"""Passive calibration utilities. No command in this module controls motors.

Subcommands:
  intrinsics  Calibrate a fixed-resolution USB camera from chessboard images.
  probe       Measure read blocking time and delivered frame intervals only.
  v4l2-probe  Inspect real DQBUF timestamp, flags, sequence and queue timing.
  latency-record  Record GPIO LED edges and their first camera observations.
  latency-fit Fit/check an empirical optical delay bound from separate sessions.
  depth       Fit an empirical metric-depth error envelope from measured CSV.

The probe cannot measure exposure age. No utility sets accepted=True or derives
a safe motion speed. See docs/DATA_AND_CALIBRATION_RU.md for the optical delay
experiment, separate validation sessions, camera mounting, and limitations.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import hashlib
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from .perception import CameraCalibration, DEFAULT_DEPTH_MODEL


def _write_json(path: str | Path, value: dict[str, Any], *, overwrite: bool = False) -> None:
    path = Path(path)
    if path.exists() and not overwrite:
        raise FileExistsError(f"Refusing to overwrite {path}; use --overwrite explicitly")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")


def distribution(values: list[float] | np.ndarray) -> dict[str, float] | None:
    values = np.asarray(values, dtype=float)
    if not values.size:
        return None
    return {"min": float(values.min()), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "p99": float(np.percentile(values, 99)),
            "max": float(values.max()), "mean": float(values.mean())}


class ImageRectifier:
    """Undistort without cropping/resizing; channel order is preserved.

    Pass valid_mask to CameraPerception.observe when rectification is performed
    on the PC. If done before transmission, preserve the corresponding rectified
    intrinsics and the valid-border mask at the receiving side.
    """

    def __init__(self, intrinsics_json: str | Path) -> None:
        import cv2
        self.cv2 = cv2
        raw = Path(intrinsics_json).read_bytes()
        self.file_sha256 = hashlib.sha256(raw).hexdigest()
        values = json.loads(raw.decode("utf-8"))
        self.width, self.height = int(values["image_width"]), int(values["image_height"])
        matrix = np.asarray(values["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(values["distortion_coefficients"], dtype=np.float64)
        rectified = np.asarray(values["rectified_camera_matrix"], dtype=np.float64)
        if matrix.shape != (3, 3) or rectified.shape != (3, 3):
            raise ValueError("Intrinsic matrices must be 3x3")
        if not all(np.all(np.isfinite(x)) for x in (matrix, distortion, rectified)):
            raise ValueError("Intrinsics must be finite")
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            matrix, distortion, None, rectified, (self.width, self.height), cv2.CV_32FC1)
        self.valid_mask = ((self.map_x >= 0) & (self.map_x < self.width - 1)
                           & (self.map_y >= 0) & (self.map_y < self.height - 1))
        self.rectified_camera_matrix = rectified

    @classmethod
    def from_json(cls, path: str | Path) -> "ImageRectifier":
        return cls(path)

    def validate_calibration(self, calibration: CameraCalibration) -> None:
        expected = np.array([[calibration.fx, 0, calibration.cx],
                             [0, calibration.fy, calibration.cy], [0, 0, 1]])
        if ((calibration.width, calibration.height) != (self.width, self.height)
                or not np.allclose(expected, self.rectified_camera_matrix, rtol=1e-6, atol=1e-5)):
            raise ValueError("camera.json must use this rectifier's resolution and rectified camera matrix")

    def __call__(self, image: np.ndarray) -> np.ndarray:
        if image.shape[:2] != (self.height, self.width):
            raise ValueError("Resolution changed after camera calibration")
        return self.cv2.remap(image, self.map_x, self.map_y, self.cv2.INTER_LINEAR,
                              borderMode=self.cv2.BORDER_CONSTANT, borderValue=0)

    def rectify_rgb(self, rgb: np.ndarray) -> np.ndarray:
        return self(rgb)


Rectifier = ImageRectifier


def calibrate_intrinsics(folder: str | Path, cols: int, rows: int, square_size_m: float,
                         *, min_views: int = 15, alpha: float = 0.0) -> dict[str, Any]:
    import cv2
    if cols < 3 or rows < 3 or square_size_m <= 0 or not math.isfinite(square_size_m):
        raise ValueError("Provide inner-corner cols/rows >=3 and a positive measured square size")
    if not 0 <= alpha <= 1 or min_views < 6:
        raise ValueError("alpha must be [0,1] and min_views at least 6")
    folder = Path(folder)
    paths = sorted(p for p in folder.iterdir() if p.suffix.casefold() in {".jpg", ".jpeg", ".png", ".bmp"})
    if not paths:
        raise ValueError("No calibration images in folder")
    object_template = np.zeros((cols * rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m
    object_points, image_points, accepted_paths, rejected = [], [], [], []
    image_size = None
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejected.append({"file": path.name, "reason": "cannot_decode"})
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            raise ValueError("All chessboard images must use exactly the same camera resolution")
        # SB returns subpixel corners directly; standard corner refinement is
        # retained as a fallback for boards not found by SB.
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, .001))
        if not found:
            rejected.append({"file": path.name, "reason": "board_not_found"})
            continue
        object_points.append(object_template.copy())
        image_points.append(np.asarray(corners, dtype=np.float32))
        accepted_paths.append(path.name)
    if len(object_points) < min_views:
        raise ValueError(f"Only {len(object_points)} valid board views; need {min_views} diverse views")
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None)
    rectified, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, image_size, alpha, image_size)
    errors = []
    for obj, img, rotation, translation in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rotation, translation, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((img - projected) ** 2, axis=-1)))))
    template = CameraCalibration(width=image_size[0], height=image_size[1], fx=float(rectified[0, 0]),
                                 fy=float(rectified[1, 1]), cx=float(rectified[0, 2]), cy=float(rectified[1, 2]),
                                 accepted=False)
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "image_width": image_size[0], "image_height": image_size[1],
        "distortion_model": "opencv_pinhole", "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.ravel().tolist(),
        "rectified_camera_matrix": rectified.tolist(), "rectification_alpha": alpha,
        "valid_roi": [int(x) for x in roi], "crop_applied": False,
        "board_inner_corners": [cols, rows], "square_size_m": square_size_m,
        "fit_rms_reprojection_px": float(rms),
        "rms_per_image_px": dict(zip(accepted_paths, errors)), "rejected_images": rejected,
        "camera_calibration_template": asdict(template),
        "extrinsics_and_metric_depth_not_calibrated": True,
        "accepted": False,
    }


def probe_camera(device: str | int = 0, *, width: int = 640, height: int = 480,
                 requested_fps: float = 15.0, frames: int = 100, warmup: int = 5,
                 backend: str = "v4l2", display: bool = False) -> dict[str, Any]:
    """Measure API blocking and delivery only; exposure latency stays unknown."""
    import cv2
    if frames < 2 or warmup < 0 or width <= 0 or height <= 0 or requested_fps <= 0:
        raise ValueError("Invalid passive camera probe settings")
    backends = {"v4l2": cv2.CAP_V4L2, "any": cv2.CAP_ANY,
                "dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}
    if backend not in backends:
        raise ValueError("Unsupported backend")
    device = int(device) if isinstance(device, str) and device.isdigit() else device
    capture = cv2.VideoCapture(device, backends[backend])
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("Cannot open camera; stop other capture processes first")
    records: list[dict[str, Any]] = []
    setting_results = {}
    effective_size = None
    try:
        settings = {
            "width": (cv2.CAP_PROP_FRAME_WIDTH, width),
            "height": (cv2.CAP_PROP_FRAME_HEIGHT, height),
            "fps": (cv2.CAP_PROP_FPS, requested_fps),
            "buffersize": (cv2.CAP_PROP_BUFFERSIZE, 1),
            "mjpg": (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
        }
        for name, (prop, value) in settings.items():
            setting_results[name] = bool(capture.set(prop, value))
        backend_name = capture.getBackendName()
        fps_property = float(capture.get(cv2.CAP_PROP_FPS))
        for i in range(frames + warmup):
            start = time.monotonic_ns()
            ok, image = capture.read()
            end = time.monotonic_ns()
            if not ok or image is None:
                raise RuntimeError(f"Camera read failed on frame {i}")
            effective_size = [int(image.shape[1]), int(image.shape[0])]
            if i >= warmup:
                records.append({"frame": i - warmup, "read_start_monotonic_ns": start,
                                "read_end_monotonic_ns": end, "read_duration_ms": (end - start) / 1e6})
            if display:
                cv2.putText(image, "READ COMPLETION ONLY; EXPOSURE TIME UNKNOWN", (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 255), 1)
                cv2.imshow("Passive camera probe (q to stop)", image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        capture.release()
        if display:
            cv2.destroyAllWindows()
    ends = np.array([x["read_end_monotonic_ns"] for x in records], dtype=np.float64)
    intervals_ms = np.diff(ends) / 1e6
    delivered_fps = 1e3 / float(np.mean(intervals_ms)) if len(intervals_ms) and np.mean(intervals_ms) > 0 else None
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(), "device": str(device), "backend": backend_name,
        "requested_size": [width, height], "actual_size": effective_size,
        "requested_fps": requested_fps, "reported_fps_property": fps_property,
        "property_set_return_values": setting_results,
        "property_acceptance_does_not_prove_device_behavior": True,
        "frames": len(records), "read_duration_ms": distribution([x["read_duration_ms"] for x in records]),
        "read_end_interval_ms": distribution(intervals_ms), "delivered_fps": delivered_fps,
        "optical_exposure_latency_measured": False, "capture_delay_bound_s": None,
        "motor_enable_allowed": False, "records": records,
    }


def fit_depth_bounds(csv_path: str | Path, *, absolute_error_m: float = .05,
                     extra_relative_margin: float = .05, min_per_split: int = 10) -> dict[str, Any]:
    """Fit symmetric residual bounds on fit sessions; only evaluate check ones.

    CSV columns: predicted_m,reference_m,session,split; split is fit or check.
    reference_m is camera-axis Z (not oblique tape/radial range). This empirical
    envelope can fail outside its measured sample and never sets accepted=True.
    """
    if not math.isfinite(absolute_error_m) or absolute_error_m < 0:
        raise ValueError("Absolute margin must be finite and nonnegative")
    if not math.isfinite(extra_relative_margin) or not 0 <= extra_relative_margin < 1:
        raise ValueError("Extra relative margin must be in [0,1)")
    with Path(csv_path).open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"predicted_m", "reference_m", "session", "split"} <= set(reader.fieldnames or []):
            raise ValueError("CSV requires predicted_m,reference_m,session,split columns")
        rows = list(reader)
    predictions, references, splits, sessions = [], [], [], []
    for row in rows:
        predicted, reference = float(row["predicted_m"]), float(row["reference_m"])
        if not all(math.isfinite(x) and x > 0 for x in (predicted, reference)):
            raise ValueError("Depth measurements must be positive finite metres")
        split, session = row["split"].strip().lower(), row["session"].strip()
        if split not in {"fit", "check"} or not session:
            raise ValueError("Use nonempty session names and split=fit/check")
        predictions.append(predicted)
        references.append(reference)
        splits.append(split)
        sessions.append(session)
    p, target = np.asarray(predictions), np.asarray(references)
    fit, check = np.asarray(splits) == "fit", np.asarray(splits) == "check"
    if fit.sum() < min_per_split or check.sum() < min_per_split:
        raise ValueError(f"Need at least {min_per_split} measurements in each fit/check split")
    fit_sessions = {sessions[i] for i in range(len(rows)) if fit[i]}
    check_sessions = {sessions[i] for i in range(len(rows)) if check[i]}
    if fit_sessions & check_sessions:
        raise ValueError("A session cannot appear in both fit and check; avoid adjacent-frame leakage")
    scale = float(np.median(target[fit] / p[fit]))
    scaled = p * scale
    errors = np.abs(scaled - target)
    relative = max(0.0, float(np.max((errors[fit] - absolute_error_m) / scaled[fit]))) + extra_relative_margin
    lower = np.maximum(0.0, scaled * (1 - relative) - absolute_error_m)
    upper = scaled * (1 + relative) + absolute_error_m
    violations = (target < lower - 1e-9) | (target > upper + 1e-9)
    usable = relative < 1.0 and not bool(np.any(violations[check]))
    suggestion = {"depth_scale_factor": scale, "depth_relative_error": relative,
                  "depth_absolute_error_m": absolute_error_m, "accepted": False} if relative < 1 else None
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(), "source_model": DEFAULT_DEPTH_MODEL,
        "kind": "empirical_symmetric_depth_residual_envelope_not_safety_certificate",
        "reference_semantics": "camera_axis_Z_metres", "fit_count": int(fit.sum()), "check_count": int(check.sum()),
        "fit_sessions": sorted(fit_sessions), "check_sessions": sorted(check_sessions),
        "fit_absolute_error_m": distribution(errors[fit]), "check_absolute_error_m": distribution(errors[check]),
        "check_envelope_violations": int(violations[check].sum()),
        "check_underestimated_closeness_count": int((target[check] < lower[check] - 1e-9).sum()),
        "empirical_holdout_pass": usable, "suggested_camera_update": suggestion,
        "motor_enable_allowed": False, "accepted": False,
        "violating_csv_line_numbers": [i + 2 for i in range(len(rows)) if check[i] and violations[i]],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    intrinsic = sub.add_parser("intrinsics", help="chessboard calibration without motion")
    intrinsic.add_argument("--images", required=True, type=Path)
    intrinsic.add_argument("--cols", required=True, type=int)
    intrinsic.add_argument("--rows", required=True, type=int)
    intrinsic.add_argument("--square-size-m", required=True, type=float)
    intrinsic.add_argument("--min-views", type=int, default=15)
    intrinsic.add_argument("--alpha", type=float, default=0.0)
    intrinsic.add_argument("--camera-config-output", type=Path)
    probe = sub.add_parser("probe", help="read timing/FPS only, exposure latency unknown")
    probe.add_argument("--device", default="0")
    probe.add_argument("--width", type=int, default=640)
    probe.add_argument("--height", type=int, default=480)
    probe.add_argument("--fps", type=float, default=15)
    probe.add_argument("--frames", type=int, default=100)
    probe.add_argument("--warmup", type=int, default=5)
    probe.add_argument("--backend", choices=["v4l2", "any", "dshow", "msmf"], default="v4l2")
    probe.add_argument("--display", action="store_true")
    native = sub.add_parser("v4l2-probe", help="real Linux DQBUF timestamps and flags, no motor control")
    native.add_argument("--config", required=True, type=Path)
    native.add_argument("--frames", type=int, default=100)
    native.add_argument("--warmup", type=int, default=5)
    optical = sub.add_parser("latency-record", help="toggle an explicitly selected GPIO LED and record visual latency")
    optical.add_argument("--config", required=True, type=Path)
    optical.add_argument("--csv-output", required=True, type=Path)
    optical.add_argument("--gpio-pin", required=True, type=int, help="LED GPIO pin in BCM numbering")
    optical.add_argument("--roi", required=True, type=int, nargs=4, metavar=("X", "Y", "WIDTH", "HEIGHT"))
    optical.add_argument("--split", required=True, choices=["fit", "check"])
    optical.add_argument("--condition", required=True, help="lighting/exposure/USB/CPU measurement condition")
    optical.add_argument("--events", type=int, default=40)
    optical.add_argument("--min-hold-s", type=float, default=.5)
    optical.add_argument("--max-hold-s", type=float, default=1.5)
    optical.add_argument("--event-timeout-s", type=float, default=8.)
    optical.add_argument("--off-max", type=float, default=60.)
    optical.add_argument("--on-min", type=float, default=180.)
    optical.add_argument("--stable-frames", type=int, default=2)
    optical.add_argument("--seed", type=int, default=2026)
    optical.add_argument("--session")
    latency = sub.add_parser("latency-fit", help="fit optical max-plus-margin; evaluate independent held-out sessions")
    latency.add_argument("--csv", required=True, type=Path, nargs="+")
    latency.add_argument("--fps", required=True, type=float)
    latency.add_argument("--extra-frame-periods", type=float, default=2.)
    latency.add_argument("--extra-margin-s", type=float, default=.05)
    latency.add_argument("--min-per-split", type=int, default=20)
    depth = sub.add_parser("depth", help="empirical depth bounds from separate measured sessions")
    depth.add_argument("--csv", required=True, type=Path)
    depth.add_argument("--absolute-error-m", type=float, default=.05)
    depth.add_argument("--extra-relative-margin", type=float, default=.05)
    depth.add_argument("--camera-config", type=Path)
    depth.add_argument("--camera-config-output", type=Path)
    for child in (intrinsic, probe, native, optical, latency, depth):
        child.add_argument("--output", required=True, type=Path)
        child.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.output.exists() and not args.overwrite:
        parser.error("Output exists; use another filename or explicit --overwrite")
    if args.command == "intrinsics":
        result = calibrate_intrinsics(args.images, args.cols, args.rows, args.square_size_m,
                                      min_views=args.min_views, alpha=args.alpha)
        if args.camera_config_output:
            _write_json(args.camera_config_output, result["camera_calibration_template"], overwrite=args.overwrite)
    elif args.command == "probe":
        result = probe_camera(args.device, width=args.width, height=args.height,
                              requested_fps=args.fps, frames=args.frames, warmup=args.warmup,
                              backend=args.backend, display=args.display)
    elif args.command == "v4l2-probe":
        from .configuration import load_config
        from .timing_calibration import probe_v4l2
        camera = load_config(args.config)["camera"]
        if camera.get("backend", "v4l2") != "v4l2":
            parser.error("v4l2-probe requires camera.backend=v4l2 in the chosen config")
        result = probe_v4l2(camera["device"], width=camera["width"], height=camera["height"],
            requested_fps=camera["requested_fps"], pixel_format=camera.get("pixel_format", "MJPG"),
            frames=args.frames, warmup=args.warmup, buffer_count=camera.get("buffer_count", 4),
            timing_config=camera.get("timing"), controls=camera.get("controls"))
    elif args.command == "latency-record":
        from .configuration import load_config
        from .timing_calibration import collect_optical_latency
        if args.output.resolve() == args.csv_output.resolve():
            parser.error("JSON report and optical CSV must use different output files")
        result = collect_optical_latency(load_config(args.config)["camera"], args.csv_output,
            gpio_pin=args.gpio_pin, roi=tuple(args.roi), split=args.split, condition=args.condition,
            events=args.events, min_hold_s=args.min_hold_s, max_hold_s=args.max_hold_s,
            event_timeout_s=args.event_timeout_s, off_max=args.off_max, on_min=args.on_min,
            stable_frames=args.stable_frames, seed=args.seed, session=args.session, overwrite=args.overwrite)
    elif args.command == "latency-fit":
        from .timing_calibration import fit_optical_latency
        if any(args.output.resolve() == path.resolve() for path in args.csv):
            parser.error("Latency report must not overwrite its source CSV")
        result = fit_optical_latency(args.csv, requested_fps=args.fps,
            extra_frame_periods=args.extra_frame_periods, extra_margin_s=args.extra_margin_s,
            min_per_split=args.min_per_split)
    elif args.command == "depth":
        result = fit_depth_bounds(args.csv, absolute_error_m=args.absolute_error_m,
                                  extra_relative_margin=args.extra_relative_margin)
        if args.camera_config_output:
            if args.camera_config is None:
                parser.error("--camera-config-output requires --camera-config")
            if not result["empirical_holdout_pass"]:
                _write_json(args.output, result, overwrite=args.overwrite)
                parser.error("Depth envelope failed held-out checks; camera config was not changed")
            camera = CameraCalibration.from_json(args.camera_config)
            updated = replace(camera, **result["suggested_camera_update"])
            _write_json(args.camera_config_output, asdict(updated), overwrite=args.overwrite)
    _write_json(args.output, result, overwrite=args.overwrite)
    summary = {key: value for key, value in result.items() if key not in {"records", "rms_per_image_px"}}
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
