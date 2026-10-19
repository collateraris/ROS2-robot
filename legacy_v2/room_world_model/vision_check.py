"""Inspect one saved RGB camera image using the runtime path checks, offline.

This module has no ROS, serial, camera capture or arming entry point. A file has
no trustworthy exposure age. Even a passing image/path check is never a motion
permission. Three shadow/teleop calls deliberately exercise ControlEngine's
actual PWM quantization, stopping distance and swept-footprint queries.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from .calibrate import ImageRectifier
from .configuration import load_config, perception_identity, resolve_path
from .engine import ControlEngine, MANUAL_ACTIONS
from .perception import CameraCalibration, CameraPerception, PerceptionConfig
from .protocol import FrameRef


class _RecordedObservation:
    """Record exact queries without changing the perception result or policy."""

    def __init__(self, observed):
        self.observed = observed
        self.queries: list[dict[str, Any]] = []

    def __getattr__(self, name):
        return getattr(self.observed, name)

    def path_clearance(self, v, w, radius, horizon, margin):
        result = self.observed.path_clearance(v, w, radius, horizon, margin)
        self.queries.append({
            "linear_mps": float(v), "angular_rps": float(w),
            "footprint_radius_m": float(radius), "horizon_s": float(horizon),
            "motion_margin_m": float(margin), "result": result,
        })
        return result


class _RecordedPerception:
    def __init__(self, perception):
        self.perception = perception
        self.last = None
        self.elapsed_s = 0.0

    def observe(self, rgb, timestamp_ns, image_valid_mask=None):
        started = time.monotonic()
        observed = self.perception.observe(rgb, timestamp_ns, image_valid_mask=image_valid_mask)
        self.elapsed_s = time.monotonic() - started
        self.last = _RecordedObservation(observed)
        return self.last


def _rectifier(config: dict, calibration: CameraCalibration):
    path = config["camera"].get("intrinsics_file")
    result = ImageRectifier(resolve_path(config, path)) if path else None
    _validate_rectification(config, calibration, result)
    return result


def _validate_rectification(config, calibration, rectifier):
    if rectifier is not None:
        rectifier.validate_calibration(calibration)
    elif calibration.accepted and not config["camera"].get("already_rectified", False):
        raise ValueError("Accepted camera needs intrinsics_file, or explicitly verified already_rectified=true")


def inspect_image(config: dict, raw_rgb: np.ndarray, perception, *, rectifier=None,
                  image_decode_s: float = 0.0) -> dict[str, Any]:
    """Return diagnostics only; injected perception supports tests without weights.

    The caller supplies the same rectifier that the PC runtime would load, or
    this function loads it from config. Inference is repeated for each candidate
    so its measured latency is not misleadingly reduced by a cached observation.
    The file's synthetic internal frame ID/timestamp are never exported as frame
    evidence. No result changes a configuration, calibration or arming state.
    """
    calibration = CameraCalibration.from_json(resolve_path(config, config["camera"]["calibration_file"]))
    if rectifier is None:
        rectifier = _rectifier(config, calibration)
    else:
        _validate_rectification(config, calibration, rectifier)
    if not isinstance(raw_rgb, np.ndarray) or raw_rgb.dtype != np.uint8 or raw_rgb.shape != (
            calibration.height, calibration.width, 3):
        raise ValueError("Image must be raw uint8 RGB at the exact calibrated resolution; no automatic resizing")
    if not math.isfinite(image_decode_s) or image_decode_s < 0:
        raise ValueError("image_decode_s must be finite and nonnegative")
    budget_s = float(config["safety"]["max_inference_s"])
    if not math.isfinite(budget_s) or budget_s <= 0:
        raise ValueError("max_inference_s must be positive and finite")

    recording = _RecordedPerception(perception)
    engine = ControlEngine(config, recording, rectifier=rectifier,
                           mode="shadow", shadow_controller="teleop")
    candidates = []
    for index, direction in enumerate(("forward", "left", "right"), 1):
        engine.set_task({"goal_id": "offline_" + direction, "kind": "manual", "direction": direction})
        # settled=False is intentional: a saved file cannot establish a real
        # completed stop/new exposure. Shadow mode independently zeros commands.
        packet = {"frame": FrameRef("offline_file", "offline_only", index).to_dict(),
                  "calibration_fingerprint": engine.fingerprint,
                  "exposure_estimate_s": 1.0, "settled": False}
        started = time.monotonic()
        proposal = engine.process(packet, raw_rgb, lease_id="")
        elapsed_s = max(0.0, time.monotonic() - started)
        diagnostic = proposal["diagnostics"]
        required_m = float(diagnostic["required_clearance_pc_m"])
        # Reuse the exact PWM quantization. Rejected turns have straight arming
        # geometry in proposal['geometry']; that fallback is NOT the turn's
        # evidence. Select the recorded query for the requested curve instead.
        v, w = engine._quantize(MANUAL_ACTIONS[direction])
        query = next((q for q in recording.last.queries
                      if math.isclose(q["linear_mps"], v, abs_tol=1e-10)
                      and math.isclose(q["angular_rps"], w, abs_tol=1e-10)), None)
        geometry = query["result"] if query is not None else None
        clearance = float(geometry.clearance_m) if geometry is not None else 0.0
        metric = bool(geometry.metric_valid) if geometry is not None else False
        covered = bool(geometry.covers_swept_footprint) if geometry is not None else False
        pipeline_s = image_decode_s + elapsed_s
        command_is_zero = proposal["command"]["linear_mps"] == proposal["command"]["angular_rps"] == 0
        if not command_is_zero:
            raise RuntimeError("Offline shadow invariant violated: no motion output is permitted")
        candidates.append({
            "direction": direction, "run_order": index,
            "requested_action": list(MANUAL_ACTIONS[direction]),
            "quantized_linear_mps": float(v), "quantized_angular_rps": float(w),
            "required_clearance_pc_m": required_m,
            "clearance_m": clearance, "metric_valid": metric,
            "covers_swept_footprint": covered,
            "clearance_sufficient": clearance >= required_m,
            "geometry_conditions_satisfied": bool(calibration.accepted and metric and covered and clearance >= required_m),
            "query": None if query is None else {k: value for k, value in query.items() if k != "result"},
            "path_reason": geometry.reason if geometry is not None else "invalid_or_unqueried_motion",
            "engine_reason": diagnostic["reason"],
            "perception": diagnostic["perception"],
            "timing": {"perception_s": recording.elapsed_s, "engine_s": elapsed_s,
                       "image_decode_plus_engine_s": pipeline_s,
                       "max_inference_s": budget_s, "budget_exceeded": pipeline_s > budget_s},
            "motor_enable_allowed": False, "shadow_command_zero": command_is_zero,
        })

    return {
        "schema_version": 1, "type": "offline_vision_inspection", "status": "completed",
        "accepted": False, "motor_enable_allowed": False,
        "reason": "saved_image_has_no_live_exposure_or_stop_evidence",
        "freshness_evidence_available": False, "source_exposure_time_s": None,
        "calibration_fingerprint": engine.fingerprint,
        "camera_calibration": asdict(calibration),
        "runtime_switches_observed_only": {k: config["runtime"].get(k) for k in (
            "allow_actuation", "allow_camera_only_research", "calibration_confirmed", "world_model_validated")},
        "perception_models": perception_identity(config),
        "rectification": {"applied": rectifier is not None,
                          "input_already_rectified": bool(config["camera"].get("already_rectified", False)),
                          "intrinsics_sha256": rectifier.file_sha256 if rectifier is not None else None,
                          "valid_pixel_fraction": float(np.mean(rectifier.valid_mask)) if rectifier is not None else None},
        "image_decode_s": image_decode_s,
        "timing_scope": "Three separate inferences; first may be cold. Includes image decode and PC computation; excludes model loading, ROS queue/network/base64, camera exposure latency and motor response. Passing is not a live deadline guarantee.",
        "candidates": candidates,
        "any_candidate_exceeds_budget": any(c["timing"]["budget_exceeded"] for c in candidates),
        "any_candidate_covers_swept_footprint": any(c["covers_swept_footprint"] for c in candidates),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline camera/person/depth/path inspection; no ROS or motor interface")
    parser.add_argument("--config", required=True)
    parser.add_argument("--image", required=True, help="One original raw RGB JPEG/PNG at the calibrated size")
    parser.add_argument("--output", required=True, help="Diagnostic JSON, never a motion permission")
    parser.add_argument("--device", help="Override runtime.device, for example cpu or cuda")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    output = Path(args.output)
    if output.exists() and not args.overwrite:
        parser.error("Output exists; use another path or --overwrite")
    report = {"schema_version": 1, "type": "offline_vision_inspection", "accepted": False,
              "motor_enable_allowed": False, "freshness_evidence_available": False,
              "source_exposure_time_s": None, "source_image": str(Path(args.image).resolve())}
    exit_code = 0
    try:
        import cv2
        config = load_config(args.config)
        calibration = CameraCalibration.from_json(resolve_path(config, config["camera"]["calibration_file"]))
        report["camera_calibration"] = asdict(calibration)
        rectifier = _rectifier(config, calibration)
        encoded = Path(args.image).read_bytes()
        started = time.monotonic()
        bgr = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("OpenCV could not decode the image")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        decode_s = time.monotonic() - started
        if rgb.shape != (calibration.height, calibration.width, 3):
            raise ValueError("Image size does not match camera calibration; do not resize the saved frame")
        started = time.monotonic()
        perception = CameraPerception.from_pretrained(
            calibration, detector_weights=str(resolve_path(config, config["perception"]["detector_weights"])),
            depth_model_id=config["perception"]["depth_model_id"],
            depth_model_revision=config["perception"].get("depth_model_revision"),
            device=args.device or config["runtime"]["device"],
            local_files_only=config["perception"]["local_files_only"],
            config=PerceptionConfig(depth_scale_m=config["perception"]["depth_scale_m"]))
        load_s = time.monotonic() - started
        report.update(inspect_image(config, rgb, perception, rectifier=rectifier, image_decode_s=decode_s))
        report["model_load_s"] = load_s
    except Exception as exc:
        report.update(status="error", error_type=type(exc).__name__, error=str(exc))
        exit_code = 1
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: {output}; accepted=false; motor_enable_allowed=false")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
