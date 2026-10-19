"""RGB semantic observations for a deliberately conservative camera-only prototype.

The depth adapter loads the *metric indoor* Depth Anything V2 checkpoint, never a
colourised/normalised depth image.  Error margins below are an engineering model
to be measured in the user's room; they are NOT statistical guarantees.  A valid
flag means calibration was explicitly accepted and the image passed basic
checks, not that monocular depth can certify collision avoidance.

Robot coordinates are x forward, y left, z up. Camera coordinates are x right,
y down, z forward; a positive pitch points the camera downwards. Timestamps are
the original camera exposure timestamp supplied by the caller. No function
replaces them with the time inference completed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Any, Protocol, Sequence

import numpy as np


AUX_NAMES = (
    "person_visible", "person_bearing", "person_proximity",
    "clearance_left", "clearance_center", "clearance_right",
)
DEFAULT_DEPTH_MODEL = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    camera_height_m: float = 0.20
    camera_pitch_down_rad: float = 0.26
    camera_forward_m: float = 0.0
    camera_left_m: float = 0.0
    robot_radius_m: float = 0.18
    robot_height_m: float = 0.25
    obstacle_margin_m: float = 0.05
    floor_tolerance_m: float = 0.025
    depth_scale_factor: float = 1.0
    depth_relative_error: float = 0.30
    depth_absolute_error_m: float = 0.20
    max_range_m: float = 3.0
    accepted: bool = False

    def __post_init__(self) -> None:
        numeric = [v for k, v in asdict(self).items() if k != "accepted"]
        if not all(np.isfinite(v) for v in numeric):
            raise ValueError("Calibration values must be finite")
        if self.width < 16 or self.height < 16 or self.fx <= 0 or self.fy <= 0:
            raise ValueError("Invalid image size or focal length")
        if not (0 <= self.cx < self.width and 0 <= self.cy < self.height):
            raise ValueError("Principal point must be inside the calibrated image")
        if not (0 <= self.camera_pitch_down_rad < math.pi / 2):
            raise ValueError("Only a forward camera with a downward pitch is supported")
        if self.camera_height_m <= 0 or self.robot_radius_m <= 0 or self.robot_height_m <= 0:
            raise ValueError("Physical dimensions must be positive")
        if self.max_range_m <= 0 or self.depth_scale_factor <= 0:
            raise ValueError("Range and metric scale must be positive")
        if not 0 <= self.depth_relative_error < 1:
            raise ValueError("depth_relative_error must be in [0, 1)")
        if min(self.depth_absolute_error_m, self.obstacle_margin_m, self.floor_tolerance_m) < 0:
            raise ValueError("Margins cannot be negative")
        if not isinstance(self.accepted, bool):
            raise ValueError("accepted must be a JSON boolean")

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraCalibration":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    @property
    def horizontal_fov_rad(self) -> float:
        return math.atan(self.cx / self.fx) + math.atan((self.width - self.cx) / self.fx)

    @property
    def blind_front_distance_m(self) -> float:
        """Ground range of bottom-centre pixel, from robot centre, not a free zone."""
        y = (self.height - 1 - self.cy) / self.fy
        s, c = math.sin(self.camera_pitch_down_rad), math.cos(self.camera_pitch_down_rad)
        denominator = s + c * y
        if denominator <= 0:
            return float("inf")
        z = self.camera_height_m / denominator
        return max(0.0, self.camera_forward_m + z * (c - s * y))


@dataclass(frozen=True)
class PersonDetection:
    xyxy: tuple[float, float, float, float]
    confidence: float


class PersonDetector(Protocol):
    def __call__(self, rgb: np.ndarray) -> Sequence[PersonDetection]: ...


class DepthEstimator(Protocol):
    def __call__(self, rgb: np.ndarray) -> np.ndarray: ...


@dataclass(frozen=True)
class PerceptionConfig:
    person_confidence: float = 0.60
    person_ambiguity_confidence: float = 0.35
    min_person_box_size_px: float = 12.0
    min_depth_m: float = 0.05
    max_raw_depth_m: float = 19.5
    min_valid_fraction: float = 0.98
    depth_scale_m: float = 3.0

    def __post_init__(self) -> None:
        if not 0 < self.person_confidence <= 1 or not 0 < self.min_valid_fraction <= 1:
            raise ValueError("Confidence and valid fraction must be in (0, 1]")
        if not 0 < self.person_ambiguity_confidence <= self.person_confidence or self.depth_scale_m <= 0:
            raise ValueError("Invalid ambiguity confidence or depth scale")
        if (not all(math.isfinite(x) for x in asdict(self).values())
                or self.min_person_box_size_px < 1
                or not 0 < self.min_depth_m < self.max_raw_depth_m):
            raise ValueError("Invalid person box size or depth range")


@dataclass(frozen=True)
class PathClearance:
    clearance_m: float
    covers_swept_footprint: bool
    metric_valid: bool
    reason: str


@dataclass
class PerceptionFrame:
    timestamp_ns: int
    clearances_m: tuple[float, float, float]
    clearance_valid: tuple[bool, bool, bool]
    person_visible: bool
    person_count: int
    person_bearing_rad: float
    person_distance_m: float
    person_distance_valid: bool
    person_confidence: float
    horizontal_fov_rad: float
    blind_front_distance_m: float
    calibrated: bool
    source: str = "metric_monocular_pseudo_measurement"
    reason: str = ""
    person_bbox: tuple[float, float, float, float] | None = None
    person_distance_estimated: bool = False
    person_observation_valid: bool = True
    clearance_observed: tuple[bool, bool, bool] = (False, False, False)
    # Geometry is intentionally not included in to_dict()/episode metadata.
    _geometry: "_FrameGeometry | None" = field(default=None, repr=False, compare=False)

    @property
    def clearance_m(self) -> tuple[float, float, float]:
        """Compatibility alias; order is left, centre, right."""
        return self.clearances_m

    @property
    def target_ambiguous(self) -> bool:
        return self.person_count > 1

    def to_dict(self) -> dict[str, Any]:
        return {name: value for name, value in vars(self).items() if not name.startswith("_")}

    def to_aux(self, depth_scale_m: float = 3.0, *, require_calibration: bool = True
               ) -> tuple[np.ndarray, np.ndarray]:
        """Six supervised targets and masks. Labels are not a motor permission.

        Setting require_calibration=False is only for explicitly marked offline
        pseudo-labels. It never changes this frame's metric_valid flags.
        """
        if not math.isfinite(depth_scale_m) or depth_scale_m <= 0:
            raise ValueError("depth_scale_m must be positive and finite")
        aux = np.zeros(6, dtype=np.float32)
        mask = np.zeros(6, dtype=bool)
        # Multiple people have no selected target; do not teach an arbitrary one.
        aux[0] = float(self.person_visible)
        mask[0] = self.person_observation_valid and self.person_count <= 1
        if self.person_visible:
            aux[1] = np.clip(self.person_bearing_rad / (self.horizontal_fov_rad / 2), -1, 1)
            mask[1] = True
            distance_known = self.person_distance_valid if require_calibration else self.person_distance_estimated
            if distance_known:
                aux[2] = np.clip(1 - self.person_distance_m / depth_scale_m, 0, 1)
                mask[2] = True
        for i, value in enumerate(self.clearances_m):
            known = self.clearance_valid[i] if require_calibration else self.clearance_observed[i]
            if known:
                aux[3 + i] = np.clip(value / depth_scale_m, 0, 1)
                mask[3 + i] = True
        return aux, mask

    def path_clearance(self, linear_mps: float, angular_rps: float,
                       footprint_radius_m: float, horizon_s: float,
                       motion_margin_m: float = 0.05) -> PathClearance:
        """Inspect a particular forward arc against this *old* camera frame.

        Distance is forward arc length until the first inflated obstacle, not
        range from the lens. Freshness, braking, pulse TTL and motion since the
        exposure remain the caller's responsibility. Coverage is a conservative
        sampled FOV/valid-pixel check, NOT a proof of free space. Unknown near or
        side regions reject the path; a central clear sector never permits a
        blind turn. No reverse or pure rotation is supported.
        """
        if self._geometry is None:
            return PathClearance(0.0, False, False, "no_projected_geometry")
        return self._geometry.path_clearance(linear_mps, angular_rps, footprint_radius_m,
                                             horizon_s, motion_margin_m)

    def clearance_for_motion(self, v: float, w: float, horizon_s: float,
                             footprint_radius_m: float | None = None,
                             motion_margin_m: float = 0.05) -> PathClearance:
        if self._geometry is None:
            return PathClearance(0.0, False, False, "no_projected_geometry")
        radius = self._geometry.calibration.robot_radius_m if footprint_radius_m is None else footprint_radius_m
        return self.path_clearance(v, w, radius, horizon_s, motion_margin_m)


@dataclass
class _FrameGeometry:
    calibration: CameraCalibration
    points_xy: np.ndarray
    point_errors_m: np.ndarray
    depth_valid: np.ndarray
    radial_lower_m: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.radial_lower_m = np.linalg.norm(self.points_xy, axis=1) - self.point_errors_m

    def path_clearance(self, v: float, w: float, radius: float,
                       horizon: float, margin: float) -> PathClearance:
        if not all(math.isfinite(x) for x in (v, w, radius, horizon, margin)):
            return PathClearance(0.0, False, False, "non_finite_motion")
        if v <= 0 or radius <= 0 or horizon <= 0 or margin < 0:
            return PathClearance(0.0, False, False, "only_positive_forward_arcs_supported")
        if abs(w * horizon) > 0.35:
            return PathClearance(0.0, False, False, "heading_change_outside_camera_envelope")
        c = self.calibration
        if not c.accepted:
            return PathClearance(0.0, False, False, "calibration_not_accepted")
        path_length = min(v * horizon, c.max_range_m)
        # Extra radius covers spatial sampling gaps; the final result is still
        # conditional on monocular depth accuracy and visibility assumptions.
        ds = min(0.015, max(path_length / 12, 0.001))
        steps = min(1000, max(2, int(math.ceil(path_length / ds)) + 1))
        distances = np.linspace(0.0, path_length, steps)
        times = distances / v
        if abs(w) < 1e-8:
            centres = np.stack((distances, np.zeros_like(distances)), axis=1)
        else:
            centres = np.stack((v / w * np.sin(w * times), v / w * (1 - np.cos(w * times))), axis=1)
        inflated = radius + margin + ds
        # Safe broad-phase rejection: a path of length s never moves its centre
        # farther than s from the origin. Obstacles whose uncertainty disc lies
        # entirely beyond s+inflated cannot touch any point of that path.
        nearby = self.radial_lower_m <= path_length + inflated
        nearby_points = self.points_xy[nearby]
        nearby_radii_squared = (inflated + self.point_errors_m[nearby]) ** 2
        clearance = path_length
        end = len(centres)
        for i, centre in enumerate(centres):
            if nearby_points.size:
                offset = nearby_points - centre
                squared = np.sum(offset * offset, axis=1)
                if np.any(squared <= nearby_radii_squared):
                    clearance = max(0.0, float(distances[i]) - ds)
                    end = i + 1
                    break

        # Check all newly swept ground cells, not only the centreline. We exclude
        # only the initially occupied physical footprint, never an assumed free
        # neighbourhood or a persistent "operator checked it" radius.
        grid_step = 0.025
        axis = np.arange(-inflated, inflated + grid_step / 2, grid_step)
        xx, yy = np.meshgrid(axis, axis)
        offsets = np.stack((xx.ravel(), yy.ravel()), axis=1)
        offsets = offsets[np.linalg.norm(offsets, axis=1) <= inflated]
        angles = np.linspace(0, 2 * math.pi, 96, endpoint=False)
        rim = np.stack((inflated * np.cos(angles), inflated * np.sin(angles)), axis=1)
        offsets = np.concatenate((offsets, rim), axis=0)
        swept = (centres[:end, None, :] + offsets[None, :, :]).reshape(-1, 2)
        new = swept[np.linalg.norm(swept, axis=1) > radius + 1e-5]
        if new.size == 0:
            return PathClearance(clearance, False, True, "no_observed_new_footprint")
        dx, dy = new[:, 0] - c.camera_forward_m, new[:, 1] - c.camera_left_m
        sp, cp = math.sin(c.camera_pitch_down_rad), math.cos(c.camera_pitch_down_rad)
        z_camera = cp * dx + sp * c.camera_height_m
        y_camera = -sp * dx + cp * c.camera_height_m
        with np.errstate(divide="ignore", invalid="ignore"):
            u = c.cx - c.fx * dy / z_camera
            row = c.cy + c.fy * y_camera / z_camera
        # Leave an image-edge guard for distortion and pixel sampling.
        inside = (z_camera > 0) & (u >= 2) & (u < c.width - 2) & (row >= 2) & (row < c.height - 2)
        if not bool(np.all(inside)):
            return PathClearance(clearance, False, True, "swept_footprint_enters_camera_blind_region")
        ui, vi = u.astype(np.int64), row.astype(np.int64)
        if not bool(np.all(self.depth_valid[vi, ui])):
            return PathClearance(clearance, False, True, "unknown_depth_in_swept_footprint")
        return PathClearance(clearance, True, True, "sampled_visible_metric_estimate")


class YOLOPersonDetector:
    """Local YOLO11 detector. Weight downloads require allow_download=True."""

    def __init__(self, weights: str = "models/yolo11n.pt", device: str = "cuda",
                 confidence: float = 0.35, *, allow_download: bool = False) -> None:
        if not allow_download and not Path(weights).is_file():
            raise FileNotFoundError(f"Local YOLO weights not found: {weights}; download explicitly before robot use")
        from ultralytics import YOLO
        self.weights = str(Path(weights).expanduser().resolve())
        self.model = YOLO(weights)
        names = self.model.names
        person_name = names.get(0, names.get("0", "")) if isinstance(names, dict) else names[0]
        if str(person_name).casefold() != "person":
            raise ValueError("Detector class 0 must be 'person'; refusing a mismatched class mapping")
        self.device = "0" if device == "cuda" else device
        self.confidence = confidence

    def __call__(self, rgb: np.ndarray) -> list[PersonDetection]:
        # Ultralytics numpy inputs use BGR, unlike PIL/transformers inputs.
        bgr = np.ascontiguousarray(rgb[..., ::-1])
        results = self.model.predict(bgr, classes=[0], conf=self.confidence,
                                     device=self.device, verbose=False, imgsz=640)
        if not results or results[0].boxes is None:
            return []
        boxes = results[0].boxes
        xyxy, confidence = boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy()
        return [PersonDetection(tuple(map(float, box)), float(score)) for box, score in zip(xyxy, confidence)]


class MetricDepthAnything:
    """Transformers adapter for calibrated metric indoor depth (metres).

    The model's depth_estimation_type is checked at runtime. A relative-depth
    checkpoint, colourised pipeline["depth"], or normalised image is rejected.
    Raw predictions still have room/camera dependent metric errors.
    """

    def __init__(self, model_id: str = DEFAULT_DEPTH_MODEL, device: str = "cuda",
                 *, local_files_only: bool = True, revision: str | None = None) -> None:
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch = torch
        self.device = device
        self.model_id = model_id
        self.revision = revision
        self.processor = AutoImageProcessor.from_pretrained(model_id, local_files_only=local_files_only, revision=revision)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_id, local_files_only=local_files_only, revision=revision)
        if getattr(self.model.config, "depth_estimation_type", None) != "metric":
            raise ValueError("Refusing non-metric depth checkpoint; relative depth has no metre scale")
        self.model.to(device).eval()

    def __call__(self, rgb: np.ndarray) -> np.ndarray:
        from PIL import Image
        torch = self.torch
        inputs = self.processor(images=Image.fromarray(rgb, mode="RGB"), return_tensors="pt")
        inputs = {name: tensor.to(self.device) for name, tensor in inputs.items()}
        with torch.inference_mode():
            output = self.model(**inputs).predicted_depth
            depth = torch.nn.functional.interpolate(output.unsqueeze(1), size=rgb.shape[:2],
                                                     mode="bilinear", align_corners=False)
        return depth[0, 0].float().cpu().numpy()


class CameraPerception:
    def __init__(self, calibration: CameraCalibration, detector: PersonDetector,
                 depth_model: DepthEstimator, config: PerceptionConfig | None = None) -> None:
        self.calibration = calibration
        self.detector = detector
        self.depth_model = depth_model
        self.config = config or PerceptionConfig()

    @classmethod
    def from_pretrained(cls, calibration: CameraCalibration,
                        detector_weights: str = "models/yolo11n.pt",
                        depth_model_id: str = DEFAULT_DEPTH_MODEL, device: str = "cuda",
                        local_files_only: bool = True,
                        config: PerceptionConfig | None = None,
                        depth_model_revision: str | None = None) -> "CameraPerception":
        return cls(calibration,
                   YOLOPersonDetector(detector_weights, device, allow_download=not local_files_only),
                   MetricDepthAnything(depth_model_id, device, local_files_only=local_files_only,
                                       revision=depth_model_revision), config)

    def observe(self, rgb: np.ndarray, timestamp_ns: int,
                image_valid_mask: np.ndarray | None = None) -> PerceptionFrame:
        c, cfg = self.calibration, self.config
        if not isinstance(timestamp_ns, (int, np.integer)) or timestamp_ns <= 0:
            raise ValueError("A positive original exposure timestamp in nanoseconds is required")
        if rgb.dtype != np.uint8 or rgb.shape != (c.height, c.width, 3):
            raise ValueError(f"Expected calibrated RGB uint8[{c.height},{c.width},3], got {rgb.shape}/{rgb.dtype}")
        depth_raw = np.asarray(self.depth_model(rgb), dtype=np.float64)
        if depth_raw.shape != rgb.shape[:2]:
            raise ValueError("Depth adapter must return the original calibrated image size")
        depth = depth_raw * c.depth_scale_factor
        valid = np.isfinite(depth) & (depth_raw >= cfg.min_depth_m) & (depth_raw <= cfg.max_raw_depth_m)
        if image_valid_mask is not None:
            if image_valid_mask.shape != depth.shape or image_valid_mask.dtype != bool:
                raise ValueError("image_valid_mask must be bool[height,width] for rectification borders")
            valid &= image_valid_mask
        # NaNs never become far/free values. Zero is a placeholder only.
        safe_depth = np.where(valid, depth, 0.0)
        error = c.depth_relative_error * safe_depth + c.depth_absolute_error_m
        lower_depth = np.maximum(0.0, safe_depth - error)
        yy, xx = np.indices(depth.shape, dtype=np.float64)
        xray, yray = (xx - c.cx) / c.fx, (yy - c.cy) / c.fy
        sp, cp = math.sin(c.camera_pitch_down_rad), math.cos(c.camera_pitch_down_rad)
        forward_ray = cp - sp * yray
        up_ray = -sp - cp * yray
        forward = c.camera_forward_m + forward_ray * safe_depth
        left = c.camera_left_m - xray * safe_depth
        height = c.camera_height_m + up_ray * safe_depth
        height_error = np.abs(up_ray) * error
        # Keep any pixel whose depth-error interval can intersect the robot's
        # vertical envelope. Exclude only definite floor/above-body surfaces.
        obstacle = (valid & (forward > 0)
                    & (height + height_error > c.floor_tolerance_m)
                    & (height - height_error < c.robot_height_m + c.obstacle_margin_m))
        horizontal_error = np.hypot(forward_ray, xray) * error
        # Keep every potential obstacle so a thin chair leg is not skipped merely
        # because it fell between downsampled point-cloud pixels.
        points = np.stack((forward[obstacle], left[obstacle]), axis=1).astype(np.float32)
        point_error = horizontal_error[obstacle].astype(np.float32)
        # A finite depth from an occluding surface is insufficient evidence that
        # a ground cell is visible. Require consistency with the calibrated
        # planar floor. Stairs, holes and unseen ground remain unsupported.
        floor_observed = (valid & (height - height_error <= c.floor_tolerance_m)
                          & (height + height_error >= -c.floor_tolerance_m))
        geometry = _FrameGeometry(c, points, point_error, floor_observed)

        # Sector estimates use angular footprint inflation. They are convenient
        # labels only; runtime approval uses the complete proposed path above.
        image_bearing = np.arctan2(-xray, forward_ray)
        half = c.horizontal_fov_rad / 2
        edges = np.array([half, half / 3, -half / 3, -half])
        ranges: list[float] = []
        observed: list[bool] = []
        distance = np.linalg.norm(points, axis=1) if len(points) else np.empty(0)
        point_bearing = np.arctan2(points[:, 1], points[:, 0]) if len(points) else np.empty(0)
        inflation = c.robot_radius_m + c.obstacle_margin_m + point_error
        angle_error = np.arcsin(np.clip(inflation / np.maximum(distance, 1e-6), 0, 1))
        # Only the lower field sees the traversable floor; missing pixels there
        # invalidate a sector rather than turning an occlusion into free space.
        floor_ray = up_ray < -1e-5
        for i in range(3):
            region = (image_bearing <= edges[i]) & (image_bearing >= edges[i + 1]) & floor_ray
            coverage = float(valid[region].mean()) if np.any(region) else 0.0
            seen = coverage >= cfg.min_valid_fraction
            selected = ((point_bearing + angle_error >= edges[i + 1])
                        & (point_bearing - angle_error <= edges[i]))
            estimate = c.max_range_m
            if np.any(selected):
                estimate = max(0.0, min(c.max_range_m, float(np.min(distance[selected] - inflation[selected]))))
            # Finite depth alone cannot establish an unbounded empty room.
            # Cap range by the farthest observed lower ground range per sector.
            if seen:
                visible_range = forward_ray[region] * lower_depth[region] + c.camera_forward_m
                estimate = min(estimate, max(0.0, float(np.percentile(visible_range, 90))))
            ranges.append(float(estimate if seen else 0.0))
            observed.append(bool(seen))

        detections: list[PersonDetection] = []
        for det in self.detector(rgb):
            box = np.asarray(det.xyxy, dtype=np.float64)
            if box.shape != (4,) or not np.all(np.isfinite(box)) or not math.isfinite(det.confidence):
                continue
            x1, y1, x2, y2 = box
            if (det.confidence >= cfg.person_ambiguity_confidence and x2 - x1 >= cfg.min_person_box_size_px
                    and y2 - y1 >= cfg.min_person_box_size_px and x2 > 0 and y2 > 0
                    and x1 < c.width and y1 < c.height):
                detections.append(det)
        unique = len(detections) == 1 and detections[0].confidence >= cfg.person_confidence
        bearing, person_distance, person_confidence = 0.0, 0.0, 0.0
        person_estimated = False
        person_box = None
        if unique:
            det = detections[0]
            x1, y1, x2, y2 = det.xyxy
            x1, x2 = np.clip([x1, x2], 0, c.width - 1)
            y1, y2 = np.clip([y1, y2], 0, c.height - 1)
            person_box = (float(x1), float(y1), float(x2), float(y2))
            person_confidence = float(det.confidence)
            centre_x, centre_y = (x1 + x2) / 2, (y1 + y2) / 2
            b_xray = (centre_x - c.cx) / c.fx
            b_yray = (centre_y - c.cy) / c.fy
            bearing = math.atan2(-b_xray, cp - sp * b_yray)
            # Torso crop reduces background leakage; nearest valid torso estimate
            # is used for a conservative stopping distance, not median background.
            px1, px2 = int(x1 + .25 * (x2 - x1)), int(x1 + .75 * (x2 - x1)) + 1
            py1, py2 = int(y1 + .20 * (y2 - y1)), int(y1 + .75 * (y2 - y1)) + 1
            patch_valid = valid[py1:py2, px1:px2]
            if patch_valid.size and float(patch_valid.mean()) >= cfg.min_valid_fraction:
                radial = np.hypot(forward[py1:py2, px1:px2], left[py1:py2, px1:px2])
                radial -= horizontal_error[py1:py2, px1:px2]
                person_distance = max(0.0, float(np.percentile(radial[patch_valid], 10)))
                person_estimated = True
        if not c.accepted:
            reason = "calibration_not_accepted"
        elif len(detections) > 1:
            reason = "multiple_people_no_target_selection"
        elif not unique:
            reason = "person_not_visible"
        else:
            reason = "metric_monocular_estimate_requires_runtime_safety_gate"
        return PerceptionFrame(
            timestamp_ns=int(timestamp_ns), clearances_m=tuple(ranges),
            clearance_valid=tuple(bool(c.accepted and x) for x in observed),
            person_visible=unique, person_count=len(detections), person_bearing_rad=float(bearing),
            person_distance_m=float(person_distance), person_distance_valid=bool(c.accepted and person_estimated),
            person_confidence=person_confidence, horizontal_fov_rad=c.horizontal_fov_rad,
            blind_front_distance_m=c.blind_front_distance_m, calibrated=c.accepted,
            reason=reason, person_bbox=person_box, person_distance_estimated=person_estimated,
            person_observation_valid=len(detections) == 0 or unique,
            clearance_observed=tuple(observed), _geometry=geometry,
        )
