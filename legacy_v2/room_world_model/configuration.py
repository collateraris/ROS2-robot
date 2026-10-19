"""Configuration and calibration provenance, shared by collection and runtime."""
from __future__ import annotations
from pathlib import Path
import hashlib
import json
import math
import yaml


def load_config(path: str | Path) -> dict:
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("Expected a YAML configuration mapping")
    for section in ("runtime", "robot", "camera", "safety", "perception", "planner"):
        if not isinstance(value.get(section), dict):
            raise ValueError(f"Missing configuration section {section}")
    for name in ("allow_actuation", "allow_camera_only_research", "calibration_confirmed",
                 "world_model_validated", "allow_wm_trials"):
        if name in value["runtime"] and type(value["runtime"][name]) is not bool:
            raise ValueError(f"runtime.{name} must be a YAML boolean")
    value["_path"] = str(path)
    value["_root"] = str(path.parent.parent)
    return value


def resolve_path(config: dict, path: str) -> Path:
    result = Path(path).expanduser()
    return result if result.is_absolute() else Path(config["_root"]) / result


def perception_identity(config: dict) -> dict:
    """Shared logical model identity; never hash or open GPU weights on the Pi.

    The default detector identifier is its filename, so Windows/PC/Pi cache
    directories do not create false mismatches. This cannot detect a modified
    file kept under the same name; use a fixed trusted release and explicit
    detector_model_id when distributing custom weights. A pinned depth revision
    is passed to Transformers by the downloader and inference adapters.
    """
    values = config["perception"]
    depth = values.get("depth_model_id")
    if not isinstance(depth, str) or not depth.strip():
        raise ValueError("perception.depth_model_id must be a nonempty model identifier")
    detector = values.get("detector_model_id")
    if detector is None:
        weights = values.get("detector_weights")
        if not isinstance(weights, str) or not weights.strip():
            raise ValueError("perception.detector_weights must name a detector")
        detector = weights.replace("\\", "/").rstrip("/").rsplit("/", 1)[-1]
    if not isinstance(detector, str) or not detector.strip() or detector in {".", ".."}:
        raise ValueError("Invalid logical detector identifier")
    revision = values.get("depth_model_revision")
    if revision is not None and (not isinstance(revision, str) or not revision.strip()):
        raise ValueError("depth_model_revision must be a nonempty revision or null")
    return {"depth_model_id": depth.strip(), "depth_model_revision": revision,
            "detector_model_id": detector.strip()}


def calibration_fingerprint(config: dict) -> str:
    """Calibration and perception identity; host paths and runtime switches excluded."""
    camera_file = resolve_path(config, config["camera"]["calibration_file"])
    calibration = yaml.safe_load(camera_file.read_text(encoding="utf-8"))
    # 'accepted' is an operational switch, not a camera parameter.
    if isinstance(calibration, dict):
        calibration = {k: v for k, v in calibration.items() if k != "accepted"}
    camera = {k: v for k, v in config["camera"].items()
              if k not in {"device", "calibration_file", "intrinsics_file"}}
    robot = {k: v for k, v in config["robot"].items() if k not in {"serial_port"}}
    intrinsics = config["camera"].get("intrinsics_file", "")
    intrinsics_sha = hashlib.sha256(resolve_path(config, intrinsics).read_bytes()).hexdigest() if intrinsics else None
    blob = json.dumps({"camera": camera, "robot": robot, "intrinsics": calibration, "rectification_sha256": intrinsics_sha,
                       "depth_scale_m": config["perception"]["depth_scale_m"],
                       "perception_models": perception_identity(config)},
                      sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(blob.encode()).hexdigest()


def finite(value, name: str, low: float = 0.0, high: float = 100.0) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"Invalid {name}")
    return value


def recording_provenance(config: dict) -> dict:
    """Hash local source/config once before collection; no model downloads on Pi."""
    source = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob("*.py")):
        source.update(path.name.encode())
        source.update(path.read_bytes())
    firmware = resolve_path(config, "firmware/roomwm_drive/roomwm_drive.ino")
    return {
        "software_version": "0.2.0", "source_tree_sha256": source.hexdigest(),
        "robot_config_sha256": hashlib.sha256(Path(config["_path"]).read_bytes()).hexdigest(),
        "firmware_sha256": hashlib.sha256(firmware.read_bytes()).hexdigest() if firmware.is_file() else None,
        "camera_calibration_sha256": hashlib.sha256(
            resolve_path(config, config["camera"]["calibration_file"]).read_bytes()).hexdigest(),
        "perception_revisions": perception_identity(config),
    }
