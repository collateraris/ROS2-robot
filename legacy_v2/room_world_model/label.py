"""Offline RGB teacher labels and auditable manual corrections for episodes.

Example:
    python -m room_world_model.label episode.npz --output episode_labeled.npz \
        --config config/robot.yaml

Input schema: images[N,128,128], timestamps[N] in seconds, actions[N-1,2],
dt[N-1], aux[N,6], aux_mask[N,6], transition_valid[N-1], metadata_json.
The teacher also requires rgb_images[N,H,W,3], or rgb_paths[N] relative to the
episode file. Original RGB is necessary for person recognition and calibrated
geometry; grayscale world-model frames are not silently upscaled as a substitute.

Corrections JSON accepts {"frames": [{"frame": 3, "aux": [...6...],
"aux_mask": [...6 booleans...]}]}. The dictionary form {"frames": {"3": {...}}}
also works. Manual corrections affect training labels only; they cannot approve
runtime camera calibration or bypass the independent motor gate.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np

from .configuration import calibration_fingerprint, load_config, perception_identity, resolve_path
from .perception import AUX_NAMES, CameraCalibration, CameraPerception, DEFAULT_DEPTH_MODEL, PerceptionConfig


def _decode_scalar_text(array: np.ndarray) -> str:
    value = array.item()
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def _check_recording_fingerprint(metadata: dict[str, Any], config: dict) -> str:
    expected = calibration_fingerprint(config)
    if metadata.get("calibration_fingerprint") != expected:
        raise ValueError("Recording and label --config calibration/model fingerprints differ; do not relabel provenance")
    return expected


def _check_teacher_config(config: dict, perception, rectifier, already_rectified: bool,
                          depth_scale_m: float) -> None:
    """Check supplied adapters too, so the Python API cannot mix teachers accidentally."""
    settings = config["perception"]
    if not math.isclose(depth_scale_m, float(settings["depth_scale_m"]), rel_tol=0, abs_tol=1e-9):
        raise ValueError("Label depth_scale_m differs from --config")
    if perception is None:
        return
    expected_calibration = CameraCalibration.from_json(resolve_path(config, config["camera"]["calibration_file"]))
    if perception.calibration != expected_calibration:
        raise ValueError("Teacher camera calibration differs from --config")
    if getattr(perception, "config", None) != PerceptionConfig(depth_scale_m=settings["depth_scale_m"]):
        raise ValueError("Teacher perception settings differ from --config runtime settings")
    weights = getattr(getattr(perception, "detector", None), "weights", None)
    expected_weights = resolve_path(config, settings["detector_weights"]).resolve()
    if not isinstance(weights, (str, Path)) or Path(weights).expanduser().resolve() != expected_weights:
        raise ValueError("Teacher detector weights differ from --config")
    depth = getattr(perception, "depth_model", None)
    if (getattr(depth, "model_id", None) != settings["depth_model_id"]
            or getattr(depth, "revision", None) != settings.get("depth_model_revision")):
        raise ValueError("Teacher depth model/revision differs from --config")
    intrinsics = config["camera"].get("intrinsics_file")
    expected_sha = hashlib.sha256(resolve_path(config, intrinsics).read_bytes()).hexdigest() if intrinsics else None
    if ((rectifier is not None) != bool(intrinsics)
            or (rectifier is not None and rectifier.file_sha256 != expected_sha)
            or already_rectified != bool(config["camera"].get("already_rectified", False))):
        raise ValueError("Teacher RGB rectification differs from --config")


def _read_rgb(data: dict[str, np.ndarray], index: int, episode_path: Path) -> np.ndarray:
    if "rgb_images" in data:
        rgb = data["rgb_images"][index]
    elif "rgb_paths" in data:
        from PIL import Image
        raw = data["rgb_paths"][index]
        path = Path(raw.decode("utf-8") if isinstance(raw, bytes) else str(raw))
        if not path.is_absolute():
            path = episode_path.parent / path
        with Image.open(path) as image:
            rgb = np.asarray(image.convert("RGB"))
    else:
        raise ValueError("No original RGB. Record rgb_images or rgb_paths; use --manual-only for existing corrections")
    if rgb.ndim != 3 or rgb.shape[-1] != 3 or rgb.dtype != np.uint8:
        raise ValueError(f"Invalid RGB frame {index}: expected uint8[H,W,3]")
    return rgb


def _correction_items(corrections: dict[str, Any] | list[dict[str, Any]]
                      ) -> list[tuple[int, dict[str, Any]]]:
    frames = corrections.get("frames", corrections) if isinstance(corrections, dict) else corrections
    if isinstance(frames, dict):
        result = []
        for key, value in frames.items():
            if not isinstance(value, dict):
                raise ValueError("Every manual frame correction must be an object")
            result.append((int(key), value))
        return result
    if isinstance(frames, list):
        result = []
        for value in frames:
            if not isinstance(value, dict) or "frame" not in value:
                raise ValueError("Every correction list entry needs a frame index")
            result.append((int(value["frame"]), value))
        return result
    raise ValueError("Corrections must contain frames as an object or a list")


def apply_manual_corrections(aux: np.ndarray, mask: np.ndarray,
                             corrections: dict[str, Any] | list[dict[str, Any]],
                             *, depth_scale_m: float = 3.0,
                             horizontal_fov_rad: float | None = None) -> list[int]:
    """Apply exact aux/mask or metric semantic corrections in-place.

    Metric format fields: person_visible, person_bearing_rad,
    person_distance_m, clearance_m (left/centre/right), clearance_valid.
    A supplied finite measurement is assumed to have been manually checked.
    Setting a valid flag false masks it without implying free space.
    """
    if aux.ndim != 2 or aux.shape[1] != 6 or mask.shape != aux.shape:
        raise ValueError("Expected aux and mask with shape [N,6]")
    if not math.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError("depth_scale_m must be positive")
    changed = []
    for index, item in _correction_items(corrections):
        supported = {"frame", "aux", "aux_mask", "person_visible", "person_bearing_rad",
                     "person_distance_m", "person_distance_valid", "clearance_m", "clearance_valid", "note"}
        if set(item) - supported:
            raise ValueError(f"Unsupported manual correction fields: {sorted(set(item) - supported)}")
        if not 0 <= index < len(aux):
            raise ValueError(f"Manual frame index out of bounds: {index}")
        row, flags = aux[index].copy(), mask[index].copy()
        if "aux" in item:
            values = np.asarray(item["aux"], dtype=np.float32)
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise ValueError(f"Frame {index}: aux must contain six finite values")
            row[:] = values
            if "aux_mask" not in item:
                raise ValueError("Direct aux corrections require explicit aux_mask")
        if "aux_mask" in item:
            values = np.asarray(item["aux_mask"])
            if values.shape != (6,) or values.dtype != bool:
                raise ValueError("aux_mask must contain six JSON booleans")
            flags[:] = values
        if "person_visible" in item:
            if not isinstance(item["person_visible"], bool):
                raise ValueError("person_visible must be a JSON boolean")
            row[0], flags[0] = float(item["person_visible"]), True
            if not item["person_visible"]:
                row[1:3], flags[1:3] = 0, False
        if "person_bearing_rad" in item:
            if horizontal_fov_rad is None or horizontal_fov_rad <= 0:
                raise ValueError("Metric bearing correction requires camera horizontal FOV")
            bearing = float(item["person_bearing_rad"])
            if not math.isfinite(bearing):
                raise ValueError("Bearing must be finite")
            row[1], flags[1] = np.clip(bearing / (horizontal_fov_rad / 2), -1, 1), True
        if "person_distance_m" in item:
            distance = float(item["person_distance_m"])
            if not math.isfinite(distance) or distance < 0:
                raise ValueError("Person distance must be nonnegative and finite")
            row[2], flags[2] = np.clip(1 - distance / depth_scale_m, 0, 1), True
        if "person_distance_valid" in item:
            if not isinstance(item["person_distance_valid"], bool):
                raise ValueError("person_distance_valid must be boolean")
            flags[2] = item["person_distance_valid"]
        if "clearance_m" in item:
            values = np.asarray(item["clearance_m"], dtype=np.float64)
            if values.shape != (3,) or np.any(values < 0) or not np.all(np.isfinite(values)):
                raise ValueError("clearance_m must contain three nonnegative finite values")
            row[3:], flags[3:] = np.clip(values / depth_scale_m, 0, 1), True
        if "clearance_valid" in item:
            values = np.asarray(item["clearance_valid"])
            if values.shape != (3,) or values.dtype != bool:
                raise ValueError("clearance_valid must contain three JSON booleans")
            flags[3:] = values
        lower = np.array([0, -1, 0, 0, 0, 0], dtype=np.float32)
        if np.any(row < lower) or np.any(row > 1) or not np.all(np.isfinite(row)):
            raise ValueError("Aux values must lie in [0,1], except bearing in [-1,1]")
        if flags[0] and row[0] == 0:
            row[1:3], flags[1:3] = 0, False
        row[~flags] = 0.0
        aux[index], mask[index] = row, flags
        changed.append(index)
    return changed


def label_episode(input_path: str | Path, output_path: str | Path,
                  perception: CameraPerception | None, *,
                  corrections: dict[str, Any] | list[dict[str, Any]] | None = None,
                  depth_scale_m: float = 3.0, allow_uncalibrated_pseudo: bool = False,
                  overwrite: bool = False, teacher_metadata: dict[str, Any] | None = None,
                  rectifier: Any | None = None, input_already_rectified: bool = False,
                  config: dict | None = None) -> dict[str, Any]:
    source, destination = Path(input_path), Path(output_path)
    if destination.exists() and not overwrite:
        raise FileExistsError(f"Refusing to overwrite {destination}; pass --overwrite explicitly")
    if perception is not None and rectifier is not None:
        rectifier.validate_calibration(perception.calibration)
    if perception is not None and perception.calibration.accepted and rectifier is None and not input_already_rectified:
        raise ValueError("Accepted calibration requires --intrinsics-json or explicitly --already-rectified RGB")
    with np.load(source, allow_pickle=False) as archive:
        data = {name: archive[name].copy() for name in archive.files}
    if "images" not in data or "timestamps" not in data:
        raise ValueError("Episode requires images and timestamps")
    count = len(data["images"])
    if data["timestamps"].shape != (count,) or not np.all(np.isfinite(data["timestamps"])):
        raise ValueError("timestamps must contain one finite exposure time per frame")
    if "rgb_images" in data and len(data["rgb_images"]) != count:
        raise ValueError("rgb_images length differs from images")
    if "rgb_paths" in data and len(data["rgb_paths"]) != count:
        raise ValueError("rgb_paths length differs from images")
    metadata = {}
    if "metadata_json" in data:
        metadata = json.loads(_decode_scalar_text(data["metadata_json"]))
        if not isinstance(metadata, dict):
            raise ValueError("metadata_json must contain an object")
    verified_fingerprint = None
    if config is not None:
        verified_fingerprint = _check_recording_fingerprint(metadata, config)
        _check_teacher_config(config, perception, rectifier, input_already_rectified, depth_scale_m)
    previous_scale = float(metadata.get("depth_scale_m", 3.0))
    if perception is None and not math.isclose(previous_scale, depth_scale_m):
        raise ValueError("Partial manual corrections must keep the episode's existing depth_scale_m")
    aux = np.asarray(data.get("aux", np.zeros((count, 6))), dtype=np.float32).copy()
    mask = np.asarray(data.get("aux_mask", np.zeros((count, 6))), dtype=bool).copy()
    if aux.shape != (count, 6) or mask.shape != aux.shape:
        raise ValueError("aux/aux_mask shape must be [N,6]")
    label_sources = np.asarray(data.get("label_source", np.full(count, "unlabeled")), dtype="U32").copy()
    if label_sources.shape != (count,):
        raise ValueError("label_source must have shape [N]")
    summaries = []
    if perception is not None:
        for i in range(count):
            timestamp_ns = int(round(float(data["timestamps"][i]) * 1_000_000_000))
            rgb = _read_rgb(data, i, source)
            if rectifier is not None:
                rgb = rectifier(rgb)
            frame = perception.observe(rgb, timestamp_ns,
                image_valid_mask=rectifier.valid_mask if rectifier is not None else None)
            aux[i], mask[i] = frame.to_aux(depth_scale_m, require_calibration=not allow_uncalibrated_pseudo)
            label_sources[i] = "pseudo_teacher"
            summaries.append(frame.to_dict())
    elif corrections is None:
        raise ValueError("Provide a teacher or manual corrections")
    changed = []
    if corrections is not None:
        changed = apply_manual_corrections(
            aux, mask, corrections, depth_scale_m=depth_scale_m,
            horizontal_fov_rad=perception.calibration.horizontal_fov_rad if perception else None)
        label_sources[changed] = "manual_correction"
    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "aux_names": list(AUX_NAMES), "depth_scale_m": depth_scale_m,
        "kind": "pseudo_labels_with_optional_manual_corrections" if perception else "manual_corrections",
        "not_safety_ground_truth": True,
        "replaces_all_targets": perception is not None,
        "allow_uncalibrated_pseudo": allow_uncalibrated_pseudo,
        "manually_corrected_frames": sorted(set(changed)),
        "manual_corrections": corrections,
        "calibration": as_calibration_dict(perception),
        "teacher": teacher_metadata or {},
        "verified_calibration_fingerprint": verified_fingerprint,
        "rgb_rectification": {"applied": rectifier is not None,
                              "input_already_rectified": input_already_rectified,
                              "intrinsics_sha256": rectifier.file_sha256 if rectifier is not None else None},
    }
    metadata["depth_scale_m"] = depth_scale_m
    if perception is not None:
        metadata["uncalibrated_pseudo_labels"] = bool(allow_uncalibrated_pseudo and not perception.calibration.accepted)
        metadata["labels_from_accepted_calibration"] = perception.calibration.accepted
        # A full offline re-teach invalidates any older verified teacher flag.
        # Manual-only corrections retain it only with unchanged normalization.
        metadata["label_fingerprint_verified"] = verified_fingerprint is not None
    else:
        metadata["label_fingerprint_verified"] = metadata.get("label_fingerprint_verified") is True
    provenance["label_fingerprint_verified"] = metadata["label_fingerprint_verified"]
    metadata.setdefault("annotation_history", []).append(provenance)
    data.update(aux=aux, aux_mask=mask, label_source=label_sources,
                metadata_json=np.array(json.dumps(metadata, ensure_ascii=False)))
    if summaries:
        data["perception_json"] = np.array([json.dumps(x, ensure_ascii=False) for x in summaries])
    if "rgb_paths" in data and source.parent.resolve() != destination.parent.resolve():
        # Preserve what each relative RGB path means after moving the labelled
        # episode to another folder (e.g. raw/ -> train/).
        paths = []
        for raw in data["rgb_paths"]:
            original = Path(raw.decode("utf-8") if isinstance(raw, bytes) else str(raw))
            resolved = original if original.is_absolute() else source.parent / original
            paths.append(os.path.relpath(resolved.resolve(), destination.parent.resolve()))
        data["rgb_paths"] = np.asarray(paths)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=destination.stem + "_", suffix=".npz", delete=False) as tmp:
        temporary = Path(tmp.name)
    try:
        np.savez_compressed(temporary, **data)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return {"frames": count, "manual_frames": len(set(changed)),
            "valid_labels_per_dimension": mask.sum(axis=0).tolist(), "output": str(destination),
            "label_fingerprint_verified": metadata["label_fingerprint_verified"]}


def as_calibration_dict(perception: CameraPerception | None) -> dict[str, Any] | None:
    if perception is None:
        return None
    from dataclasses import asdict
    return asdict(perception.calibration)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path, help="Authoritative recording/runtime configuration; verifies label provenance")
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--intrinsics-json", type=Path)
    parser.add_argument("--already-rectified", action="store_true", default=None)
    parser.add_argument("--detector-weights")
    parser.add_argument("--depth-model")
    parser.add_argument("--depth-model-revision", help="Use the same pinned HF revision as runtime")
    parser.add_argument("--device")
    parser.add_argument("--depth-scale-m", type=float)
    parser.add_argument("--corrections", type=Path)
    parser.add_argument("--manual-only", action="store_true")
    parser.add_argument("--allow-uncalibrated-pseudo", action="store_true")
    parser.add_argument("--allow-model-download", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    config = None
    if args.config is not None:
        owned = ("calibration", "intrinsics_json", "already_rectified", "detector_weights",
                 "depth_model", "depth_model_revision", "depth_scale_m")
        if any(getattr(args, key) is not None for key in owned) or args.allow_model_download:
            parser.error("--config owns camera, rectification, detector, depth, revision and scale; do not combine it with their individual flags or --allow-model-download")
        config = load_config(args.config)
        p, c = config["perception"], config["camera"]
        args.calibration = resolve_path(config, c["calibration_file"])
        args.intrinsics_json = resolve_path(config, c["intrinsics_file"]) if c.get("intrinsics_file") else None
        args.already_rectified = bool(c.get("already_rectified", False))
        args.detector_weights = str(resolve_path(config, p["detector_weights"]))
        args.depth_model, args.depth_model_revision = p["depth_model_id"], p.get("depth_model_revision")
        args.depth_scale_m = float(p["depth_scale_m"])
        args.allow_model_download = not p["local_files_only"]
        args.device = args.device or config["runtime"]["device"]
        # Reject a wrong recording/config before loading any heavyweight model.
        with np.load(args.episode, allow_pickle=False) as archive:
            metadata = json.loads(_decode_scalar_text(archive["metadata_json"])) if "metadata_json" in archive else {}
        _check_recording_fingerprint(metadata, config)
    else:
        args.detector_weights = args.detector_weights or "models/yolo11n.pt"
        args.depth_model = args.depth_model or DEFAULT_DEPTH_MODEL
        args.depth_scale_m = 3.0 if args.depth_scale_m is None else args.depth_scale_m
        args.already_rectified = bool(args.already_rectified)
        args.device = args.device or "cuda"
    if not args.manual_only and args.calibration is None:
        parser.error("--config or --calibration is required for teacher labels")
    if args.manual_only and args.corrections is None:
        parser.error("--manual-only requires --corrections")
    corrections = json.loads(args.corrections.read_text(encoding="utf-8")) if args.corrections else None
    if config is None and args.intrinsics_json and args.already_rectified:
        parser.error("Choose --intrinsics-json for raw RGB or --already-rectified, not both")
    from .calibrate import ImageRectifier
    rectifier = ImageRectifier(args.intrinsics_json) if args.intrinsics_json else None
    perception = None if args.manual_only else CameraPerception.from_pretrained(
        CameraCalibration.from_json(args.calibration), detector_weights=args.detector_weights,
        depth_model_id=args.depth_model, device=args.device, local_files_only=not args.allow_model_download,
        depth_model_revision=args.depth_model_revision,
        config=PerceptionConfig(depth_scale_m=args.depth_scale_m))
    report = label_episode(
        args.episode, args.output, perception, corrections=corrections, depth_scale_m=args.depth_scale_m,
        allow_uncalibrated_pseudo=args.allow_uncalibrated_pseudo, overwrite=args.overwrite,
        teacher_metadata={"detector_weights": args.detector_weights, "depth_model": args.depth_model,
                          "depth_model_revision": args.depth_model_revision,
                          "perception_identity": perception_identity(config) if config is not None else None},
        rectifier=rectifier, input_already_rectified=args.already_rectified, config=config)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
