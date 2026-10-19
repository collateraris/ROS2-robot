"""Leakage-resistant episode validation and chronological sequence sampling.

NPZ is loaded with allow_pickle=False. Nothing here explores on the robot.
Inputs are demonstrations collected under human control and motor ACK timing.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np


AUX_NAMES = (
    "person_visible", "person_bearing", "person_proximity",
    "clearance_left", "clearance_center", "clearance_right",
)
DATASET_SCHEMA_VERSION = 2
ACTION_SEMANTICS = "mcu_confirmed_piecewise_normalized_forward_yaw"
CONTROL_REPRESENTATION = "piecewise_mcu_confirmed"
MAX_SEGMENTS = 24


@dataclass(frozen=True)
class EpisodeInfo:
    path: Path
    n_frames: int
    metadata: dict[str, Any]
    valid_transitions: int
    aux_valid_counts: tuple[int, ...]
    has_exact_segments: bool

    @property
    def split_group(self) -> str:
        return str(self.metadata.get("session_id") or self.metadata.get("episode_id") or self.path.resolve())

    def group_value(self, axis: str) -> str:
        value = self.metadata.get(f"{axis}_id")
        if not isinstance(value, str) or not value.strip() or value.lower() in {"unknown", "unspecified"}:
            raise ValueError(f"{self.path.name}: provide a recorded {axis}_id for --split-by {axis}")
        return value


def read_episode(path: str | Path) -> dict[str, np.ndarray]:
    with np.load(Path(path), allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def episode_metadata(data: dict[str, np.ndarray]) -> dict[str, Any]:
    value = data.get("metadata_json")
    if value is None:
        return {}
    try:
        metadata = json.loads(str(np.asarray(value).item()))
    except (ValueError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("metadata_json must be a scalar JSON object string") from exc
    if not isinstance(metadata, dict):
        raise ValueError("metadata_json must contain an object")
    return metadata


def validate_episode(path: str | Path, *, max_dt_s: float = 5.0) -> EpisodeInfo:
    path = Path(path)
    data = read_episode(path)
    metadata = episode_metadata(data)
    if type(metadata.get("schema_version")) is not int or metadata["schema_version"] != DATASET_SCHEMA_VERSION:
        raise ValueError(f"{path.name}: v2 schema_version=2 required. Legacy v1 data must be re-exported "
                         "from MCU-confirmed raw logs; changing metadata cannot recover actual pulse timing. "
                         "Use python -m room_world_model.data --audit-v1 PATH for a read-only audit.")
    if metadata.get("action_semantics") != ACTION_SEMANTICS or metadata.get("control_representation") != CONTROL_REPRESENTATION:
        raise ValueError("v2 requires exact ordered MCU-confirmed segments; interval means are diagnostics only")
    if type(metadata.get("synthetic")) is not bool:
        raise ValueError("metadata synthetic must be an explicit JSON boolean")
    if not metadata["synthetic"] and metadata.get("mcu_confirmation_validated") is not True:
        raise ValueError("Real v2 episodes require validated START/STOP MCU confirmation provenance")
    if not isinstance(metadata.get("session_id"), str) or not metadata["session_id"].strip():
        raise ValueError("v2 episodes require an explicit nonempty session_id")
    required = {"images", "timestamps", "actions", "dt", "aux", "aux_mask", "transition_valid"}
    if missing := required - data.keys():
        raise ValueError(f"{path.name}: missing keys {sorted(missing)}")
    images = data["images"]
    if images.dtype != np.uint8 or images.ndim != 3 or images.shape[1:] != (128, 128):
        raise ValueError(f"{path.name}: images must be uint8 [N,128,128]")
    n = len(images)
    if n < 3:
        raise ValueError(f"{path.name}: at least three chronological frames required")
    if not metadata["synthetic"]:
        for key in ("exposure_earliest_ns", "exposure_latest_ns"):
            if key not in data or data[key].shape != (n,) or data[key].dtype.kind not in "ui":
                raise ValueError("Real v2 data requires per-frame integer exposure bounds [N]")
        exposure_low = data["exposure_earliest_ns"].astype(np.float64) / 1e9
        exposure_high = data["exposure_latest_ns"].astype(np.float64) / 1e9
        if np.any(exposure_low > exposure_high) or np.any(exposure_low < 0):
            raise ValueError("Invalid camera exposure bounds")
        if np.any(data["timestamps"] < exposure_low-1e-8) or np.any(data["timestamps"] > exposure_high+1e-8):
            raise ValueError("Exposure timestamp lies outside recorded acquisition interval")
    shapes = {"timestamps": (n,), "actions": (n - 1, 2), "dt": (n - 1,),
              "aux": (n, 6), "aux_mask": (n, 6), "transition_valid": (n - 1,)}
    for name, shape in shapes.items():
        if data[name].shape != shape:
            raise ValueError(f"{path.name}: {name} shape {data[name].shape} != {shape}")
    for name in ("timestamps", "actions", "dt", "aux"):
        if not np.isfinite(data[name]).all():
            raise ValueError(f"{path.name}: {name} contains nonfinite values (unknown labels use mask=False)")
    if data["aux_mask"].dtype != np.bool_ or data["transition_valid"].dtype != np.bool_:
        raise ValueError("aux_mask and transition_valid must be boolean arrays")
    elapsed = np.diff(data["timestamps"].astype(np.float64))
    if np.any(elapsed <= 0) or not np.allclose(elapsed, data["dt"], rtol=1e-4, atol=0.002):
        raise ValueError(f"{path.name}: timestamps must increase and dt must match exposure-to-exposure time")
    if (np.abs(data["actions"]) > 1.00001).any():
        raise ValueError(f"{path.name}: normalized controls exceed [-1,1]")
    mask = data["aux_mask"]
    low = np.array([0, -1, 0, 0, 0, 0])
    high = np.ones(6)
    if np.any(mask & ((data["aux"] < low - 1e-5) | (data["aux"] > high + 1e-5))):
        raise ValueError(f"{path.name}: valid auxiliary targets outside normalized bounds")
    exact = "action_segments" in data
    if exact != ("segment_mask" in data):
        raise ValueError("action_segments and segment_mask must appear together")
    if not exact:
        raise ValueError("v2 requires exact action_segments and segment_mask; no interval-mean fallback")
    valid = data["transition_valid"]
    if not metadata["synthetic"]:
        limit = float(metadata.get("camera", {}).get("max_training_exposure_uncertainty_s", .1))
        if not np.isfinite(limit) or limit <= 0:
            raise ValueError("Invalid maximum training exposure uncertainty")
        uncertain = (exposure_high-exposure_low)/2 > limit+1e-8
        if np.any(valid & (uncertain[:-1] | uncertain[1:])):
            raise ValueError("Training transition exceeds configured camera exposure uncertainty")
    if np.any(valid & (data["dt"] > max_dt_s)):
        raise ValueError("Valid camera interval exceeds max_dt_s; invalidate and reset across gaps")
    if exact:
        segments, segment_mask = data["action_segments"], data["segment_mask"]
        if segments.ndim != 3 or segments.shape[0] != n - 1 or segments.shape[-1] != 3:
            raise ValueError("action_segments must be [N-1,K,3]")
        if not 1 <= segments.shape[1] <= MAX_SEGMENTS:
            raise ValueError(f"Each interval may contain 1..{MAX_SEGMENTS} segments; coalesce equal commands")
        if segment_mask.shape != segments.shape[:2] or segment_mask.dtype != np.bool_:
            raise ValueError("segment_mask must be boolean [N-1,K]")
        if not np.isfinite(segments).all():
            raise ValueError("Action segments contain nonfinite values")
        if np.any(segment_mask & (segments[..., 2] <= 0)) or np.any(
                segment_mask & valid[:, None] & (segments[..., 2] > max_dt_s)):
            raise ValueError("Active segment durations must be >0 and <= max_dt_s; split long known holds")
        if np.any(segment_mask[..., None] & (np.abs(segments[..., :2]) > 1.00001)):
            raise ValueError("Segment controls exceed [-1,1]")
        # Prefix padding makes segment order unambiguous.
        if np.any(np.diff(segment_mask.astype(np.int8), axis=1) > 0):
            raise ValueError("Active segments must form a contiguous prefix")
        durations = np.where(segment_mask, segments[..., 2], 0.0)
        if not np.allclose(durations.sum(1)[valid], data["dt"][valid], rtol=1e-4, atol=0.002):
            raise ValueError("Exact segment durations do not cover valid camera intervals")
        means = (segments[..., :2] * durations[..., None]).sum(1) / np.maximum(data["dt"][:, None], 1e-9)
        if not np.allclose(means[valid], data["actions"][valid], rtol=1e-4, atol=0.002):
            raise ValueError("Interval means disagree with exact effective control segments")
    elif np.any(valid & (data["dt"] > max_dt_s)):
        raise ValueError("Valid interval exceeds model max_dt_s; invalidate gaps or preserve exact segments")
    return EpisodeInfo(path, n, metadata, int(valid.sum()),
                       tuple(int(x) for x in mask.sum(0)), exact)


def discover_episodes(folder: str | Path) -> list[Path]:
    paths = sorted(Path(folder).rglob("*.npz"))
    if not paths:
        raise ValueError(f"No episode .npz files under {folder}")
    return paths


def split_episodes(paths: Iterable[str | Path], *, validation_fraction: float = 0.2,
                   seed: int = 42, max_dt_s: float = 5.0,
                   split_by: str = "session") -> tuple[list[EpisodeInfo], list[EpisodeInfo]]:
    if not 0 < validation_fraction < 1:
        raise ValueError("validation_fraction must lie strictly between 0 and 1")
    infos = [validate_episode(path, max_dt_s=max_dt_s) for path in paths]
    if len({str(info.path.resolve()) for info in infos}) != len(infos):
        raise ValueError("Duplicate episode paths are not permitted")
    groups = episode_groups(infos, split_by=split_by)
    if len(groups) < 2:
        raise ValueError("Collect at least two separate sessions for held-out validation; no frame-level split")
    keys = sorted(groups)
    np.random.default_rng(seed).shuffle(keys)
    n_validation = min(len(keys) - 1, max(1, round(len(keys) * validation_fraction)))
    val_groups = set(keys[:n_validation])
    train = [info for key, values in groups.items() if key not in val_groups for info in values]
    val = [info for key, values in groups.items() if key in val_groups for info in values]
    return train, val


def episode_groups(infos: list[EpisodeInfo], *, split_by: str = "session") -> dict[str, list[EpisodeInfo]]:
    """Connected components preserve BOTH each session and the selected axis.

    A session spanning layouts A and B must not join train A to test B. The
    union of equal session IDs and equal selected-axis IDs handles that case.
    Lighting/day overlaps on other axes are reported, never claimed unseen.
    """
    if split_by not in {"session", "layout", "day"}:
        raise ValueError("split_by must be session, layout or day")
    parent = list(range(len(infos)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    seen: dict[tuple[str, str], int] = {}
    for i, info in enumerate(infos):
        keys = [("session", info.group_value("session")), (split_by, info.group_value(split_by))]
        for key in keys:
            if key in seen:
                parent[root(i)] = root(seen[key])
            else:
                seen[key] = i
    groups: dict[int, list[EpisodeInfo]] = {}
    for i, info in enumerate(infos):
        groups.setdefault(root(i), []).append(info)
    return {min(info.split_group for info in values): values for values in groups.values()}


def partition_episodes(paths: Iterable[str | Path], *, validation_fraction: float = 0.2,
                       test_fraction: float = 0.2, split_by: str = "layout", seed: int = 42,
                       max_dt_s: float = 5.0) -> dict[str, list[EpisodeInfo]]:
    """Three disjoint partitions; test never participates in checkpoint selection."""
    if not (0 < validation_fraction < 1 and 0 < test_fraction < 1 and validation_fraction + test_fraction < 1):
        raise ValueError("Positive validation/test fractions must sum to less than one")
    infos = [validate_episode(path, max_dt_s=max_dt_s) for path in paths]
    if len({str(info.path.resolve()) for info in infos}) != len(infos):
        raise ValueError("Duplicate episode paths are not permitted")
    groups = episode_groups(infos, split_by=split_by)
    if len(groups) < 3:
        raise ValueError(f"Need at least three independent {split_by} groups for train/validation/test; "
                         "sessions spanning groups remain indivisible")
    keys = sorted(groups)
    np.random.default_rng(seed).shuffle(keys)
    n_val = min(len(keys) - 2, max(1, round(len(keys) * validation_fraction)))
    n_test = min(len(keys) - n_val - 1, max(1, round(len(keys) * test_fraction)))
    selection = {"validation": keys[:n_val], "test": keys[n_val:n_val + n_test], "train": keys[n_val + n_test:]}
    partitions = {name: [info for key in chosen for info in groups[key]] for name, chosen in selection.items()}
    # Identical recordings with altered filenames/session IDs are not independent.
    fingerprints: dict[str, str] = {}
    for name, values in partitions.items():
        for info in values:
            digest = episode_content_sha256(info.path)
            if digest in fingerprints and fingerprints[digest] != name:
                raise ValueError("Duplicate recorded content crosses dataset partitions; remove copied episodes")
            fingerprints[digest] = name
    return partitions


def episode_content_sha256(path: str | Path) -> str:
    data = read_episode(path)
    digest = hashlib.sha256()
    for key in ("images", "timestamps", "action_segments", "segment_mask"):
        array = np.ascontiguousarray(data[key])
        digest.update(key.encode())
        digest.update(str((array.dtype.str, array.shape)).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def split_audit(partitions: dict[str, list[EpisodeInfo]], split_by: str) -> dict[str, Any]:
    axes = ("session", "layout", "day", "lighting")
    coverage = {name: {axis: sorted({str(info.metadata.get(f"{axis}_id", "unknown")) for info in infos})
                      for axis in axes} for name, infos in partitions.items()}
    overlap = {}
    names = list(partitions)
    for i, first in enumerate(names):
        for second in names[i + 1:]:
            overlap[f"{first}__{second}"] = {
                axis: sorted(set(coverage[first][axis]) & set(coverage[second][axis])) for axis in axes}
    return {"split_by": split_by, "coverage": coverage, "overlaps": overlap,
            "interpretation": "Session and selected split axis are held out. Overlap on another axis "
                              "does not establish generalization on that axis. Test is unused during training."}


class SequenceDataset:
    """Each sample is an unbroken, chronological window from one episode.

    Plain numpy returns work with torch DataLoader's default collate; this
    module can also validate recordings on the Pi without importing torch.
    """

    def __init__(self, episodes: Iterable[EpisodeInfo | str | Path], *, sequence_length: int = 6,
                 stride: int = 2, max_cache: int = 4, max_dt_s: float = 5.0,
                 max_segments: int = MAX_SEGMENTS):
        if sequence_length < 2 or stride < 1 or max_cache < 1:
            raise ValueError("sequence_length>=2, stride>=1 and max_cache>=1 required")
        self.episodes = [x if isinstance(x, EpisodeInfo) else validate_episode(x, max_dt_s=max_dt_s)
                         for x in episodes]
        self.sequence_length = sequence_length
        self.max_cache = max_cache
        self.max_segments = max_segments
        self._cache: OrderedDict[int, dict[str, np.ndarray]] = OrderedDict()
        self.windows: list[tuple[int, int]] = []
        for episode_i, info in enumerate(self.episodes):
            data = read_episode(info.path)
            if info.has_exact_segments and data["action_segments"].shape[1] > max_segments:
                raise ValueError("Dataset segment padding smaller than a recorded interval")
            for start in range(0, info.n_frames - sequence_length, stride):
                if data["transition_valid"][start:start + sequence_length].all():
                    self.windows.append((episode_i, start))
        if not self.windows:
            raise ValueError("No complete valid sequences: collect longer sessions or inspect timing/ACK gaps")

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, index: int) -> dict[str, np.ndarray]:
        episode_i, start = self.windows[index]
        if episode_i not in self._cache:
            self._cache[episode_i] = read_episode(self.episodes[episode_i].path)
            while len(self._cache) > self.max_cache:
                self._cache.popitem(last=False)
        self._cache.move_to_end(episode_i)
        data = self._cache[episode_i]
        end = start + self.sequence_length
        result = {
            "images": data["images"][start:end + 1, None].astype(np.float32) / 255.0,
            "actions": data["actions"][start:end].astype(np.float32),
            "dt": data["dt"][start:end].astype(np.float32),
            "aux": data["aux"][start:end + 1].astype(np.float32),
            "aux_mask": data["aux_mask"][start:end + 1].copy(),
        }
        segments = np.zeros((self.sequence_length, self.max_segments, 3), dtype=np.float32)
        segment_mask = np.zeros((self.sequence_length, self.max_segments), dtype=np.bool_)
        if "action_segments" in data:
            k = data["action_segments"].shape[1]
            segments[:, :k] = data["action_segments"][start:end]
            segment_mask[:, :k] = data["segment_mask"][start:end]
        else:
            raise ValueError("Missing v2 action chronology; interval means cannot be substituted")
        result["action_segments"], result["segment_mask"] = segments, segment_mask
        return result


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Read-only v1 provenance audit; no fabricated migration")
    parser.add_argument("--audit-v1", required=True, type=Path)
    args = parser.parse_args()
    data = read_episode(args.audit_v1)
    metadata = episode_metadata(data)
    print(json.dumps({"file": str(args.audit_v1), "schema_version": metadata.get("schema_version"),
                      "has_segments": "action_segments" in data,
                      "mcu_confirmation_validated": metadata.get("mcu_confirmation_validated") is True,
                      "migration": "Re-export original raw logs only if they include boot/pulse identity, "
                                   "matched MCU START and STOP ticks, actual PWM, and bounded camera timing. "
                                   "v1 ACK-receipt timestamps or interval means cannot recover that evidence. "
                                   "Otherwise retain v1 for legacy offline study and collect new v2 data; "
                                   "v1 weights must be retrained."}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
