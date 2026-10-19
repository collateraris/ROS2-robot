"""Versioned transition data, masks, geometry and session-disjoint splits."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import numpy as np

SCHEMA_VERSION = 3
STOP = "STOP"
ODOMETRY_SOURCE = "rf2o_laser_odometry_guarded"
ACTION_SOURCE = "mcu_acknowledged_nominal_velocity_setpoint"
ACTION_SOURCES = {"hardware": ACTION_SOURCE, "gazebo": "simulator_accepted_velocity_setpoint", "synthetic": "synthetic_command"}
DEFAULT_LIMITS_S = {"image_age": .2, "odom_age": .15, "scan_age": .25,
                    "health_age": .3, "ack_gap": .25, "armed_age": .8}


def load_metadata(path):
    meta = json.loads(Path(path).read_text(encoding="utf-8"))
    if meta.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported schema_version")
    environment = meta.get("environment")
    if environment not in ACTION_SOURCES or (environment == "synthetic") != (meta.get("synthetic") is True):
        raise ValueError("environment must explicitly identify hardware/gazebo/synthetic")
    expected_odom = "synthetic" if environment == "synthetic" else ODOMETRY_SOURCE
    if meta.get("odometry_source") != expected_odom:
        raise ValueError("odometry_source must identify guarded RF2O (or explicitly synthetic data)")
    if meta.get("action_source") != ACTION_SOURCES[environment]:
        raise ValueError("action_source must match the declared environment")
    limits = meta.get("recording_limits_s", {})
    if set(limits) != set(DEFAULT_LIMITS_S) or any(not isinstance(v, (float, int)) or not np.isfinite(v) or not 0 < v <= 2 for v in limits.values()):
        raise ValueError("recording_limits_s must contain positive bounded freshness limits")
    if not isinstance(meta.get("map_id"), str) or not meta["map_id"]:
        raise ValueError("map_id must identify the exact saved map used for labels")
    candidates = meta["candidates"]
    if not candidates or candidates[0] != {"name": STOP, "pose": None}:
        raise ValueError("Candidate 0 must be {'name':'STOP', 'pose':null}")
    names = [c["name"] for c in candidates]
    if len(set(names)) != len(names):
        raise ValueError("Candidate names must be unique")
    for c in candidates[1:]:
        if not isinstance(c["name"], str) or len(c["pose"]) != 3 or not np.isfinite(c["pose"]).all():
            raise ValueError("Candidate pose must contain finite map x,y,yaw")
    if not 0 < meta["lidar_max_m"] <= 100:
        raise ValueError("Invalid lidar_max_m")
    if not 8 <= int(meta["sectors"]) <= 720:
        raise ValueError("Invalid sectors")
    return meta


def metadata_fingerprint(meta):
    stable = {k: meta[k] for k in ("schema_version", "environment", "candidates", "lidar_max_m", "sectors", "map_id", "odometry_source", "action_source", "recording_limits_s")}
    return hashlib.sha256(json.dumps(stable, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def wrap_angle(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def relative_motion(odom, nxt):
    """Body-frame displacement and wrapped yaw delta, not a motor command."""
    d = np.asarray(nxt) - np.asarray(odom)
    c, s = np.cos(odom[..., 2]), np.sin(odom[..., 2])
    return np.stack((c*d[..., 0] + s*d[..., 1], -s*d[..., 0] + c*d[..., 1], wrap_angle(d[..., 2])), -1).astype(np.float32)


def candidate_geometry(odom, meta):
    """STOP=(0,0); remaining known map goals expressed in robot frame."""
    odom = np.asarray(odom)
    poses = np.array([[0., 0., 0.] if c["pose"] is None else c["pose"] for c in meta["candidates"]])
    dx = poses[:, 0] - odom[..., 0, None]
    dy = poses[:, 1] - odom[..., 1, None]
    c, s = np.cos(odom[..., 2, None]), np.sin(odom[..., 2, None])
    result = np.stack((c*dx+s*dy, -s*dx+c*dy), -1).astype(np.float32)
    result[..., 0, :] = 0
    return result


def sector_scan(ranges, angle_min, angle_increment, range_min, range_max, sectors, max_m):
    """Full-circle sectors. Missing/NaN/too-near beams stay invalid, never free.

    Nonfinite and outside-range values are invalid. Unknown sectors remain
    masked. A finite max-range return is accepted; range_max < max_m is retained
    as that shorter measured clearance. Sector positions are scan-frame angles.
    """
    r = np.asarray(ranges, np.float64)
    values = np.zeros(sectors, np.float32)
    mask = np.zeros(sectors, bool)
    angles = angle_min + np.arange(len(r))*angle_increment
    ids = np.floor(((angles + np.pi) % (2*np.pi))/(2*np.pi)*sectors).astype(int)
    usable = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    clipped = np.minimum(np.where(usable,r,0.), min(max_m, range_max)) / max_m
    for k in range(sectors):
        hit = (ids == k) & usable
        if hit.any():
            values[k] = clipped[hit].min()
            mask[k] = True
    return values, mask


def grouped_split(sessions, seed=17):
    """Three disjoint session groups; no random frame split."""
    sessions = np.asarray(sessions)
    names = np.unique(sessions)
    if len(names) < 3:
        raise ValueError("At least three distinct recording sessions are required for train/val/test")
    names = names.copy()
    np.random.default_rng(seed).shuffle(names)
    n_val = max(1, len(names)//5)
    n_test = max(1, len(names)//5)
    groups = {"test": names[:n_test], "val": names[n_test:n_test+n_val], "train": names[n_test+n_val:]}
    return {name: np.flatnonzero(np.isin(sessions, values)) for name, values in groups.items()}


def load_dataset(paths, meta):
    chunks = []
    mandatory = ("rgb", "lidar", "lidar_mask", "odom", "map_pose", "map_pose_valid", "next_lidar", "next_lidar_mask", "next_odom", "action", "dt", "action_valid", "instruction", "label", "session", "start_ns", "end_ns", "ack_ns", "rgb_ns", "next_rgb_ns", "scan_ns", "next_scan_ns", "odom_ns", "next_odom_ns", "odom_healthy", "next_odom_healthy", "snapshot_received_ns", "next_snapshot_received_ns", "health_received_ns", "next_health_received_ns")
    for path in paths:
        with np.load(path, allow_pickle=False) as z:
            if int(z["schema_version"]) != SCHEMA_VERSION or str(z["metadata_fingerprint"]) != metadata_fingerprint(meta):
                raise ValueError(f"{path}: metadata/schema mismatch")
            if str(z["environment"]) != meta["environment"]:
                raise ValueError(f"{path}: environment source mismatch")
            if str(z["odometry_source"]) != meta["odometry_source"] or str(z["action_source"]) != meta["action_source"]:
                raise ValueError(f"{path}: sensor/action source mismatch")
            d = {key: z[key] for key in mandatory}
        n = len(d["rgb"])
        if any(len(v) != n for v in d.values()):
            raise ValueError(f"{path}: unequal row counts")
        for key in ("dt", "action_valid", "map_pose_valid", "instruction", "label", "session", "start_ns", "end_ns", "ack_ns", "rgb_ns", "next_rgb_ns", "scan_ns", "next_scan_ns", "odom_ns", "next_odom_ns", "odom_healthy", "next_odom_healthy", "snapshot_received_ns", "next_snapshot_received_ns", "health_received_ns", "next_health_received_ns"):
            if d[key].shape != (n,):
                raise ValueError(f"Invalid {key} row shape")
        for key in ("odom_healthy", "next_odom_healthy"):
            if d[key].dtype != np.bool_:
                raise ValueError("Odometry health must be explicit bool")
        if d["rgb"].shape != (n,128,128,3) or d["rgb"].dtype != np.uint8:
            raise ValueError("rgb must be uint8 [N,128,128,3]")
        for key in ("lidar", "lidar_mask", "next_lidar", "next_lidar_mask"):
            if d[key].shape != (n,meta["sectors"]):
                raise ValueError(f"Invalid {key} shape")
        for key in ("odom", "next_odom", "map_pose"):
            if d[key].shape != (n,3) or not np.isfinite(d[key]).all():
                raise ValueError(f"Invalid {key}")
        if d["action"].shape != (n,2):
            raise ValueError("action must have shape [N,2]")
        if not np.isfinite(d["dt"]).all() or np.any(d["dt"] <= 0):
            raise ValueError("dt must be positive")
        if np.any(d["end_ns"] <= d["start_ns"]) or not np.allclose(d["dt"], (d["end_ns"]-d["start_ns"])*1e-9, rtol=0, atol=1e-6):
            raise ValueError("Interval timestamps disagree with dt")
        limits = meta["recording_limits_s"]
        for prefix in ("", "next_"):
            scan = d[prefix + "scan_ns"]
            if not np.array_equal(scan, d["end_ns" if prefix else "start_ns"]):
                raise ValueError("Scan timestamps must identify transition endpoints")
            received = d[prefix + "snapshot_received_ns"]
            for key, maximum, reference in (("rgb_ns", "image_age", scan), ("odom_ns", "odom_age", scan), ("scan_ns", "scan_age", received), ("health_received_ns", "health_age", received)):
                source = d[prefix + key]
                age = reference - source
                if source.dtype != np.int64 or np.any(source <= 0) or np.any(age < 0) or np.any(age > int(limits[maximum] * 1e9)):
                    raise ValueError(f"Stale/future/invalid {prefix + key}")
            if received.dtype != np.int64 or not np.all(d[prefix + "odom_healthy"]):
                raise ValueError("Unhealthy odometry cannot be used as a training target")
        if np.any(d["next_odom_ns"] <= d["odom_ns"]):
            raise ValueError("Transition requires two distinct source odometry stamps")
        valid = d["action_valid"].astype(bool)
        if not np.isfinite(d["action"][valid]).all():
            raise ValueError("Acknowledged actions must be finite")
        if np.any(d["ack_ns"][valid] > d["start_ns"][valid]) or np.any(d["ack_ns"][valid] <= 0):
            raise ValueError("Applied action ACK must precede transition")
        if np.any(d["start_ns"][valid] - d["ack_ns"][valid] > int(limits["ack_gap"] * 1e9)):
            raise ValueError("Acknowledged nominal setpoint is stale")
        if np.any((d["label"] < -1) | (d["label"] >= len(meta["candidates"]))):
            raise ValueError("Invalid label index")
        for field,mask in (("lidar","lidar_mask"),("next_lidar","next_lidar_mask")):
            observed = d[field][d[mask].astype(bool)]
            if not np.isfinite(observed).all() or np.any((observed < 0)|(observed > 1)):
                raise ValueError("Valid lidar values must be normalized to [0,1]")
            d[field] = np.where(d[mask], d[field], 0).astype(np.float32)
        chunks.append(d)
    if not chunks:
        raise ValueError("No datasets")
    return {key: np.concatenate([d[key] for d in chunks]) for key in mandatory}


def save_dataset(path, rows, meta):
    if not rows:
        raise ValueError("No synchronized transitions were collected")
    data = {key: np.stack([row[key] for row in rows]) for key in rows[0]}
    np.savez_compressed(path, schema_version=np.array(SCHEMA_VERSION), metadata_fingerprint=np.array(metadata_fingerprint(meta)),
                        environment=np.array(meta["environment"]), odometry_source=np.array(meta["odometry_source"]), action_source=np.array(meta["action_source"]), **data)


def text_bytes(texts, limit=128):
    out = np.zeros((len(texts),limit),np.int64)
    for i,text in enumerate(texts):
        value = list(str(text).encode("utf-8")[:limit])
        out[i,:len(value)] = np.array(value)+1
    return out
