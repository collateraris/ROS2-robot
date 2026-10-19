"""Pi-local episodes, acknowledged PWM segments and leakage-safe dataset export."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import hashlib
import time
from pathlib import Path
import queue
import threading
import uuid
import numpy as np
from .actuator import unmix_pwm, validate_motor_interval


class EpisodeRecorder:
    def __init__(self, session_dir: str | Path, metadata: dict):
        self.session_dir = Path(session_dir)
        episode_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
        self.directory = self.session_dir / episode_id
        (self.directory / "rgb").mkdir(parents=True, exist_ok=False)
        self.metadata = {"schema_version": 2, "session_id": self.session_dir.name,
                         "layout_id": "unknown", "day_id": "unknown", "lighting_id": "unknown",
                         **metadata, "episode_id": episode_id, "log_complete": False}
        manifest = {k: v for k, v in self.metadata.items() if k not in {"episode_id", "log_complete"}}
        manifest_file = self.session_dir / "session_manifest.json"
        if manifest_file.exists():
            if json.loads(manifest_file.read_text(encoding="utf-8")) != manifest:
                raise ValueError("Session configuration changed: select a NEW record_session directory")
        else:
            with manifest_file.open("x", encoding="utf-8") as stream:
                json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
        self.failed = False
        self._queue = queue.Queue(maxsize=128)
        self._thread = threading.Thread(target=self._run, daemon=True, name="episode-writer")
        self._thread.start()
        self._save_metadata()

    def _save_metadata(self):
        (self.directory / "metadata.json").write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    def _enqueue(self, item):
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.failed = True  # entire recording is marked incomplete, never quietly trusted

    def frame(self, jpeg: bytes, value: dict):
        seq = value["frame"]["frame_seq"]
        event = {k: v for k, v in value.items() if k not in {"jpeg", "history"}}
        epoch=value["frame"]["camera_epoch"]
        event.update(type="camera", rgb_path=f"rgb/{epoch}_{seq:09d}.jpg")
        self._enqueue((event, bytes(jpeg)))

    def event(self, value: dict):
        self._enqueue((dict(value), None))

    def boundary(self, reason: str, time_s: float | None = None):
        self.event({"type": "episode_boundary", "reason": reason,
                    "time_s": time.monotonic() if time_s is None else time_s})

    def _run(self):
        try:
            with (self.directory / "events.jsonl").open("w", encoding="utf-8") as stream:
                while True:
                    item = self._queue.get()
                    if item is None:
                        self._queue.task_done()
                        break
                    value, jpeg = item
                    if jpeg is not None:
                        (self.directory / value["rgb_path"]).write_bytes(jpeg)
                    stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
                    stream.flush()
                    self._queue.task_done()
        except Exception:
            self.failed = True

    def close(self):
        try:
            self._queue.put(None, timeout=1)
        except queue.Full:
            self.failed = True
        self._thread.join(timeout=5)
        self.metadata["log_complete"] = not self.failed and not self._thread.is_alive()
        self._save_metadata()


def validate_confirmed_interval(event):
    """Same MCU validation on disk, in typed transport and in the live filter."""
    validate_motor_interval(event)


def intervals_to_segments(start_s: float, end_s: float, intervals: list[dict], robot: dict,
                          *, max_timing_uncertainty_s: float = 0.1,
                          history_start_s: float | None = None,
                          start_interval=None, end_interval=None,
                          require_mcu_confirmation: bool = True) -> tuple[np.ndarray, bool]:
    """Ordered actual PWM pulses and zero-PWM gaps, never an averaged action.

    Exact MCU elapsed time determines pulse duration. Pi bounds only place it
    relative to camera exposure. A camera interval intersecting an uncertain
    pulse is ineligible. Zero means PWM is off, not measured wheel velocity.
    A caller must supply the start of complete, exclusively owned history.
    """
    if not np.isfinite([start_s, end_s]).all() or end_s <= start_s:
        raise ValueError("Invalid interval endpoints")
    if not np.isfinite(max_timing_uncertainty_s) or max_timing_uncertainty_s < 0:
        raise ValueError("Invalid motor timing uncertainty limit")
    fallback = np.array([[0., 0., end_s-start_s]], np.float32)
    if (history_start_s is None or not np.isfinite(history_start_s)
            or history_start_s > start_s + 1e-9):
        return fallback, False
    # Disabling confirmation is only useful when auditing old logs. It never
    # changes the validity return or licenses a legacy transition for training.
    if not require_mcu_confirmation:
        return fallback, False
    try:
        for event in intervals:
            validate_motor_interval(event)
    except (ValueError, TypeError, KeyError):
        return fallback, False
    for bound, point in ((start_interval, start_s), (end_interval, end_s)):
        bounds = (point, point) if bound is None else bound
        if (len(bounds) != 2 or not np.isfinite(bounds).all()
                or not bounds[0] <= point <= bounds[1]):
            return fallback, False
        if history_start_s > bounds[0] + 1e-9:
            return fallback, False
        for event in intervals:
            if bounds[0] <= event["end_upper_s"] and bounds[1] >= event["start_lower_s"]:
                return fallback, False
    selected = sorted((e for e in intervals if e["end_s"] > start_s and e["start_s"] < end_s),
                      key=lambda e: e["start_s"])
    cursor, pieces = start_s, []
    for event in selected:
        a, b, duration = event["start_s"], event["end_s"], event["mcu_duration_s"]
        if (a < cursor-1e-8 or a < start_s or b > end_s
                or event["timing_uncertainty_s"] > max_timing_uncertainty_s
                or event["duration_uncertainty_s"] > max_timing_uncertainty_s):
            return fallback, False
        if a > cursor+1e-9:
            pieces.append([0., 0., a-cursor])
        v, w = unmix_pwm(event["left"], event["right"], robot["track_width_m"],
                        robot["wheel_command_scale_mps"], robot["pwm_limit"])
        action = [v/robot["linear_command_scale_mps"], w/robot["angular_command_scale_rps"]]
        if not np.isfinite(action).all() or max(abs(x) for x in action) > 1.00001:
            return fallback, False
        if duration > 1e-9:
            pieces.append([*np.clip(action, -1, 1), duration])
        cursor = b
    if end_s > cursor+1e-9:
        pieces.append([0., 0., end_s-cursor])
    if not pieces:
        pieces = [[0., 0., end_s-start_s]]
    coalesced = []
    for piece in pieces:
        if coalesced and np.allclose(coalesced[-1][:2], piece[:2], rtol=0., atol=1e-8):
            coalesced[-1][2] += piece[2]
        else:
            coalesced.append(piece)
    result = np.asarray(coalesced, dtype=np.float32)
    if not np.isclose(result[:, 2].sum(), end_s-start_s, atol=1e-5, rtol=1e-6):
        return fallback, False
    return result, True


def export_episode(directory: str | Path, output: str | Path, *, max_segments: int = 24,
                   max_dt_s: float = 5.0) -> dict:
    import cv2
    import os
    from .frames import validate_frame
    if not 1 <= max_segments <= 24 or not np.isfinite(max_dt_s) or max_dt_s <= 0:
        raise ValueError("Require 1..24 segments and a positive finite max_dt_s")
    directory, output = Path(directory), Path(output)
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    if metadata.get("schema_version") != 2:
        raise ValueError("v1 raw log lacks MCU/timestamp evidence: recollect, do not relabel its version")
    if not metadata.get("log_complete"):
        raise ValueError("Incomplete/crashed recording: cannot infer zero controls in missing history")
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text(encoding="utf-8").splitlines() if line]
    frames = sorted((x for x in events if x["type"] == "camera" and x.get("settled", False)
                     and x.get("timing_valid") is True), key=lambda x: x["exposure_estimate_s"])
    intervals = [x for x in events if x["type"] == "motor_interval"]
    faults = [x.get("time_s",0.) for x in events if x["type"] in
              {"motor_fault", "motor_unknown", "motor_reboot", "episode_boundary"}]
    ready = [x for x in events if x["type"] == "motor_ready"
             and x.get("zero_confirmed") is True and x.get("exclusive_pulse_firmware") is True]
    def pulse_key(event):
        key = (event.get("boot_id"), event.get("nonce"), event.get("seq"))
        if any(type(value) is not int or not 0 < value <= 0xFFFFFFFF for value in key):
            raise ValueError("Motor event lacks valid (boot_id, nonce, seq); v1 logs cannot prove completion")
        return key
    sends = [pulse_key(x) for x in events if x["type"] == "motor_sent"]
    dones = [pulse_key(x) for x in intervals]
    if len(set(sends)) != len(sends) or len(set(dones)) != len(dones):
        raise ValueError("Duplicate (boot_id, nonce, seq) in recorded motor events")
    if set(sends) != set(dones):
        raise ValueError("A motor pulse has no matching MCU completion; do not invent its timing")
    for event in intervals:
        validate_motor_interval(event)
    if not ready:
        raise ValueError("No exclusive MCU zero handshake: shadow camera recording is not confirmed-action training data")
    if len(frames) < 4:
        raise ValueError("At least four settled frames with calibrated exposure intervals are required")
    n = len(frames)
    images, paths = [], []
    for frame in frames:
        # Recorder removes history/jpeg from frames; validate the rest with an
        # empty history (its content is reconstructed from the immutable log).
        from .frames import FRAME_FIELDS
        packet = {k: v for k, v in frame.items() if k in FRAME_FIELDS}
        packet["type"] = "frame"
        packet["history"] = []
        validate_frame(packet)
        rgb_path = (directory / frame["rgb_path"]).resolve()
        if not rgb_path.is_relative_to(directory.resolve()):
            raise ValueError("Unsafe RGB path in recording")
        image = cv2.imread(str(rgb_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Missing image: {rgb_path}")
        images.append(cv2.resize(image, (128,128), interpolation=cv2.INTER_AREA))
        paths.append(os.path.relpath(rgb_path, output.parent.resolve()))
    timestamps = np.asarray([f["exposure_estimate_s"] for f in frames], np.float64)
    dt = np.diff(timestamps).astype(np.float32)
    if np.any(dt <= 0):
        raise ValueError("Exposure timestamps are not strictly increasing")
    actions = np.zeros((n-1,2), np.float32)  # audit-only diagnostic mean
    segments = np.zeros((n-1,max_segments,3), np.float32)
    masks = np.zeros((n-1,max_segments), bool)
    valid = (dt <= max_dt_s) & (dt >= .02)
    robot = metadata["robot"]
    for i in range(n-1):
        a,b = timestamps[i:i+2]
        prior_ready = [x for x in ready if x["history_start_s"] <= a]
        history_start = max((x["history_start_s"] for x in prior_ready), default=None)
        previous_fault = max((x.get("time_s",0.) for x in events if x["type"] in
            {"motor_fault", "motor_unknown", "motor_reboot"} and x.get("time_s",0.) <= a), default=-1.)
        if history_start is not None and history_start <= previous_fault:
            history_start = None
        if history_start is not None:
            history_start = max(history_start, frames[i+1]["history_start_s"])
        pieces, ok = intervals_to_segments(a,b,intervals,robot,
            max_timing_uncertainty_s=robot["motor_time_uncertainty_max_s"], history_start_s=history_start,
            start_interval=(frames[i]["exposure_earliest_ns"]/1e9, frames[i]["exposure_latest_ns"]/1e9),
            end_interval=(frames[i+1]["exposure_earliest_ns"]/1e9, frames[i+1]["exposure_latest_ns"]/1e9))
        if len(pieces)>max_segments:
            ok=False
            pieces=np.array([[0,0,b-a]],np.float32)
        if any(frames[i]["frame"][key] != frames[i+1]["frame"][key] for key in ("session_id", "camera_epoch")):
            ok=False
        if any(frames[i]["exposure_earliest_ns"]/1e9 <= t <= frames[i+1]["exposure_latest_ns"]/1e9 for t in faults):
            ok=False
        exposure_limit=metadata.get("camera",{}).get("max_training_exposure_uncertainty_s",0.1)
        if not np.isfinite(exposure_limit) or exposure_limit <= 0:
            raise ValueError("Invalid max_training_exposure_uncertainty_s")
        if any(f["exposure_uncertainty_s"] > exposure_limit for f in frames[i:i+2]):
            ok=False
        valid[i] &= ok
        segments[i,:len(pieces)] = pieces
        masks[i,:len(pieces)] = True
        actions[i] = np.sum(pieces[:,:2]*pieces[:,2:3],axis=0)/(b-a)
    metadata.update(schema_version=2, action_semantics="mcu_confirmed_piecewise_normalized_forward_yaw",
        control_representation="piecewise_mcu_confirmed", mcu_confirmation_validated=True,
        source="real_robot", synthetic=False, timestamp_semantics="calibrated_exposure_interval_pi_monotonic",
        number_of_frames=n, valid_transitions=int(valid.sum()),
        raw_events_sha256=hashlib.sha256((directory/"events.jsonl").read_bytes()).hexdigest())
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():
        raise ValueError("Export output already exists: choose a new derived dataset path")
    np.savez_compressed(output, images=np.stack(images), rgb_paths=np.asarray(paths),
        timestamps=timestamps, dt=dt, actions=actions, action_segments=segments, segment_mask=masks,
        exposure_earliest_ns=np.asarray([f["exposure_earliest_ns"] for f in frames],np.uint64),
        exposure_latest_ns=np.asarray([f["exposure_latest_ns"] for f in frames],np.uint64),
        aux=np.zeros((n,6),np.float32), aux_mask=np.zeros((n,6),bool),
        transition_valid=valid, metadata_json=np.asarray(json.dumps(metadata)))
    return {"file":str(output),"frames":n,"valid_transitions":int(valid.sum()),
            "invalid_transitions":int((~valid).sum()),"schema_version":2}


def main():
    parser=argparse.ArgumentParser(description="Export complete Pi raw episodes to world-model NPZ")
    parser.add_argument("--session",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    directories=sorted(p.parent for p in args.session.rglob("metadata.json") if (p.parent/"events.jsonl").is_file())
    if not directories:
        raise SystemExit("No recorded episodes found")
    for directory in directories:
        print(json.dumps(export_episode(directory,args.output/(directory.name+".npz")),ensure_ascii=False))


if __name__ == "__main__":
    main()
