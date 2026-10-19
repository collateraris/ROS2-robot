"""Typed ROS v2 boundary. The perception/ML/safety core has no rclpy dependency.

Generated classes are imported lazily so offline training needs no ROS install.
The old JSON codecs remain for audit files; no critical live topic uses String.
"""
from __future__ import annotations

import math
import time
from .protocol import FrameRef, MetricGeometry, MotionCommand, ProtocolError

SCHEMA_VERSION = 2


def message_types():
    try:
        from roomwm_msgs import msg
        return msg
    except ImportError as exc:
        raise RuntimeError(
            "roomwm_msgs is not built/sourced. Run bash scripts/build_ros.sh, "
            "then source scripts/ros_env.sh in this terminal.") from exc


def _finite(value, name, low=0., high=1e12):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be numeric")
    if not math.isfinite(value) or not low <= value <= high:
        raise ProtocolError(f"Invalid {name}")
    return float(value)


def ref_to_ros(value, types=None):
    t = types or message_types()
    ref = value if isinstance(value, FrameRef) else FrameRef.from_dict(value)
    ref.validate()
    msg = t.FrameRef()
    msg.session_id, msg.camera_epoch, msg.frame_seq = ref.session_id, ref.camera_epoch, ref.frame_seq
    return msg


def ref_from_ros(msg):
    ref = FrameRef(msg.session_id, msg.camera_epoch, msg.frame_seq)
    ref.validate()
    return ref


INTERVAL_FIELDS = (
    "schema_version", "boot_id", "nonce", "seq", "pulse_id", "left", "right",
    "accept_tick_ms", "mcu_start_tick_ms", "mcu_stop_tick_ms", "mcu_elapsed_ms",
    "requested_duration_ms", "mcu_duration_s", "duration_uncertainty_s",
    "start_s", "end_s", "start_lower_s", "start_upper_s", "end_lower_s", "end_upper_s",
    "timing_uncertainty_s", "complete", "actuation_confirmed", "pwm_off_confirmed",
    "normal_completion", "deadline_exceeded", "duration_mismatch", "confirmation_source", "stop_reason")


def interval_to_ros(event, types=None):
    from .actuator import validate_motor_interval
    validate_motor_interval(event)
    t = types or message_types()
    msg = t.ActuationInterval()
    for name in INTERVAL_FIELDS:
        setattr(msg, name, event[name])
    return msg


def interval_from_ros(msg):
    from .actuator import validate_motor_interval
    event = {name: getattr(msg, name) for name in INTERVAL_FIELDS}
    event["type"] = "motor_interval"
    validate_motor_interval(event)
    return event


def frame_to_ros(body, jpeg, *, ros_now_ns=None, pi_now_ns=None, types=None):
    from .frames import validate_frame
    validate_frame(body)
    t = types or message_types()
    msg, info = t.FramePacket(), t.FrameInfo()
    info.schema_version = SCHEMA_VERSION
    info.frame = ref_to_ros(body["frame"], t)
    info.pi_dequeue_monotonic_ns = int(body["dequeue_mono_ns"])
    info.timing_valid = body["timing_valid"]
    info.exposure_earliest_ns = int(body["exposure_earliest_ns"] or 0)
    info.exposure_latest_ns = int(body["exposure_latest_ns"] or 0)
    info.publish_monotonic_ns = time.monotonic_ns() if pi_now_ns is None else pi_now_ns
    info.has_v4l2_metadata = body["v4l2_timestamp_ns"] is not None
    info.v4l2_timestamp_ns = int(body["v4l2_timestamp_ns"] or 0)
    info.v4l2_sequence = int(body["hardware_sequence"] or 0)
    info.v4l2_flags = int(body["v4l2_flags"] or 0)
    for name in ("timestamp_source", "timestamp_clock", "timing_evidence_id", "timing_reason"):
        setattr(info, name, str(body[name]))
    for name in ("exposure_estimate_s", "exposure_uncertainty_s", "capture_latency_bound_s"):
        setattr(info, name, float(body[name] or 0.))
    info.age_upper_at_publish_s = (
        max(0., (info.publish_monotonic_ns-info.exposure_earliest_ns)/1e9) if info.timing_valid else 0.)
    info.calibration_fingerprint = body["calibration_fingerprint"]
    info.settled, info.mode = body["settled"], body["mode"]
    # Only for ROS viewers/bags. Pi monotonic interval above remains authoritative.
    if ros_now_ns is not None and info.timing_valid:
        estimate_ns = int(body["exposure_estimate_s"]*1e9)
        stamp = max(0, ros_now_ns-(info.publish_monotonic_ns-estimate_ns))
        info.header.stamp.sec, info.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    info.header.frame_id = "camera_optical" if info.timing_valid else "camera_timing_unknown"
    msg.info = info
    msg.image.header = info.header
    msg.image.format, msg.image.data = "jpeg", bytes(jpeg)
    msg.history = [interval_to_ros(e, t) for e in body["history"]]
    if len(msg.history) > 256:
        raise ProtocolError("Too much MCU history")
    msg.history_start_s = float(body["history_start_s"])
    return msg


def frame_from_ros(msg):
    from .frames import decode_frame_bytes
    info = msg.info
    if info.schema_version != SCHEMA_VERSION:
        raise ProtocolError("Unsupported typed frame schema")
    if len(msg.history) > 256 or msg.image.format != "jpeg" or len(msg.image.data) > 2_000_000:
        raise ProtocolError("Invalid typed image/history")
    body = {"version": 2, "type": "frame", "frame": ref_from_ros(info.frame).to_dict(),
            "read_mono_s": info.pi_dequeue_monotonic_ns/1e9,
            "dequeue_mono_ns": info.pi_dequeue_monotonic_ns,
            "exposure_estimate_s": info.exposure_estimate_s,
            "capture_latency_bound_s": info.capture_latency_bound_s if info.timing_valid else None,
            "exposure_uncertainty_s": info.exposure_uncertainty_s if info.timing_valid else None,
            "exposure_earliest_ns": info.exposure_earliest_ns if info.timing_valid else None,
            "exposure_latest_ns": info.exposure_latest_ns if info.timing_valid else None,
            "v4l2_timestamp_ns": info.v4l2_timestamp_ns if info.has_v4l2_metadata else None,
            "hardware_sequence": info.v4l2_sequence if info.has_v4l2_metadata else None,
            "v4l2_flags": info.v4l2_flags if info.has_v4l2_metadata else None,
            "timestamp_source": info.timestamp_source, "timestamp_clock": info.timestamp_clock,
            "timing_valid": info.timing_valid, "timing_reason": info.timing_reason,
            "timing_evidence_id": info.timing_evidence_id,
            "calibration_fingerprint": info.calibration_fingerprint,
            "settled": info.settled, "mode": info.mode,
            "history_start_s": msg.history_start_s,
            "history": [interval_from_ros(x) for x in msg.history]}
    return decode_frame_bytes(body, bytes(msg.image.data))


def perception_to_ros(value, fingerprint, types=None):
    t = types or message_types()
    msg = t.PerceptionState()
    geometry = MetricGeometry.from_dict(value["geometry"])
    msg.frame = ref_to_ros(geometry.frame, t)
    msg.calibration_fingerprint = fingerprint
    for name in ("clearance_m", "path_linear_mps", "path_angular_rps", "observed_half_fov_rad",
                 "metric_valid", "covers_swept_footprint"):
        setattr(msg, name, getattr(geometry, name))
    observed = value["diagnostics"].get("perception", {})
    msg.person_count = int(observed.get("person_count", 0))
    msg.person_visible = bool(observed.get("person_visible", False))
    msg.person_distance_valid = bool(observed.get("person_distance_valid", False))
    msg.person_distance_m = float(observed.get("person_distance_m", 0.))
    msg.person_bearing_rad = float(observed.get("person_bearing_rad", 0.))
    msg.path_reason = str(value["diagnostics"].get("path_reason", ""))[:256]
    return msg


def proposal_to_ros(value, fingerprint, model_hash="", types=None):
    t = types or message_types()
    msg = t.ActuationProposal()
    command = MotionCommand.from_dict(value["command"])
    msg.schema_version = SCHEMA_VERSION
    msg.goal_id, msg.controller = value["goal_id"], value["controller"]
    msg.frame = ref_to_ros(command.frame, t)
    for name in ("lease_id", "command_seq", "linear_mps", "angular_rps", "valid_for_ms", "inference_ms"):
        setattr(msg, name, getattr(command, name))
    msg.pulse_ms = command.valid_for_ms
    msg.abort = value["abort"]
    uncertainty = value["diagnostics"].get("planner", {}).get("max_aux_disagreement")
    msg.model_uncertainty_valid = (isinstance(uncertainty, (int, float))
                                    and math.isfinite(uncertainty) and uncertainty >= 0.)
    msg.model_uncertainty = float(uncertainty) if msg.model_uncertainty_valid else 0.
    msg.model_hash = model_hash
    msg.reason = str(value["diagnostics"].get("reason", ""))[:256]
    msg.current_perception = perception_to_ros(value, fingerprint, t)
    return msg


def proposal_from_ros(msg):
    if msg.schema_version != SCHEMA_VERSION:
        raise ProtocolError("Unsupported typed proposal schema")
    if msg.controller not in {"shadow", "teleop", "baseline", "wm", "wm_trial"}:
        raise ProtocolError("Unknown controller")
    if msg.pulse_ms != msg.valid_for_ms:
        raise ProtocolError("Pulse and bounded validity differ")
    frame = ref_from_ros(msg.frame)
    command = MotionCommand(frame, msg.lease_id, msg.command_seq, msg.linear_mps,
                            msg.angular_rps, msg.valid_for_ms, msg.inference_ms)
    command.validate()
    p = msg.current_perception
    if ref_from_ros(p.frame) != frame:
        raise ProtocolError("Proposal/perception frame mismatch")
    geometry = MetricGeometry(frame, p.clearance_m, p.path_angular_rps, p.metric_valid,
                              p.observed_half_fov_rad, p.covers_swept_footprint, p.path_linear_mps)
    geometry.validate()
    _finite(msg.model_uncertainty, "model uncertainty", 0, 100)
    _finite(p.person_distance_m, "person distance", 0, 100)
    _finite(p.person_bearing_rad, "person bearing", -math.pi, math.pi)
    if len(p.calibration_fingerprint) != 64:
        raise ProtocolError("Missing current-perception calibration hash")
    return {"version": 1, "type": "proposal", "goal_id": msg.goal_id,
            "controller": msg.controller, "abort": msg.abort,
            "command": command.to_dict(), "geometry": geometry.to_dict(),
            "diagnostics": {"reason": msg.reason, "path_reason": p.path_reason,
                "calibration_fingerprint": p.calibration_fingerprint,
                "model_hash": msg.model_hash, "model_uncertainty": msg.model_uncertainty,
                "model_uncertainty_valid": msg.model_uncertainty_valid,
                "perception": {"person_count": p.person_count, "person_visible": p.person_visible,
                    "person_distance_valid": p.person_distance_valid,
                    "person_distance_m": p.person_distance_m, "person_bearing_rad": p.person_bearing_rad}}}


def task_to_ros(value, types=None):
    t = types or message_types()
    msg = t.TaskRequest()
    msg.schema_version = SCHEMA_VERSION
    msg.goal_id, msg.kind = value["goal_id"], value["kind"]
    msg.text, msg.direction = value.get("text", ""), value.get("direction", "")
    task_from_ros(msg)
    return msg


def task_from_ros(msg):
    if msg.schema_version != SCHEMA_VERSION or not msg.goal_id or len(msg.goal_id) > 128:
        raise ProtocolError("Invalid typed task")
    if msg.kind == "text" and not msg.direction and len(msg.text) <= 500:
        return {"goal_id": msg.goal_id, "kind": "text", "text": msg.text}
    if msg.kind == "manual" and not msg.text and msg.direction in {"forward", "left", "right", "stop"}:
        return {"goal_id": msg.goal_id, "kind": "manual", "direction": msg.direction}
    raise ProtocolError("Unsupported typed task")


def stop_to_ros(reason, goal_id="", types=None):
    t = types or message_types()
    msg = t.Stop()
    msg.goal_id, msg.reason = goal_id[:128], str(reason)[:256]
    return msg


def ack_to_ros(event, types=None):
    t = types or message_types()
    msg = t.ActuationAck()
    msg.schema_version, msg.event_type = SCHEMA_VERSION, event["type"][:32]
    msg.goal_id = event.get("goal_id", "")
    msg.pi_receive_monotonic_ns = int(max(0., event.get("time_s", event.get("end_s", 0.)))*1e9)
    for name in ("boot_id", "nonce", "seq", "pulse_id"):
        setattr(msg, name, int(event.get(name, 0) or 0))
    msg.mcu_tick = int(event.get("mcu_tick_ms", event.get("mcu_tick", 0)))
    msg.left_pwm, msg.right_pwm = int(event.get("left", 0)), int(event.get("right", 0))
    msg.reason = str(event.get("reason", event.get("stop_reason", "")))[:256]
    msg.has_interval = event["type"] == "motor_interval"
    if msg.has_interval:
        msg.interval = interval_to_ros(event, t)
    return msg


def status_to_ros(status, types=None):
    t = types or message_types()
    msg = t.SafetyStatus()
    msg.schema_version = SCHEMA_VERSION
    for name in ("session_id", "goal_id", "lease_id", "state", "reason", "armed", "arm_pending",
                 "motor_ready", "actuation_enabled", "required_clearance_m", "speed_bound_mps",
                 "linear_mps", "angular_rps", "pulse_id", "pulse_remaining_s", "settle_until_mono_s",
                 "calibration_fingerprint"):
        setattr(msg, name, status[name])
    msg.has_frame = status["frame"] is not None
    if msg.has_frame:
        msg.frame = ref_to_ros(status["frame"], t)
    msg.frame_age_valid = status["frame_age_s"] is not None
    msg.frame_age_s = float(status["frame_age_s"] or 0.)
    return msg


def planner_status_to_ros(proposal, types=None, filter_steps=0):
    t = types or message_types()
    msg = t.PlannerStatus()
    msg.schema_version = SCHEMA_VERSION
    for name in ("goal_id", "controller", "reason", "inference_ms", "current_perception",
                 "model_uncertainty_valid", "model_uncertainty"):
        setattr(msg, name, getattr(proposal, name))
    msg.filter_steps = int(filter_steps)
    return msg
