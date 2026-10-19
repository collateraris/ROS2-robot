"""Validated v2 camera/provenance envelopes for a best-effort depth-one topic.

The native ROS adapter transports JPEG bytes separately. JSON/base64 helpers are
retained for offline diagnostics. v1 envelopes are not promoted to calibrated
v2 timing: a read-completion estimate cannot establish acquisition provenance.
"""
from __future__ import annotations
import base64
from dataclasses import fields
import json
import math
import re
import numpy as np
from .protocol import FrameRef, ProtocolError, decode_json
from .timebase import CaptureTiming, exposure_bounds

MAX_FRAME_MESSAGE_BYTES = 3_000_000
TIMING_FIELDS = {f.name for f in fields(CaptureTiming)}
FRAME_FIELDS = {"version", "type", "frame", "read_mono_s", "exposure_estimate_s",
                "capture_latency_bound_s", "exposure_uncertainty_s", "calibration_fingerprint",
                "history", "history_start_s", "settled", "mode"} | TIMING_FIELDS


def make_frame(ref, jpeg=None, read_mono_s=None, latency_estimate_s=None, latency_bound_s=None,
               fingerprint="", history=None, history_start_s=0.0, settled=False, mode="shadow", *, timing=None):
    """Build metadata; jpeg=None avoids base64 for native ROS transport.

    Legacy positional latency arguments are accepted for source compatibility
    only. They never generate calibrated exposure bounds. Pass timing=sample.timing.
    """
    if timing is None:
        if read_mono_s is None or not math.isfinite(read_mono_s) or read_mono_s < 0:
            raise ValueError("Frame needs CaptureTiming or a diagnostic read completion")
        timing = exposure_bounds(dequeue_mono_ns=round(read_mono_s * 1e9))
    if isinstance(timing, dict):
        timing = CaptureTiming.from_dict(timing)
    if not isinstance(timing, CaptureTiming):
        raise TypeError("timing must be CaptureTiming")
    # Metadata comes from a single acquisition event, not independent arguments.
    body = {
        "version": 2, "type": "frame", "frame": ref.to_dict(),
        "read_mono_s": timing.dequeue_mono_ns / 1e9,
        "exposure_estimate_s": timing.exposure_estimate_s,
        "capture_latency_bound_s": timing.age_upper_ns(timing.dequeue_mono_ns) / 1e9 if timing.timing_valid else None,
        "exposure_uncertainty_s": timing.exposure_uncertainty_s,
        "calibration_fingerprint": fingerprint,
        "history": history if history is not None else [], "history_start_s": history_start_s,
        "settled": bool(settled and timing.timing_valid), "mode": mode,
        **timing.to_dict(),
    }
    validate_frame(body)
    if jpeg is not None:
        body["jpeg"] = base64.b64encode(jpeg).decode("ascii")
    return body


def encode_frame(value: dict) -> str:
    return json.dumps(value, separators=(",", ":"), allow_nan=False)


def validate_frame(value: dict) -> CaptureTiming:
    """Validate metadata independent of JPEG/ROS; returns its timing object."""
    if (not isinstance(value, dict) or set(value) != FRAME_FIELDS or value["type"] != "frame"
            or type(value["version"]) is not int or value["version"] != 2):
        raise ProtocolError("Unexpected v2 frame envelope; legacy read timestamps are not calibrated")
    FrameRef.from_dict(value["frame"])
    try:
        timing = CaptureTiming.from_dict({k: value[k] for k in TIMING_FIELDS})
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"Invalid capture timing: {exc}") from exc
    for name in ("read_mono_s", "exposure_estimate_s", "history_start_s"):
        v = value[name]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
            raise ProtocolError(f"Invalid frame {name}")
    if (type(value["settled"]) is not bool or not isinstance(value["history"], list) or len(value["history"]) > 256
            or any(not isinstance(x, dict) for x in value["history"])):
        raise ProtocolError("Invalid frame control history")
    if not isinstance(value["calibration_fingerprint"], str) or not re.fullmatch(r"[0-9a-f]{64}", value["calibration_fingerprint"]):
        raise ProtocolError("Missing calibration fingerprint")
    if value["mode"] not in {"shadow", "teleop", "baseline", "wm"}:
        raise ProtocolError("Unknown frame runtime mode")
    # A newly completed MCU handshake can postdate a queued camera frame.
    # Keep that honest coverage boundary; the filter/exporter must reset or
    # reject the transition instead of clamping the history into the past.
    expected = {"read_mono_s": timing.dequeue_mono_ns / 1e9,
                "exposure_estimate_s": timing.exposure_estimate_s,
                "capture_latency_bound_s": timing.age_upper_ns(timing.dequeue_mono_ns) / 1e9 if timing.timing_valid else None,
                "exposure_uncertainty_s": timing.exposure_uncertainty_s}
    for name, target in expected.items():
        value_field = value[name]
        if target is None:
            if value_field is not None:
                raise ProtocolError(f"Uncalibrated {name} must be null")
        elif (isinstance(value_field, bool) or not isinstance(value_field, (int, float))
              or not math.isfinite(value_field) or not math.isclose(value_field, target, rel_tol=0, abs_tol=2e-9)):
            raise ProtocolError(f"Frame {name} disagrees with explicit acquisition bounds")
    if not timing.timing_valid and value["settled"]:
        raise ProtocolError("Uncalibrated exposure cannot establish a settled frame")
    return timing


def _jpeg_dimensions(jpeg: bytes) -> tuple[int, int]:
    """Read SOF before image allocation to reject oversized JPEG dimensions."""
    if len(jpeg) < 4 or jpeg[:2] != b"\xff\xd8":
        raise ProtocolError("Frame is not JPEG")
    position = 2
    sof_markers = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while position < len(jpeg):
        if jpeg[position] != 0xFF:
            raise ProtocolError("Malformed JPEG marker")
        while position < len(jpeg) and jpeg[position] == 0xFF:
            position += 1
        if position >= len(jpeg):
            break
        marker = jpeg[position]
        position += 1
        if marker in {0xD9, 0xDA}:
            break
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:
            continue
        if position + 2 > len(jpeg):
            break
        size = int.from_bytes(jpeg[position:position + 2], "big")
        if size < 2 or position + size > len(jpeg):
            break
        if marker in sof_markers:
            if size < 8:
                break
            return int.from_bytes(jpeg[position + 3:position + 5], "big"), int.from_bytes(jpeg[position + 5:position + 7], "big")
        position += size
    raise ProtocolError("JPEG is missing valid dimensions")


def decode_frame_bytes(value: dict, jpeg: bytes) -> tuple[dict, np.ndarray]:
    """Native ROS byte transport entrypoint; no base64 encode/decode roundtrip."""
    import cv2
    validate_frame(value)
    if not isinstance(jpeg, (bytes, bytearray)) or len(jpeg) > 2_000_000:
        raise ProtocolError("Invalid/oversized JPEG payload")
    height, width = _jpeg_dimensions(jpeg)
    if not 16 <= height <= 1080 or not 16 <= width <= 1920:
        raise ProtocolError("Invalid/oversized frame image dimensions")
    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None or bgr.shape != (height, width, 3):
        raise ProtocolError("Invalid frame image")
    return value, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def decode_frame(text: str) -> tuple[dict, np.ndarray]:
    value = decode_json(text, MAX_FRAME_MESSAGE_BYTES)
    if not isinstance(value, dict) or set(value) != FRAME_FIELDS | {"jpeg"}:
        raise ProtocolError("Unexpected v2 JSON frame envelope")
    try:
        jpeg = base64.b64decode(value["jpeg"], validate=True)
    except (ValueError, TypeError) as exc:
        raise ProtocolError("Invalid JPEG base64") from exc
    metadata = {key: item for key, item in value.items() if key != "jpeg"}
    _, rgb = decode_frame_bytes(metadata, jpeg)
    return value, rgb
