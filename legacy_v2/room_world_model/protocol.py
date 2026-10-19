"""Small, strict wire messages shared by the Pi governor and the PC planner.

No PC timestamp can make a picture fresh.  A proposal names a frame originally
registered on the Pi, and echoes an explicitly issued arming lease.  These
messages prevent accidental replay; they are not network authentication.  Use a
private robot network and keep motor topics private to the Pi governor.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import re
from typing import Any, Mapping


PROTOCOL_VERSION = 1
# Version 1 applies only to the strict in-process/diagnostic JSON codecs above.
# The generated ROS interfaces carry schema 2; motor serial transport is version 3.
ACTUATION_SCHEMA_VERSION = 2
MAX_CONTROL_MESSAGE_BYTES = 16384
_ID = re.compile(r"^[A-Za-z0-9_-]{1,128}$")


class ProtocolError(ValueError):
    """Malformed control or geometry input; adapters must command zero on it."""


def _object(value: Any, keys: set[str], name: str) -> Mapping[str, Any]:
    if not isinstance(value, dict) or set(value) != keys:
        raise ProtocolError(f"{name}: wrong object fields")
    return value


def _number(value: Any, name: str, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name}: expected a finite number")
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ProtocolError(f"{name}: out of range or non-finite")
    return result


def _integer(value: Any, name: str, low: int = 0, high: int = 2**63 - 1) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ProtocolError(f"{name}: expected an integer in range")
    return value


def _boolean(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise ProtocolError(f"{name}: expected a boolean")
    return value


def _identifier(value: Any, name: str, allow_empty: bool = False) -> str:
    if allow_empty and value == "":
        return ""
    if not isinstance(value, str) or _ID.fullmatch(value) is None:
        raise ProtocolError(f"{name}: expected a short session identifier")
    return value


@dataclass(frozen=True)
class FrameRef:
    """Pi boot/session, camera stream epoch, and strictly increasing frame ID."""

    session_id: str
    camera_epoch: str
    frame_seq: int

    def validate(self) -> None:
        _identifier(self.session_id, "session_id")
        _identifier(self.camera_epoch, "camera_epoch")
        _integer(self.frame_seq, "frame_seq")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)

    @classmethod
    def from_dict(cls, value: Any) -> "FrameRef":
        obj = _object(value, {"session_id", "camera_epoch", "frame_seq"}, "frame")
        frame = cls(**obj)
        frame.validate()
        return frame


@dataclass(frozen=True)
class MotionCommand:
    """One proposal for one real frame.  TTL starts on Pi receipt and is capped.

    ``inference_ms`` is diagnostic and an additional rejection check.  The Pi
    independently checks local frame age, so lying about this value never makes
    an old frame acceptable.  ``lease_id=''`` is useful for shadow predictions.
    """

    frame: FrameRef
    lease_id: str
    command_seq: int
    linear_mps: float
    angular_rps: float
    valid_for_ms: int = 120
    inference_ms: float = 0.0

    def validate(self) -> None:
        if not isinstance(self.frame, FrameRef):
            raise ProtocolError("frame: expected FrameRef")
        self.frame.validate()
        _identifier(self.lease_id, "lease_id", allow_empty=True)
        _integer(self.command_seq, "command_seq")
        _number(self.linear_mps, "linear_mps", -5.0, 5.0)
        _number(self.angular_rps, "angular_rps", -20.0, 20.0)
        _integer(self.valid_for_ms, "valid_for_ms", 1, 1000)
        _number(self.inference_ms, "inference_ms", 0.0, 60000.0)

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return {"version": PROTOCOL_VERSION, "type": "motion", **asdict(self)}

    @classmethod
    def from_dict(cls, value: Any) -> "MotionCommand":
        obj = dict(_object(value, {
            "version", "type", "frame", "lease_id", "command_seq",
            "linear_mps", "angular_rps", "valid_for_ms", "inference_ms",
        }, "motion"))
        _header(obj, "motion")
        obj["frame"] = FrameRef.from_dict(obj["frame"])
        command = cls(**obj)
        command.validate()
        return command


@dataclass(frozen=True)
class MetricGeometry:
    """Current observed clearance for the exact proposed curve.

    ``clearance_m`` is conservative path distance FROM the inflated robot
    footprint, after depth/scale error and unobserved-space handling; it is not
    raw camera range.  A caller cannot obtain this just by labelling normalized
    monocular depth as metres. ``covers_swept_footprint`` must be false when
    relevant unseen space is being assumed free. Camera-only research mode
    still offers no certified collision protection or moving-person guarantee.
    """

    frame: FrameRef
    clearance_m: float
    path_angular_rps: float = 0.0
    metric_valid: bool = False
    observed_half_fov_rad: float = 0.0
    covers_swept_footprint: bool = False
    path_linear_mps: float = 0.0

    def validate(self) -> None:
        if not isinstance(self.frame, FrameRef):
            raise ProtocolError("frame: expected FrameRef")
        self.frame.validate()
        _number(self.clearance_m, "clearance_m", 0.0, 100.0)
        _number(self.path_angular_rps, "path_angular_rps", -20.0, 20.0)
        _number(self.path_linear_mps, "path_linear_mps", -5.0, 5.0)
        _boolean(self.metric_valid, "metric_valid")
        _boolean(self.covers_swept_footprint, "covers_swept_footprint")
        _number(self.observed_half_fov_rad, "observed_half_fov_rad", 0.0, math.pi)

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return {"version": PROTOCOL_VERSION, "type": "geometry", **asdict(self)}

    @classmethod
    def from_dict(cls, value: Any) -> "MetricGeometry":
        obj = dict(_object(value, {
            "version", "type", "frame", "clearance_m", "path_angular_rps",
            "metric_valid", "observed_half_fov_rad", "covers_swept_footprint",
            "path_linear_mps",
        }, "geometry"))
        _header(obj, "geometry")
        obj["frame"] = FrameRef.from_dict(obj["frame"])
        evidence = cls(**obj)
        evidence.validate()
        return evidence


def _header(obj: dict[str, Any], expected_type: str) -> None:
    if _integer(obj.pop("version"), "version", 1, 100) != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if obj.pop("type") != expected_type:
        raise ProtocolError("wrong message type")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ProtocolError(f"invalid JSON constant: {value}")


def decode_json(payload: str | bytes, max_bytes: int = MAX_CONTROL_MESSAGE_BYTES) -> Any:
    if not isinstance(payload, (str, bytes)):
        raise ProtocolError("expected UTF-8 JSON text")
    try:
        if (len(payload.encode("utf-8")) if isinstance(payload, str) else len(payload)) > max_bytes:
            raise ProtocolError("message too large")
        return json.loads(payload, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    except (ValueError, TypeError, UnicodeError, RecursionError) as exc:
        raise ProtocolError(f"invalid JSON: {exc}") from exc


def encode_command(command: MotionCommand) -> str:
    return json.dumps(command.to_dict(), separators=(",", ":"), allow_nan=False)


def decode_command(payload: str | bytes) -> MotionCommand:
    return MotionCommand.from_dict(decode_json(payload))


def encode_geometry(geometry: MetricGeometry) -> str:
    return json.dumps(geometry.to_dict(), separators=(",", ":"), allow_nan=False)


def decode_geometry(payload: str | bytes) -> MetricGeometry:
    return MetricGeometry.from_dict(decode_json(payload))
