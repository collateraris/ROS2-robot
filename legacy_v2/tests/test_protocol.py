"""Fault injections at the untrusted PC command boundary; no ROS/hardware needed."""

from dataclasses import replace
import json

import pytest

from room_world_model.protocol import (
    FrameRef, MetricGeometry, MotionCommand, ProtocolError,
    decode_command, decode_geometry, decode_json, encode_command, encode_geometry,
)


FRAME = FrameRef("pi_boot", "camera_1", 123)
COMMAND = MotionCommand(FRAME, "arm_lease", 1, 0.05, 0.0, 100, 25.0)


def test_round_trip_preserves_frame_lease_and_values():
    assert decode_command(encode_command(COMMAND)) == COMMAND
    geometry = MetricGeometry(FRAME, 1.2, 0.0, True, 0.7, True, 0.05)
    assert decode_geometry(encode_geometry(geometry)) == geometry


@pytest.mark.parametrize("payload", [
    "", "[]", "null", "{", "{\"x\":NaN}", "{\"x\":Infinity}",
    '{"x":1,"x":2}', b"\xff", "\ud800", "x" * 16385,
])
def test_corrupt_packets_are_protocol_errors(payload):
    with pytest.raises(ProtocolError):
        decode_command(payload)


@pytest.mark.parametrize("field,value", [
    ("linear_mps", float("nan")), ("angular_rps", float("inf")),
    ("linear_mps", "0.06"), ("linear_mps", True),
    ("command_seq", True), ("command_seq", -1), ("command_seq", 2**64),
    ("valid_for_ms", 0), ("valid_for_ms", 100.0),
    ("inference_ms", -1.0), ("lease_id", "a\nb"),
])
def test_invalid_typed_command_values_rejected(field, value):
    with pytest.raises(ProtocolError):
        replace(COMMAND, **{field: value}).validate()


@pytest.mark.parametrize("mutation", [
    lambda x: x.update(extra="not allowed"),
    lambda x: x.pop("lease_id"),
    lambda x: x.update(version=2),
    lambda x: x.update(version=True),
    lambda x: x.update(type="geometry"),
    lambda x: x["frame"].update(frame_seq=True),
    lambda x: x["frame"].update(timestamp=0),
])
def test_unknown_fields_and_protocol_mismatch_rejected(mutation):
    obj = COMMAND.to_dict()
    mutation(obj)
    with pytest.raises(ProtocolError):
        decode_command(json.dumps(obj))


def test_duplicate_nested_key_cannot_override_original_frame():
    payload = encode_command(COMMAND).replace('"frame_seq":123', '"frame_seq":123,"frame_seq":999')
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_a_numeric_string_is_not_a_frame_sequence():
    with pytest.raises(ProtocolError):
        FrameRef.from_dict({"session_id": "pi", "camera_epoch": "camera", "frame_seq": "1"})


def test_geometry_requires_real_booleans_and_finite_metric_evidence():
    for field, value in [("metric_valid", "true"), ("clearance_m", float("inf")),
                         ("covers_swept_footprint", 1), ("observed_half_fov_rad", -1)]:
        evidence = replace(MetricGeometry(FRAME, 1.0), **{field: value})
        with pytest.raises(ProtocolError):
            encode_geometry(evidence)


def test_json_limit_is_configurable_for_image_envelope_adapter():
    assert decode_json('{"x":0}', max_bytes=20) == {"x": 0}
    with pytest.raises(ProtocolError):
        decode_json('{"x":0}', max_bytes=4)
