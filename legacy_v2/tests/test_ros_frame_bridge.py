"""ROS adapter contract tests with message-shaped fixtures, not generated ROS/DDS.

No rclpy, ROS build or network is exercised. Actual JPEG decoding and the same
CaptureTiming/MCU validators used by live consumers are exercised end to end.
"""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from room_world_model.actuator import PendingPulse, confirmed_interval
from room_world_model.frames import make_frame, validate_frame
from room_world_model.protocol import FrameRef, ProtocolError
from room_world_model.record import EpisodeRecorder
from room_world_model.ros_interfaces import frame_to_ros, frame_from_ros, INTERVAL_FIELDS
from room_world_model.timebase import TimingConfig, TIMESTAMP_MONOTONIC, TSTAMP_SRC_SOE, exposure_bounds


def header():
    return SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="")


class FrameInfoFixture:
    def __init__(self):
        self.header = header()


class FramePacketFixture:
    def __init__(self):
        self.image = SimpleNamespace(header=header(), format="", data=b"")
        self.history = []


TYPES = SimpleNamespace(FrameInfo=FrameInfoFixture, FramePacket=FramePacketFixture,
                        FrameRef=SimpleNamespace, ActuationInterval=SimpleNamespace)


def timing(kind):
    kwargs = dict(dequeue_mono_ns=12_000_000_000)
    if kind == "SOE":
        kwargs.update(v4l2_timestamp_ns=11_950_000_000, v4l2_flags=TIMESTAMP_MONOTONIC | TSTAMP_SRC_SOE,
            hardware_sequence=0, config=TimingConfig(accepted=True, timestamp_uncertainty_s=.001, evidence_id="optics-1"))
    elif kind == "EOF":
        kwargs.update(v4l2_timestamp_ns=11_990_000_000, v4l2_flags=TIMESTAMP_MONOTONIC, hardware_sequence=15,
            config=TimingConfig(accepted=True, timestamp_uncertainty_s=.002,
                                exposure_readout_bound_s=.050, evidence_id="optics-1"))
    elif kind == "EMPIRICAL":
        kwargs.update(config=TimingConfig(accepted=True, empirical_latency_bound_s=.8, evidence_id="optics-1"))
    elif kind == "UNKNOWN_NATIVE":
        kwargs.update(v4l2_timestamp_ns=0, v4l2_flags=0, hardware_sequence=0)
    return exposure_bounds(**kwargs)


@pytest.mark.parametrize("kind", ["SOE", "EOF", "EMPIRICAL", "UNKNOWN", "UNKNOWN_NATIVE"])
def test_typed_image_frame_roundtrip_preserves_exact_pi_bounds_and_null_semantics(kind):
    capture = timing(kind)
    body = make_frame(FrameRef("boot", "camera-session", 101), fingerprint="f" * 64,
        history_start_s=0., settled=True, timing=capture)
    bgr = np.full((32, 48, 3), (20, 80, 180), np.uint8)
    ok, jpeg = cv2.imencode(".jpg", bgr)
    assert ok
    wire = frame_to_ros(body, jpeg.tobytes(), ros_now_ns=1_760_000_000_000_000_000,
                        pi_now_ns=12_100_000_000, types=TYPES)
    decoded, rgb = frame_from_ros(wire)
    assert decoded == body
    assert rgb.shape == bgr.shape
    np.testing.assert_allclose(rgb[10, 10], [180, 80, 20], atol=5)
    assert wire.info.timing_valid == capture.timing_valid
    assert wire.info.has_v4l2_metadata == (capture.v4l2_timestamp_ns is not None)
    if capture.timing_valid:
        assert wire.info.age_upper_at_publish_s == pytest.approx(capture.age_upper_ns(12_100_000_000) / 1e9)
        assert wire.image.header.stamp.sec > 0
    else:
        assert wire.info.exposure_earliest_ns == wire.info.exposure_latest_ns == 0
        assert decoded["exposure_earliest_ns"] is decoded["exposure_latest_ns"] is None
        assert wire.image.header.stamp.sec == wire.image.header.stamp.nanosec == 0


def test_typed_frame_keeps_confirmed_mcu_segments_and_rejects_tampered_bounds():
    pending = PendingPulse(1, 1, 31, 33, 80, 10.48, accepted_s=10.49,
                           accept_tick_ms=499, acknowledged_s=10.52, start_tick_ms=500)
    event = confirmed_interval(pending, boot_id=33, nonce=22, stop_tick_ms=580,
                               stop_received_s=10.65, stop_reason="completed")
    body = make_frame(FrameRef("boot", "camera-session", 101), fingerprint="f" * 64,
                      history=[event], history_start_s=10., settled=True, timing=timing("SOE"))
    _, jpeg = cv2.imencode(".jpg", np.zeros((32, 48, 3), np.uint8))
    wire = frame_to_ros(body, jpeg.tobytes(), pi_now_ns=12_100_000_000, types=TYPES)
    decoded, _ = frame_from_ros(wire)
    expected = {key: event[key] for key in INTERVAL_FIELDS}
    expected["type"] = "motor_interval"
    assert decoded["history"] == [expected]
    # Extra serial audit fields remain in the Pi recording, outside ROS's
    # minimal interval representation required by the online recurrent filter.
    wire.info.exposure_earliest_ns = wire.info.pi_dequeue_monotonic_ns + 1
    with pytest.raises(ProtocolError):
        frame_from_ros(wire)


def test_recorder_frame_metadata_can_be_reconstructed_without_inventing_timing(tmp_path):
    import json
    from room_world_model.frames import FRAME_FIELDS
    body = make_frame(FrameRef("boot", "camera-session", 101), fingerprint="f" * 64,
                      history_start_s=0., settled=True, timing=timing("EOF"))
    _, jpeg = cv2.imencode(".jpg", np.zeros((32, 48, 3), np.uint8))
    recorder = EpisodeRecorder(tmp_path / "one-session", {"synthetic": False})
    try:
        recorder.frame(jpeg.tobytes(), body)
    finally:
        recorder.close()
    recorded = json.loads((recorder.directory / "events.jsonl").read_text().splitlines()[0])
    assert recorded["type"] == "camera" and "history" not in recorded
    reconstructed = {key: value for key, value in recorded.items() if key in FRAME_FIELDS}
    reconstructed.update(type="frame", history=[])
    assert validate_frame(reconstructed) == timing("EOF")
    assert reconstructed == body
