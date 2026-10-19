"""Boundary and provenance faults: acquisition age is never read() duration."""
from dataclasses import replace
import json

import numpy as np
import pytest

from room_world_model.timebase import (
    CaptureTiming, TimingConfig, TIMESTAMP_MONOTONIC, TIMESTAMP_COPY,
    TSTAMP_SRC_SOE, exposure_bounds,
)
from room_world_model.frames import make_frame, encode_frame, decode_frame, decode_frame_bytes, validate_frame
from room_world_model.protocol import FrameRef, ProtocolError


def config(**kwargs):
    return TimingConfig(accepted=True, evidence_id="measured-optical-report", **kwargs)


def test_soe_bound_includes_camera_queue_and_does_not_use_read_duration():
    timing = exposure_bounds(dequeue_mono_ns=20_000_000_000, v4l2_timestamp_ns=18_000_000_000,
        v4l2_flags=TIMESTAMP_MONOTONIC | TSTAMP_SRC_SOE, hardware_sequence=500,
        config=config(timestamp_uncertainty_s=.002))
    assert timing.exposure_earliest_ns == 17_998_000_000
    assert timing.exposure_latest_ns == 18_002_000_000
    assert timing.age_upper_ns(20_250_000_000) == 2_252_000_000
    assert timing.timestamp_source == "SOE"
    assert CaptureTiming.from_dict(timing.to_dict()) == timing


def test_eof_needs_exposure_readout_and_software_timestamp_error():
    options = dict(dequeue_mono_ns=20_000_000_000, v4l2_timestamp_ns=19_900_000_000,
                   v4l2_flags=TIMESTAMP_MONOTONIC)
    missing = exposure_bounds(**options, config=config(timestamp_uncertainty_s=.003))
    assert not missing.timing_valid and missing.exposure_earliest_ns is None
    bounded = exposure_bounds(**options, config=config(timestamp_uncertainty_s=.003, exposure_readout_bound_s=.150))
    assert bounded.exposure_earliest_ns == 19_747_000_000
    assert bounded.exposure_latest_ns == 19_903_000_000
    assert bounded.age_upper_ns(20_000_000_000) == 253_000_000


@pytest.mark.parametrize("flags", [None, 0, TIMESTAMP_COPY, 0x6000, TIMESTAMP_MONOTONIC | 0x20000])
def test_unknown_clock_or_source_only_uses_independent_empirical_bound(flags):
    value = exposure_bounds(dequeue_mono_ns=30_000_000_000, v4l2_timestamp_ns=1_700_000_000_000_000_000,
        v4l2_flags=flags, config=config(empirical_latency_bound_s=.9))
    if flags == TIMESTAMP_MONOTONIC | 0x20000:
        # A future advertised monotonic time is an actual fault, not fallback.
        assert not value.timing_valid
    else:
        assert value.timestamp_source == "EMPIRICAL" and value.timing_valid
        assert value.exposure_earliest_ns == 29_100_000_000
        assert value.exposure_latest_ns == 30_000_000_000


def test_unaccepted_example_values_and_unknown_timestamp_fail_closed():
    value = exposure_bounds(dequeue_mono_ns=20_000_000_000,
        config=TimingConfig(empirical_latency_bound_s=.85, timestamp_uncertainty_s=.001))
    assert not value.timing_valid
    assert value.exposure_earliest_ns is value.exposure_latest_ns is value.exposure_uncertainty_s is None
    assert value.age_upper_ns(20_050_000_000) is None
    with pytest.raises(ValueError, match="evidence_id"):
        TimingConfig(accepted=True, empirical_latency_bound_s=1.)


def test_future_monotonic_stamp_is_not_repaired_by_empirical_fallback():
    value = exposure_bounds(dequeue_mono_ns=10_000_000_000, v4l2_timestamp_ns=11_000_000_000,
        v4l2_flags=TIMESTAMP_MONOTONIC | TSTAMP_SRC_SOE,
        config=config(timestamp_uncertainty_s=.001, empirical_latency_bound_s=2.))
    assert not value.timing_valid and value.timing_reason == "v4l2_timestamp_in_future"


def test_pi_age_cannot_be_evaluated_before_its_dequeue():
    value = exposure_bounds(dequeue_mono_ns=10_000_000_000, config=config(empirical_latency_bound_s=1.))
    with pytest.raises(ValueError, match="same Pi"):
        value.age_upper_ns(9_000_000_000)
    with pytest.raises(ValueError):
        replace(value, exposure_latest_ns=11_000_000_000)
    with pytest.raises(ValueError):
        replace(value, timing_valid=False)


def frame_metadata(timing):
    return make_frame(FrameRef("pi_boot", "camera_a", 1), fingerprint="a" * 64,
        history=[], history_start_s=0., settled=True, mode="shadow", timing=timing)


def test_legacy_latency_arguments_cannot_create_a_valid_exposure():
    value = make_frame(FrameRef("pi_boot", "camera_a", 1), None, 10., .35, .85,
                       "a" * 64, [], 0., True, "shadow")
    assert value["version"] == 2
    assert value["timing_valid"] is False and value["settled"] is False
    assert value["exposure_earliest_ns"] is value["capture_latency_bound_s"] is None


def test_native_and_diagnostic_frame_paths_preserve_identical_acquisition_metadata():
    cv2 = pytest.importorskip("cv2")
    timing = exposure_bounds(dequeue_mono_ns=10_000_000_000, config=config(empirical_latency_bound_s=.8))
    value = frame_metadata(timing)
    ok, jpeg = cv2.imencode(".jpg", np.full((32, 48, 3), 80, np.uint8))
    assert ok
    received, rgb = decode_frame_bytes(value, jpeg.tobytes())
    assert received == value and rgb.shape == (32, 48, 3)
    diagnostic = make_frame(FrameRef("pi_boot", "camera_a", 1), jpeg.tobytes(),
        fingerprint="a" * 64, timing=timing, settled=True)
    decoded, second = decode_frame(encode_frame(diagnostic))
    assert {k: v for k, v in decoded.items() if k != "jpeg"} == value
    np.testing.assert_array_equal(rgb, second)


@pytest.mark.parametrize("change", [
    {"exposure_estimate_s": 0.01}, {"capture_latency_bound_s": 0.}, {"timing_valid": 1},
    {"exposure_earliest_ns": -1}, {"timestamp_source": "ROS_NOW"}, {"version": 1},
    {"version": True}, {"dequeue_mono_ns": True}, {"extra": "untrusted"},
])
def test_frame_timing_tampering_rejected(change):
    value = frame_metadata(exposure_bounds(dequeue_mono_ns=10_000_000_000,
                           config=config(empirical_latency_bound_s=.8)))
    value.update(change)
    with pytest.raises(ProtocolError):
        validate_frame(value)


def test_unbounded_frame_can_be_viewed_but_cannot_claim_settled():
    value = frame_metadata(exposure_bounds(dequeue_mono_ns=10_000_000_000))
    validate_frame(value)
    value["settled"] = True
    with pytest.raises(ProtocolError, match="settled"):
        validate_frame(value)


def test_history_reset_after_capture_is_preserved_for_downstream_transition_rejection():
    value = frame_metadata(exposure_bounds(dequeue_mono_ns=10_000_000_000,
                           config=config(empirical_latency_bound_s=.8)))
    value["history_start_s"] = 10.05
    validate_frame(value)
    assert value["history_start_s"] > value["read_mono_s"]


def test_jpeg_large_dimensions_rejected_before_decode(monkeypatch):
    cv2 = pytest.importorskip("cv2")
    value = frame_metadata(exposure_bounds(dequeue_mono_ns=10_000_000_000))
    # Minimal SOF header advertising an enormous allocation.
    jpeg = bytes.fromhex("FFD8 FFC0 0008 08 FFFF FFFF 03 FFD9")
    monkeypatch.setattr(cv2, "imdecode", lambda *args: pytest.fail("Must reject oversized SOF before allocating"))
    with pytest.raises(ProtocolError, match="dimensions"):
        decode_frame_bytes(value, jpeg)
