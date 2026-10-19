"""Optical calibration split integrity and simulated GPIO/camera event pairing."""
import csv
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
import pytest

from room_world_model.camera import CameraSample
from room_world_model.timebase import exposure_bounds
from room_world_model.timing_calibration import (
    OPTICAL_COLUMNS, camera_mode_identity, collect_optical_latency, fit_optical_latency,
)


def write_events(path, latencies, *, session, split, mode="a" * 64):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OPTICAL_COLUMNS)
        writer.writeheader()
        for index, latency in enumerate(latencies):
            start = (index + 100) * 10**9
            writer.writerow({"session": session, "split": split, "camera_mode_id": mode,
                "event_id": index, "condition": "simulated", "target_on": index % 2,
                "event_earliest_ns": start, "event_latest_ns": start + 1_000_000,
                "first_seen_dequeue_ns": start + int(latency * 1e9)})
    return path


def test_max_plus_margin_is_fit_only_and_does_not_hide_holdout_outliers(tmp_path):
    fit = write_events(tmp_path / "fit.csv", [.2, .3, .25], session="training", split="fit")
    check = write_events(tmp_path / "check.csv", [.32, .7, .28], session="later-room-run", split="check")
    report = fit_optical_latency([fit, check], requested_fps=10., min_per_split=3)
    assert report["candidate_latency_bound_s"] == pytest.approx(.55)
    assert not report["empirical_holdout_pass"]
    assert report["suggested_timing_config"] is None
    assert report["check_violations"] == [{"file": "check.csv", "line": 3, "latency_s": .7}]
    assert report["accepted"] is report["motor_enable_allowed"] is False


def test_success_still_requires_manual_review_and_never_infers_positive_minimum(tmp_path):
    fit = write_events(tmp_path / "fit.csv", [.2, .3, .25], session="training", split="fit")
    check = write_events(tmp_path / "check.csv", [.32, .3, .28], session="later-room-run", split="check")
    report = fit_optical_latency([fit, check], requested_fps=10., min_per_split=3)
    suggested = report["suggested_timing_config"]
    assert report["empirical_holdout_pass"] and suggested["empirical_latency_bound_s"] == pytest.approx(.55)
    assert suggested["empirical_latency_min_s"] == 0.0
    assert suggested["timestamp_uncertainty_s"] is suggested["exposure_readout_bound_s"] is None
    assert suggested["accepted"] is False
    assert len(suggested["evidence_id"]) == 64
    assert "p99_9" not in report["fit_latency_upper_s"]


@pytest.mark.parametrize("fault", ["same_session", "different_mode", "duplicate_file"])
def test_calibration_provenance_prevents_leakage_and_mixed_camera_modes(tmp_path, fault):
    fit = write_events(tmp_path / "fit.csv", [.2, .3, .25], session="training", split="fit")
    check = write_events(tmp_path / "check.csv", [.32, .31, .28],
        session="training" if fault == "same_session" else "later-room-run", split="check",
        mode="b" * 64 if fault == "different_mode" else "a" * 64)
    paths = [fit, check, fit] if fault == "duplicate_file" else [fit, check]
    with pytest.raises(ValueError):
        fit_optical_latency(paths, requested_fps=10., min_per_split=3)


def test_camera_mode_hash_tracks_resolution_controls_and_backend_not_timing_acceptance():
    config = {"device": "/dev/video0", "width": 640, "height": 480, "requested_fps": 15}
    original, _ = camera_mode_identity(config)
    assert camera_mode_identity({**config, "timing": {"accepted": True}})[0] == original
    assert camera_mode_identity({**config, "width": 1280})[0] != original
    assert camera_mode_identity({**config, "controls": {"exposure_auto": 1}})[0] != original
    assert camera_mode_identity({**config, "backend": "opencv"})[0] != original


def test_gpio_collector_pairs_events_and_closes_resources_without_any_real_hardware(tmp_path):
    class LED:
        state = False
        closed = False
        def on(self):
            self.state = True
        def off(self):
            self.state = False
        def close(self):
            self.closed = True
    led = LED()

    class Camera:
        metadata = {"backend": "simulated", "reported_fps": 15.}
        error = None
        sequence = 0
        closed = False
        def take_latest(self):
            self.sequence += 1
            now = time.monotonic_ns()
            image = np.full((16, 24, 3), 255 if led.state else 0, np.uint8)
            return CameraSample(image, now / 1e9, .0, self.sequence, "fake-camera-session",
                                exposure_bounds(dequeue_mono_ns=now), now / 1e9)
        def close(self):
            self.closed = True
    camera = Camera()
    csv_path = tmp_path / "fit.csv"
    report = collect_optical_latency({"width": 24, "height": 16, "requested_fps": 15.}, csv_path,
        gpio_pin=17, roi=(0, 0, 24, 16), split="fit", condition="simulated", events=4,
        min_hold_s=.001, max_hold_s=.002, event_timeout_s=.2, camera_factory=lambda _: camera,
        led_factory=lambda _: led)
    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert report["events"] == 4 and report["accepted"] is False
    assert [int(row["target_on"]) for row in rows] == [1, 0, 1, 0]
    for row in rows:
        assert int(row["event_earliest_ns"]) <= int(row["event_latest_ns"]) <= int(row["first_seen_dequeue_ns"])
    assert camera.closed and led.closed and not led.state
