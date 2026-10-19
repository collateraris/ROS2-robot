"""Inspect the requested curve, even when engine arming geometry falls back."""
from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from room_world_model.configuration import load_config
from room_world_model.perception import CameraCalibration, PathClearance
from room_world_model.vision_check import inspect_image


@pytest.mark.parametrize("accepted", [False, True])
def test_offline_report_keeps_turn_blindness_stopping_bound_and_no_permission(tmp_path, monkeypatch, accepted):
    config = load_config(Path(__file__).parents[1] / "config/robot.yaml")
    calibration = replace(CameraCalibration.from_json(Path(config["_root"]) / "config/camera.json"), accepted=accepted)
    camera_path = tmp_path / "camera.json"
    camera_path.write_text(json.dumps(asdict(calibration)), encoding="utf-8")
    config["camera"]["calibration_file"] = str(camera_path)
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr("room_world_model.vision_check.time.monotonic", lambda: clock.now)
    # A changed engine bound must propagate to both the report and path query;
    # the offline CLI may not carry its own stopping-distance formula.
    monkeypatch.setattr("room_world_model.engine.stopping_distance", lambda *args: .83)

    class FakeRectifier:
        valid_mask = np.ones((calibration.height, calibration.width), dtype=bool)
        file_sha256 = "a" * 64

        def validate_calibration(self, value):
            assert value == calibration

        def __call__(self, raw):
            result = raw.copy()
            result[0, 0] = [12, 34, 56]
            return result

    class FakeObservation:
        horizontal_fov_rad = calibration.horizontal_fov_rad

        def to_dict(self):
            return {"person_visible": True, "person_count": 1, "person_distance_m": 2.,
                    "person_distance_valid": accepted, "calibrated": accepted}

        def path_clearance(self, v, w, radius, horizon, margin):
            assert v * horizon >= .83 + .05 - 1e-9
            if w > 0:
                # Straight ahead is observed; the left swept side is blind.
                return PathClearance(2., False, accepted, "left_side_blind")
            if w < 0:
                return PathClearance(.1, True, accepted, "right_obstacle_too_close")
            return PathClearance(2., True, accepted, "straight_observed")

    class FakePerception:
        calls = 0

        def observe(self, rgb, timestamp_ns, image_valid_mask=None):
            self.calls += 1
            assert rgb[0, 0].tolist() == [12, 34, 56]
            assert image_valid_mask is FakeRectifier.valid_mask
            assert timestamp_ns == 1_000_000_000  # explicitly synthetic offline time
            clock.now += .5  # a real slow inference must remain visible in JSON
            return FakeObservation()

    perception = FakePerception()
    raw = np.zeros((calibration.height, calibration.width, 3), dtype=np.uint8)
    report = inspect_image(config, raw, perception, rectifier=FakeRectifier(), image_decode_s=.02)
    by_direction = {entry["direction"]: entry for entry in report["candidates"]}
    assert perception.calls == 3 and not raw.any()
    assert by_direction["forward"]["covers_swept_footprint"] is True
    assert by_direction["left"]["covers_swept_footprint"] is False
    assert by_direction["left"]["path_reason"] == "left_side_blind"
    assert by_direction["left"]["query"]["angular_rps"] > 0
    assert by_direction["right"]["clearance_sufficient"] is False
    for entry in report["candidates"]:
        assert entry["required_clearance_pc_m"] == .83
        assert entry["timing"]["budget_exceeded"] is True
        assert entry["timing"]["image_decode_plus_engine_s"] == pytest.approx(.52)
        assert entry["shadow_command_zero"] is True
        assert entry["motor_enable_allowed"] is False
    assert report["camera_calibration"]["accepted"] is accepted
    assert report["accepted"] is report["motor_enable_allowed"] is False
    assert not report["freshness_evidence_available"]
    assert report["source_exposure_time_s"] is None
    assert report["rectification"]["intrinsics_sha256"] == "a" * 64
    json.dumps(report, allow_nan=False)
