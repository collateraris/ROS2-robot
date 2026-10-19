from dataclasses import replace
import json
import math

import numpy as np
import pytest

from room_world_model.label import apply_manual_corrections, label_episode
from room_world_model.calibrate import ImageRectifier, fit_depth_bounds, probe_camera
from room_world_model.perception import (
    CameraCalibration, CameraPerception, PersonDetection, PerceptionConfig,
    MetricDepthAnything, YOLOPersonDetector,
)


def calibration(**kwargs):
    values = dict(width=160, height=120, fx=100, fy=100, cx=80, cy=60,
                  camera_height_m=.2, camera_pitch_down_rad=.15,
                  robot_radius_m=.12, depth_relative_error=.05, depth_absolute_error_m=.05)
    values.update(kwargs)
    return CameraCalibration(**values)


def observe(cal=None, detections=(), depth=None):
    cal = cal or calibration()
    depth = np.full((cal.height, cal.width), 2.0) if depth is None else depth
    engine = CameraPerception(cal, lambda rgb: detections, lambda rgb: depth)
    frame = engine.observe(np.zeros((cal.height, cal.width, 3), dtype=np.uint8), 123_456_789)
    return engine, frame


def test_exposure_timestamp_is_never_restamped():
    _, frame = observe()
    assert frame.timestamp_ns == 123_456_789


def test_no_calibration_acceptance_cannot_grant_metric_motion():
    _, frame = observe(detections=[PersonDetection((50, 15, 110, 110), .9)])
    assert frame.person_visible and frame.person_distance_estimated
    assert not frame.person_distance_valid
    assert not any(frame.clearance_valid)
    assert not frame.path_clearance(.1, 0, .12, .1).metric_valid
    aux, mask = frame.to_aux()
    assert mask[:2].all() and not mask[2:].any()
    assert aux[2:].tolist() == [0, 0, 0, 0]


def test_explicit_offline_pseudo_option_does_not_enable_runtime_motion():
    _, frame = observe(detections=[PersonDetection((50, 15, 110, 110), .9)])
    _, mask = frame.to_aux(require_calibration=False)
    assert mask[2]
    assert not frame.person_distance_valid


def test_unique_person_bearing_sign_and_conservative_distance():
    _, frame = observe(calibration(accepted=True), [PersonDetection((15, 20, 55, 110), .9)])
    assert frame.person_bearing_rad > 0  # left in image -> left robot bearing
    assert frame.person_distance_valid
    assert 0 < frame.person_distance_m < 2.3


def test_two_people_do_not_silently_select_largest_or_nearest():
    _, frame = observe(calibration(accepted=True), [
        PersonDetection((5, 15, 65, 110), .98), PersonDetection((90, 20, 130, 105), .91),
    ])
    assert frame.person_count == 2 and frame.target_ambiguous
    assert not frame.person_visible
    assert not frame.to_aux()[1][:3].any()


def test_uncertain_second_person_also_blocks_target_selection():
    _, frame = observe(calibration(accepted=True), [
        PersonDetection((5, 15, 65, 110), .98), PersonDetection((90, 20, 130, 105), .4),
    ])
    assert frame.target_ambiguous and not frame.person_visible


def test_unknown_depth_never_becomes_free_space():
    _, frame = observe(calibration(accepted=True), depth=np.full((120, 160), np.nan))
    assert frame.clearances_m == (0, 0, 0)
    assert not any(frame.clearance_valid)


def test_uncalibrated_resize_and_new_timestamp_are_rejected():
    engine, _ = observe()
    with pytest.raises(ValueError):
        engine.observe(np.zeros((128, 128, 3), np.uint8), 100)
    with pytest.raises(ValueError):
        engine.observe(np.zeros((120, 160, 3), np.uint8), 0)


def floor_depth(cal):
    yy, _ = np.indices((cal.height, cal.width), dtype=float)
    denominator = math.sin(cal.camera_pitch_down_rad) + math.cos(cal.camera_pitch_down_rad) * (yy - cal.cy) / cal.fy
    return np.where(denominator > 0, cal.camera_height_m / np.maximum(denominator, 1e-6), np.nan)


def test_clear_centre_cannot_approve_blind_side_sweep():
    cal = calibration(accepted=True, depth_absolute_error_m=0, depth_relative_error=0)
    _, frame = observe(cal, depth=floor_depth(cal))
    result = frame.path_clearance(.1, .1, .12, .1, .02)
    assert not result.covers_swept_footprint
    assert "blind" in result.reason


def test_ground_visible_overhead_fixture_can_pass_coverage_check():
    # Synthetic geometry, not a claim about the user's forward USB camera.
    cal = calibration(accepted=True, camera_height_m=.7, camera_pitch_down_rad=1.15,
                      camera_forward_m=-.35, depth_absolute_error_m=0, depth_relative_error=0,
                      fx=80, fy=80)
    _, frame = observe(cal, depth=floor_depth(cal))
    result = frame.path_clearance(.1, 0, .12, .1, .02)
    assert result.metric_valid
    assert result.covers_swept_footprint
    assert result.clearance_m == pytest.approx(.01)


def test_single_pixel_obstacle_limits_path_without_point_subsampling():
    cal = calibration(accepted=True, camera_height_m=.7, camera_pitch_down_rad=1.15,
                      camera_forward_m=-.35, depth_absolute_error_m=0, depth_relative_error=0,
                      fx=80, fy=80)
    depth = floor_depth(cal)
    # A tiny visible obstacle at about x=.35m and height=.15m. It must survive
    # point-cloud construction even if represented by just one depth pixel.
    dx, rel_z = .35 - cal.camera_forward_m, .15 - cal.camera_height_m
    sp, cp = math.sin(cal.camera_pitch_down_rad), math.cos(cal.camera_pitch_down_rad)
    zc, yc = cp * dx - sp * rel_z, -sp * dx - cp * rel_z
    column, row = round(cal.cx), round(cal.cy + cal.fy * yc / zc)
    depth[row, column] = zc
    _, frame = observe(cal, depth=depth)
    result = frame.path_clearance(.1, 0, .12, 5.0, .02)
    assert 0 < result.clearance_m < .25
    assert result.covers_swept_footprint


def test_rectification_invalid_pixels_are_never_free():
    engine, _ = observe(calibration(accepted=True))
    frame = engine.observe(np.zeros((120, 160, 3), np.uint8), 999,
                           image_valid_mask=np.zeros((120, 160), bool))
    assert not any(frame.clearance_valid)


@pytest.mark.parametrize("v,w", [(0, .5), (-.1, 0), (.1, float("nan"))])
def test_unsupported_motion_is_rejected(v, w):
    _, frame = observe(calibration(accepted=True))
    assert not frame.path_clearance(v, w, .12, .1).covers_swept_footprint


def test_metric_configuration_has_no_silent_scale_acceptance():
    with pytest.raises(ValueError):
        calibration(depth_scale_factor=0)
    with pytest.raises(ValueError):
        calibration(accepted="yes")


def test_manual_corrections_mask_absent_target_and_keep_units_explicit():
    aux, mask = np.zeros((2, 6), np.float32), np.zeros((2, 6), bool)
    changed = apply_manual_corrections(aux, mask, {"frames": {"1": {
        "person_visible": False, "clearance_m": [1.5, .6, 0], "clearance_valid": [True, True, False],
    }}})
    assert changed == [1]
    assert np.allclose(aux[1], [0, 0, 0, .5, .2, 0])
    assert mask[1].tolist() == [True, False, False, True, True, False]


def test_direct_corrections_require_explicit_mask():
    with pytest.raises(ValueError):
        apply_manual_corrections(np.zeros((1, 6)), np.zeros((1, 6), bool),
                                 {"0": {"aux": [1, 0, .5, 1, 1, 1]}})


def test_label_episode_preserves_actions_timing_and_marks_teacher(tmp_path):
    cal = calibration(accepted=False)
    engine, _ = observe(cal, [PersonDetection((45, 20, 115, 115), .9)])
    source, target = tmp_path / "episode.npz", tmp_path / "labels.npz"
    actions = np.array([[.06, .1], [0, 0]], np.float32)
    times = np.array([1.0, 1.1, 1.3])
    np.savez_compressed(source, images=np.zeros((3, 128, 128), np.uint8), timestamps=times,
                        rgb_images=np.zeros((3, 120, 160, 3), np.uint8), actions=actions,
                        dt=np.diff(times).astype(np.float32), transition_valid=np.ones(2, bool),
                        metadata_json=np.array(json.dumps({"room": "room_A", "split": "train"})))
    report = label_episode(source, target, engine, allow_uncalibrated_pseudo=True)
    assert report["frames"] == 3
    with np.load(target, allow_pickle=False) as data:
        assert np.array_equal(data["actions"], actions)
        assert np.array_equal(data["timestamps"], times)
        assert data["aux"].shape == (3, 6)
        metadata = json.loads(data["metadata_json"].item())
        assert metadata["room"] == "room_A"
        assert metadata["annotation_history"][0]["not_safety_ground_truth"]
        assert not metadata["annotation_history"][0]["calibration"]["accepted"]


def test_teacher_refuses_grayscale_only_episode(tmp_path):
    engine, _ = observe()
    source = tmp_path / "gray_only.npz"
    np.savez(source, images=np.zeros((2, 128, 128), np.uint8), timestamps=[1, 2])
    with pytest.raises(ValueError, match="original RGB"):
        label_episode(source, tmp_path / "bad.npz", engine)
    assert not (tmp_path / "bad.npz").exists()


def test_depth_envelope_uses_separate_sessions_and_never_accepts_motor(tmp_path):
    source = tmp_path / "depth.csv"
    rows = ["predicted_m,reference_m,session,split"]
    for split, session in [("fit", "a"), ("check", "b")]:
        for index in range(12):
            reference = .5 + index * .2
            rows.append(f"{reference * 1.1},{reference},{session},{split}")
    source.write_text("\n".join(rows), encoding="utf-8")
    report = fit_depth_bounds(source)
    assert report["empirical_holdout_pass"]
    assert report["suggested_camera_update"]["depth_scale_factor"] == pytest.approx(1 / 1.1)
    assert report["accepted"] is False and report["motor_enable_allowed"] is False
    rows[-1] = "2.97,0.2,b,check"
    source.write_text("\n".join(rows), encoding="utf-8")
    report = fit_depth_bounds(source)
    assert not report["empirical_holdout_pass"]
    assert report["check_underestimated_closeness_count"] == 1


def test_depth_calibration_rejects_session_leakage(tmp_path):
    source = tmp_path / "leak.csv"
    source.write_text("predicted_m,reference_m,session,split\n" +
        "\n".join(f"1,1,same,{split}" for split in ["fit"] * 10 + ["check"] * 10), encoding="utf-8")
    with pytest.raises(ValueError, match="session"):
        fit_depth_bounds(source)


def test_rectification_must_match_perception_intrinsics(tmp_path):
    cal = calibration()
    matrix = [[cal.fx, 0, cal.cx], [0, cal.fy, cal.cy], [0, 0, 1]]
    source = tmp_path / "intrinsics.json"
    source.write_text(json.dumps({"image_width": cal.width, "image_height": cal.height,
        "camera_matrix": matrix, "rectified_camera_matrix": matrix,
        "distortion_coefficients": [0, 0, 0, 0, 0]}), encoding="utf-8")
    rectifier = ImageRectifier(source)
    rectifier.validate_calibration(cal)
    assert rectifier(np.zeros((120, 160, 3), np.uint8)).shape == (120, 160, 3)
    with pytest.raises(ValueError, match="matrix"):
        rectifier.validate_calibration(replace(cal, fx=cal.fx + 1))


def test_passive_probe_does_not_claim_exposure_latency(monkeypatch):
    import cv2
    class FakeCapture:
        def isOpened(self): return True
        def set(self, *_): return True
        def get(self, *_): return 15
        def getBackendName(self): return "FAKE"
        def read(self): return True, np.zeros((120, 160, 3), np.uint8)
        def release(self): pass
    monkeypatch.setattr(cv2, "VideoCapture", lambda *_: FakeCapture())
    report = probe_camera(frames=4, warmup=0, width=160, height=120)
    assert report["frames"] == 4
    assert report["read_duration_ms"] is not None
    assert report["capture_delay_bound_s"] is None
    assert report["optical_exposure_latency_measured"] is False


def test_manual_relabel_cannot_silently_change_normalization(tmp_path):
    source = tmp_path / "old.npz"
    np.savez(source, images=np.zeros((2, 128, 128), np.uint8), timestamps=[1, 2],
             aux=np.zeros((2, 6), np.float32), aux_mask=np.zeros((2, 6), bool),
             metadata_json=np.array(json.dumps({"depth_scale_m": 3.0})))
    with pytest.raises(ValueError, match="depth_scale_m"):
        label_episode(source, tmp_path / "changed.npz", None, depth_scale_m=4,
                      corrections={"0": {"person_visible": False}})


def test_relative_checkpoint_cannot_be_misinterpreted_as_metres(monkeypatch):
    import sys
    from types import SimpleNamespace
    class FakeLoader:
        @classmethod
        def from_pretrained(cls, *_args, **_kwargs):
            return SimpleNamespace(config=SimpleNamespace(depth_estimation_type="relative"))
    monkeypatch.setitem(sys.modules, "transformers", SimpleNamespace(
        AutoImageProcessor=FakeLoader, AutoModelForDepthEstimation=FakeLoader))
    with pytest.raises(ValueError, match="non-metric"):
        MetricDepthAnything("relative-model", device="cpu")


def test_detector_custom_class_zero_cannot_become_a_person(monkeypatch, tmp_path):
    import sys
    from types import SimpleNamespace
    weights = tmp_path / "fake.pt"
    weights.write_bytes(b"not loaded: fake backend")
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(
        YOLO=lambda _: SimpleNamespace(names={0: "chair"})))
    with pytest.raises(ValueError, match="class 0"):
        YOLOPersonDetector(str(weights), device="cpu")


def test_depth_revision_reaches_processor_and_model_loaders(monkeypatch):
    import sys
    from types import SimpleNamespace
    calls = []
    class FakeModel:
        config = SimpleNamespace(depth_estimation_type="metric")
        def to(self, _device): return self
        def eval(self): return self
    class FakeLoader:
        @classmethod
        def from_pretrained(cls, model_id, **kwargs):
            calls.append((model_id, kwargs))
            return FakeModel()
    monkeypatch.setitem(sys.modules, "transformers", SimpleNamespace(
        AutoImageProcessor=FakeLoader, AutoModelForDepthEstimation=FakeLoader))
    model = MetricDepthAnything("metric/model", device="cpu", revision="fixed-commit")
    assert model.revision == "fixed-commit"
    assert len(calls) == 2
    assert all(kwargs["revision"] == "fixed-commit" and kwargs["local_files_only"] for _, kwargs in calls)
