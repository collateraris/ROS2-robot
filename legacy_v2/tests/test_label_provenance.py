"""The real labelling CLI must bind its actual teacher to recorded provenance."""
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from room_world_model.configuration import calibration_fingerprint, load_config, resolve_path
from room_world_model.label import label_episode, main
from room_world_model.perception import CameraCalibration, CameraPerception, PerceptionConfig, PersonDetection


@pytest.fixture
def context(tmp_path):
    config = load_config(Path(__file__).parents[1] / "config/robot.yaml")
    config = {key: value for key, value in config.items() if not key.startswith("_")}
    directory = tmp_path / "config"
    directory.mkdir()
    camera = CameraCalibration(width=48, height=32, fx=50, fy=50, cx=23.5, cy=15.5, accepted=True)
    (directory / "camera.json").write_text(json.dumps(asdict(camera)), encoding="utf-8")
    config["camera"].update(width=48, height=32, calibration_file="config/camera.json",
                            intrinsics_file="", already_rectified=True)
    config["perception"]["depth_model_revision"] = "fixed_test_revision"
    config_path = directory / "robot.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    config = load_config(config_path)
    source = tmp_path / "episode.npz"
    np.savez_compressed(source, images=np.zeros((3, 128, 128), np.uint8),
                        rgb_images=np.zeros((3, 32, 48, 3), np.uint8), timestamps=[10., 12., 14.],
                        actions=np.zeros((2, 2), np.float32), dt=np.array([2., 2.], np.float32),
                        aux=np.zeros((3, 6), np.float32), aux_mask=np.zeros((3, 6), bool),
                        transition_valid=np.ones(2, bool), metadata_json=np.array(json.dumps({
                            "calibration_fingerprint": calibration_fingerprint(config),
                            "synthetic": False, "depth_scale_m": 3.0})))
    return config, camera, source


def fake_teacher(config, camera):
    class Detector:
        weights = str(resolve_path(config, config["perception"]["detector_weights"]))

        def __call__(self, rgb):
            return [PersonDetection((15., 3., 34., 30.), .95)]

    class Depth:
        model_id = config["perception"]["depth_model_id"]
        revision = config["perception"]["depth_model_revision"]

        def __call__(self, rgb):
            return np.full(rgb.shape[:2], 2., dtype=np.float32)

    return CameraPerception(camera, Detector(), Depth(), PerceptionConfig(depth_scale_m=3.))


def metadata(path):
    with np.load(path, allow_pickle=False) as archive:
        return json.loads(str(archive["metadata_json"].item()))


def test_config_cli_then_manual_and_offline_relabel_provenance(context, tmp_path, monkeypatch):
    config, camera, source = context
    teacher = fake_teacher(config, camera)
    calls = []

    def factory(cls, cal, **values):
        assert cal == camera
        assert values["detector_weights"] == teacher.detector.weights
        assert values["depth_model_id"] == teacher.depth_model.model_id
        assert values["depth_model_revision"] == teacher.depth_model.revision
        assert values["config"] == teacher.config
        assert values["local_files_only"] == config["perception"]["local_files_only"]
        calls.append(values)
        return teacher

    monkeypatch.setattr(CameraPerception, "from_pretrained", classmethod(factory))
    labeled = tmp_path / "labeled.npz"
    main([str(source), "--output", str(labeled), "--config", config["_path"]])
    assert len(calls) == 1
    verified = metadata(labeled)
    assert verified["label_fingerprint_verified"] is True
    assert verified["calibration_fingerprint"] == calibration_fingerprint(config)
    assert verified["labels_from_accepted_calibration"] is True
    assert verified["annotation_history"][-1]["verified_calibration_fingerprint"] == calibration_fingerprint(config)

    manual = tmp_path / "manual.npz"
    label_episode(labeled, manual, None, corrections={"0": {"person_visible": False}})
    assert metadata(manual)["label_fingerprint_verified"] is True
    with pytest.raises(ValueError, match="existing depth_scale_m"):
        label_episode(labeled, tmp_path / "bad_scale.npz", None,
                      corrections={"0": {"person_visible": False}}, depth_scale_m=4.)

    offline = tmp_path / "offline.npz"
    label_episode(manual, offline, teacher, input_already_rectified=True)
    assert metadata(offline)["label_fingerprint_verified"] is False
    assert metadata(offline)["calibration_fingerprint"] == verified["calibration_fingerprint"]


def test_changed_accepted_camera_rejects_before_pretrained_loading(context, tmp_path, monkeypatch):
    config, camera, source = context
    other = replace(camera, fx=55.)
    resolve_path(config, config["camera"]["calibration_file"]).write_text(json.dumps(asdict(other)), encoding="utf-8")

    def forbidden(*args, **kwargs):
        pytest.fail("A mismatched recording must be rejected before model loading")

    monkeypatch.setattr(CameraPerception, "from_pretrained", forbidden)
    destination = tmp_path / "mismatched.npz"
    with pytest.raises(ValueError, match="fingerprints differ"):
        main([str(source), "--output", str(destination), "--config", config["_path"]])
    assert not destination.exists()


@pytest.mark.parametrize("field", ["camera", "detector", "depth"])
def test_python_api_rejects_actual_teacher_different_from_config(context, tmp_path, field):
    config, camera, source = context
    teacher = fake_teacher(config, camera)
    if field == "camera":
        teacher.calibration = replace(camera, depth_scale_factor=1.2)
    elif field == "detector":
        teacher.detector.weights = str(tmp_path / "another_detector.pt")
    else:
        teacher.depth_model.revision = "different_revision"
    destination = tmp_path / "wrong_teacher.npz"
    with pytest.raises(ValueError, match="differs? from --config"):
        label_episode(source, destination, teacher, input_already_rectified=True, config=config)
    assert not destination.exists()


def test_cli_refuses_individual_model_override_with_config(context, tmp_path):
    config, _, source = context
    with pytest.raises(SystemExit) as caught:
        main([str(source), "--output", str(tmp_path / "conflict.npz"), "--config", config["_path"],
              "--depth-model", "another_teacher"])
    assert caught.value.code == 2
