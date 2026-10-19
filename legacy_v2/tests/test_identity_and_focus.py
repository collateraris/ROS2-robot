"""Cross-host model provenance and hold-to-run focus regression checks."""

from copy import deepcopy
import json

import pytest

from room_world_model.configuration import calibration_fingerprint, perception_identity
from room_world_model.operator import window_has_focus


def example_config(tmp_path):
    (tmp_path / "camera.json").write_text(json.dumps({"fx": 550, "accepted": False}), encoding="utf-8")
    return {
        "_root": str(tmp_path),
        "camera": {"calibration_file": "camera.json", "device": "/dev/video0", "intrinsics_file": "",
                   "width": 640, "height": 480},
        "robot": {"serial_port": "/dev/ttyACM0", "footprint_radius_m": .18},
        "perception": {"depth_model_id": "depth-anything/metric-indoor", "depth_scale_m": 3.0,
                       "detector_weights": "/home/pi/models/yolo11n.pt"},
        "runtime": {"mode": "shadow"},
    }


def test_same_models_in_different_host_paths_share_fingerprint(tmp_path):
    pi = example_config(tmp_path)
    pc = deepcopy(pi)
    pc["perception"]["detector_weights"] = "C:\\Users\\iulian\\robot\\models\\yolo11n.pt"
    pc["camera"]["device"] = "camera_on_other_host"
    pc["robot"]["serial_port"] = "COM7"
    assert calibration_fingerprint(pi) == calibration_fingerprint(pc)
    assert perception_identity(pi)["detector_model_id"] == "yolo11n.pt"


@pytest.mark.parametrize("key,value", [
    ("depth_model_id", "depth-anything/other-metric-model"),
    ("depth_model_revision", "0123456789abcdef"),
    ("detector_weights", "models/yolo11s.pt"),
    ("detector_model_id", "custom/person-v2"),
])
def test_changing_model_identity_invalidates_calibration_fingerprint(tmp_path, key, value):
    first = example_config(tmp_path)
    changed = deepcopy(first)
    changed["perception"][key] = value
    assert calibration_fingerprint(first) != calibration_fingerprint(changed)


def test_rectifier_contents_matter_not_host_filename(tmp_path):
    first = example_config(tmp_path)
    a, b = tmp_path / "pi_intrinsics.json", tmp_path / "pc_intrinsics.json"
    a.write_text('{"same":"calibration"}', encoding="utf-8")
    b.write_bytes(a.read_bytes())
    first["camera"]["intrinsics_file"] = str(a)
    second = deepcopy(first)
    second["camera"]["intrinsics_file"] = str(b)
    assert calibration_fingerprint(first) == calibration_fingerprint(second)
    b.write_text('{"changed":"calibration"}', encoding="utf-8")
    assert calibration_fingerprint(first) != calibration_fingerprint(second)


def test_runtime_switch_does_not_relabel_physical_geometry(tmp_path):
    config = example_config(tmp_path)
    original = calibration_fingerprint(config)
    config["runtime"]["mode"] = "baseline"
    (tmp_path / "camera.json").write_text(json.dumps({"fx": 550, "accepted": True}), encoding="utf-8")
    assert calibration_fingerprint(config) == original


def test_invalid_revision_is_not_silently_ignored(tmp_path):
    config = example_config(tmp_path)
    config["perception"]["depth_model_revision"] = ""
    with pytest.raises(ValueError, match="revision"):
        calibration_fingerprint(config)


class FocusWindow:
    def __init__(self):
        self.focused = None

    def focus_displayof(self):
        return self.focused


class FocusWidget:
    def __init__(self, top):
        self.top = top

    def winfo_toplevel(self):
        return self.top


def test_deadman_focus_requires_this_top_level():
    window = FocusWindow()
    assert not window_has_focus(window)
    window.focused = FocusWidget(window)
    assert window_has_focus(window)
    window.focused = FocusWidget(FocusWindow())
    assert not window_has_focus(window)


def test_focus_query_error_releases_permission():
    class BrokenWindow:
        def focus_displayof(self):
            raise RuntimeError("window destroyed")
    assert not window_has_focus(BrokenWindow())
