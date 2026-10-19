"""PC integration faults and real-frame/actual-control provenance, without ROS."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("cv2")
torch = pytest.importorskip("torch")

from room_world_model.configuration import calibration_fingerprint, load_config
from room_world_model.engine import ControlEngine
from room_world_model.model import ACTION_SEMANTICS
from room_world_model.protocol import FrameRef, MetricGeometry, MotionCommand


class FakeState:
    def detach(self):
        return self


class FilterSpy:
    def __init__(self, metadata=None):
        self.config = SimpleNamespace(max_dt_s=5.0)
        self.device = torch.device("cpu")
        self.checkpoint_metadata = metadata or {"synthetic": True}
        self.resets, self.assimilations = 0, 0
        self.replayed = []

    def initial_state(self, tensor):
        assert tensor.shape == (1, 1, 128, 128)
        self.resets += 1
        return FakeState()

    def advance_interval(self, state, action, dt, segments, mask):
        self.replayed.append((float(dt), segments.cpu().numpy().copy(), action.cpu().numpy().copy()))
        return state

    def assimilate(self, state, tensor):
        self.assimilations += 1
        return state


class FakeObservation:
    def __init__(self):
        self.person_visible = True
        self.person_count = 1
        self.person_distance_valid = True
        self.person_distance_m = 2.0
        self.person_bearing_rad = 0.0
        self.horizontal_fov_rad = 1.2
        self.clearance = 2.0
        self.covered = True
        self.path_checks = []

    def to_dict(self):
        return {"person_visible": self.person_visible, "person_count": self.person_count,
                "person_distance_m": self.person_distance_m}

    def path_clearance(self, v, w, radius, horizon, margin):
        self.path_checks.append((v, w, radius, horizon))
        return SimpleNamespace(clearance_m=self.clearance, metric_valid=True,
                               covers_swept_footprint=self.covered, reason="test_geometry")


class FakePerception:
    def __init__(self):
        self.observation = FakeObservation()
        self.calls = 0

    def observe(self, rgb, timestamp_ns, image_valid_mask=None):
        self.calls += 1
        return self.observation


class FakePlanner:
    def __init__(self):
        self.calls = []

    def plan(self, state, **kwargs):
        self.calls.append(kwargs)
        options = kwargs["first_action_options"]
        accepted = [action for action in options if action[0] > 0 and kwargs["action_allowed"](action)]
        action = accepted[0] if accepted else np.zeros(2, np.float32)
        return SimpleNamespace(action=action, reason="test_planned_pulse", diagnostics={"test": True})


@pytest.fixture
def config():
    return load_config(Path(__file__).parents[1] / "config/robot.yaml")


@pytest.fixture
def clock(monkeypatch):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr("room_world_model.engine.time.monotonic", lambda: clock.now)
    return clock


def packet(config, seq=1, exposure=10.0, *, epoch="camera_a", history=None, history_start=0.0, settled=True):
    return {"frame": FrameRef("pi_boot", epoch, seq).to_dict(), "exposure_estimate_s": exposure,
            "timing_valid": True, "exposure_earliest_ns": int((exposure-.02)*1e9),
            "exposure_latest_ns": int((exposure+.02)*1e9),
            "calibration_fingerprint": calibration_fingerprint(config), "history": history or [],
            "history_start_s": history_start, "settled": settled, "exposure_uncertainty_s": 0.02,
            "read_mono_s": exposure + 0.35, "capture_latency_bound_s": 0.85}


def real_metadata(config):
    return {"synthetic": False, "provenance_complete": True, "calibration_confirmed": True,
            "limited_batch_smoke": False, "uncalibrated_pseudo_labels": False,
            "labels_from_accepted_calibration": True,
            "label_fingerprint_verified": True,
            "action_semantics": ACTION_SEMANTICS, "control_representation": "piecewise_mcu_confirmed",
            "dataset_schema_version": 2, "mcu_confirmation_validated": True,
            "calibration_fingerprint": calibration_fingerprint(config), "depth_scale_m": 3.0,
            "held_out_validation": True}


def confirmed_pulse(start):
    return {"type":"motor_interval", "schema_version":2, "boot_id":11, "nonce":101, "seq":1,"pulse_id":1,
            "confirmation_source":"mcu_start_stop", "actuation_confirmed":True,"complete":True,
            "pwm_off_confirmed":True,"left":31,"right":33,
            "mcu_start_tick_ms":500,"mcu_stop_tick_ms":580,"mcu_elapsed_ms":80,"mcu_duration_s":.08,
            "requested_duration_ms":80,
            "start_s":start,"end_s":start+.08,"start_lower_s":start-.02,"start_upper_s":start+.02,
            "end_lower_s":start+.06,"end_upper_s":start+.10,"timing_uncertainty_s":.02,
            "duration_uncertainty_s":.002,"stop_reason":"completed","normal_completion":True,
            "deadline_exceeded":False,"duration_mismatch":False}


def test_manual_geometry_and_wire_frame_correspond(config, clock):
    perception = FakePerception()
    engine = ControlEngine(config, perception, mode="teleop")
    engine.set_task({"goal_id": "manual_1", "kind": "manual", "direction": "forward"})
    value = engine.process(packet(config), np.zeros((16, 24, 3), np.uint8), "lease_1")
    command, geometry = MotionCommand.from_dict(value["command"]), MetricGeometry.from_dict(value["geometry"])
    assert command.frame == geometry.frame == FrameRef("pi_boot", "camera_a", 1)
    assert command.lease_id == "lease_1"
    assert command.linear_mps > 0
    assert command.linear_mps == pytest.approx(geometry.path_linear_mps)
    assert command.angular_rps == pytest.approx(geometry.path_angular_rps)
    assert perception.observation.path_checks
    perception.observation.covered = False
    blocked = engine.process(packet(config, seq=2, exposure=12), np.zeros((16, 24, 3), np.uint8), "lease_1")
    assert blocked["command"]["linear_mps"] == 0


def test_shadow_or_unsettled_frames_never_propose_motion(config, clock):
    for mode, settled in (("shadow", True), ("teleop", False)):
        engine = ControlEngine(config, FakePerception(), mode=mode)
        engine.set_task({"goal_id": "goal_1", "kind": "text", "text": "Подъехать к человеку и обходить препятствия"})
        value = engine.process(packet(config, settled=settled), np.zeros((16, 24, 3), np.uint8), "lease_1")
        assert value["command"]["linear_mps"] == value["command"]["angular_rps"] == 0


def test_filter_replays_pulse_then_holds_and_resets_on_unknown_history(config, clock):
    model = FilterSpy()
    engine = ControlEngine(config, FakePerception(), model=model, mode="shadow")
    rgb = np.zeros((16, 24, 3), np.uint8)
    engine.process(packet(config), rgb, "")
    pulse = confirmed_pulse(10.5)
    engine.process(packet(config, seq=2, exposure=12, history=[pulse]), rgb, "")
    assert model.resets == 1 and model.assimilations == 1
    assert engine.filter_steps == 2
    dt, segments, action = model.replayed[0]
    assert dt == 2.0
    np.testing.assert_allclose(segments[0, :, 2], [0.50, 0.08, 1.42], atol=1e-5)
    assert segments[0, 1, 0] > action[0, 0] > 0
    # A recorded motor fault truncates coverage; empty recent history cannot
    # be interpreted as a stopped interval spanning that fault.
    engine.process(packet(config, seq=3, exposure=14, history_start=13.0), rgb, "")
    assert model.resets == 2 and engine.filter_steps == 1
    engine.process(packet(config, seq=4, exposure=21), rgb, "")
    assert model.resets == 3
    engine.process(packet(config, seq=5, exposure=22, epoch="camera_restart"), rgb, "")
    assert model.resets == 4


def test_incomplete_ack_resets_instead_of_imagining_zero_controls(config, clock):
    model = FilterSpy()
    engine = ControlEngine(config, FakePerception(), model=model, mode="shadow")
    rgb = np.zeros((16, 24, 3), np.uint8)
    engine.process(packet(config), rgb, "")
    bad = {**confirmed_pulse(10.5), "complete": False}
    engine.process(packet(config, seq=2, exposure=12, history=[bad]), rgb, "")
    assert model.resets == 2 and not model.replayed


def test_unknown_camera_timing_cannot_warm_up_or_propose_motion(config, clock):
    model = FilterSpy()
    engine = ControlEngine(config, FakePerception(), model=model, mode="teleop")
    engine.set_task({"goal_id":"manual", "kind":"manual", "direction":"forward"})
    bad = packet(config)
    bad.update(timing_valid=False, exposure_earliest_ns=None, exposure_latest_ns=None)
    result = engine.process(bad,np.zeros((16,24,3),np.uint8),"lease")
    assert engine.filter_steps == 0 and engine.state is None
    assert result["command"]["linear_mps"] == 0
    assert model.resets == 0


@pytest.mark.parametrize("bad",["unsettled","wide_exposure"])
def test_unsettled_or_uncertain_frames_reset_recurrent_context(config,clock,bad):
    model=FilterSpy()
    engine=ControlEngine(config,FakePerception(),model=model,mode="shadow")
    rgb=np.zeros((16,24,3),np.uint8)
    engine.process(packet(config),rgb,"")
    changed=packet(config,seq=2,exposure=12.)
    if bad=="unsettled":changed["settled"]=False
    else:changed.update(exposure_earliest_ns=int(11e9),exposure_latest_ns=int(13e9))
    engine.process(changed,rgb,"")
    assert engine.state is None and engine.filter_steps==0 and not model.replayed


def test_legacy_ack_arrival_timing_never_enters_filter(config, clock):
    model = FilterSpy()
    engine = ControlEngine(config, FakePerception(), model=model, mode="shadow")
    rgb = np.zeros((16,24,3),np.uint8)
    engine.process(packet(config),rgb,"")
    legacy = {"start_s":10.5,"end_s":10.58,"complete":True,"timing_uncertainty_s":.02,"left":31,"right":33}
    engine.process(packet(config,seq=2,exposure=12.,history=[legacy]),rgb,"")
    assert model.resets == 2 and not model.replayed


def test_wrong_pi_or_model_calibration_and_synthetic_model_are_blocked(config, clock):
    perception = FakePerception()
    engine = ControlEngine(config, perception, mode="baseline")
    wrong = packet(config)
    wrong["calibration_fingerprint"] = "0" * 64
    with pytest.raises(ValueError, match="Pi/PC"):
        engine.process(wrong, np.zeros((16, 24, 3), np.uint8), "lease")
    assert perception.calls == 0
    with pytest.raises(ValueError, match="Synthetic"):
        ControlEngine(config, perception, model=FilterSpy(), planner=FakePlanner(), mode="wm")
    metadata = real_metadata(config)
    metadata["calibration_fingerprint"] = "0" * 64
    with pytest.raises(ValueError, match="calibration differ"):
        ControlEngine(config, perception, model=FilterSpy(metadata), planner=FakePlanner(), mode="wm")


@pytest.mark.parametrize("failure", ["lost", "multiple", "unknown_distance", "close", "jump"])
def test_person_goal_stops_on_loss_ambiguity_close_distance_or_target_jump(config, clock, failure):
    perception = FakePerception()
    engine = ControlEngine(config, perception, mode="baseline")
    engine.set_task({"goal_id": "person_1", "kind": "text", "text": "подъедь к человеку"})
    rgb = np.zeros((16, 24, 3), np.uint8)
    engine.process(packet(config), rgb, "lease")
    observed = perception.observation
    if failure == "lost":
        observed.person_visible = False
    elif failure == "multiple":
        observed.person_count = 2
    elif failure == "unknown_distance":
        observed.person_distance_valid = False
    elif failure == "close":
        observed.person_distance_m = 0.7
    else:
        observed.person_bearing_rad = 0.8
    result = engine.process(packet(config, seq=2, exposure=12), rgb, "lease")
    assert result["abort"]
    assert result["command"]["linear_mps"] == result["command"]["angular_rps"] == 0
    engine.set_task({"goal_id": "stop_2", "kind": "text", "text": "стоп"})
    assert engine.task.kind == "stop" and engine.last_person is None


def test_world_model_warmup_then_exact_discrete_options_reach_planner(config, clock):
    model, planner = FilterSpy(real_metadata(config)), FakePlanner()
    engine = ControlEngine(config, FakePerception(), model=model, planner=planner, mode="wm")
    engine.set_task({"goal_id": "person_1", "kind": "text", "text": "подъезд к человеку"})
    assert engine.task.kind == "stop"  # unsupported phrase never starts a goal
    engine.set_task({"goal_id": "person_2", "kind": "text", "text": "подъедь к человеку"})
    rgb = np.zeros((16, 24, 3), np.uint8)
    for i, exposure in enumerate((10., 12., 14.), start=1):
        result = engine.process(packet(config, seq=i, exposure=exposure), rgb, "lease")
        if i < 3:
            assert result["command"]["linear_mps"] == 0
    assert len(planner.calls) == 1
    assert planner.calls[0]["for_live"] is True
    options=planner.calls[0]["first_action_options"]
    assert np.array_equal(options[0],[0,0])
    for option in options[1:]:
        v,w=engine._quantize(option)
        np.testing.assert_allclose(option,[v/config["robot"]["linear_command_scale_mps"],
                                          w/config["robot"]["angular_command_scale_rps"]],atol=1e-6)
    assert result["command"]["linear_mps"] > 0


def test_warmup_matches_checkpoint_burn_in_after_history_reset(config, clock):
    metadata = real_metadata(config)
    metadata["burn_in_frames"] = 4
    model, planner = FilterSpy(metadata), FakePlanner()
    engine = ControlEngine(config, FakePerception(), model=model, planner=planner, mode="wm")
    engine.set_task({"goal_id": "person_1", "kind": "text", "text": "подъедь к человеку"})
    rgb = np.zeros((16, 24, 3), np.uint8)
    for i in range(1, 6):
        result = engine.process(packet(config, seq=i, exposure=10. + 2 * (i - 1)), rgb, "lease")
        assert result["diagnostics"]["required_filter_steps"] == 5
        if i < 5:
            assert result["command"]["linear_mps"] == 0 and not planner.calls
    assert len(planner.calls) == 1 and result["command"]["linear_mps"] > 0
    # A gap invalidates recurrent context and must start the same warmup again.
    result = engine.process(packet(config, seq=6, exposure=26.), rgb, "lease")
    assert len(planner.calls) == 1 and result["command"]["linear_mps"] == 0
    assert engine.filter_steps == 1
