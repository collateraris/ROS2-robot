"""Recorded motor timing must survive settled-frame selection and export."""
import json
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from room_world_model.actuator import PulseSerial, packet as motor_packet, unmix_pwm, PendingPulse, confirmed_interval
from room_world_model.configuration import calibration_fingerprint, load_config, resolve_path
from room_world_model.data import SequenceDataset, read_episode, validate_episode
from room_world_model.frames import make_frame
from room_world_model.timebase import CaptureTiming
from room_world_model.protocol import FrameRef
from room_world_model.record import EpisodeRecorder, export_episode, intervals_to_segments


@pytest.fixture
def robot():
    return load_config(Path(__file__).parents[1] / "config/robot.yaml")["robot"].copy()


def pulse(start, seq=1, *, nonce=101, complete=True, uncertainty=0.02, left=31, right=33):
    p=PendingPulse(seq,seq,left,right,80,start-.02,accepted_s=start-.01,accept_tick_ms=499,
                   acknowledged_s=start+.02,start_tick_ms=500)
    event=confirmed_interval(p,boot_id=7,nonce=nonce,stop_tick_ms=580,
        stop_received_s=start+.11,stop_reason="completed")
    event["complete"]=complete
    if not np.isfinite(uncertainty):
        event["timing_uncertainty_s"]=uncertainty
    return event


def recording(tmp_path, robot, *, times=(10., 12., 14., 16.), change_session_at=None,
              include_moving_frame=True, missing_done=False, fingerprint="f" * 64):
    recorder = EpisodeRecorder(tmp_path / "sessions" / "layout_a", {
        "robot": robot, "calibration_fingerprint": fingerprint, "depth_scale_m": 3.0,
        "actuation_allowed": True, "calibration_confirmed": True, "synthetic": False,
        "layout_id": "layout_a", "day_id": "day1", "lighting_id": "test",
        "camera": {"max_training_exposure_uncertainty_s": .1},
    })
    bgr = np.zeros((32, 48, 3), dtype=np.uint8)
    bgr[:] = (20, 80, 180)
    encoded, jpeg = cv2.imencode(".jpg", bgr)
    assert encoded
    seq = 0
    recorder.event({"type":"motor_ready","time_s":0.,"history_start_s":0.,
        "zero_confirmed":True,"exclusive_pulse_firmware":True,"boot_id":7,"nonce":101})

    def frame(time, settled=True, session="pi_boot_a"):
        nonlocal seq
        seq += 1
        timing=CaptureTiming(round((time+.035)*1e9),None,None,None,"UNKNOWN","EMPIRICAL",
            round((time-.005)*1e9),round((time+.005)*1e9),True,"test_measured","test-evidence")
        body=make_frame(FrameRef(session,"camera_a",seq),jpeg.tobytes(),fingerprint=fingerprint,
                        history=[],history_start_s=0.,settled=settled,mode="teleop",timing=timing)
        recorder.frame(jpeg.tobytes(), body)

    try:
        for i, time in enumerate(times):
            session = "pi_boot_b" if change_session_at is not None and i >= change_session_at else "pi_boot_a"
            frame(time, session=session)
            if i < len(times) - 1:
                event = pulse(time + 0.50, seq=i + 1)
                recorder.event({"type": "motor_sent", "boot_id":7, "nonce": 101, "seq": i + 1, "time_s": time + 0.48})
                if not (missing_done and i == 0):
                    recorder.event(event)
                if include_moving_frame:
                    frame(time + 0.55, settled=False, session=session)
    finally:
        recorder.close()
    return recorder.directory


def test_settled_export_retains_skipped_frame_pulse_and_rgb_references(tmp_path, robot):
    directory = recording(tmp_path, robot)
    output = tmp_path / "datasets" / "raw.npz"
    report = export_episode(directory, output)
    assert report["frames"] == 4 and report["valid_transitions"] == 3
    info = validate_episode(output)
    assert info.split_group == "layout_a"
    data = read_episode(output)
    np.testing.assert_allclose(data["timestamps"], [10, 12, 14, 16])
    first_segments = data["action_segments"][0, data["segment_mask"][0]]
    assert first_segments.shape == (3, 3)
    np.testing.assert_allclose(first_segments[:, 2], [0.50, 0.08, 1.42], atol=1e-5)
    v, w = unmix_pwm(31, 33, robot["track_width_m"], robot["wheel_command_scale_mps"], robot["pwm_limit"])
    expected = np.array([v / robot["linear_command_scale_mps"], w / robot["angular_command_scale_rps"]])
    np.testing.assert_allclose(first_segments[1, :2], expected, atol=1e-6)
    np.testing.assert_allclose(data["actions"][0], expected * 0.08 / 2.0, atol=1e-6)
    for value in data["rgb_paths"]:
        assert (output.parent / str(value)).resolve().is_file()
    metadata = json.loads(str(data["metadata_json"].item()))
    assert metadata["calibration_confirmed"] and metadata["synthetic"] is False
    assert metadata["control_representation"] == "piecewise_mcu_confirmed"
    assert not data["aux_mask"].any()


def test_missing_mcu_completion_never_becomes_zero_action(tmp_path, robot):
    directory = recording(tmp_path, robot, missing_done=True)
    with pytest.raises(ValueError, match="matching MCU completion"):
        export_episode(directory, tmp_path / "bad.npz")
    _, valid = intervals_to_segments(10, 12, [pulse(10.5, complete=False)], robot, history_start_s=0.)
    assert not valid
    _, valid = intervals_to_segments(10, 12, [pulse(10.5, uncertainty=float("nan"))], robot, history_start_s=0.)
    assert not valid


def test_actuator_nonce_survives_recording_and_old_completion_cannot_cover_reset(tmp_path, robot, monkeypatch):
    """Exercise public CRC transport, restart, raw log and NPZ export together."""
    class VirtualPort:
        def __init__(self):
            self.inbox = bytearray()

        @property
        def in_waiting(self):
            return len(self.inbox)

        def read(self, count):
            value = bytes(self.inbox[:count])
            del self.inbox[:count]
            return value

        def write(self, value):
            return len(value)

        def reset_input_buffer(self):
            self.inbox.clear()

        def inject(self, value):
            self.inbox.extend(motor_packet(value))

    values = iter([100, 201])
    monkeypatch.setattr("room_world_model.actuator.secrets.randbelow", lambda upper: next(values))
    port, events = VirtualPort(), []
    adapter = PulseSerial(port, on_event=events.append)

    def ready_and_start(time):
        adapter.poll(time)
        port.inject("V 3 7 100 140 40 250 0")
        adapter.poll(time + .01)
        port.inject(f"R 7 {adapter.nonce} 100")
        adapter.poll(time + .02)
        assert adapter.send_pulse(1, 31, 33, .08, time + .50)
        port.inject(f"C 7 {adapter.nonce} 1 499")
        adapter.poll(time+.51)
        port.inject(f"A 7 {adapter.nonce} 1 500 31 33")
        adapter.poll(time+.52)

    ready_and_start(10.)
    port.inject(f"D 7 {adapter.nonce} 1 580 0")
    adapter.poll(10.65)
    adapter.reset_handshake(12.)
    ready_and_start(12.)
    assert [(event["nonce"], event["seq"]) for event in events if event["type"] == "motor_sent"] == [(101, 1), (202, 1)]
    assert all(event["nonce"] in (101, 202) for event in events
               if event["type"] in {"motor_sent", "motor_start", "motor_interval"})

    directory = recording(tmp_path, robot, include_moving_frame=False)
    events_file = directory / "events.jsonl"
    frames = [json.loads(line) for line in events_file.read_text().splitlines()
              if json.loads(line)["type"] == "camera"]

    def save_events():
        events_file.write_text("\n".join(json.dumps(event) for event in frames + events) + "\n")

    save_events()
    with pytest.raises(ValueError, match="matching MCU completion"):
        export_episode(directory, tmp_path / "missing_after_reset.npz")
    port.inject(f"D 7 {adapter.nonce} 1 580 0")
    adapter.poll(12.65)
    save_events()
    report = export_episode(directory, tmp_path / "both_sessions_complete.npz")
    assert report["valid_transitions"] <= 3  # reset creates a boundary; never stitches histories


@pytest.mark.parametrize("corruption", ["missing_nonce", "duplicate_completion"])
def test_ambiguous_legacy_or_duplicate_motor_keys_refuse_export(tmp_path, robot, corruption):
    directory = recording(tmp_path, robot, include_moving_frame=False)
    events_file = directory / "events.jsonl"
    events = [json.loads(line) for line in events_file.read_text().splitlines()]
    completion = next(event for event in events if event["type"] == "motor_interval")
    if corruption == "missing_nonce":
        completion.pop("nonce")
    else:
        events.append(completion.copy())
    events_file.write_text("\n".join(json.dumps(event) for event in events) + "\n")
    with pytest.raises(ValueError, match="nonce, seq"):
        export_episode(directory, tmp_path / "ambiguous.npz")


def test_long_gap_stays_in_episode_but_never_enters_training_window(tmp_path, robot):
    directory = recording(tmp_path, robot, times=(10., 12., 14., 23., 25., 27., 29.))
    output = tmp_path / "long_gap.npz"
    export_episode(directory, output)
    info = validate_episode(output)
    assert info.valid_transitions == 5
    data = read_episode(output)
    assert not data["transition_valid"][2]
    dataset = SequenceDataset([info], sequence_length=2, stride=1)
    assert dataset.windows
    assert all(not (start <= 2 < start + 2) for _, start in dataset.windows)


def test_pi_restart_invalidates_transition_even_with_same_camera_epoch(tmp_path, robot):
    directory = recording(tmp_path, robot, change_session_at=2)
    output = tmp_path / "restarted.npz"
    export_episode(directory, output)
    data = read_episode(output)
    np.testing.assert_array_equal(data["transition_valid"], [True, False, True])


def test_idle_is_one_segment_and_overlapping_pulses_are_rejected(robot):
    segments,valid=intervals_to_segments(10,12,[],robot,history_start_s=0.)
    assert valid
    np.testing.assert_array_equal(segments,[[0,0,2]])
    _,valid=intervals_to_segments(10,12,[pulse(10.5),pulse(10.55,seq=2)],robot,history_start_s=0.)
    assert not valid


def test_uncertain_exposure_or_uncovered_handshake_cannot_seed_a_transition(robot):
    event=pulse(10.4)
    _,valid=intervals_to_segments(10,12,[event],robot,history_start_s=0.,
        start_interval=(9.995,10.005),end_interval=(11.995,12.005))
    assert valid
    _,valid=intervals_to_segments(10,12,[event],robot,history_start_s=0.,
        start_interval=(9.99,10.43),end_interval=(11.995,12.005))
    assert not valid
    _,valid=intervals_to_segments(10,12,[],robot,history_start_s=10.,start_interval=(9.99,10.01))
    assert not valid
    _,valid=intervals_to_segments(10,12,[],robot)  # no proof of zero-PWM history
    assert not valid


def test_record_label_and_training_provenance_and_relocated_rgb_paths(tmp_path, robot):
    pytest.importorskip("torch")
    from room_world_model.label import _read_rgb, label_episode
    from room_world_model.model import check_live_metadata
    from room_world_model.perception import CameraCalibration, PerceptionConfig
    from room_world_model.train import build_metadata

    class LabelFrame:
        def to_aux(self, depth_scale_m, require_calibration=True):
            assert require_calibration
            return np.array([1, 0, .3, .8, .8, .8], np.float32), np.ones(6, bool)

        def to_dict(self):
            return {"test_teacher_only": True}

    class Teacher:
        calibration = CameraCalibration(width=48, height=32, fx=40, fy=40, cx=23.5, cy=15.5, accepted=True)
        config = PerceptionConfig(depth_scale_m=3.0)

        def observe(self, rgb, timestamp_ns, image_valid_mask=None):
            assert rgb.shape == (32, 48, 3)
            # JPEG was encoded from BGR (20,80,180); teacher must receive RGB.
            np.testing.assert_allclose(rgb[10, 10], [180, 80, 20], atol=5)
            return LabelFrame()

    config = load_config(Path(__file__).parents[1] / "config/robot.yaml")
    camera_file = tmp_path / "camera.json"
    camera_file.write_text(json.dumps(asdict(Teacher.calibration)))
    config["camera"].update(width=48, height=32, calibration_file=str(camera_file), already_rectified=True)
    fingerprint = calibration_fingerprint(config)
    Teacher.detector = SimpleNamespace(weights=str(resolve_path(config, config["perception"]["detector_weights"])))
    Teacher.depth_model = SimpleNamespace(model_id=config["perception"]["depth_model_id"],
                                          revision=config["perception"].get("depth_model_revision"))
    infos = []
    for i in range(2):
        directory = recording(tmp_path / str(i), robot, fingerprint=fingerprint)
        metadata_file = directory / "metadata.json"
        metadata = json.loads(metadata_file.read_text())
        metadata["session_id"] = f"independent_session_{i}"
        metadata_file.write_text(json.dumps(metadata))
        raw = tmp_path / "raw" / f"episode_{i}.npz"
        export_episode(directory, raw)
        labeled = tmp_path / "labeled" / "nested" / f"episode_{i}.npz"
        label_episode(raw, labeled, Teacher(), input_already_rectified=True, config=config)
        data = read_episode(labeled)
        assert data["aux_mask"].all()
        rgb = _read_rgb(data, 0, labeled)
        np.testing.assert_allclose(rgb[10, 10], [180, 80, 20], atol=5)
        infos.append(validate_episode(labeled))
    provenance = build_metadata(infos[:1], infos[1:])
    assert provenance["calibration_confirmed"] is True
    assert provenance["uncalibrated_pseudo_labels"] is False
    assert provenance["labels_from_accepted_calibration"] is True
    assert provenance["label_fingerprint_verified"] is True
    assert provenance["action_scale"]["linear_command_scale_mps"] == robot["linear_command_scale_mps"]
    check_live_metadata(provenance, calibration_fingerprint=fingerprint, depth_scale_m=3.0)
    provenance["labels_from_accepted_calibration"] = False
    with pytest.raises(ValueError, match="accepted metric calibration"):
        check_live_metadata(provenance, calibration_fingerprint=fingerprint, depth_scale_m=3.0)

    unverified_path = tmp_path / "unverified" / "episode_0.npz"
    label_episode(tmp_path / "raw" / "episode_0.npz", unverified_path, Teacher(), input_already_rectified=True)
    mixed_provenance = build_metadata([validate_episode(unverified_path)], infos[1:])
    assert mixed_provenance["labels_from_accepted_calibration"] is True
    assert mixed_provenance["label_fingerprint_verified"] is False
    with pytest.raises(ValueError, match="label configuration fingerprint"):
        check_live_metadata(mixed_provenance, calibration_fingerprint=fingerprint, depth_scale_m=3.0)
