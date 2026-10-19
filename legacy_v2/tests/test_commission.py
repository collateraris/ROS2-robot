"""No-port dry-run checks and a mocked one-shot commissioning integration."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from room_world_model import commission
from room_world_model.actuator import PendingPulse, confirmed_interval
from room_world_model.engine import ControlEngine


CONFIG = str(Path(__file__).parents[1] / "config" / "robot.yaml")
BASE_ARGS = ["--config", CONFIG, "--left-pwm", "30", "--right-pwm", "30", "--duration-ms", "50"]


def test_dry_run_does_not_import_serial_or_open_a_port(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "serial", None)
    assert commission.main(BASE_ARGS) == 0
    assert "DRY RUN" in capsys.readouterr().out


@pytest.mark.parametrize("extra", [["--execute"], ["--duration-ms", "101"], ["--left-pwm", "141"],
                                  ["--execute", "--wheels-raised", "--clear-test-lane"]])
def test_execution_context_and_hardware_caps_are_required(extra):
    with pytest.raises(SystemExit) as error:
        commission.main(BASE_ARGS + extra)
    assert error.value.code == 2


@pytest.mark.parametrize("answer,expected", [("PULSE", True), ("pulse", False), ("", False), ("PULSE PULSE", False)])
def test_only_one_exact_local_trigger_is_accepted(monkeypatch, answer, expected):
    fake = SimpleNamespace(poll=lambda: None, ready=True, _faulted=False, fault="")
    monkeypatch.setattr("builtins.input", lambda _: answer)
    assert commission.local_trigger(fake, timeout_s=1.0) is expected


@pytest.mark.parametrize("stop_reason,return_code,duration_ms", [("completed", 0, 50), ("watchdog", 1, 20)])
def test_execute_path_emits_exactly_one_mocked_pulse_and_closes(monkeypatch, tmp_path,
                                                               stop_reason, return_code, duration_ms):
    instances = []

    class FakePort:
        def reset_input_buffer(self):
            pass

        def close(self):
            pass

    class FakeAdapter:
        def __init__(self, port, on_event, clock_rate_bound_ppm):
            assert clock_rate_bound_ppm == 20000.0
            self.event, self.ready, self._faulted = on_event, True, False
            self.fault, self.pending, self.sent, self.closed = "", None, [], False
            instances.append(self)

        def poll(self):
            if self.pending:
                p = PendingPulse(1, 1, 30, 30, 50, 10., 10.02, 1000, 10.03, 1000)
                self.event(confirmed_interval(p, boot_id=7, nonce=42,
                    stop_tick_ms=1000+duration_ms, stop_received_s=10.10, stop_reason=stop_reason))
                self._faulted = stop_reason != "completed"
                self.fault = "MCU watchdog" if self._faulted else ""
                self.pending = None

        def send_pulse(self, *args):
            self.sent.append(args)
            self.pending = object()

        def close(self):
            self.closed = True

    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=lambda *a, **kw: FakePort()))
    monkeypatch.setattr(commission, "PulseSerial", FakeAdapter)
    monkeypatch.setattr(commission.time, "sleep", lambda _: None)
    triggers = []
    monkeypatch.setattr(commission, "local_trigger", lambda _: triggers.append(True) or True)
    target = tmp_path / "measurement.json"
    assert commission.main(BASE_ARGS + ["--execute", "--wheels-raised", "--record", str(target)]) == return_code
    assert triggers == [True]
    assert instances[0].sent == [(1, 30, 30, 0.05)]
    assert instances[0].closed
    result = json.loads(target.read_text())
    assert result["matched_mcu_completion"] is True
    assert result["normal_pulse_completion"] is (stop_reason == "completed")
    assert result["confirmed_mcu_elapsed_ms"] == duration_ms
    assert result["confirmed_stop_reason"] == stop_reason
    assert result["measured_distance_to_full_stop_mm"] is None
    assert result["events"][0]["mcu_duration_s"] == pytest.approx(duration_ms/1000)


def test_example_max_speed_quantization_retains_a_bounded_straight_action():
    engine = ControlEngine.__new__(ControlEngine)
    engine.config = {"robot": {
        "linear_command_scale_mps": 0.06, "angular_command_scale_rps": 0.15,
        "track_width_m": 0.18, "wheel_command_scale_mps": 0.25, "pwm_limit": 140,
    }}
    v, w = engine._quantize([1.0, 0.0])
    assert 0.0 < v <= 0.06
    assert w == pytest.approx(0.0)
    assert v == pytest.approx(33.0 / 140.0 * 0.25)
    for action in ([1.0, 1.0], [1.0, -1.0], [1.0, 0.25], [1.0, -0.25]):
        v, w = engine._quantize(action)
        assert 0 < v <= 0.06
        assert abs(w) <= 0.15
