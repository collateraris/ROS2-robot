"""Virtual serial only: no port is opened and no robot is controlled."""
import math
from dataclasses import replace

import pytest

from room_world_model.actuator import (
    MotorProtocolError, PulseSerial, PendingPulse, confirmed_interval,
    crc16, mix_pwm, packet, parse_packet, tick_elapsed_ms, unmix_pwm, validate_motor_interval,
)


class FakePort:
    def __init__(self):
        self.inbox = bytearray()
        self.writes = []
        self.partial = False
        self.flushed_output = 0
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.inbox)

    def read(self, n):
        data = bytes(self.inbox[:n])
        del self.inbox[:n]
        return data

    def write(self, data):
        self.writes.append(data)
        return len(data) - 1 if self.partial else len(data)

    def reset_output_buffer(self):
        self.flushed_output += 1

    def reset_input_buffer(self):
        self.inbox.clear()

    def close(self):
        self.closed = True

    def inject(self, body):
        self.inbox.extend(packet(body))


def ready_adapter():
    port, events = FakePort(), []
    adapter = PulseSerial(port, on_event=events.append)
    adapter.poll(10.0)
    assert parse_packet(port.writes[-1]) == ["Q"]
    port.inject("V 3 7 100 140 40 250 0")
    adapter.poll(10.01)
    assert parse_packet(port.writes[-1]) == ["Z", "7", str(adapter.nonce)]
    port.inject(f"R 7 {adapter.nonce} 1000")
    adapter.poll(10.02)
    assert adapter.ready and adapter.history_start_s == 10.02
    return port, adapter, events


def begin_pulse():
    port, adapter, events = ready_adapter()
    assert adapter.send_pulse(1, 40, 45, 0.08, 10.03)
    return port, adapter, events


def start(port, adapter, tick=1050, now=10.10):
    port.inject(f"C 7 {adapter.nonce} 1 {tick}")
    port.inject(f"A 7 {adapter.nonce} 1 {tick} 40 45")
    adapter.poll(now)


def complete(port, adapter, stop=1130, reason=0, now=10.18):
    port.inject(f"D 7 {adapter.nonce} 1 {stop} {reason}")
    adapter.poll(now)


def test_crc_ccitt_standard_check_vector_and_roundtrip():
    assert crc16(b"123456789") == 0x29B1
    assert parse_packet(packet("P 7 123 1 40 45 80")) == ["P", "7", "123", "1", "40", "45", "80"]


def test_no_motion_before_versioned_zero_handshake():
    adapter = PulseSerial(FakePort())
    with pytest.raises(MotorProtocolError):
        adapter.send_pulse(1, 40, 40, .08, 10.)


def test_old_serial_firmware_cannot_arm_v2():
    port, events = FakePort(), []
    adapter = PulseSerial(port, on_event=events.append)
    port.inject("V 2 100 140")
    adapter.poll(10.)
    assert not adapter.ready and adapter._faulted
    assert not any(w.startswith(b"P ") for w in port.writes)


def test_duplicate_polling_never_emits_another_nonzero_packet():
    port, adapter, _ = begin_pulse()
    count = len(port.writes)
    for _ in range(100):
        assert not adapter.send_pulse(1, 40, 45, .08, 10.04)
    assert len(port.writes) == count
    with pytest.raises(MotorProtocolError):
        adapter.send_pulse(2, 40, 45, .08, 10.04)


def test_three_confirmations_retain_mcu_duration_pwm_and_time_uncertainty():
    port, adapter, events = begin_pulse()
    start(port, adapter)
    complete(port, adapter)
    event = next(e for e in events if e["type"] == "motor_interval")
    validate_motor_interval(event)
    assert event["mcu_duration_s"] == pytest.approx(.08)
    assert event["mcu_elapsed_ms"] == 80
    assert event["start_lower_s"] == pytest.approx(10.03)
    assert event["start_upper_s"] == pytest.approx(10.10)
    assert event["timing_uncertainty_s"] >= .035
    assert event["duration_uncertainty_s"] == pytest.approx(.0026)
    assert event["end_s"] - event["start_s"] == pytest.approx(.08)
    assert event["stop_reason"] == "completed" and event["normal_completion"]
    assert event["boot_id"] == 7 and event["left"] == 40 and event["right"] == 45
    assert [e["type"] for e in events][:6] == [
        "motor_ready", "motor_sent", "motor_accept", "motor_start", "motor_stop", "motor_interval"]
    assert adapter.pending is None and adapter.ready


def test_completion_supports_uint32_millisecond_wrap():
    port, adapter, events = begin_pulse()
    tick = 0xFFFFFFF0
    start(port, adapter, tick)
    complete(port, adapter, (tick+80)&0xFFFFFFFF)
    assert next(e for e in events if e["type"] == "motor_interval")["mcu_duration_s"] == pytest.approx(.08)


@pytest.mark.parametrize("ack", [
    "A 7 {nonce} 1 1050 41 45",  # applied PWM differs
    "A 7 {nonce} 2 1050 40 45",  # sequence differs
    "A 8 {nonce} 1 1050 40 45",  # prior/new MCU boot
    "A 7 0 1 1050 40 45",       # wrong session
    "A 7 {nonce} 1 1050 -40 45", # signed PWM
    "A 7 {nonce} 1 4294967296 40 45", # overflow
])
def test_mismatched_ack_latches_and_flushes_nonzero_queue(ack):
    port, adapter, _ = begin_pulse()
    port.inject(f"C 7 {adapter.nonce} 1 1050")
    port.inject(ack.format(nonce=adapter.nonce))
    adapter.poll(10.10)
    assert not adapter.ready and adapter._faulted
    assert port.flushed_output == 1
    assert parse_packet(port.writes[-1]) == ["S", "7", str(adapter.nonce)]
    with pytest.raises(MotorProtocolError):
        adapter.send_pulse(2, 40, 45, .08, 10.11)


@pytest.mark.parametrize("reason,name", [(1,"host_stop"), (2,"bluetooth_stop"), (3,"protocol_fault"),
                                         (4,"watchdog"), (5,"ack_backpressure")])
def test_aborts_record_confirmed_actual_elapsed_then_latch(reason, name):
    port, adapter, events = begin_pulse()
    start(port, adapter)
    complete(port, adapter, stop=1082, reason=reason)
    event = next(e for e in events if e["type"] == "motor_interval")
    assert event["mcu_duration_s"] == pytest.approx(.032)
    assert event["stop_reason"] == name and not event["normal_completion"]
    assert event["actuation_confirmed"] and event["complete"]
    assert not adapter.ready and adapter._faulted and adapter.pending is None


def test_explicit_stop_keeps_pending_until_confirmed_and_still_records_interval():
    port, adapter, events = begin_pulse()
    start(port, adapter)
    adapter.emergency_stop("operator", 10.11)
    assert adapter.pending is not None
    complete(port, adapter, stop=1080, reason=1)
    event = next(e for e in events if e["type"] == "motor_interval")
    assert event["mcu_duration_s"] == pytest.approx(.03)
    assert not adapter.ready


def test_overshoot_is_recorded_honestly_then_fails_closed():
    port, adapter, events = begin_pulse()
    start(port, adapter)
    complete(port, adapter, stop=1160, now=10.25)
    event = next(e for e in events if e["type"] == "motor_interval")
    assert event["deadline_exceeded"] and event["mcu_duration_s"] == pytest.approx(.11)
    assert not adapter.ready and adapter._faulted


@pytest.mark.parametrize("missing", ["accept", "start", "stop"])
def test_missing_confirmation_never_becomes_an_action(missing):
    port, adapter, events = begin_pulse()
    if missing != "accept":
        port.inject(f"C 7 {adapter.nonce} 1 1050")
    if missing != "start":
        port.inject(f"A 7 {adapter.nonce} 1 1050 40 45")
    adapter.poll(10.10)
    if missing != "stop":
        complete(port, adapter)
    adapter.poll(11.)
    assert not adapter.ready and adapter.pending is None
    assert not any(e["type"] == "motor_interval" for e in events)
    assert any(e["type"] == "motor_unknown" for e in events)


def test_ack_timeout_never_retransmits_a_pulse_and_never_auto_rearms():
    port, adapter, events = begin_pulse()
    adapter.poll(10.60)
    assert not adapter.ready and adapter.pending is None
    assert len([w for w in port.writes if w.startswith(b"P ")]) == 1
    assert any(e["type"] == "motor_unknown" for e in events)
    port.inject("V 3 7 100 140 40 250 0")
    adapter.poll(10.70)
    assert not adapter.ready
    assert len([w for w in port.writes if w.startswith(b"Z ")]) == 1


def test_mcu_reboot_midpulse_does_not_invent_a_stop_or_rearm():
    port, adapter, events = begin_pulse()
    start(port, adapter)
    port.inject("V 3 8 100 140 40 250 8")
    adapter.poll(10.2)
    assert not adapter.ready and adapter.pending is None
    unknown = next(e for e in events if e["type"] == "motor_unknown")
    assert unknown["reason"] == "mcu_reboot" and unknown["start_confirmed"]
    assert not any(e["type"] == "motor_interval" for e in events)
    assert len([w for w in port.writes if w.startswith(b"Z ")]) == 1


def test_explicit_recovery_uses_new_session_and_resets_pulse_ids():
    port, adapter, _ = begin_pulse()
    old_nonce = adapter.nonce
    adapter.poll(10.60)
    adapter.reset_handshake(10.70)
    assert adapter.nonce != old_nonce and adapter._last_pulse_id is None
    adapter.poll(10.71)
    port.inject("V 3 8 100 140 40 250 0")
    adapter.poll(10.72)
    port.inject(f"R 8 {adapter.nonce} 100")
    adapter.poll(10.73)
    assert adapter.send_pulse(1, 40, 45, .08, 10.74)


def test_heartbeat_echo_records_pi_interval_and_nonce_tick_without_clock_subtraction():
    port, adapter, events = ready_adapter()
    adapter.poll(10.51)
    assert parse_packet(port.writes[-1])[:3] == ["T", "7", str(adapter.nonce)]
    port.inject(f"T 7 {adapter.nonce} 1 1480")
    adapter.poll(10.59)
    sync = next(e for e in events if e["type"] == "motor_clock_sync")
    assert sync["pi_lower_s"] == 10.51 and sync["pi_upper_s"] == 10.59
    assert sync["mcu_tick_ms"] == 1480
    adapter.poll(11.11)
    adapter.poll(11.62)
    assert not adapter.ready and "heartbeat" in adapter.fault


def test_corrupt_serial_stream_stops_but_trailing_matched_acks_are_preserved():
    port, adapter, events = begin_pulse()
    port.inbox.extend(b"garbage*0000\n")
    port.inject(f"C 7 {adapter.nonce} 1 1050")
    port.inject(f"A 7 {adapter.nonce} 1 1050 40 45")
    port.inject(f"D 7 {adapter.nonce} 1 1080 3")
    adapter.poll(10.18)
    assert not adapter.ready
    assert next(e for e in events if e["type"] == "motor_interval")["mcu_duration_s"] == pytest.approx(.03)


@pytest.mark.parametrize("duration", [float("nan"), float("inf"), -1., 0., .1001, .101, True])
def test_invalid_pulse_duration_never_emits_nonzero(duration):
    port, adapter, _ = ready_adapter()
    count = len(port.writes)
    with pytest.raises(MotorProtocolError):
        adapter.send_pulse(1, 40, 40, duration, 10.03)
    assert len(port.writes) == count


def test_negative_wheels_pivot_and_nonfinite_calibration_are_rejected():
    for args in [(0., 1., .2, .1, 140), (float("nan"), 0, .2, .1, 140),
                 (.05, 0, 0, .1, 140), (.05, 0, .2, .1, 141)]:
        with pytest.raises(MotorProtocolError):
            mix_pwm(*args)


def test_mixer_saturation_preserves_ratio_without_minimum_pwm_boost():
    left, right = mix_pwm(.20, .20, .20, .10, 140)
    assert max(left, right) == 140 and min(left, right) > 0
    assert mix_pwm(.000001, 0., .2, .1, 140) == (0, 0)
    v, w = unmix_pwm(70, 70, .2, .1, 140)
    assert v == pytest.approx(.05) and w == 0.


def test_packet_rejects_control_delimiters_and_non_hex_checksum():
    for body in ["Q\nP 1 1 1 1 1", "Q*0000", "Q\r", " Q", "Q  1", "Q\t", "é"]:
        with pytest.raises(MotorProtocolError):
            packet(body)
    for raw in [b"Q*+ABC\n", b"Q* 123\n", b"Q*-001\n", b"Q*D6A3\n\n"]:
        with pytest.raises(MotorProtocolError):
            parse_packet(raw)


def test_partial_write_latches_and_emits_unknown_before_returning_error():
    port, adapter, events = ready_adapter()
    port.partial = True
    with pytest.raises(OSError):
        adapter.send_pulse(1, 40, 40, .08, 10.03)
    assert adapter.pending is None and adapter._faulted and not adapter.ready
    assert any(e["type"] == "motor_unknown" for e in events)


def test_sequence_exhaustion_latches_before_a_wrapped_packet():
    port, adapter, _ = ready_adapter()
    adapter.seq = 0xFFFFFFFE
    with pytest.raises(MotorProtocolError):
        adapter.send_pulse(1, 40, 40, .08, 10.03)
    assert adapter._faulted
    assert not any(w.startswith(b"P ") for w in port.writes)


def test_stale_backward_or_ambiguous_tick_is_rejected():
    assert tick_elapsed_ms(0xFFFFFFF0, 64) == 80
    for a, b in [(100, 99), (0, 0x80000000), (-1, 2), (0, True)]:
        with pytest.raises(MotorProtocolError):
            tick_elapsed_ms(a, b)


def test_confirmation_helper_rejects_tampered_and_legacy_intervals():
    p = PendingPulse(1, 1, 40, 45, 80, 10.03, 10.08, 1000, 10.10, 1000)
    event = confirmed_interval(p, boot_id=7, nonce=123, stop_tick_ms=1080,
                               stop_received_s=10.20, stop_reason="completed")
    for field, value in [("schema_version", 1), ("confirmation_source", "planner"),
                         ("actuation_confirmed", False), ("complete", 1),
                         ("mcu_duration_s", .079), ("mcu_elapsed_ms", 79),
                         ("mcu_stop_tick_ms", 1079), ("left", -2),
                         ("end_lower_s", 11), ("boot_id", 0)]:
        with pytest.raises(MotorProtocolError):
            validate_motor_interval({**event, field: value})
    with pytest.raises(MotorProtocolError):
        confirmed_interval(replace(p, accept_tick_ms=None), boot_id=7, nonce=123,
                           stop_tick_ms=1080, stop_received_s=10.20, stop_reason="completed")
