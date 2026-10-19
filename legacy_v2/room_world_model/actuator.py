"""Nonblocking MCU-confirmed pulse transport (serial revision 3, action schema 2).

C/A/D are ACK_ACCEPT / ACK_START / ACK_STOP. A persistent MCU boot counter and
an independent host nonce bind every pulse to exactly one boot and session.
PWM/ticks confirm electrical setpoints, never wheel speed or physical rest.
No legacy protocol fallback, pulse retransmission, or automatic rearming exists.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import re
import secrets
import time
from typing import Callable

from .protocol import ACTUATION_SCHEMA_VERSION

MOTOR_PROTOCOL_VERSION = 3
STOP_REASONS = {0: "completed", 1: "host_stop", 2: "bluetooth_stop",
                3: "protocol_fault", 4: "watchdog", 5: "ack_backpressure"}


class MotorProtocolError(ValueError):
    pass


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


def packet(body: str) -> bytes:
    try:
        encoded = body.encode("ascii")
    except (AttributeError, UnicodeError) as exc:
        raise MotorProtocolError("Expected ASCII motor body") from exc
    if not encoded or len(encoded) > 84 or any(c < 32 or c > 126 or c == 42 for c in encoded):
        raise MotorProtocolError("Invalid motor body")
    if any(not field for field in body.split(" ")):
        raise MotorProtocolError("Invalid motor fields")
    return encoded + f"*{crc16(encoded):04X}\n".encode()


def parse_packet(raw: bytes) -> list[str]:
    if not isinstance(raw, bytes) or not raw or len(raw) > 96:
        raise MotorProtocolError("Invalid motor line")
    try:
        line = raw.removesuffix(b"\n").removesuffix(b"\r")
        body, checksum = line.rsplit(b"*", 1)
        if (not body or any(c < 32 or c > 126 or c == 42 for c in body)
                or re.fullmatch(rb"[0-9A-Fa-f]{4}", checksum) is None
                or crc16(body) != int(checksum, 16)):
            raise MotorProtocolError("Motor CRC/framing mismatch")
        words = body.decode("ascii").split(" ")
        if any(not x for x in words):
            raise MotorProtocolError("Invalid motor fields")
        return words
    except (ValueError, UnicodeError) as exc:
        raise MotorProtocolError(str(exc)) from exc


def uint(text: str, upper: int = 0xFFFFFFFF) -> int:
    if not isinstance(text, str) or not text.isascii() or not text.isdigit() or len(text) > 10:
        raise MotorProtocolError("Expected unsigned integer")
    result = int(text)
    if result > upper:
        raise MotorProtocolError("Motor integer out of range")
    return result


def tick_elapsed_ms(start: int, stop: int, *, max_elapsed_ms: int = 1000) -> int:
    """One unambiguous forward uint32 wrap, bounded to a short pulse/sync span."""
    if any(type(v) is not int or not 0 <= v <= 0xFFFFFFFF for v in (start, stop)):
        raise MotorProtocolError("MCU tick is not uint32")
    if type(max_elapsed_ms) is not int or not 0 <= max_elapsed_ms < 0x80000000:
        raise MotorProtocolError("Invalid tick interval bound")
    elapsed = (stop - start) & 0xFFFFFFFF
    if elapsed > max_elapsed_ms:
        raise MotorProtocolError("MCU ticks are reversed, stale, or span an ambiguous wrap")
    return elapsed


def mix_pwm(v: float, w: float, track: float, wheel_scale: float, limit: int) -> tuple[int, int]:
    if not all(math.isfinite(x) for x in (v, w, track, wheel_scale)) or track <= 0 or wheel_scale <= 0:
        raise MotorProtocolError("Invalid motor calibration")
    if type(limit) is not int or not 1 <= limit <= 140:
        raise MotorProtocolError("PWM limit must be 1..140")
    left, right = v - w * track / 2, v + w * track / 2
    if left < -1e-9 or right < -1e-9:
        raise MotorProtocolError("Reverse wheel / pivot is forbidden in this camera-only profile")
    scale = max(1.0, left / wheel_scale, right / wheel_scale)
    return round(left / scale / wheel_scale * limit), round(right / scale / wheel_scale * limit)


def unmix_pwm(left: int, right: int, track: float, wheel_scale: float, limit: int) -> tuple[float, float]:
    if (any(type(x) is not int or not 0 <= x <= 140 for x in (left, right))
            or type(limit) is not int or not 1 <= limit <= 140
            or not all(math.isfinite(x) and x > 0 for x in (track, wheel_scale))):
        raise MotorProtocolError("Invalid applied PWM/calibration")
    l, r = left / limit * wheel_scale, right / limit * wheel_scale
    return (l + r) / 2, (r - l) / track


@dataclass
class PendingPulse:
    seq: int
    pulse_id: int
    left: int
    right: int
    duration_ms: int
    sent_s: float
    accepted_s: float | None = None
    accept_tick_ms: int | None = None
    acknowledged_s: float | None = None
    start_tick_ms: int | None = None
    stop_requested_s: float | None = None


def confirmed_interval(p: PendingPulse, *, boot_id: int, nonce: int, stop_tick_ms: int,
                       stop_received_s: float, stop_reason: str,
                       clock_rate_bound_ppm: float = 20000.0) -> dict:
    """Reconstruct a matched MCU pulse while retaining honest Pi-time bounds.

    The millisecond tick duration is exact in the MCU clock. Its conversion to
    SI seconds uses the explicitly recorded oscillator-rate *assumption* plus
    one tick of quantization. The default 2% is not a hardware calibration.
    Timing is sandwiched by local serial send/receive times; PC clocks are unused.
    No stop tick is fabricated when an ACK or a boot identity is missing.
    """
    if (p.accepted_s is None or p.accept_tick_ms is None or p.acknowledged_s is None
            or p.start_tick_ms is None or stop_reason not in STOP_REASONS.values()):
        raise MotorProtocolError("A confirmed interval requires ACCEPT, START and STOP")
    if not 0 <= clock_rate_bound_ppm <= 100000 or not math.isfinite(clock_rate_bound_ppm):
        raise MotorProtocolError("Invalid clock-rate bound")
    elapsed_ms = tick_elapsed_ms(p.start_tick_ms, stop_tick_ms, max_elapsed_ms=1000)
    tick_elapsed_ms(p.accept_tick_ms, p.start_tick_ms, max_elapsed_ms=100)
    if not p.sent_s <= p.accepted_s <= p.acknowledged_s <= stop_received_s:
        raise MotorProtocolError("Non-monotonic Pi motor timestamps")
    elapsed = elapsed_ms / 1000.0
    duration_error = 0.001 + elapsed * clock_rate_bound_ppm / 1e6
    duration_lower, duration_upper = max(0., elapsed - duration_error), elapsed + duration_error
    start_lower = p.sent_s
    start_upper = min(p.acknowledged_s, stop_received_s - duration_lower)
    if start_upper < start_lower:
        raise MotorProtocolError("MCU duration conflicts with serial timestamps")
    start = (start_lower + start_upper) / 2
    end = start + elapsed
    end_lower = start_lower + duration_lower
    end_upper = min(stop_received_s, start_upper + duration_upper)
    uncertainty = max(start - start_lower, start_upper - start,
                      end - end_lower, end_upper - end)
    interval = {
        "type": "motor_interval", "schema_version": ACTUATION_SCHEMA_VERSION,
        "confirmation_source": "mcu_start_stop", "actuation_confirmed": True,
        "complete": True, "boot_id": boot_id, "nonce": nonce, "seq": p.seq,
        "pulse_id": p.pulse_id, "time_s": stop_received_s,
        "sent_s": p.sent_s, "accept_received_s": p.accepted_s,
        "start_received_s": p.acknowledged_s, "stop_received_s": stop_received_s,
        "accept_tick_ms": p.accept_tick_ms, "mcu_start_tick_ms": p.start_tick_ms,
        "mcu_stop_tick_ms": stop_tick_ms, "mcu_elapsed_ms": elapsed_ms,
        "mcu_duration_s": elapsed, "duration_uncertainty_s": duration_error,
        "clock_rate_bound_ppm": clock_rate_bound_ppm, "clock_rate_calibrated": False,
        "start_s": start, "end_s": end,
        "start_lower_s": start_lower, "start_upper_s": start_upper,
        "end_lower_s": end_lower, "end_upper_s": end_upper,
        "timing_uncertainty_s": uncertainty,
        "left": p.left, "right": p.right, "requested_duration_ms": p.duration_ms,
        "stop_reason": stop_reason, "normal_completion": stop_reason == "completed",
        "deadline_exceeded": elapsed_ms > p.duration_ms + 3,
        "duration_mismatch": stop_reason == "completed" and not p.duration_ms <= elapsed_ms <= p.duration_ms + 3,
        "pwm_off_confirmed": True,
    }
    validate_motor_interval(interval)
    return interval


def validate_motor_interval(event: dict) -> None:
    """Reject legacy, invented, malformed or internally inconsistent evidence.

    A caller still needs complete event-log ownership and matching SENT/ACK keys;
    checking this dictionary alone cannot prove that the MCU ever sent it.
    """
    if (not isinstance(event, dict) or event.get("type") != "motor_interval"
            or event.get("schema_version") != ACTUATION_SCHEMA_VERSION
            or event.get("confirmation_source") != "mcu_start_stop"
            or event.get("actuation_confirmed") is not True or event.get("complete") is not True
            or event.get("pwm_off_confirmed") is not True):
        raise MotorProtocolError("Missing v2 MCU START/STOP confirmation")
    for field in ("boot_id", "nonce", "seq", "pulse_id"):
        value = event.get(field)
        if type(value) is not int or not 0 < value <= 0xFFFFFFFF:
            raise MotorProtocolError(f"Invalid motor {field}")
    for field in ("left", "right"):
        value = event.get(field)
        if type(value) is not int or not 0 <= value <= 140:
            raise MotorProtocolError("Invalid confirmed PWM")
    if not (event["left"] or event["right"]):
        raise MotorProtocolError("Pulse confirmation cannot contain only zero PWM")
    numbers = ("start_s", "end_s", "start_lower_s", "start_upper_s", "end_lower_s",
               "end_upper_s", "timing_uncertainty_s", "mcu_duration_s", "duration_uncertainty_s")
    for field in numbers:
        value = event.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise MotorProtocolError(f"Invalid motor time {field}")
    start, end, dt = event["start_s"], event["end_s"], event["mcu_duration_s"]
    if (not event["start_lower_s"] <= start <= event["start_upper_s"]
            or not event["end_lower_s"] <= end <= event["end_upper_s"]
            or not math.isclose(end-start, dt, abs_tol=1e-8, rel_tol=1e-8)):
        raise MotorProtocolError("Inconsistent confirmed action placement")
    elapsed_ms = tick_elapsed_ms(event.get("mcu_start_tick_ms"), event.get("mcu_stop_tick_ms"))
    if (event.get("mcu_elapsed_ms") != elapsed_ms or not math.isclose(dt, elapsed_ms/1000., abs_tol=1e-9)
            or event.get("stop_reason") not in STOP_REASONS.values()
            or event.get("normal_completion") is not (event.get("stop_reason") == "completed")):
        raise MotorProtocolError("Inconsistent MCU duration/stop reason")
    requested = event.get("requested_duration_ms")
    if type(requested) is not int or not 1 <= requested <= 100:
        raise MotorProtocolError("Invalid requested pulse limit")
    uncertainty = event["timing_uncertainty_s"]
    if (uncertainty + 1e-9 < max(start-event["start_lower_s"], event["start_upper_s"]-start,
                               end-event["end_lower_s"], event["end_upper_s"]-end)
            or event["duration_uncertainty_s"] < .001
            or event.get("deadline_exceeded") is not (elapsed_ms > requested+3)
            or event.get("duration_mismatch") is not (event["normal_completion"]
                                                       and not requested <= elapsed_ms <= requested+3)):
        raise MotorProtocolError("Understated uncertainty or inconsistent pulse limit")


class PulseSerial:
    """One nonzero write per pulse ID; zero/fault recovery is always explicit.

    poll() must continue after a stop to receive its ACK. A transport fault locks
    motion immediately; a later matching STOP still records what actually ran.
    A reset/timeout produces motor_unknown, never an invented normal duration.
    """
    def __init__(self, port, *, on_event: Callable[[dict], None] | None = None,
                 ack_timeout_s: float = 0.5, clock_rate_bound_ppm: float = 20000.0):
        if not math.isfinite(ack_timeout_s) or not .05 <= ack_timeout_s <= 2.0:
            raise MotorProtocolError("Invalid acknowledgement timeout")
        if not math.isfinite(clock_rate_bound_ppm) or not 0 <= clock_rate_bound_ppm <= 100000:
            raise MotorProtocolError("Invalid clock-rate bound")
        self.port = port
        self.on_event = on_event or (lambda event: None)
        self.ack_timeout_s = ack_timeout_s
        self.clock_rate_bound_ppm = clock_rate_bound_ppm
        self.nonce = secrets.randbelow(0xFFFFFFFE) + 1
        self.boot_id: int | None = None
        self.seq = 0
        self.ready = False
        self.capabilities_seen = False
        self.fault = "firmware_handshake_required"
        self.pending: PendingPulse | None = None
        self._rx = bytearray()
        self._last_probe = -math.inf
        self._last_rx = -math.inf
        self._last_poll = -math.inf
        self._last_pulse_id = None
        self._zero_sent = False
        self._zero_sent_s = None
        self._faulted = False
        self.max_pulse_ms = 0
        self.history_start_s: float | None = None
        self._ping_seq = 0
        self._ping: tuple[int, float] | None = None

    def _event(self, kind: str, now: float, **fields) -> None:
        self.on_event({"type": kind, "schema_version": ACTUATION_SCHEMA_VERSION,
                       "time_s": now, "boot_id": self.boot_id, "nonce": self.nonce, **fields})

    def _write(self, body: str) -> None:
        data = packet(body)
        if self.port.write(data) != len(data):
            raise OSError("Partial nonblocking Arduino write")

    def _unknown(self, reason: str, now: float) -> None:
        p = self.pending
        if p is not None:
            self._event("motor_unknown", now, seq=p.seq, pulse_id=p.pulse_id,
                        sent_s=p.sent_s, left=p.left, right=p.right,
                        requested_duration_ms=p.duration_ms, reason=reason,
                        start_confirmed=p.start_tick_ms is not None,
                        mcu_start_tick_ms=p.start_tick_ms, complete=False,
                        actuation_confirmed=False)
            self.pending = None

    def emergency_stop(self, reason: str, now_s: float | None = None) -> None:
        now = time.monotonic() if now_s is None else now_s
        was_faulted = self._faulted
        self.ready = False
        self._faulted = True
        self.fault = reason
        if self.pending is not None and self.pending.stop_requested_s is None:
            self.pending.stop_requested_s = now
        try:
            if hasattr(self.port, "reset_output_buffer"):
                self.port.reset_output_buffer()
        except (OSError, ValueError):
            pass
        try:
            # A partial line is terminated first. S always cuts PWM regardless of
            # the host's boot/session, but cannot unlock or change the owner.
            self.port.write(b"\n")
            self._write(f"S {self.boot_id or 0} {self.nonce}")
        except (OSError, ValueError):
            pass
        if not was_faulted:
            self._event("motor_fault", now, reason=reason)

    def reset_handshake(self, now_s: float | None = None) -> None:
        """Explicit local recovery with a fresh nonce, while governor disarmed."""
        if self.pending is not None:
            raise MotorProtocolError("Wait for pulse confirmation/timeout before handshake recovery")
        old_nonce = self.nonce
        while self.nonce == old_nonce:
            self.nonce = secrets.randbelow(0xFFFFFFFE) + 1
        self._faulted = False
        self.ready = self.capabilities_seen = self._zero_sent = False
        self.fault = "firmware_handshake_required"
        self.boot_id = None
        self.seq = 0
        self._last_probe = self._last_rx = self._last_poll = -math.inf
        self._last_pulse_id = self._zero_sent_s = None
        self.history_start_s = None
        self._ping = None
        self._rx.clear()
        if hasattr(self.port, "reset_input_buffer"):
            self.port.reset_input_buffer()

    def send_pulse(self, pulse_id: int, left: int, right: int, duration_s: float,
                   now_s: float | None = None) -> bool:
        now = time.monotonic() if now_s is None else now_s
        if type(pulse_id) is not int or not 0 < pulse_id <= 0xFFFFFFFF:
            raise MotorProtocolError("Pulse id must be a positive uint32")
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now) or now < 0:
            raise MotorProtocolError("Invalid monotonic time")
        if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)) or not math.isfinite(duration_s):
            raise MotorProtocolError("Invalid pulse duration")
        if pulse_id == self._last_pulse_id:
            return False
        if self._last_pulse_id is not None and pulse_id < self._last_pulse_id:
            raise MotorProtocolError("Pulse IDs cannot move backwards")
        if not self.ready or self._faulted or self.pending is not None or self.boot_id is None:
            raise MotorProtocolError("Arduino not idle/ready")
        duration_ms = int(math.floor(duration_s * 1000))
        if not 0 < duration_s <= .100 or not 1 <= duration_ms <= min(100, self.max_pulse_ms):
            raise MotorProtocolError("Pulse duration outside firmware limit")
        if any(type(x) is not int or not 0 <= x <= 140 for x in (left, right)) or not (left or right):
            raise MotorProtocolError("Pulse PWM must be forward, nonzero and <=140")
        if self.seq >= 0xFFFFFFFE:
            self.emergency_stop("Sequence exhausted; explicit recovery required", now)
            raise MotorProtocolError(self.fault)
        self.seq += 1
        self.pending = PendingPulse(self.seq, pulse_id, left, right, duration_ms, now)
        self._last_pulse_id = pulse_id
        self._event("motor_sent", now, seq=self.seq, pulse_id=pulse_id, left=left, right=right,
                    duration_ms=duration_ms, sent_s=now, actuation_confirmed=False)
        try:
            self._write(f"P {self.boot_id} {self.nonce} {self.seq} {left} {right} {duration_ms}")
        except (OSError, ValueError):
            self._unknown("serial_write_incomplete", now)
            self.emergency_stop("serial_write_incomplete", now)
            raise
        return True

    def _identity(self, fields: list[str], *, pending: bool = False) -> PendingPulse | None:
        if uint(fields[1]) != self.boot_id or uint(fields[2]) != self.nonce:
            raise MotorProtocolError("MCU boot/session mismatch")
        p = self.pending
        if pending and (p is None or uint(fields[3]) != p.seq):
            raise MotorProtocolError("Unmatched MCU pulse sequence")
        return p

    def _handle(self, fields: list[str], now: float) -> None:
        kind = fields[0]
        if kind == "V" and len(fields) == 8:
            version, boot, max_ms, pwm, loop_ms, reset_ms, reset_flags = (uint(x) for x in fields[1:])
            if (version != MOTOR_PROTOCOL_VERSION or boot == 0 or not 1 <= max_ms <= 100
                    or pwm != 140 or not 1 <= loop_ms <= 40 or not 100 <= reset_ms <= 250 or reset_flags > 255):
                raise MotorProtocolError("Incompatible firmware capabilities")
            if self.boot_id is not None and boot != self.boot_id:
                self._unknown("mcu_reboot", now)
                self._event("motor_reboot", now, previous_boot_id=self.boot_id, observed_boot_id=boot,
                            reset_flags=reset_flags)
                raise MotorProtocolError("MCU reboot; explicit recovery required")
            self.boot_id = boot
            self.capabilities_seen, self.max_pulse_ms = True, max_ms
            if not self._zero_sent and not self._faulted:
                self._write(f"Z {self.boot_id} {self.nonce}")
                self._zero_sent, self._zero_sent_s = True, now
        elif kind == "R" and len(fields) == 4:
            self._identity(fields)
            tick = uint(fields[3])
            if not self.capabilities_seen or not self._zero_sent or self.ready or self._zero_sent_s is None:
                raise MotorProtocolError("Unexpected zero handshake")
            if not self._faulted:
                self.ready, self.fault, self.history_start_s = True, "", now
                self._event("motor_ready", now, zero_confirmed=True, mcu_tick_ms=tick,
                            zero_lower_s=self._zero_sent_s, zero_upper_s=now,
                            history_start_s=now, exclusive_pulse_firmware=True)
        elif kind == "C" and len(fields) == 5:
            p = self._identity(fields, pending=True)
            tick = uint(fields[4])
            if p.accepted_s is not None:
                raise MotorProtocolError("Duplicate ACCEPT")
            p.accepted_s, p.accept_tick_ms = now, tick
            self._event("motor_accept", now, seq=p.seq, pulse_id=p.pulse_id,
                        mcu_tick_ms=tick, sent_s=p.sent_s)
        elif kind == "A" and len(fields) == 7:
            p = self._identity(fields, pending=True)
            tick, left, right = (uint(x) for x in fields[4:])
            if (p.accepted_s is None or p.acknowledged_s is not None
                    or (left, right) != (p.left, p.right)):
                raise MotorProtocolError("Unmatched, duplicate or mismatched START")
            tick_elapsed_ms(p.accept_tick_ms, tick, max_elapsed_ms=100)
            p.acknowledged_s, p.start_tick_ms = now, tick
            self._event("motor_start", now, seq=p.seq, pulse_id=p.pulse_id, left=left, right=right,
                        mcu_tick_ms=tick, start_tick_ms=tick, sent_s=p.sent_s,
                        start_lower_s=p.sent_s, start_upper_s=now)
        elif kind == "D" and len(fields) == 6:
            p = self._identity(fields, pending=True)
            tick, reason_id = (uint(x) for x in fields[4:])
            if reason_id not in STOP_REASONS:
                raise MotorProtocolError("Unknown MCU stop reason")
            interval = confirmed_interval(p, boot_id=self.boot_id, nonce=self.nonce,
                stop_tick_ms=tick, stop_received_s=now, stop_reason=STOP_REASONS[reason_id],
                clock_rate_bound_ppm=self.clock_rate_bound_ppm)
            self._event("motor_stop", now, seq=p.seq, pulse_id=p.pulse_id,
                        mcu_tick_ms=tick, stop_reason=STOP_REASONS[reason_id], pwm_off_confirmed=True)
            self.on_event(interval)
            self.pending = None
            if reason_id != 0 or interval["deadline_exceeded"] or interval["duration_mismatch"]:
                self.emergency_stop("MCU stopped pulse: " + interval["stop_reason"]
                    + (" (duration outside requested bound)" if interval["duration_mismatch"] else "")
                    + (" (deadline exceeded)" if interval["deadline_exceeded"] else ""), now)
        elif kind == "T" and len(fields) == 5:
            self._identity(fields)
            ping_seq, tick = uint(fields[3]), uint(fields[4])
            if self._ping is None or ping_seq != self._ping[0]:
                raise MotorProtocolError("Unmatched MCU clock echo")
            self._event("motor_clock_sync", now, probe_seq=ping_seq, mcu_tick_ms=tick,
                        pi_lower_s=self._ping[1], pi_upper_s=now,
                        round_trip_s=now-self._ping[1], clock_rate_calibrated=False)
            self._ping = None
        elif kind == "E" and len(fields) == 5:
            # STOP is queued before E when a pulse was active. A valid E from a
            # different boot is evidence of desync, not a completion for this one.
            self._identity(fields)
            uint(fields[3])
            reason = uint(fields[4])
            if reason not in STOP_REASONS:
                raise MotorProtocolError("Unknown MCU fault code")
            raise MotorProtocolError("Arduino reports a latched fault: " + STOP_REASONS[reason])
        else:
            raise MotorProtocolError("Unexpected firmware message; serial revision 3 required")
        self._last_rx = now

    def poll(self, now_s: float | None = None) -> None:
        now = time.monotonic() if now_s is None else now_s
        try:
            if (isinstance(now, bool) or not isinstance(now, (int, float))
                    or not math.isfinite(now) or now < 0 or now < self._last_poll):
                raise MotorProtocolError("Invalid/non-monotonic Pi poll time")
            self._last_poll = now
            incoming = self.port.read(min(1024, max(1, self.port.in_waiting)))
            self._rx.extend(incoming)
            if len(self._rx) > 2048:
                raise MotorProtocolError("Arduino RX overflow")
            # Process every complete frame even after a fault. START/STOP that
            # follow an earlier transport error can still identify actual PWM.
            while b"\n" in self._rx:
                line, _, rest = self._rx.partition(b"\n")
                self._rx[:] = rest
                if line:
                    try:
                        self._handle(parse_packet(bytes(line)), now)
                    except (OSError, ValueError, TypeError) as exc:
                        if not self._faulted:
                            self.emergency_stop(str(exc), now)
            p = self.pending
            if p is not None:
                base = p.sent_s if p.acknowledged_s is None else p.acknowledged_s + p.duration_ms/1000
                if p.stop_requested_s is not None:
                    base = max(base, p.stop_requested_s)
                if now > base + self.ack_timeout_s:
                    self._unknown("acknowledgement_or_completion_timeout", now)
                    raise MotorProtocolError("Arduino acknowledgement/completion timeout")
            if not self._faulted and self.pending is None and now - self._last_probe >= .5:
                self._last_probe = now
                if self.ready:
                    if self._ping is not None:
                        raise MotorProtocolError("MCU clock echo/heartbeat lost")
                    self._ping_seq = (self._ping_seq % 0xFFFFFFFE) + 1
                    self._ping = (self._ping_seq, now)
                    self._write(f"T {self.boot_id} {self.nonce} {self._ping_seq}")
                else:
                    self._write("Q")
            if self.ready and now - self._last_rx > 1.2:
                raise MotorProtocolError("Arduino heartbeat lost")
        except (OSError, ValueError, TypeError) as exc:
            if not self._faulted:
                self.emergency_stop(str(exc), now if isinstance(now, (int, float)) and math.isfinite(now) else time.monotonic())

    def close(self) -> None:
        self.emergency_stop("shutdown")
        if self.pending is not None:
            self._unknown("port_closed_before_stop_confirmation", time.monotonic())
        self.port.close()
