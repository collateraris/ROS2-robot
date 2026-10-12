"""Pure command gate, open-loop mixing and bounded ASCII/CRC serial protocol.

9600 8N1. Request: M <uint32 seq> <signed left PWM> <signed right PWM>*<CRC4>\n
Explicit global stop: E <uint32 seq>*<CRC4>\n
Reply: A <same seq> <applied left PWM> <right PWM> <flags>*<CRC4>\n
CRC16-CCITT-FALSE: ASCII bytes before '*', poly=0x1021, init=0xFFFF, xorout=0.
USB targets are in [-140,140]; ACK PWM may reach ±255 only with BT_ACTIVE.
ACK values are logical USB PWM normally, electrical signed PWM during Bluetooth;
Bluetooth forward signs must not be interpreted as ROS forward or odometry.
Flags: 1=USB locked, 2=USB watchdog, 4=bad frame, 8=Bluetooth active,
16=sticky Bluetooth handoff, 32=explicit emergency-stop event.
M0 never interrupts Bluetooth. An inactive M0 clears the USB lock and reports
sticky handoff before clearing it. E stops both and latches Bluetooth until S/D.
No encoder feedback is present: acknowledged PWM is not measured wheel speed.
"""
from dataclasses import dataclass
import math
import re

MAX_PWM, MAX_LINE = 140, 96
BOOT_LOCK, WATCHDOG_EVENT, BAD_FRAME_EVENT = 1, 2, 4
BT_ACTIVE, HANDOFF_EVENT, ESTOP_EVENT = 8, 16, 32
KNOWN_FLAGS = BOOT_LOCK | WATCHDOG_EVENT | BAD_FRAME_EVENT | BT_ACTIVE | HANDOFF_EVENT | ESTOP_EVENT
ACK_DEADLINE, ACK_FRESH, SEND_INTERVAL, RECOVERY_HOLD = .18, .30, .08, .50


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def frame(payload):
    body = payload.encode("ascii")
    return body + f"*{crc16(body):04X}\n".encode("ascii")


def encode_command(seq, left, right):
    if type(seq) is not int or not 0 <= seq < 2**32:
        raise ValueError("Sequence must be uint32")
    if not all(type(v) is int and -MAX_PWM <= v <= MAX_PWM for v in (left, right)):
        raise ValueError("PWM targets must be integers in [-140,140]")
    return frame(f"M {seq} {left} {right}")


def encode_emergency_stop(seq):
    if type(seq) is not int or not 0 <= seq < 2**32:
        raise ValueError("Sequence must be uint32")
    return frame(f"E {seq}")


@dataclass(frozen=True)
class Ack:
    sequence: int
    left: int
    right: int
    flags: int


def decode_ack(line):
    if not 1 <= len(line) <= MAX_LINE:
        raise ValueError("ACK line length out of bounds")
    match = re.fullmatch(rb"(A ([0-9]{1,10}) (-?[0-9]{1,3}) (-?[0-9]{1,3}) ([0-9]{1,3}))"
                         rb"\*([0-9A-Fa-f]{4})\r?\n", line)
    if match is None or crc16(match[1]) != int(match[6], 16):
        raise ValueError("Malformed ACK or CRC mismatch")
    seq, left, right, flags = (int(match[i]) for i in range(2, 6))
    limit = 255 if flags & BT_ACTIVE else MAX_PWM
    if seq >= 2**32 or flags & ~KNOWN_FLAGS or max(abs(left), abs(right)) > limit:
        raise ValueError("ACK sequence/PWM/flags out of bounds")
    return Ack(seq, left, right, flags)


def sequence_newer(current, previous):
    return previous is None or 0 < (current - previous) % 2**32 < 2**31


def mix_velocity(v, w, track=.18, max_wheel_speed=.40, pwm_limit=140):
    if (not all(math.isfinite(x) for x in (v, w, track, max_wheel_speed))
            or track <= 0 or max_wheel_speed <= 0
            or type(pwm_limit) is not int or not 1 <= pwm_limit <= MAX_PWM):
        raise ValueError("Invalid velocity/motor configuration")
    left, right = v - w * track / 2, v + w * track / 2
    scale = max(1.0, abs(left) / max_wheel_speed, abs(right) / max_wheel_speed)
    if not all(math.isfinite(x) for x in (left, right, scale)):
        raise ValueError("Velocity arithmetic overflow")
    return (round(left / scale / max_wheel_speed * pwm_limit),
            round(right / scale / max_wheel_speed * pwm_limit))


class VelocityGate:
    """Only new, finite, stamped commands can move after a stop or reconnect."""
    def __init__(self, timeout=.25, stamp_max_age=.25):
        if not 0 < timeout <= .25 or not 0 < stamp_max_age <= .25:
            raise ValueError("Command/stamp timeouts must be in (0,0.25] seconds")
        self.timeout, self.stamp_max_age = timeout, stamp_max_age
        self.ready, self.velocity, self.received = False, (0.0, 0.0), None
        self.barrier_ns = self.last_stamp_ns = 0

    def clear(self, ros_now_ns=None):
        self.velocity, self.received = (0.0, 0.0), None
        if ros_now_ns is not None:
            self.barrier_ns = max(self.barrier_ns, self.last_stamp_ns, ros_now_ns)

    def set_ready(self, ready, ros_now_ns):
        if not ready or ready != self.ready:
            self.clear(ros_now_ns)
        self.ready = ready

    def accept(self, v, w, stamp_ns, ros_now_ns, monotonic_now):
        if not all(math.isfinite(x) for x in (v, w, monotonic_now)):
            self.clear(ros_now_ns)
            return False
        self.target(monotonic_now, ros_now_ns)  # Expire even if a callback beats the timer.
        if v == 0 and w == 0:
            self.clear(ros_now_ns)  # Zero is accepted even without a valid timestamp.
            return True
        age = (ros_now_ns - stamp_ns) / 1e9
        if (not self.ready or stamp_ns <= max(self.barrier_ns, self.last_stamp_ns)
                or not 0 <= age <= self.stamp_max_age):
            self.clear(ros_now_ns)
            return False
        self.velocity, self.received = (v, w), monotonic_now
        self.last_stamp_ns = stamp_ns
        return True

    def target(self, monotonic_now, ros_now_ns):
        if (self.received is not None
                and (not self.ready or not 0 <= monotonic_now - self.received < self.timeout)):
            self.clear(ros_now_ns)
        return self.velocity
