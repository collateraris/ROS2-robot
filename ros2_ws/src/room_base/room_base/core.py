"""ROS-free wire protocol and *measured* encoder kinematics. No PWM odometry."""
from dataclasses import dataclass
import math


def crc16(data: bytes) -> int:
    crc = 0xffff
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (crc << 1) & 0xffff
    return crc


def frame(body: str) -> bytes:
    data = body.encode('ascii')
    if not data or len(data) > 140 or b'*' in data or b'\n' in data or b'\r' in data:
        raise ValueError('invalid frame body')
    return data + f'*{crc16(data):04X}\n'.encode('ascii')


def parse_frame(raw: bytes) -> list[str]:
    if len(raw) > 160 or not raw.endswith(b'\n'):
        raise ValueError('oversize or incomplete frame')
    try:
        body, checksum = raw[:-1].split(b'*')
        if len(checksum) != 4 or crc16(body) != int(checksum, 16):
            raise ValueError('CRC mismatch')
        text = body.decode('ascii')
    except (UnicodeError, ValueError) as exc:
        raise ValueError('invalid frame') from exc
    if not text or text != text.strip() or '  ' in text:
        raise ValueError('invalid whitespace')
    return text.split(' ')


def uint32(text: str) -> int:
    if not text.isascii() or not text.isdigit() or len(text) > 10:
        raise ValueError('not uint32')
    result = int(text)
    if result > 0xffffffff:
        raise ValueError('uint32 overflow')
    return result


def signed_delta(new: int, old: int) -> int:
    return (new - old + 0x80000000) % 0x100000000 - 0x80000000


def wheel_targets(linear: float, angular: float, track: float, max_mm_s: int) -> tuple[int, int]:
    if not all(math.isfinite(x) for x in (linear, angular, track)) or track <= 0 or max_mm_s <= 0:
        raise ValueError('invalid velocity or geometry')
    left = 1000.0 * (linear - .5 * track * angular)
    right = 1000.0 * (linear + .5 * track * angular)
    factor = min(1.0, max_mm_s / max(abs(left), abs(right), 1.0))
    return round(left * factor), round(right * factor)


def body_velocity(left_mm_s: int, right_mm_s: int, track: float) -> tuple[float, float]:
    return (left_mm_s + right_mm_s) / 2000., (right_mm_s - left_mm_s) / (1000. * track)


@dataclass
class EncoderOdometry:
    track: float
    ticks_per_meter: float
    x: float = 0.
    y: float = 0.
    yaw: float = 0.
    previous: tuple[int, int, int] | None = None

    def update(self, mcu_ms: int, left: int, right: int):
        current = (mcu_ms, left, right)
        old = self.previous
        if old is None:
            self.previous = current
            return None
        dt_ms = (mcu_ms - old[0]) & 0xffffffff
        if dt_ms == 0:
            return None
        if dt_ms > 1000:
            raise ValueError('encoder telemetry gap requires new baseline')
        dl = signed_delta(left, old[1]) / self.ticks_per_meter
        dr = signed_delta(right, old[2]) / self.ticks_per_meter
        # Reject physically impossible deltas; never integrate reset counters.
        dt = dt_ms / 1000.
        if max(abs(dl), abs(dr)) > .40 * dt + 2. / self.ticks_per_meter:
            raise ValueError('encoder jump or wheel speed above physical limit')
        self.previous = current
        ds, dtheta = .5 * (dl + dr), (dr - dl) / self.track
        # Exact constant-curvature integration, stable near zero curvature.
        half = .5 * dtheta
        distance = ds * (math.sin(half) / half if abs(half) > 1e-8 else 1.)
        self.x += distance * math.cos(self.yaw + half)
        self.y += distance * math.sin(self.yaw + half)
        self.yaw = math.atan2(math.sin(self.yaw + dtheta), math.cos(self.yaw + dtheta))
        return ds / dt, dtheta / dt


def fresh(age: float, timeout: float) -> bool:
    return math.isfinite(age) and 0. <= age <= timeout
