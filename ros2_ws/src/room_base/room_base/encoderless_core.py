"""ROS-free open-loop calibration and freshness checks. No invented odometry.

The returned speed is a NOMINAL calibrated setpoint; only RF2O measures motion.
Small requests below the measured continuous running speed become zero, rather
than a minimum-PWM kick that would silently make the robot go faster.
"""
from dataclasses import dataclass
import math

from .core import body_velocity, fresh, wheel_targets


@dataclass(frozen=True)
class SpeedCurve:
    min_mm_s: float = 20.
    max_mm_s: float = 150.
    min_pwm: int = 60
    max_pwm: int = 120

    def __post_init__(self):
        if (not all(math.isfinite(x) for x in (self.min_mm_s, self.max_mm_s)) or
                not 0 < self.min_mm_s < self.max_mm_s or
                type(self.min_pwm) is not int or type(self.max_pwm) is not int or
                not 0 < self.min_pwm < self.max_pwm <= 255):
            raise ValueError('invalid continuous speed/PWM calibration')

    def convert(self, speed_mm_s: float) -> tuple[int, float]:
        if not math.isfinite(speed_mm_s) or speed_mm_s < 0:
            raise ValueError('curve requires finite nonnegative speed')
        if speed_mm_s < self.min_mm_s:
            return 0, 0.
        speed = min(speed_mm_s, self.max_mm_s)
        fraction = (speed - self.min_mm_s) / (self.max_mm_s - self.min_mm_s)
        pwm = round(self.min_pwm + fraction * (self.max_pwm - self.min_pwm))
        # The speed associated with the quantised PWM, not a measured velocity.
        nominal = self.min_mm_s + (pwm - self.min_pwm) * (
            self.max_mm_s - self.min_mm_s) / (self.max_pwm - self.min_pwm)
        return pwm, nominal


@dataclass(frozen=True)
class OpenLoopMapping:
    left_forward: SpeedCurve
    left_reverse: SpeedCurve
    right_forward: SpeedCurve
    right_reverse: SpeedCurve
    track_m: float = .160
    max_wheel_mm_s: int = 150

    def __post_init__(self):
        if not math.isfinite(self.track_m) or self.track_m <= 0:
            raise ValueError('invalid wheel separation')
        if type(self.max_wheel_mm_s) is not int or self.max_wheel_mm_s <= 0:
            raise ValueError('invalid maximum nominal wheel speed')
        if self.max_wheel_mm_s > min(c.max_mm_s for c in self.curves):
            raise ValueError('speed limit exceeds a calibrated direction')

    @property
    def curves(self):
        return (self.left_forward, self.left_reverse,
                self.right_forward, self.right_reverse)

    @property
    def max_pwm(self):
        return max(c.max_pwm for c in self.curves)

    def convert(self, linear_m_s: float, angular_rad_s: float):
        left, right = wheel_targets(linear_m_s, angular_rad_s,
                                    self.track_m, self.max_wheel_mm_s)
        def one(value, positive, negative):
            pwm, nominal = (positive if value >= 0 else negative).convert(abs(value))
            return (pwm, nominal) if value >= 0 else (-pwm, -nominal)
        lp, ln = one(left, self.left_forward, self.left_reverse)
        rp, rn = one(right, self.right_forward, self.right_reverse)
        v, w = body_velocity(ln, rn, self.track_m)
        return lp, rp, v, w


@dataclass
class SafetyState:
    allow_actuation: bool = False
    calibration_confirmed: bool = False
    estop: bool = False
    connected: bool = False
    boot_known: bool = False
    deadman: bool = False
    odom_healthy: bool = False
    deadman_at: float = -math.inf
    scan_at: float = -math.inf
    scan_age: float = math.inf
    odom_at: float = -math.inf
    odom_age: float = math.inf
    health_at: float = -math.inf
    feedback_at: float = -math.inf

    def reason(self, now: float, limits: dict) -> str:
        if not self.allow_actuation: return 'allow_actuation is false'
        if not self.calibration_confirmed: return 'calibration_confirmed is false'
        if self.estop: return 'emergency stop latched'
        if not self.connected or not self.boot_known: return 'encoderless firmware handshake unavailable'
        if not self.deadman or not fresh(now-self.deadman_at, limits['deadman_timeout_s']):
            return 'deadman missing or stale'
        if not fresh(now-self.scan_at, limits['scan_timeout_s']) or not fresh(
                self.scan_age, limits['scan_timeout_s']): return 'scan missing or stale'
        if not self.odom_healthy or not fresh(now-self.health_at, limits['health_timeout_s']):
            return 'RF2O odometry health false or stale'
        if not fresh(now-self.odom_at, limits['odom_timeout_s']) or not fresh(
                self.odom_age, limits['odom_timeout_s']): return 'RF2O odometry missing or stale'
        if not fresh(now-self.feedback_at, limits['feedback_timeout_s']):
            return 'MCU heartbeat stale'
        return ''
