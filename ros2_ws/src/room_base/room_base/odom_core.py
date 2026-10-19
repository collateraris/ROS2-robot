"""ROS-free gate for RF2O odometry. Checks are plausibility checks, not confidence."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ScanSample:
    stamp: float
    received: float
    usable: bool


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float
    frame: str = 'odom'
    child: str = 'base_link'


def usable_scan(ranges, range_min, range_max, angle_increment, minimum=60, sectors=16, minimum_sectors=8):
    """Require finite returns distributed around the scan; no observability claim."""
    if (len(ranges) < minimum or not all(math.isfinite(x) for x in
            (range_min, range_max, angle_increment)) or not 0 <= range_min < range_max
            or abs(angle_increment) * (len(ranges)-1) < math.pi):
        return False
    occupied = set()
    count = 0
    for i, value in enumerate(ranges):
        if math.isfinite(value) and range_min <= value <= range_max and value > 0:
            count += 1
            occupied.add(min(sectors-1, i*sectors//len(ranges)))
    return count >= minimum and len(occupied) >= minimum_sectors


class OdomGate:
    def __init__(self, scan_timeout=.35, odom_timeout=.35, max_gap=.40,
                 max_linear=.40, max_angular=1.20, position_slack=.015,
                 angle_slack=.04, good_frames=3, stamp_tolerance=.002,
                 future_tolerance=.03, odom_frame='odom', base_frame='base_link'):
        values = (scan_timeout, odom_timeout, max_gap, max_linear, max_angular,
                  position_slack, angle_slack, stamp_tolerance, future_tolerance)
        if any(not math.isfinite(x) or x < 0 for x in values) or min(values[:5]) <= 0 or good_frames < 1:
            raise ValueError('invalid odometry guard limits')
        self.scan_timeout, self.odom_timeout, self.max_gap = scan_timeout, odom_timeout, max_gap
        self.max_linear, self.max_angular = max_linear, max_angular
        self.position_slack, self.angle_slack = position_slack, angle_slack
        self.good_frames, self.stamp_tolerance = good_frames, stamp_tolerance
        self.future_tolerance = future_tolerance
        self.odom_frame, self.base_frame = odom_frame, base_frame
        self.scans = deque(maxlen=64)
        self.previous = None
        self.last_odom_received = None
        self.good_count = 0
        self.latched_fault = ''
        self.reason = 'waiting for laser odometry'

    def reject(self, reason, latch=False):
        self.good_count = 0
        self.reason = reason
        if latch:
            self.latched_fault = reason
        return False

    def add_scan(self, stamp, received, usable):
        if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(received):
            return self.reject('invalid scan timestamp')
        if self.scans and stamp <= self.scans[-1].stamp:
            return self.reject('scan timestamp stopped or moved backwards')
        self.scans.append(ScanSample(stamp, received, bool(usable)))
        if not usable:
            return self.reject('insufficient valid laser returns')
        return True

    def _age_ok(self, age, timeout):
        return math.isfinite(age) and -self.future_tolerance <= age <= timeout

    def accept(self, pose, received, ros_now, monotonic_now=None):
        if self.latched_fault:
            return self.reject(self.latched_fault)
        if not all(math.isfinite(x) for x in (pose.stamp, pose.x, pose.y, pose.yaw, received, ros_now)) or pose.stamp <= 0:
            return self.reject('nonfinite pose/timestamp', latch=True)
        if pose.frame != self.odom_frame or pose.child != self.base_frame:
            return self.reject('unexpected odometry frames', latch=True)
        if not self._age_ok(ros_now-pose.stamp, self.odom_timeout):
            return self.reject('stale or future odometry acquisition time')
        matching = [s for s in self.scans if abs(s.stamp-pose.stamp) <= self.stamp_tolerance]
        if not matching:
            return self.reject('odometry does not match a received laser scan')
        scan = min(matching, key=lambda s: abs(s.stamp-pose.stamp))
        processed = received if monotonic_now is None else monotonic_now
        if not scan.usable or not self._age_ok(processed-scan.received, self.scan_timeout):
            return self.reject('unusable or stale source scan')
        old = self.previous
        velocity = (0., 0., 0.)
        if old is not None:
            dt = pose.stamp-old.stamp
            if dt <= 0:
                return self.reject('replayed odometry timestamp')
            if dt > self.max_gap:
                return self.reject('odometry gap: restart RF2O and navigation', latch=True)
            dx, dy = pose.x-old.x, pose.y-old.y
            da = math.atan2(math.sin(pose.yaw-old.yaw), math.cos(pose.yaw-old.yaw))
            if math.hypot(dx,dy) > self.max_linear*dt+self.position_slack or abs(da) > self.max_angular*dt+self.angle_slack:
                return self.reject('implausible odometry jump: restart RF2O and navigation', latch=True)
            # Body-frame finite differences use the source scan interval, not a configured timer frequency.
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            velocity = ((c*dx+s*dy)/dt, (-s*dx+c*dy)/dt, da/dt)
        self.previous, self.last_odom_received = pose, received
        self.good_count += 1
        self.reason = 'ok' if self.good_count >= self.good_frames else 'collecting initial good scans'
        return velocity

    def healthy(self, received_now, ros_now):
        if self.latched_fault or self.good_count < self.good_frames or self.previous is None or not self.scans:
            return False
        scan = self.scans[-1]
        checks = (scan.usable,
                  self._age_ok(received_now-scan.received, self.scan_timeout),
                  self._age_ok(ros_now-scan.stamp, self.scan_timeout),
                  self._age_ok(received_now-self.last_odom_received, self.odom_timeout),
                  self._age_ok(ros_now-self.previous.stamp, self.odom_timeout))
        if not all(checks):
            return self.reject('scan or laser odometry freshness lost')
        return True
