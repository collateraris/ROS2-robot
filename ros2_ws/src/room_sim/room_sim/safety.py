"""ROS-free gating of simulated motor commands. World truth is deliberately absent."""
from dataclasses import dataclass
from math import isfinite, inf


def fresh(now, received, timeout):
    return isfinite(now) and isfinite(received) and 0.0 <= now-received <= timeout


@dataclass
class Source:
    received: float = -inf
    stamp: float = -inf

    def accept(self, stamp, received, ros_now, timeout):
        if not all(isfinite(v) for v in (stamp, received, ros_now)):
            self.received = -inf
            return False
        if stamp <= 0 or not -0.03 <= ros_now-stamp <= timeout:
            self.received = -inf
            return False
        if stamp <= self.stamp:
            return False  # repeating source data never refreshes its watchdog
        self.received, self.stamp = received, stamp
        return True

    def healthy(self, now, ros_now, timeout):
        return fresh(now, self.received, timeout) and -0.03 <= ros_now-self.stamp <= timeout


class SimulationDriveGate:
    def __init__(self, *, simulation_autonomy=False, command_timeout=.25,
                 sensor_timeout=.35, health_timeout=.20, deadman_timeout=.25,
                 max_linear=.10, max_angular=.40, first_command_wait=10.):
        for value in (command_timeout, sensor_timeout, health_timeout,
                      deadman_timeout, max_linear, max_angular, first_command_wait):
            if not isfinite(value) or value <= 0:
                raise ValueError('all simulation safety limits must be finite and positive')
        self.simulation_autonomy = bool(simulation_autonomy)
        self.command_timeout, self.sensor_timeout = command_timeout, sensor_timeout
        self.health_timeout, self.deadman_timeout = health_timeout, deadman_timeout
        self.max_linear, self.max_angular = max_linear, max_angular
        self.first_command_wait = first_command_wait
        self.scan, self.odom = Source(), Source()
        self.health_at = self.deadman_at = self.command_at = self.armed_at = -inf
        self.health_ok = self.deadman = self.armed = self.estop = False
        self.command = None
        self.status = 'simulation disarmed; explicit ARM required'

    def disarm(self, reason):
        self.armed = False
        self.command = None
        self.command_at = -inf
        self.status = reason

    def on_health(self, value, now):
        self.health_ok, self.health_at = bool(value), now
        if not value:
            self.disarm('RF2O odometry unhealthy; explicit rearm required')

    def on_deadman(self, value, now):
        self.deadman, self.deadman_at = bool(value), now
        if not value and not self.simulation_autonomy:
            self.disarm('deadman released')

    def on_estop(self, value):
        if value:
            self.estop = True
            self.disarm('simulation emergency stop latched')

    def reset_estop(self):
        if self.armed:
            return False, 'stop before resetting latch'
        self.estop = False
        self.status = 'latch reset; remains disarmed'
        return True, self.status

    def reason(self, now, ros_now):
        if self.estop:
            return 'simulation emergency stop latched'
        if not self.simulation_autonomy and (
            not self.deadman or not fresh(now, self.deadman_at, self.deadman_timeout)):
            return 'deadman absent or expired'
        if not self.scan.healthy(now, ros_now, self.sensor_timeout):
            return 'laser scan absent, paused or stale'
        if not self.odom.healthy(now, ros_now, self.sensor_timeout):
            return 'validated RF2O odometry absent, paused or stale'
        if not self.health_ok or not fresh(now, self.health_at, self.health_timeout):
            return 'RF2O health absent or stale'
        return ''

    def arm(self, now, ros_now):
        if self.armed:
            return False, 'already armed'
        reason = self.reason(now, ros_now)
        if reason:
            return False, reason
        self.armed = True
        self.armed_at = now
        self.command = None  # commands from the previous arm session cannot restart motion
        self.command_at = -inf
        self.status = 'simulation armed; waiting for a new safe velocity command'
        return True, self.status

    def on_command(self, linear, angular, now, supported=True):
        if not supported or not all(isfinite(v) for v in (linear, angular)):
            self.disarm('invalid or nonplanar Twist')
            return
        if not self.armed:
            return
        self.command = (max(-self.max_linear, min(self.max_linear, linear)),
                        max(-self.max_angular, min(self.max_angular, angular)))
        self.command_at = now

    def output(self, now, ros_now):
        if not self.armed:
            return 0., 0.
        reason = self.reason(now, ros_now)
        if reason:
            self.disarm(reason)
            return 0., 0.
        if self.command is None:
            if not fresh(now, self.armed_at, self.first_command_wait):
                self.disarm('no first command after ARM')
            return 0., 0.
        if not fresh(now, self.command_at, self.command_timeout):
            self.disarm('safe velocity command expired; explicit rearm required')
            return 0., 0.
        self.status = 'simulation armed' + ('; autonomous test deadman bypass' if self.simulation_autonomy else '')
        return self.command
