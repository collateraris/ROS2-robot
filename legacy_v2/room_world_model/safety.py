"""Transport-independent, fail-closed motion governor for supervised experiments.

The world model proposes; this Pi-local state machine decides.  All time values
here are Pi ``time.monotonic()`` seconds.  Never pass ROS time or PC timestamps.
Call ``tick`` from a steady-clock timer and independently enforce pulse expiry
in the microcontroller. Python/ROS scheduling is not a hardware safety system.

Default operation is STOP--LOOK--ACT: one short pulse, braking and settling,
then a frame whose EARLIEST possible exposure is after settling. This is useful
when a USB camera has hundreds of milliseconds of hidden buffering. No callback
automatically arms the machine; only an explicit ``arm`` creates a new lease.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import secrets
import time
from typing import Callable

from .protocol import FrameRef, MetricGeometry, MotionCommand, ProtocolError, decode_command


@dataclass(frozen=True)
class SafetyConfig:
    # Both flags must be deliberate outcomes of the commissioning procedure.
    calibration_confirmed: bool = False
    allow_camera_only_research: bool = False
    pulse_mode: bool = True
    max_motion_pulse_s: float = 0.08
    settle_extra_s: float = 0.20
    max_frame_age_s: float = 1.20
    max_geometry_age_s: float = 1.20
    max_capture_latency_bound_s: float = 0.90
    max_response_delay_s: float = 0.40
    max_inference_s: float = 0.30
    max_command_ttl_s: float = 0.12
    command_watchdog_s: float = 0.18
    deadman_timeout_s: float = 0.30
    drive_ready_timeout_s: float = 0.50
    control_period_s: float = 0.02
    max_control_gap_s: float = 0.08
    # Firmware v3 cuts PWM from Timer2, independently of the main loop. This
    # example Pi-side bound includes an unmeasured margin above 100 MCU ms.
    # Commission clock rate and the actual cutoff on the real board.
    # These are placeholders, not measurements of the user's car.
    motor_watchdog_s: float = 0.11
    driver_latency_s: float = 0.05
    max_linear_mps: float = 0.06
    max_angular_rps: float = 0.18
    max_curvature_inv_m: float = 3.0
    command_accel_mps2: float = 0.80
    speed_bound_mps: float = 0.12
    angular_speed_bound_rps: float = 0.40
    braking_decel_min_mps2: float = 0.25
    angular_braking_decel_min_rps2: float = 0.80
    static_margin_m: float = 0.15
    # This is a declared scene bound, not a motion detector or person guarantee.
    obstacle_speed_bound_mps: float = 0.0
    fov_margin_rad: float = 0.12
    frame_history_size: int = 128

    def __post_init__(self) -> None:
        booleans = {"calibration_confirmed", "allow_camera_only_research", "pulse_mode"}
        zeros_ok = {"obstacle_speed_bound_mps", "settle_extra_s"}
        for name, value in vars(self).items():
            if name in booleans:
                if type(value) is not bool:
                    raise ValueError(f"{name} must be boolean")
            elif name == "frame_history_size":
                if type(value) is not int or not 4 <= value <= 4096:
                    raise ValueError("frame_history_size must be an integer in [4,4096]")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            elif value < 0 or (value == 0 and name not in zeros_ok):
                raise ValueError(f"{name} must be positive")
        if self.max_linear_mps > self.speed_bound_mps:
            raise ValueError("speed_bound_mps must bound every allowed motor command")
        if self.max_angular_rps > self.angular_speed_bound_rps:
            raise ValueError("angular_speed_bound_rps must bound every allowed turn")
        if self.control_period_s > self.max_control_gap_s:
            raise ValueError("control period exceeds the maximum scheduling gap")
        if self.max_motion_pulse_s > self.motor_watchdog_s:
            raise ValueError("software pulse must not exceed the hardware pulse/watchdog bound")
        if self.max_motion_pulse_s > self.max_command_ttl_s:
            raise ValueError("software pulse must not exceed command TTL")


@dataclass(frozen=True)
class SafetyDecision:
    linear_mps: float
    angular_rps: float
    armed: bool
    reason: str
    frame_age_s: float | None = None
    required_clearance_m: float = 0.0
    speed_bound_mps: float = 0.0
    lease_id: str = ""
    # Metadata for the MCU adapter; repeated tick output MUST retain pulse_id.
    pulse_id: int = 0
    pulse_remaining_s: float = 0.0
    settle_until_mono_s: float = 0.0

    @property
    def moving(self) -> bool:
        return abs(self.linear_mps) > 1e-9 or abs(self.angular_rps) > 1e-9


@dataclass(frozen=True)
class _FrameState:
    ref: FrameRef
    receipt_mono_s: float
    capture_latency_bound_s: float

    def age_upper_s(self, now_s: float) -> float:
        return now_s - self.receipt_mono_s + self.capture_latency_bound_s

    @property
    def earliest_exposure_s(self) -> float:
        return self.receipt_mono_s - self.capture_latency_bound_s


def stopping_distance(
    speed_mps: float,
    reaction_s: float,
    decel_min_mps2: float,
    margin_m: float = 0.0,
    obstacle_speed_mps: float = 0.0,
) -> float:
    """Conservative 1D closing distance under explicit bounded-motion assumptions.

    For a static obstacle: v*t + v²/(2*a) + margin. For an obstacle approaching
    at a declared bounded speed, add its travel during reaction and braking.
    This formula cannot establish that a monocular estimate has metric scale.
    """
    numbers = (speed_mps, reaction_s, decel_min_mps2, margin_m, obstacle_speed_mps)
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in numbers):
        raise ValueError("stopping distance inputs must be finite numbers")
    if min(speed_mps, reaction_s, margin_m, obstacle_speed_mps) < 0 or decel_min_mps2 <= 0:
        raise ValueError("invalid stopping distance inputs")
    return (margin_m + speed_mps * reaction_s + speed_mps**2 / (2.0 * decel_min_mps2)
            + obstacle_speed_mps * (reaction_s + speed_mps / decel_min_mps2))


class SafetyGovernor:
    """Single-thread-owned state. ROS callbacks must share a mutually exclusive group.

    Integrators publish ONLY the result of ``tick`` to the motor adapter.
    Bad JSON should go through ``submit_json`` or call ``stop('malformed')``.
    ``set_deadman(True)`` is an operator heartbeat, never an autonomous timer.
    The drive-ready heartbeat must come from acknowledged MCU status.
    """

    def __init__(self, config: SafetyConfig | None = None, *, clock: Callable[[], float] = time.monotonic):
        self.cfg = config or SafetyConfig()
        self._clock = clock
        self.armed = False
        self.lease_id = ""
        self.reason = "disarmed"
        self._now_last: float | None = None
        self._tick_last: float | None = None
        self._ready_at: float | None = None
        self._ready = False
        self._deadman_at: float | None = None
        self._deadman = False
        self._frames: OrderedDict[FrameRef, _FrameState] = OrderedDict()
        self._geometry: OrderedDict[FrameRef, MetricGeometry] = OrderedDict()
        self._latest_frame: FrameRef | None = None
        self._latest_geometry: FrameRef | None = None
        self._epoch: tuple[str, str] | None = None
        self._retired_epochs: set[tuple[str, str]] = set()
        self._last_frame_seq = -1
        self._last_frame_receipt = -math.inf
        self._last_command_seq = -1
        self._last_command_frame_seq = -1
        self._command: MotionCommand | None = None
        self._command_received = 0.0
        self._command_expires = 0.0
        self._pulse_counter = 0
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._brake_at: float | None = None
        self._residual_linear_start = 0.0
        self._residual_angular_start = 0.0
        self._settle_until = 0.0
        self._ever_moved = False

    @property
    def settle_until_mono_s(self) -> float:
        return self._settle_until

    def _now(self, value: float | None) -> float | None:
        now = self._clock() if value is None else value
        if isinstance(now, bool) or not isinstance(now, (float, int)) or not math.isfinite(now) or now < 0:
            self._latch("invalid_monotonic_time")
            return None
        if self._now_last is not None and now < self._now_last - 1e-9:
            self._latch("monotonic_time_reversed")
            return None
        self._now_last = float(now)
        return float(now)

    def _latch(self, reason: str) -> None:
        self.armed = False
        self.lease_id = ""
        self._command = None
        self.reason = reason

    def _residual_speeds(self, now: float) -> tuple[float, float]:
        if self._last_linear > 0:
            return self.cfg.speed_bound_mps, self.cfg.angular_speed_bound_rps if self._last_angular else 0.0
        if self._brake_at is None:
            return 0.0, 0.0
        # A zero in software does not instantly stop a physical vehicle.
        elapsed = max(0.0, now - self._brake_at - self.cfg.driver_latency_s - self.cfg.motor_watchdog_s)
        return (
            max(0.0, self._residual_linear_start - self.cfg.braking_decel_min_mps2 * elapsed),
            max(0.0, self._residual_angular_start - self.cfg.angular_braking_decel_min_rps2 * elapsed),
        )

    def _zero(self, now: float, reason: str, *, latch: bool = False) -> SafetyDecision:
        previous_v, previous_w = self._residual_speeds(now)
        if self._last_linear > 0 or self._last_angular != 0:
            self._brake_at = now
            self._residual_linear_start = previous_v
            self._residual_angular_start = previous_w
            brake_time = max(previous_v / self.cfg.braking_decel_min_mps2,
                             previous_w / self.cfg.angular_braking_decel_min_rps2)
            self._settle_until = (now + self.cfg.driver_latency_s + self.cfg.motor_watchdog_s
                                  + brake_time + self.cfg.settle_extra_s)
        self._last_linear = self._last_angular = 0.0
        self._command = None
        if latch:
            self._latch(reason)
        else:
            self.reason = reason
        age = self._frame_age(self._latest_frame, now)
        distance = self._required_clearance(previous_v, age or 0.0)
        return self._decision(now, 0.0, 0.0, reason, age, distance, previous_v)

    def stop(self, reason: str = "operator_stop", now_s: float | None = None) -> SafetyDecision:
        now = self._now(now_s)
        return self._zero(now if now is not None else (self._now_last or 0.0), reason, latch=True)

    def set_drive_ready(self, ready: bool, now_s: float | None = None) -> None:
        now = self._now(now_s)
        if now is None:
            return
        if type(ready) is not bool:
            self._zero(now, "malformed_drive_ready", latch=True)
            return
        self._ready, self._ready_at = ready, now
        if not ready:
            self._zero(now, "drive_not_ready", latch=True)

    def set_deadman(self, held: bool, now_s: float | None = None) -> None:
        now = self._now(now_s)
        if now is None:
            return
        if type(held) is not bool:
            self._zero(now, "malformed_deadman", latch=True)
            return
        self._deadman, self._deadman_at = held, now
        if not held:
            self._zero(now, "deadman_released", latch=True)

    def register_frame(self, ref: FrameRef, receipt_mono_s: float,
                       capture_latency_bound_s: float | None, *, now_s: float | None = None) -> bool:
        """Record the original Pi camera-read time, never arrival time on the PC.

        ``capture_latency_bound_s`` must include worst-case exposure-to-read delay
        (USB/driver/buffering). Missing calibration is represented by None, which
        blocks motion. A new epoch invalidates the arming lease and all history.
        """
        now = self._now(now_s)
        if now is None:
            return False
        try:
            if not isinstance(ref, FrameRef):
                raise ProtocolError("not FrameRef")
            ref.validate()
            numeric = (receipt_mono_s, capture_latency_bound_s)
            if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in numeric):
                raise ProtocolError("missing or invalid capture latency bound")
            if not 0 <= receipt_mono_s <= now or not 0 <= capture_latency_bound_s <= self.cfg.max_capture_latency_bound_s:
                raise ProtocolError("capture timestamp/bound out of range")
        except (ProtocolError, TypeError, ValueError):
            self._zero(now, "capture_timing_uncalibrated", latch=True)
            return False
        epoch = (ref.session_id, ref.camera_epoch)
        if epoch != self._epoch:
            if epoch in self._retired_epochs:
                self._zero(now, "camera_epoch_replay", latch=True)
                return False
            if self._epoch is not None:
                self._retired_epochs.add(self._epoch)
                self._zero(now, "camera_restarted", latch=True)
            self._epoch = epoch
            self._frames.clear()
            self._geometry.clear()
            self._latest_frame = self._latest_geometry = None
            self._last_frame_seq = self._last_command_frame_seq = -1
        if ref.frame_seq <= self._last_frame_seq or receipt_mono_s < self._last_frame_receipt:
            self._zero(now, "camera_frame_replay_or_reorder", latch=True)
            return False
        self._frames[ref] = _FrameState(ref, float(receipt_mono_s), float(capture_latency_bound_s))
        self._latest_frame = ref
        self._last_frame_seq = ref.frame_seq
        self._last_frame_receipt = receipt_mono_s
        while len(self._frames) > self.cfg.frame_history_size:
            old, _ = self._frames.popitem(last=False)
            self._geometry.pop(old, None)
        return True

    def register_exposure_interval(self, ref: FrameRef, receipt_mono_s: float,
                                   exposure_earliest_s: float | None,
                                   exposure_latest_s: float | None,
                                   *, now_s: float | None = None) -> bool:
        """Register a calibrated Pi-monotonic optical interval from the driver.

        The whole interval must have ended before DQBUF. Missing/unknown timing
        is deliberately not replaced with a configured latency estimate. The
        lower endpoint defines frame age and the post-settle invariant.
        """
        now = self._now(now_s)
        if now is None:
            return False
        values = (receipt_mono_s, exposure_earliest_s, exposure_latest_s)
        if (any(isinstance(x, bool) or not isinstance(x, (int, float))
                or not math.isfinite(x) for x in values)
                or not 0 <= exposure_earliest_s <= exposure_latest_s <= receipt_mono_s <= now):
            self._zero(now, "capture_timing_uncalibrated", latch=True)
            return False
        return self.register_frame(ref, receipt_mono_s,
                                   receipt_mono_s - exposure_earliest_s, now_s=now)

    def confirm_motor_stop(self, end_latest_s: float, now_s: float | None = None) -> None:
        """A PWM-off ACK is not a physical wheel-stop sensor.

        Extend (never shorten) settling from the latest possible MCU PWM-off
        time, using the measured worst-case linear/angular brake bounds.
        """
        now = self._now(now_s)
        if now is None:
            return
        if (isinstance(end_latest_s, bool) or not isinstance(end_latest_s, (int, float))
                or not math.isfinite(end_latest_s) or not 0 <= end_latest_s <= now):
            self._zero(now, "invalid_mcu_stop_time", latch=True)
            return
        brake_s = max(self.cfg.speed_bound_mps / self.cfg.braking_decel_min_mps2,
                      self.cfg.angular_speed_bound_rps / self.cfg.angular_braking_decel_min_rps2)
        self._settle_until = max(self._settle_until, end_latest_s + brake_s + self.cfg.settle_extra_s)

    def update_geometry(self, geometry: MetricGeometry, receipt_mono_s: float | None = None,
                        *, now_s: float | None = None) -> bool:
        """Tie evidence to a known camera frame. Arrival NEVER renews frame age."""
        now = self._now(now_s)
        if now is None:
            return False
        try:
            geometry.validate()
            if receipt_mono_s is not None and (not math.isfinite(receipt_mono_s) or not 0 <= receipt_mono_s <= now):
                raise ProtocolError("bad geometry receipt time")
        except (AttributeError, TypeError, ValueError):
            self._zero(now, "malformed_geometry", latch=True)
            return False
        if geometry.frame not in self._frames:
            self._zero(now, "unknown_geometry_frame", latch=True)
            return False
        if geometry.frame in self._geometry:
            # Do not refresh, replace or re-date evidence for a used photograph.
            return False
        if self._latest_geometry and geometry.frame.frame_seq < self._latest_geometry.frame_seq:
            self._zero(now, "geometry_reordered", latch=True)
            return False
        self._geometry[geometry.frame] = geometry
        self._latest_geometry = geometry.frame
        if not geometry.metric_valid or not geometry.covers_swept_footprint:
            if self.armed:
                self._zero(now, "geometry_untrusted_or_unobserved", latch=True)
        elif self.armed and self._command is not None:
            # Newly observed obstacles must interrupt the old proposal. If the
            # new evidence covers a different curve, continuing the old curve
            # is not justified either. A camera update never renews its TTL.
            failure = self._geometry_failure(self._command, geometry, now)
            if failure:
                self._zero(now, failure, latch=True)
        return True

    def _frame_age(self, ref: FrameRef | None, now: float) -> float | None:
        item = self._frames.get(ref) if ref is not None else None
        return item.age_upper_s(now) if item is not None else None

    def _base_failure(self, now: float, *, need_geometry: bool = True) -> str | None:
        if not self.cfg.calibration_confirmed:
            return "calibration_required"
        if not self.cfg.allow_camera_only_research:
            return "camera_only_research_opt_in_required"
        if not self._ready or self._ready_at is None or now - self._ready_at > self.cfg.drive_ready_timeout_s:
            return "drive_ready_stale"
        if not self._deadman or self._deadman_at is None or now - self._deadman_at > self.cfg.deadman_timeout_s:
            return "deadman_stale"
        age = self._frame_age(self._latest_frame, now)
        if age is None or age > self.cfg.max_frame_age_s:
            return "camera_stale"
        if need_geometry:
            geometry = self._geometry.get(self._latest_geometry)
            age = self._frame_age(self._latest_geometry, now)
            if geometry is None or age is None or age > self.cfg.max_geometry_age_s:
                return "geometry_stale"
            if not geometry.metric_valid or not geometry.covers_swept_footprint:
                return "geometry_untrusted_or_unobserved"
        return None

    def arm(self, now_s: float | None = None, *, stationary_confirmed: bool = False) -> str | None:
        """Explicit operator action after confirming that the physical wheels stopped.

        An existing active arm is not renewed. No PC proposal can invoke this.
        Initial movement additionally requires a newly exposed post-settle frame.
        """
        now = self._now(now_s)
        if now is None:
            return None
        if self.armed:
            self.reason = "already_armed"
            return self.lease_id
        failure = self._base_failure(now)
        if type(stationary_confirmed) is not bool or not stationary_confirmed:
            failure = "confirm_wheels_stationary"
        if now < self._settle_until:
            failure = "braking_or_settling"
        if failure:
            self._zero(now, failure, latch=True)
            return None
        self.armed = True
        self.lease_id = secrets.token_hex(16)
        self.reason = "armed_waiting_for_post_settle_frame"
        self._command = None
        self._last_command_seq = self._last_command_frame_seq = -1
        self._tick_last = now
        self._settle_until = now + self.cfg.settle_extra_s
        return self.lease_id

    def submit_json(self, payload: str | bytes, now_s: float | None = None) -> SafetyDecision:
        try:
            command = decode_command(payload)
        except ProtocolError:
            return self.stop("malformed_command", now_s)
        return self.submit(command, now_s)

    def submit(self, command: MotionCommand, now_s: float | None = None) -> SafetyDecision:
        now = self._now(now_s)
        if now is None:
            return self._zero(self._now_last or 0.0, self.reason, latch=True)
        try:
            command.validate()
        except (AttributeError, TypeError, ValueError):
            return self._zero(now, "malformed_command", latch=True)
        if not self.armed:
            return self._zero(now, self.reason, latch=False)
        failure = self._base_failure(now)
        if failure:
            return self._zero(now, failure, latch=True)
        if command.lease_id != self.lease_id:
            return self._zero(now, "wrong_or_expired_lease", latch=True)
        if command.command_seq <= self._last_command_seq:
            return self._zero(now, "command_replay_or_reorder", latch=True)
        if (command.frame.session_id, command.frame.camera_epoch) != self._epoch:
            return self._zero(now, "wrong_camera_epoch", latch=True)
        frame = self._frames.get(command.frame)
        if frame is None:
            return self._zero(now, "unknown_command_frame", latch=True)
        if command.frame.frame_seq <= self._last_command_frame_seq:
            return self._zero(now, "reused_inference_frame", latch=True)
        self._last_command_seq = command.command_seq
        self._last_command_frame_seq = command.frame.frame_seq
        age = frame.age_upper_s(now)
        if age > self.cfg.max_frame_age_s or now - frame.receipt_mono_s > self.cfg.max_response_delay_s:
            return self._zero(now, "stale_inference_result", latch=True)
        if command.inference_ms / 1000.0 > self.cfg.max_inference_s:
            return self._zero(now, "inference_deadline_exceeded", latch=True)
        if command.valid_for_ms / 1000.0 > self.cfg.max_command_ttl_s + 1e-9:
            return self._zero(now, "command_lease_too_long", latch=True)
        if command.linear_mps == 0.0 and command.angular_rps == 0.0:
            return self._zero(now, "requested_stop")
        if command.linear_mps <= 0.0:
            return self._zero(now, "reverse_or_pivot_forbidden", latch=True)
        if command.linear_mps > self.cfg.max_linear_mps or abs(command.angular_rps) > self.cfg.max_angular_rps:
            return self._zero(now, "command_speed_limit", latch=True)
        if abs(command.angular_rps / command.linear_mps) > self.cfg.max_curvature_inv_m:
            return self._zero(now, "turn_too_sharp", latch=True)
        if self.cfg.pulse_mode:
            if self._command is not None:
                # A new packet can shorten a pulse, never lengthen or replace it.
                return self._zero(now, "pulse_interrupted_by_new_proposal")
            if now < self._settle_until or frame.earliest_exposure_s < self._settle_until:
                return self._zero(now, "waiting_for_post_settle_frame")
        geometry = self._geometry.get(command.frame)
        failure = self._geometry_failure(command, geometry, now)
        if failure:
            return self._zero(now, failure, latch=True)
        duration = min(command.valid_for_ms / 1000.0, self.cfg.max_command_ttl_s)
        if self.cfg.pulse_mode:
            duration = min(duration, self.cfg.max_motion_pulse_s)
        self._command = command
        self._command_received = now
        self._command_expires = now + duration
        self._pulse_counter += 1
        return self.tick(now)

    def _geometry_failure(self, command: MotionCommand, geometry: MetricGeometry | None, now: float) -> str | None:
        if geometry is None:
            return "missing_geometry_for_command"
        age = self._frame_age(geometry.frame, now)
        if age is None or age > self.cfg.max_geometry_age_s:
            return "geometry_stale"
        if not geometry.metric_valid or not geometry.covers_swept_footprint:
            return "geometry_untrusted_or_unobserved"
        # A curved path needs geometry computed for that exact curvature. Ramp
        # scales v and omega together, preserving this curve.
        requested_curve = command.angular_rps / command.linear_mps
        if geometry.path_linear_mps > 0.0:
            observed_curve = geometry.path_angular_rps / geometry.path_linear_mps
        elif geometry.path_angular_rps == 0.0 and requested_curve == 0.0:
            observed_curve = 0.0
        else:
            return "geometry_path_mismatch"
        if abs(observed_curve - requested_curve) > 1e-6:
            return "geometry_path_mismatch"
        if self.cfg.pulse_mode:
            # The post-settle exposure invariant establishes that the robot
            # remained stationary during pre-command camera buffering. Its
            # orientation can change only during this pulse and its stopping
            # tail; old-picture age still matters to moving obstacles below.
            horizon = (self.cfg.max_motion_pulse_s + self._reaction_s(0.0)
                       + self.cfg.angular_speed_bound_rps / self.cfg.angular_braking_decel_min_rps2)
        else:
            horizon = self._reaction_s(age) + self.cfg.angular_speed_bound_rps / self.cfg.angular_braking_decel_min_rps2
        angular_sweep = self.cfg.angular_speed_bound_rps * horizon if command.angular_rps else 0.0
        if angular_sweep + self.cfg.fov_margin_rad > geometry.observed_half_fov_rad:
            return "turn_leaves_observed_fov"
        v_now, _ = self._residual_speeds(now)
        v_bound = max(v_now, self.cfg.speed_bound_mps)
        if geometry.clearance_m < self._required_clearance(v_bound, age):
            return "insufficient_braking_clearance"
        return None

    def _reaction_s(self, frame_age_s: float) -> float:
        return max(0.0, frame_age_s) + self.cfg.max_control_gap_s + self.cfg.driver_latency_s + self.cfg.motor_watchdog_s

    def _required_clearance(self, speed_mps: float, age_s: float) -> float:
        return stopping_distance(speed_mps, self._reaction_s(age_s), self.cfg.braking_decel_min_mps2,
                                 self.cfg.static_margin_m, self.cfg.obstacle_speed_bound_mps)

    def _decision(self, now: float, v: float, w: float, reason: str, age: float | None,
                  distance: float, speed_bound: float) -> SafetyDecision:
        return SafetyDecision(v, w, self.armed, reason, age, distance, speed_bound,
                              self.lease_id, self._pulse_counter,
                              max(0.0, self._command_expires - now) if self._command else 0.0,
                              self._settle_until)

    def tick(self, now_s: float | None = None) -> SafetyDecision:
        """Publish this result at the configured control rate, including all zeros."""
        now = self._now(now_s)
        if now is None:
            return self._zero(self._now_last or 0.0, self.reason, latch=True)
        dt = self.cfg.control_period_s if self._tick_last is None else now - self._tick_last
        self._tick_last = now
        if self.armed and dt > self.cfg.max_control_gap_s + 1e-9:
            return self._zero(now, "control_loop_missed_deadline", latch=True)
        if not self.armed:
            return self._zero(now, self.reason)
        # In pulse mode, geometry may legitimately remain old while the MCU is
        # already at zero and we wait for a new post-settle exposure. Keep all
        # local heartbeat/camera checks, but require exact fresh geometry only
        # at arm, submit and during an active pulse. This does not authorize
        # movement or discard the residual physical braking/settle bound.
        need_geometry = not (self.cfg.pulse_mode and self._command is None)
        failure = self._base_failure(now, need_geometry=need_geometry)
        if failure:
            return self._zero(now, failure, latch=True)
        command = self._command
        if command is None:
            return self._zero(now, self.reason)
        if now - self._command_received > self.cfg.command_watchdog_s:
            return self._zero(now, "command_watchdog", latch=True)
        if now >= self._command_expires - 1e-9:
            return self._zero(now, "pulse_complete" if self.cfg.pulse_mode else "command_expired")
        frame_age = self._frame_age(command.frame, now)
        if frame_age is None or frame_age > self.cfg.max_frame_age_s:
            return self._zero(now, "camera_stale", latch=True)
        failure = self._geometry_failure(command, self._geometry.get(command.frame), now)
        if failure:
            return self._zero(now, failure, latch=True)
        # The MCU adapter emits only ONE nonzero packet per pulse. A changing
        # host ramp would therefore log velocities the MCU never received and
        # would hold only the first tiny PWM, potentially below wheel stiction.
        # Pulse speed is fixed and must be commissioned; the measured physical
        # upper speed bound, not that PWM/request, controls stopping clearance.
        v = command.linear_mps if self.cfg.pulse_mode else min(
            command.linear_mps, self._last_linear + self.cfg.command_accel_mps2 * max(0.0, dt))
        w = command.angular_rps * (v / command.linear_mps)
        self._last_linear, self._last_angular = v, w
        if v > 0:
            self._ever_moved = True
        self.reason = "motion_allowed" if v > 0 else "ramp_wait"
        required = self._required_clearance(self.cfg.speed_bound_mps, frame_age)
        return self._decision(now, v, w, self.reason, frame_age, required, self.cfg.speed_bound_mps)
