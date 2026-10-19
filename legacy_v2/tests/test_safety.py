"""Clock, stale-image, replay and braking fault injection without moving a robot."""

from dataclasses import replace
import math

import pytest

from room_world_model.protocol import FrameRef, MetricGeometry, MotionCommand
from room_world_model.safety import SafetyConfig, SafetyGovernor, stopping_distance


class Harness:
    def __init__(self, **config):
        self.now = 10.0
        self.gov = SafetyGovernor(SafetyConfig(calibration_confirmed=True,
            allow_camera_only_research=True, **config), clock=lambda: self.now)
        self.seq = 0
        self.command_seq = 0
        self.gov.set_drive_ready(True)
        self.gov.set_deadman(True)
        self.frame()
        self.lease = self.gov.arm(stationary_confirmed=True)
        assert self.lease is not None

    def frame(self, *, bound=0.05, geometry=True, clearance=2.0, linear=0.06, angular=0.0,
              fov=0.9, metric_valid=True, covered=True):
        self.seq += 1
        self.ref = FrameRef("pi_boot", "camera_epoch", self.seq)
        assert self.gov.register_frame(self.ref, self.now, bound)
        if geometry:
            evidence = MetricGeometry(self.ref, clearance, angular, metric_valid, fov, covered, linear)
            assert self.gov.update_geometry(evidence)
        return self.ref

    def step(self, seconds=0.02, *, deadman=True, ready=True):
        self.now += seconds
        if deadman:
            self.gov.set_deadman(True)
        if ready:
            self.gov.set_drive_ready(True)
        return self.gov.tick()

    def advance(self, seconds, *, deadman=True, ready=True):
        count = round(seconds / 0.02)
        assert math.isclose(count * 0.02, seconds, abs_tol=1e-8)
        out = None
        for _ in range(count):
            out = self.step(deadman=deadman, ready=ready)
        return out

    def command(self, *, linear=0.06, angular=0.0, **fields):
        self.command_seq += 1
        command = MotionCommand(self.ref, self.lease, self.command_seq, linear, angular)
        return replace(command, **fields)

    def ready_frame(self, **kwargs):
        self.advance(0.30)
        return self.frame(**kwargs)

    def moving(self, *, linear=0.06, angular=0.0):
        self.ready_frame(linear=linear, angular=angular)
        self.gov.submit(self.command(linear=linear, angular=angular))
        out = self.step()
        assert out.moving
        return out


def test_default_is_disarmed_and_calibration_cannot_be_implied_by_packets():
    g = SafetyGovernor(clock=lambda: 10.0)
    g.set_deadman(True)
    g.set_drive_ready(True)
    assert g.arm(stationary_confirmed=True) is None
    assert not g.tick().moving
    assert g.reason == "calibration_required"


def test_stationary_confirmation_required_and_true_heartbeats_never_rearm():
    h = Harness()
    h.gov.stop()
    h.advance(0.30)
    h.frame()
    assert not h.gov.armed
    assert h.gov.arm() is None
    assert h.gov.reason == "confirm_wheels_stationary"
    assert h.gov.arm(stationary_confirmed=True) is not None
    assert h.gov.lease_id != h.lease


def test_unknown_capture_latency_blocks_motion():
    h = Harness()
    assert not h.gov.register_frame(FrameRef("pi_boot", "camera_epoch", 99), h.now, None)
    assert not h.gov.armed
    assert h.gov.reason == "capture_timing_uncalibrated"


def test_v2_entire_exposure_must_follow_settling_not_just_its_midpoint():
    h = Harness()
    h.advance(0.30)
    h.seq += 1
    h.ref = FrameRef("pi_boot", "camera_epoch", h.seq)
    earliest, latest = 10.15, 10.29
    assert earliest < h.gov.settle_until_mono_s < (earliest + latest) / 2
    assert h.gov.register_exposure_interval(h.ref, h.now, earliest, latest)
    assert h.gov.update_geometry(MetricGeometry(h.ref, 2.0, 0.0, True, 0.9, True, 0.06))
    decision = h.gov.submit(h.command())
    assert not decision.moving
    assert decision.reason == "waiting_for_post_settle_frame"


def test_v2_frame_age_uses_earliest_optical_time():
    h = Harness(max_frame_age_s=0.20)
    h.advance(0.30)
    h.seq += 1
    ref = FrameRef("pi_boot", "camera_epoch", h.seq)
    assert h.gov.register_exposure_interval(ref, h.now, h.now - 0.35, h.now - 0.01)
    assert h.gov.tick().reason == "camera_stale"
    assert not h.gov.armed


@pytest.mark.parametrize("earliest,latest", [(None, None), (10.0, 10.4),
                                            (10.2, 10.1), (float("nan"), 10.2)])
def test_v2_missing_future_or_reversed_exposure_revokes_lease(earliest, latest):
    h = Harness()
    h.advance(0.30)
    assert not h.gov.register_exposure_interval(
        FrameRef("pi_boot", "camera_epoch", 99), h.now, earliest, latest)
    assert not h.gov.armed and not h.gov.lease_id
    assert h.gov.reason == "capture_timing_uncalibrated"


def test_v2_delayed_pwm_off_extends_settling_and_old_ack_cannot_shorten_it():
    h = Harness()
    h.moving()
    h.advance(0.06)
    software_settle = h.gov.settle_until_mono_s
    h.advance(0.30)
    latest_off = h.now - 0.03
    h.gov.confirm_motor_stop(latest_off)
    measured_brake = max(h.gov.cfg.speed_bound_mps / h.gov.cfg.braking_decel_min_mps2,
                         h.gov.cfg.angular_speed_bound_rps / h.gov.cfg.angular_braking_decel_min_rps2)
    extended = latest_off + measured_brake + h.gov.cfg.settle_extra_s
    assert extended > software_settle
    assert h.gov.settle_until_mono_s == pytest.approx(extended)
    h.gov.confirm_motor_stop(latest_off - 0.2)
    assert h.gov.settle_until_mono_s == pytest.approx(extended)
    h.frame()
    assert h.gov.submit(h.command()).reason == "waiting_for_post_settle_frame"


def test_v2_future_mcu_stop_time_revokes_lease():
    h = Harness()
    h.gov.confirm_motor_stop(h.now + 1.0)
    assert not h.gov.armed
    assert h.gov.reason == "invalid_mcu_stop_time"


def test_driver_buffer_latency_counts_even_for_just_received_frame():
    h = Harness(max_frame_age_s=0.40)
    h.frame(bound=0.75)
    assert h.gov.tick().reason == "camera_stale"
    assert not h.gov.armed


def test_frame_exposed_before_arm_settling_cannot_start_a_pulse():
    h = Harness()
    h.advance(0.30)
    h.frame(bound=0.75)
    out = h.gov.submit(h.command())
    assert out.reason == "waiting_for_post_settle_frame"
    assert not out.moving and out.armed


def test_slow_camera_can_move_only_after_post_settle_exposure():
    h = Harness()
    h.advance(1.00)
    h.frame(bound=0.75)
    out = h.gov.submit(h.command())
    assert out.reason in {"ramp_wait", "motion_allowed"}
    assert h.step().moving


def test_pulse_expiry_zeroes_output_and_requires_new_post_braking_frame():
    h = Harness()
    start = h.moving()
    pulse_id = start.pulse_id
    middle = h.step()
    assert middle.pulse_id == pulse_id
    assert middle.pulse_remaining_s < start.pulse_remaining_s
    out = h.advance(0.04)
    assert out.reason == "pulse_complete"
    assert not out.moving and out.armed
    assert out.speed_bound_mps == pytest.approx(h.gov.cfg.speed_bound_mps)
    assert out.settle_until_mono_s > h.now
    h.frame()
    blocked = h.gov.submit(h.command())
    assert not blocked.moving
    assert blocked.reason == "waiting_for_post_settle_frame"


def test_inflight_new_command_cannot_extend_a_motion_pulse():
    h = Harness()
    h.moving()
    h.frame()
    out = h.gov.submit(h.command())
    assert not out.moving
    assert out.reason == "pulse_interrupted_by_new_proposal"
    assert out.settle_until_mono_s > h.now


def test_same_inference_frame_with_increasing_command_id_is_still_a_replay():
    h = Harness()
    h.ready_frame()
    h.gov.submit(h.command())
    out = h.gov.submit(h.command())
    assert out.reason == "reused_inference_frame"
    assert not out.moving and not out.armed


def test_duplicate_command_id_latches_stop():
    h = Harness()
    h.ready_frame()
    cmd = h.command()
    h.gov.submit(cmd)
    out = h.gov.submit(cmd)
    assert out.reason == "command_replay_or_reorder"
    assert not out.armed


def test_duplicate_camera_delivery_cannot_make_an_old_frame_fresh():
    h = Harness()
    original = h.ref
    h.advance(0.10)
    assert not h.gov.register_frame(original, h.now, 0.0)
    out = h.gov.tick()
    assert out.reason == "camera_frame_replay_or_reorder"
    assert out.frame_age_s == pytest.approx(0.15)
    assert not out.armed


def test_camera_restart_revokes_lease_and_retired_epoch_cannot_return():
    h = Harness()
    old_epoch = h.ref
    new = FrameRef("pi_boot", "camera_new", 0)
    assert h.gov.register_frame(new, h.now, 0.05)
    assert not h.gov.armed and h.gov.lease_id == ""
    assert h.gov.reason == "camera_restarted"
    assert not h.gov.register_frame(replace(old_epoch, frame_seq=100), h.now, 0.05)
    assert h.gov.reason == "camera_epoch_replay"


def test_wrong_lease_cannot_move_even_with_current_picture():
    h = Harness()
    h.ready_frame()
    out = h.gov.submit(h.command(lease_id="old_lease"))
    assert out.reason == "wrong_or_expired_lease"
    assert not out.armed


def test_geometry_arrival_does_not_reset_camera_freshness():
    h = Harness(max_geometry_age_s=0.20, pulse_mode=False)
    h.advance(0.20)
    h.frame(geometry=False)
    out = h.step()
    assert out.reason == "geometry_stale"
    assert not out.armed


def test_pulse_idle_wait_allows_old_geometry_but_never_motion_from_it():
    h = Harness(max_geometry_age_s=0.20)
    h.advance(0.30)
    h.frame(geometry=False)
    idle = h.gov.tick()
    assert idle.armed and not idle.moving
    rejected = h.gov.submit(h.command())
    assert rejected.reason == "geometry_stale"
    assert not rejected.armed and not rejected.moving


def test_duplicate_geometry_cannot_replace_clearance_or_refresh_frame():
    h = Harness()
    assert not h.gov.update_geometry(MetricGeometry(h.ref, 100.0, 0.0, True, 0.9, True, 0.06))
    assert h.gov._geometry[h.ref].clearance_m == 2.0


def test_inference_deadline_uses_pi_elapsed_time_even_if_pc_reports_zero():
    h = Harness()
    h.ready_frame()
    cmd = h.command(inference_ms=0.0)
    h.advance(0.42)
    out = h.gov.submit(cmd)
    assert out.reason == "stale_inference_result"
    assert not out.moving


def test_reported_overlong_inference_is_an_additional_stop_condition():
    h = Harness()
    h.ready_frame()
    out = h.gov.submit(h.command(inference_ms=301.0))
    assert out.reason == "inference_deadline_exceeded"


def test_deadman_timeout_stops_and_new_heartbeat_does_not_rearm():
    h = Harness()
    out = h.advance(0.32, deadman=False)
    assert out.reason == "deadman_stale"
    assert not out.armed
    h.gov.set_deadman(True)
    assert not h.gov.tick().armed


def test_drive_ack_timeout_stops():
    h = Harness()
    out = h.advance(0.52, ready=False)
    assert out.reason == "drive_ready_stale"
    assert not out.armed


def test_cpu_scheduling_pause_latches_stop_even_if_heartbeats_are_fresh():
    h = Harness()
    out = h.step(0.20)
    assert out.reason == "control_loop_missed_deadline"
    assert not out.armed


def test_backwards_or_nonfinite_monotonic_clock_latches_stop():
    h = Harness()
    assert h.gov.tick(9.0).reason == "monotonic_time_reversed"
    assert not h.gov.armed
    h = Harness()
    assert h.gov.tick(float("nan")).reason == "invalid_monotonic_time"


@pytest.mark.parametrize("v,w,reason", [
    (-0.01, 0.0, "reverse_or_pivot_forbidden"),
    (0.0, 0.1, "reverse_or_pivot_forbidden"),
    (0.06, 0.19, "command_speed_limit"),
    (0.01, 0.10, "turn_too_sharp"),
    (0.07, 0.0, "command_speed_limit"),
])
def test_blind_or_overspeed_actions_stop(v, w, reason):
    h = Harness()
    h.ready_frame()
    out = h.gov.submit(h.command(linear=v, angular=w))
    assert out.reason == reason
    assert not out.armed


def test_unobserved_swept_footprint_and_relative_only_depth_block_motion():
    for field in ("covered", "metric_valid"):
        h = Harness()
        h.ready_frame(**{field: False})
        assert not h.gov.armed
        assert h.gov.reason == "geometry_untrusted_or_unobserved"


def test_geometry_for_another_curve_cannot_authorize_a_turn():
    h = Harness()
    h.ready_frame()
    out = h.gov.submit(h.command(angular=0.1))
    assert out.reason == "geometry_path_mismatch"


def test_turn_cannot_extend_outside_observed_fov():
    h = Harness()
    h.ready_frame(angular=0.1, fov=0.15)
    out = h.gov.submit(h.command(angular=0.1))
    assert out.reason == "turn_leaves_observed_fov"


def test_forward_arc_preserves_verified_curvature_during_speed_ramp():
    h = Harness(pulse_mode=False)
    out = h.moving(angular=0.12)
    assert out.linear_mps < 0.06
    assert out.angular_rps / out.linear_mps == pytest.approx(2.0)


def test_pulse_velocity_is_constant_because_mcu_receives_one_packet_only():
    h = Harness()
    h.ready_frame(angular=0.12)
    first = h.gov.submit(h.command(angular=0.12))
    second = h.step()
    assert (first.linear_mps, first.angular_rps) == pytest.approx((0.06, 0.12))
    assert (second.linear_mps, second.angular_rps) == pytest.approx((0.06, 0.12))
    assert first.pulse_id == second.pulse_id


def test_next_pulse_requires_and_accepts_a_new_exposure_after_physical_settling():
    h = Harness()
    first = h.moving()
    h.advance(0.06)
    settle = h.gov.settle_until_mono_s
    h.advance(0.90)
    assert h.now - 0.05 > settle
    h.frame()
    second = h.gov.submit(h.command())
    assert second.moving
    assert second.pulse_id > first.pulse_id


def test_newly_observed_near_obstacle_interrupts_old_approved_pulse():
    h = Harness()
    h.moving()
    h.frame(clearance=0.01)
    out = h.gov.tick()
    assert not out.moving and not out.armed
    assert out.reason == "insufficient_braking_clearance"


def test_braking_limit_uses_physical_speed_bound_not_smaller_requested_speed():
    h = Harness(pulse_mode=False)
    h.moving()
    h.frame(linear=0.01)
    out = h.gov.submit(h.command(linear=0.01))
    assert out.linear_mps == pytest.approx(0.01)
    assert out.speed_bound_mps == pytest.approx(0.12)
    misleading = stopping_distance(0.01, h.gov._reaction_s(out.frame_age_s), 0.25, 0.15)
    assert out.required_clearance_m > misleading
    h.frame(linear=0.0)
    stopped = h.gov.submit(h.command(linear=0.0))
    assert not stopped.moving
    assert stopped.speed_bound_mps == pytest.approx(0.12)
    assert stopped.required_clearance_m > misleading
    assert h.step().speed_bound_mps == pytest.approx(0.12)


def test_insufficient_clearance_cannot_be_overridden_by_a_tiny_speed_request():
    h = Harness()
    h.ready_frame(linear=0.001, clearance=0.16)
    out = h.gov.submit(h.command(linear=0.001))
    assert out.reason == "insufficient_braking_clearance"
    assert not out.armed


def test_nan_and_malformed_commands_stop_an_already_moving_robot():
    h = Harness()
    h.moving()
    out = h.gov.submit(h.command(linear=float("nan")))
    assert out.reason == "malformed_command" and not out.moving and not out.armed
    h = Harness()
    h.moving()
    out = h.gov.submit_json('{"linear_mps":NaN}')
    assert out.reason == "malformed_command" and not out.moving and not out.armed


def test_stopping_distance_and_moving_obstacle_term():
    assert stopping_distance(0.1, 0.8, 0.2, 0.15) == pytest.approx(0.255)
    assert stopping_distance(0.1, 0.8, 0.2, 0.15, 0.5) == pytest.approx(0.905)
    for args in [(0.1, 0.2, 0.0), (float("nan"), 0.2, 0.2), (-0.1, 0.2, 0.2)]:
        with pytest.raises(ValueError):
            stopping_distance(*args)


def test_invalid_safety_configuration_fails_at_startup():
    for values in [{"speed_bound_mps": 0.01}, {"braking_decel_min_mps2": 0.0},
                   {"max_frame_age_s": float("inf")}, {"pulse_mode": "yes"},
                   {"max_motion_pulse_s": 0.5}]:
        with pytest.raises(ValueError):
            SafetyConfig(**values)
