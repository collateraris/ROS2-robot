import math
import pytest
from room_base.odom_core import OdomGate, PoseSample, usable_scan


def feed(gate, stamp, x=0., yaw=0., usable=True, delay=.02):
    gate.add_scan(stamp, stamp+100., usable)
    return gate.accept(PoseSample(stamp,x,0.,yaw), stamp+100.+delay, stamp+delay)


def test_warmup_and_body_velocity_use_real_scan_interval():
    gate = OdomGate()
    assert feed(gate,1.) == (0.,0.,0.)
    assert not gate.healthy(101.03,1.03)
    assert feed(gate,1.1,.01)[0] == pytest.approx(.1)
    assert feed(gate,1.2,.02)[0] == pytest.approx(.1)
    assert gate.healthy(101.23,1.23)


def test_stale_input_stops_and_needs_new_good_sequence():
    gate = OdomGate()
    for t in (1.,1.1,1.2):
        feed(gate,t)
    assert not gate.healthy(101.56,1.56)
    assert gate.good_count == 0
    feed(gate,1.5)
    assert not gate.healthy(101.52,1.52)
    feed(gate,1.6)
    feed(gate,1.7)
    assert gate.healthy(101.72,1.72)


@pytest.mark.parametrize('x,yaw',[(1.,0.),(0.,1.)])
def test_pose_jump_latches_even_after_data_returns(x,yaw):
    gate = OdomGate()
    feed(gate,1.)
    assert feed(gate,1.1,x,yaw) is False
    assert gate.latched_fault
    assert feed(gate,1.2) is False


def test_large_gap_requires_restart():
    gate = OdomGate()
    feed(gate,1.)
    assert feed(gate,1.6) is False
    assert gate.latched_fault


def test_unmatched_replayed_and_future_stamps_rejected():
    gate = OdomGate()
    assert gate.accept(PoseSample(1.,0.,0.,0.),101.,1.) is False
    feed(gate,1.)
    assert gate.accept(PoseSample(1.,0.,0.,0.),101.02,1.02) is False
    gate.add_scan(1.1,101.1,True)
    assert gate.accept(PoseSample(1.1,0.,0.,0.),101.1,1.) is False


def test_nan_and_wrong_frame_latch():
    gate = OdomGate()
    assert feed(gate,1.,math.nan) is False
    assert gate.latched_fault
    gate = OdomGate()
    gate.add_scan(1.,101.,True)
    assert gate.accept(PoseSample(1.,0.,0.,0.,frame='map'),101.,1.) is False
    assert gate.latched_fault


def test_yaw_wrap_is_small_rotation():
    gate = OdomGate()
    feed(gate,1.,yaw=math.pi-.01)
    assert feed(gate,1.1,yaw=-math.pi+.01)[2] == pytest.approx(.2)


def test_scan_requires_distributed_finite_returns():
    assert usable_scan([2.]*360,.05,12.,math.pi/180)
    assert not usable_scan([math.inf]*360,.05,12.,math.pi/180)
    assert not usable_scan([2.]*60+[math.inf]*300,.05,12.,math.pi/180)
    assert not usable_scan([2.]*360,.05,12.,0.)


def test_fresh_receipt_does_not_make_old_acquisition_healthy():
    gate = OdomGate()
    for t in (1.,1.1,1.2):
        feed(gate,t)
    assert not gate.healthy(101.23,2.)


def test_cross_topic_order_preserves_original_raw_receipt_time():
    gate = OdomGate(good_frames=1)
    # Raw odom callback can precede the corresponding scan callback by 60 ms.
    gate.add_scan(1.,101.06,True)
    assert gate.accept(PoseSample(1.,0.,0.,0.),101.,1.07,monotonic_now=101.07) == (0.,0.,0.)
    assert gate.last_odom_received == 101.
    assert gate.healthy(101.07,1.07)
    assert not gate.healthy(101.36,1.36)
