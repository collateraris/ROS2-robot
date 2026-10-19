"""Test the actual ROS adapter's decision dispatcher with virtual motors."""

from room_world_model.ros_pi import advance_pending_arm,dispatch_motor
from room_world_model.safety import SafetyDecision


ROBOT={"track_width_m":0.18,"wheel_command_scale_mps":0.25,"pwm_limit":140}


class Governor:
    def stop(self,reason,now):
        return SafetyDecision(0.,0.,False,reason)


class Motor:
    def __init__(self):
        self.pending=None
        self.ready=True
        self._faulted=False
        self.sent=[]
        self.stops=[]

    def send_pulse(self,*args):
        self.sent.append(args)
        self.pending=object()
        return True

    def emergency_stop(self,reason,now):
        self.stops.append(reason)
        self.ready=False
        self._faulted=True


def test_callback_dispatch_starts_full_pulse_immediately_and_tick_cannot_repeat():
    g,m=Governor(),Motor()
    prev=SafetyDecision(0.,0.,True,"armed")
    current=SafetyDecision(.05,0.,True,"motion_allowed",pulse_id=1,pulse_remaining_s=.08)
    out,pid=dispatch_motor(g,m,current,prev,ROBOT,-1,10.)
    assert pid==1 and len(m.sent)==1
    assert m.sent[0][3]==.08
    out,pid=dispatch_motor(g,m,current,current,ROBOT,pid,10.02)
    assert len(m.sent)==1 and not m.stops


def test_fault_stops_pending_mcu_even_if_previous_decision_was_already_overwritten():
    g,m=Governor(),Motor()
    m.pending=object()
    fault=SafetyDecision(0.,0.,False,"malformed_command")
    out,pid=dispatch_motor(g,m,fault,fault,ROBOT,1,10.)
    assert not out.armed and m.stops==["malformed_command"]
    dispatch_motor(g,m,fault,fault,ROBOT,1,10.02)
    assert m.stops==["malformed_command"]


def test_normal_pulse_completion_does_not_lock_every_autonomous_step():
    g,m=Governor(),Motor()
    m.pending=object()
    prev=SafetyDecision(.05,0.,True,"motion_allowed")
    done=SafetyDecision(0.,0.,True,"pulse_complete")
    out,_=dispatch_motor(g,m,done,prev,ROBOT,1,10.)
    assert out.armed and not m.stops


def test_explicit_zero_interrupts_pending_pulse_and_latches():
    g,m=Governor(),Motor()
    m.pending=object()
    prev=SafetyDecision(.05,0.,True,"motion_allowed")
    zero=SafetyDecision(0.,0.,True,"requested_stop")
    out,_=dispatch_motor(g,m,zero,prev,ROBOT,1,10.)
    assert not out.armed
    assert m.stops==["requested_stop"]


def test_partial_or_failed_send_gets_immediate_stop():
    g,m=Governor(),Motor()
    def fail(*args):
        raise OSError("partial write")
    m.send_pulse=fail
    current=SafetyDecision(.05,0.,True,"motion_allowed",pulse_id=1,pulse_remaining_s=.08)
    out,pid=dispatch_motor(g,m,current,current,ROBOT,-1,10.)
    assert not out.armed and pid==-1
    assert m.stops==["motor_submit_failed:partial write"]


def test_pending_arm_waits_for_geometry_but_never_extends_original_grant():
    class PendingGovernor:
        def __init__(self):
            self.geometry_ready=False
            self.calls=[]
            self.stops=[]

        def arm(self,now,*,stationary_confirmed):
            self.calls.append(now)
            assert stationary_confirmed
            return "fresh_lease" if self.geometry_ready else None

        def stop(self,reason,now):
            self.stops.append((reason,now))

    g,m=PendingGovernor(),Motor()
    deadline=13.0
    assert advance_pending_arm(g,m,deadline,10.1)==deadline
    assert advance_pending_arm(g,m,deadline,11.9)==deadline
    g.geometry_ready=True
    assert advance_pending_arm(g,m,deadline,12.0)==0.0
    calls=len(g.calls)
    assert advance_pending_arm(g,m,0.0,12.1)==0.0
    assert len(g.calls)==calls  # cancellation/no grant cannot trigger a later arm
    g.geometry_ready=False
    assert advance_pending_arm(g,m,deadline,12.99)==deadline
    assert advance_pending_arm(g,m,deadline,13.0)==0.0
    assert g.stops==[("explicit_arm_handshake_timeout",13.0)]
    assert g.calls==[10.1,11.9,12.0,12.99]
