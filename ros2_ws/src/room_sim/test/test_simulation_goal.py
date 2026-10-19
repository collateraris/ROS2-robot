"""Goal acceptance -> fresh safe command -> one ARM, with deterministic transport."""
from types import SimpleNamespace
import unittest
from room_sim.simulation_goal import SimulationMonitor, run_simulation_goal


class Future:
    def __init__(self): self.value=None;self.completed=False
    def done(self): return self.completed
    def result(self): return self.value
    def complete(self,value): self.value=value;self.completed=True


class Transport:
    def __init__(self):
        self.time=1.
        self.monitor=SimulationMonitor(output_mode='simulation')
        self.result=Future()
        self.request=SimpleNamespace(handle=SimpleNamespace(accepted=True),result=self.result)
        self.arm_calls=[]
        self.validations=[]
        self.events=[]
        self.keep_command=False
        self.is_armed=False
        self.reject_arm=False
        self.refresh()
    def refresh(self):
        self.monitor.marker_at=self.monitor.health_at=self.time
        self.monitor.health=True
        self.monitor.armed,self.monitor.armed_at=self.is_armed,self.time
        if self.keep_command:self.monitor.safe_command(.1,0.,self.time)
    def later(self,delay,callback): self.events.append((self.time+delay,callback))
    def spin(self,timeout):
        self.time+=timeout
        due=[event for event in self.events if event[0]<=self.time+1e-10]
        self.events=[event for event in self.events if event[0]>self.time+1e-10]
        for _,callback in due: callback()
        self.refresh()
    def validate(self): self.validations.append(self.time)
    def arm(self):
        self.arm_calls.append(self.time)
        self.is_armed=not self.reject_arm
        future=Future()
        future.complete(SimpleNamespace(success=not self.reject_arm,message='rejected' if self.reject_arm else 'ok'))
        return future
    def run(self,**kwargs):
        return run_simulation_goal(self.request,self.monitor,
            validate_simulation=self.validate,arm=self.arm,spin_once=self.spin,
            now=lambda:self.time,timeout=1.,**kwargs)


class SimulationGoalTests(unittest.TestCase):
    def test_arm_only_after_accepted_goal_has_new_nonzero_safe_command(self):
        bus=Transport()
        bus.monitor.safe_command(.1,0.,.99)  # previous goal's command is not readiness
        bus.later(.15,lambda:setattr(bus,'keep_command',True))
        bus.later(.4,lambda:bus.result.complete(SimpleNamespace(status=4)))
        self.assertEqual(bus.run().status,4)
        self.assertEqual(len(bus.arm_calls),1)
        self.assertGreaterEqual(bus.arm_calls[0],1.15-1e-10)
        self.assertEqual(bus.validations,bus.arm_calls)

    def test_current_pose_goal_succeeds_without_any_arm(self):
        bus=Transport()
        bus.monitor.safe_command(0.,0.,1.)
        bus.later(.1,lambda:bus.result.complete(SimpleNamespace(status=4)))
        self.assertEqual(bus.run().status,4)
        self.assertEqual(bus.arm_calls,[])

    def test_terminal_success_during_validation_does_not_arm(self):
        bus=Transport();bus.keep_command=True;bus.refresh()
        def validation():
            bus.validations.append(bus.time)
            bus.result.complete(SimpleNamespace(status=4))
        bus.validate=validation
        self.assertEqual(bus.run().status,4)
        self.assertEqual(bus.arm_calls,[])

    def test_watchdog_disarm_fails_without_rearm_for_same_goal(self):
        bus=Transport();bus.keep_command=True;bus.refresh()
        bus.later(.2,lambda:setattr(bus,'is_armed',False))
        with self.assertRaisesRegex(RuntimeError,'disarmed; no automatic rearm'):bus.run()
        self.assertEqual(len(bus.arm_calls),1)

    def test_rejected_arm_is_not_retried(self):
        bus=Transport();bus.keep_command=True;bus.reject_arm=True;bus.refresh()
        with self.assertRaisesRegex(RuntimeError,'ARM rejected'):bus.run()
        self.assertEqual(len(bus.arm_calls),1)

    def test_accepted_goal_with_no_motion_command_times_out_disarmed(self):
        bus=Transport()
        with self.assertRaisesRegex(TimeoutError,'drive stayed disarmed'):bus.run()
        self.assertEqual(bus.arm_calls,[])

    def test_shutdown_does_not_arm(self):
        bus=Transport()
        with self.assertRaisesRegex(RuntimeError,'ROS shutdown'):bus.run(running=lambda:False)
        self.assertEqual(bus.arm_calls,[])

    def test_requires_an_accepted_goal(self):
        bus=Transport();bus.request.handle.accepted=False
        with self.assertRaisesRegex(RuntimeError,'accepted owned goal'):bus.run()
        self.assertEqual(bus.arm_calls,[])


if __name__=='__main__': unittest.main()
