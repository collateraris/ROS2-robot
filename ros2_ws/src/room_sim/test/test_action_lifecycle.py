"""Transport-stub regression tests for goals accepted after a local timeout."""
from types import SimpleNamespace
import unittest
from room_sim.action_lifecycle import OwnedGoals


class Future:
    def __init__(self):
        self.finished=False
        self.value=None
        self.callbacks=[]
    def done(self):return self.finished
    def result(self):return self.value
    def add_done_callback(self,callback):
        self.callbacks.append(callback)
        if self.finished:callback(self)
    def complete(self,value):
        self.finished,self.value=True,value
        for callback in list(self.callbacks):callback(self)


class Transport:
    def __init__(self):self.time=0.;self.events=[]
    def later(self,delay,callback):self.events.append((self.time+delay,callback))
    def spin(self,timeout):
        self.time+=timeout
        due=[event for event in self.events if event[0]<=self.time]
        self.events=[event for event in self.events if event[0]>self.time]
        for _,callback in due:callback()


class Handle:
    def __init__(self,transport,accepted=True):
        self.transport=transport
        self.accepted=accepted
        self.result=Future()
        self.cancel_count=0
    def get_result_async(self):return self.result
    def cancel_goal_async(self):
        self.cancel_count+=1
        response=Future()
        self.transport.later(.05,lambda:response.complete(SimpleNamespace(return_code=0)))
        self.transport.later(.10,lambda:self.result.complete(SimpleNamespace(status=5)))
        return response


class GoalCleanupTests(unittest.TestCase):
    def test_active_goal_cancelled_before_cleanup_returns(self):
        bus=Transport();session=OwnedGoals();acceptance=Future();handle=Handle(bus)
        session.submit(lambda:acceptance);acceptance.complete(handle)
        self.assertTrue(session.cancel_and_drain(bus.spin,timeout=.5,now=lambda:bus.time))
        self.assertEqual(handle.cancel_count,1)
        self.assertTrue(session.closed)
        with self.assertRaises(RuntimeError):session.submit(Future)

    def test_late_acceptance_after_local_timeout_is_cancelled(self):
        bus=Transport();session=OwnedGoals();acceptance=Future();handle=Handle(bus)
        session.submit(lambda:acceptance)
        bus.later(.15,lambda:acceptance.complete(handle))
        self.assertTrue(session.cancel_and_drain(bus.spin,timeout=.5,now=lambda:bus.time))
        self.assertEqual(handle.cancel_count,1)
        self.assertEqual(handle.result.result().status,5)

    def test_unconfirmed_acceptance_is_unresolved_and_wait_is_bounded(self):
        bus=Transport();session=OwnedGoals();session.submit(Future)
        self.assertFalse(session.cancel_and_drain(bus.spin,timeout=.25,now=lambda:bus.time))
        self.assertLessEqual(bus.time,.251)

    def test_send_error_after_unknown_transport_outcome_is_unresolved(self):
        bus=Transport();session=OwnedGoals()
        def partial_send(): raise RuntimeError('transport lost after write')
        with self.assertRaises(RuntimeError):session.submit(partial_send)
        self.assertFalse(session.cancel_and_drain(bus.spin,timeout=.1,now=lambda:bus.time))

    def test_cancel_ack_is_not_a_terminal_goal_confirmation(self):
        bus=Transport();session=OwnedGoals();acceptance=Future();handle=Handle(bus)
        def acknowledged_only():
            response=Future();response.complete(SimpleNamespace(return_code=0));return response
        handle.cancel_goal_async=acknowledged_only
        session.submit(lambda:acceptance);acceptance.complete(handle)
        self.assertFalse(session.cancel_and_drain(bus.spin,timeout=.1,now=lambda:bus.time))

    def test_shutdown_still_issues_cancel_and_reports_unresolved_without_spinning(self):
        bus=Transport();session=OwnedGoals();acceptance=Future();handle=Handle(bus)
        session.submit(lambda:acceptance);acceptance.complete(handle)
        self.assertFalse(session.cancel_and_drain(bus.spin,timeout=.5,now=lambda:bus.time,running=lambda:False))
        self.assertEqual(handle.cancel_count,1)
        self.assertEqual(bus.time,0.)

    def test_success_and_rejection_do_not_cancel_terminal_goals(self):
        for accepted in (True,False):
            bus=Transport();session=OwnedGoals();acceptance=Future();handle=Handle(bus,accepted)
            session.submit(lambda:acceptance);acceptance.complete(handle)
            if accepted:handle.result.complete(SimpleNamespace(status=4))
            self.assertTrue(session.cancel_and_drain(bus.spin,timeout=.5,now=lambda:bus.time))
            self.assertEqual(handle.cancel_count,0)

    def test_old_generation_acceptance_cannot_become_current_work(self):
        bus=Transport();session=OwnedGoals();old=Future();new=Future();handle=Handle(bus)
        session.submit(lambda:old);session.submit(lambda:new)
        old.complete(handle)
        self.assertEqual(handle.cancel_count,1)
        self.assertFalse(session.settled)


if __name__=='__main__':unittest.main()
