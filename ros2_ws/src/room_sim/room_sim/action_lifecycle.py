"""Own action requests across timeout/cancellation, including late acceptance.

The transport interface is Future/GoalHandle, so this can be tested without ROS.
An unacknowledged request is unresolved, never assumed to be rejected.
"""
from dataclasses import dataclass
import time


@dataclass
class GoalRequest:
    generation: int
    acceptance: object
    acceptance_handled: bool = False
    handle: object = None
    result: object = None
    cancellation: object = None
    terminal: bool = False
    error: str = ''


class OwnedGoals:
    def __init__(self):
        self.generation = 0
        self.closed = False
        self.requests = []

    def submit(self, send):
        if self.closed:
            raise RuntimeError('goal session is invalidated')
        self.generation += 1
        request = GoalRequest(self.generation, None)
        self.requests.append(request)
        try:
            request.acceptance = send()
            request.acceptance.add_done_callback(lambda _future: self._accepted(request))
        except Exception as exc:
            # A failed send call may still have put bytes on the transport.
            request.error = f'action send transport: {exc}'
            raise
        return request

    def _accepted(self, request):
        if request.acceptance is None or request.acceptance_handled or not request.acceptance.done():
            return
        request.acceptance_handled = True
        try:
            request.handle = request.acceptance.result()
            if request.handle is None:
                raise RuntimeError('empty action acceptance response')
            if not request.handle.accepted:
                request.terminal = True
                return
            request.result = request.handle.get_result_async()
            request.result.add_done_callback(lambda _future: self._result(request))
            if self.closed or request.generation != self.generation:
                self._cancel(request)
        except Exception as exc:
            # A transport error does not prove the server never accepted the goal.
            request.error = f'action acceptance/result transport: {exc}'

    def _result(self, request):
        if request.result is None or not request.result.done():
            return
        try:
            result = request.result.result()
            if result is not None and result.status in (4, 5, 6):
                request.terminal = True
            else:
                request.error = 'action returned no confirmed terminal status'
        except Exception as exc:
            request.error = f'action result transport: {exc}'

    def _cancel(self, request):
        self._result(request)
        if request.terminal or request.handle is None or not request.handle.accepted or request.cancellation is not None:
            return
        try:
            request.cancellation = request.handle.cancel_goal_async()
        except Exception as exc:
            request.error = f'action cancellation transport: {exc}'

    def poll(self):
        for request in self.requests:
            self._accepted(request)
            self._result(request)
            if self.closed:
                self._cancel(request)

    def invalidate(self):
        self.closed = True
        self.generation += 1
        self.poll()

    @property
    def settled(self):
        self.poll()
        return all(request.terminal for request in self.requests)

    def cancel_and_drain(self, spin_once, *, timeout=2., now=time.monotonic, running=lambda: True):
        """Invalidate first, then cancel/drain owned late acceptances within a bound.

        False means a goal may still exist on the server. The caller must latch
        its drive stop before exit and must not treat a new ARM as safe recovery.
        """
        self.invalidate()
        deadline = now() + timeout
        while not self.settled and running() and now() < deadline:
            spin_once(min(.05, max(0., deadline-now())))
        return self.settled
