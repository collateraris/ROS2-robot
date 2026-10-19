"""One explicit simulated ARM after an accepted goal produces a fresh safe command.

No automatic rearm is performed within a goal. The transport callbacks supplied
by goal_ab are also replaceable by deterministic test transports.
"""
from dataclasses import dataclass
import math
import time


@dataclass
class SimulationMonitor:
    marker_at: float = -math.inf
    output_mode: str = ''
    health: bool = False
    health_at: float = -math.inf
    command_at: float = -math.inf
    command_nonzero: bool = False
    armed: bool = False
    armed_at: float = -math.inf

    def safe_command(self, linear, angular, received, supported=True):
        valid = supported and all(math.isfinite(v) for v in (linear, angular, received))
        self.command_at = received if valid else -math.inf
        self.command_nonzero = valid and max(abs(linear), abs(angular)) > 1e-6

    def context_fresh(self, now):
        return (self.output_mode == 'simulation' and
                0. <= now-self.marker_at <= .20 and self.health and
                0. <= now-self.health_at <= .20)

    def command_fresh(self, now):
        return 0. <= now-self.command_at <= .20

    def ready(self, accepted_at, now):
        return (self.context_fresh(now) and self.command_fresh(now) and
                self.command_at >= accepted_at and self.command_nonzero)


def run_simulation_goal(request, monitor, *, validate_simulation, arm,
                        spin_once, running=lambda: True, now=time.monotonic,
                        timeout=90.):
    """Return the terminal result; ARM once, only after fresh command readiness.

    The caller must register/cancel the owned request and stop the drive in its
    finally block. Goal terminal success may occur without any motion/ARM.
    """
    if request.handle is None or not request.handle.accepted or request.result is None:
        raise RuntimeError('simulation ARM requires an accepted owned goal')
    accepted_at = now()
    deadline = accepted_at + timeout
    arm_future = None
    arm_requested_at = None
    arm_ack_at = None
    armed_confirmed = False
    while running() and now() < deadline:
        if request.result.done():
            result = request.result.result()
            if result is None or result.status != 4:
                raise RuntimeError('simulation goal ended without success')
            return result
        current = now()
        if arm_future is None:
            if monitor.ready(accepted_at, current):
                # This validation checks the sole provider and its sim parameters.
                # It can process DDS replies; recheck data freshness afterwards.
                validate_simulation()
                if request.result.done():
                    continue  # A zero-distance goal can finish during parameter RPCs.
                if now() < deadline and monitor.ready(accepted_at, now()):
                    arm_requested_at = now()
                    arm_future = arm()  # exactly one attempt for this goal
        elif arm_ack_at is None:
            if not arm_future.done() and now()-arm_requested_at > 3.:
                raise TimeoutError('simulation ARM response timed out; no automatic retry')
            if arm_future.done():
                response = arm_future.result()
                if response is None or not response.success:
                    message = getattr(response, 'message', 'no response')
                    raise RuntimeError('simulation ARM rejected: ' + message)
                arm_ack_at = now()
        else:
            # Never interpret a pre-ARM latched false as a new watchdog event.
            if monitor.armed and monitor.armed_at >= arm_ack_at-.05:
                armed_confirmed = True
            if not armed_confirmed and now()-arm_ack_at > .75:
                raise RuntimeError('simulation ARM not confirmed; no automatic rearm')
            if armed_confirmed:
                if not monitor.context_fresh(now()) or not monitor.command_fresh(now()):
                    raise RuntimeError('simulation sensors/command became stale; no automatic rearm')
                if not monitor.armed or now()-monitor.armed_at > .20:
                    raise RuntimeError('simulation drive disarmed; no automatic rearm')
        spin_once(.05)
    if not running():
        raise RuntimeError('ROS shutdown while executing simulated goal')
    if arm_future is None:
        raise TimeoutError('accepted goal produced no fresh nonzero safe command before timeout; drive stayed disarmed')
    raise TimeoutError('simulated goal timed out; no automatic rearm')
