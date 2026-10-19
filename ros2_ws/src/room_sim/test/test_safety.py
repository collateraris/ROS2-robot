import math
import unittest
from room_sim.safety import SimulationDriveGate, Source


def ready(autonomy=True):
    gate=SimulationDriveGate(simulation_autonomy=autonomy)
    gate.scan.accept(1.,10.,1.,.35)
    gate.odom.accept(1.,10.,1.,.35)
    gate.on_health(True,10.)
    if not autonomy:
        gate.on_deadman(True,10.)
    return gate


class SimSafetyTests(unittest.TestCase):
    def test_arm_is_explicit_and_ignores_cached_commands(self):
        gate=ready()
        gate.on_command(.1,.2,10.)
        self.assertEqual(gate.output(10.,1.),(0.,0.))
        self.assertTrue(gate.arm(10.,1.)[0])
        self.assertEqual(gate.output(10.,1.),(0.,0.))
        gate.on_command(.1,.2,10.01)
        self.assertEqual(gate.output(10.01,1.01),(.1,.2))
        gate.disarm('stop')
        self.assertTrue(gate.arm(10.02,1.02)[0])
        self.assertEqual(gate.output(10.02,1.02),(0.,0.))

    def test_human_deadman_default_and_health_recovery_never_rearms(self):
        gate=ready(False)
        self.assertTrue(gate.arm(10.,1.)[0])
        gate.on_command(.1,0.,10.)
        gate.on_deadman(False,10.01)
        self.assertEqual(gate.output(10.01,1.01),(0.,0.))
        gate.on_deadman(True,10.02)
        self.assertFalse(gate.armed)
        gate.on_health(False,10.02)
        gate.on_health(True,10.03)
        self.assertFalse(gate.armed)

    def test_stale_health_command_scan_and_clock_pause_stop(self):
        for field in ('health','command','scan','odom'):
            gate=ready()
            gate.arm(10.,1.)
            gate.on_command(.1,0.,10.)
            if field=='health':
                gate.health_at=9.
            elif field=='command':
                gate.command_at=9.
            else:
                getattr(gate,field).received=9.
            self.assertEqual(gate.output(10.01,1.01),(0.,0.))
            self.assertFalse(gate.armed)
        gate=ready()
        gate.arm(10.,1.)
        gate.on_command(.1,0.,10.)
        self.assertEqual(gate.output(10.40,1.),(0.,0.))
        self.assertFalse(gate.armed)

    def test_estop_latch_invalid_twist_and_limits(self):
        gate=ready()
        gate.arm(10.,1.)
        gate.on_command(8.,-9.,10.)
        self.assertEqual(gate.output(10.,1.),(.1,-.4))
        gate.on_command(math.nan,0.,10.)
        self.assertFalse(gate.armed)
        gate.on_estop(True)
        self.assertFalse(gate.arm(10.,1.)[0])
        gate.on_estop(False)
        self.assertTrue(gate.estop)
        self.assertTrue(gate.reset_estop()[0])
        self.assertFalse(gate.armed)

    def test_fresh_timestamps_and_repeated_scan(self):
        source=Source()
        self.assertTrue(source.accept(1.,10.,1.,.35))
        self.assertFalse(source.accept(1.,10.3,1.3,.35))
        self.assertFalse(source.healthy(10.36,1.3,.35))
        self.assertFalse(source.accept(1.5,10.4,1.,.35))
        self.assertFalse(source.accept(0.,10.,1.,.35))

    def test_first_command_wait_expires_and_requires_rearm(self):
        gate=ready()
        gate.arm(10.,1.)
        gate.first_command_wait=.1
        gate.on_health(True,10.15)
        self.assertEqual(gate.output(10.15,1.15),(0.,0.))
        self.assertFalse(gate.armed)


if __name__=='__main__':
    unittest.main()
