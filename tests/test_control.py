import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_web_teleop'))
from robot_web_teleop.control import Control


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.c = Control()

    def arm(self, now=10.0, client='a'):
        token = self.c.issue(client, now)
        self.assertTrue(self.c.arm(client, {'challenge': token}, now))
        return token

    @staticmethod
    def cmd(token, seq=0, linear=0.5, angular=-0.2):
        return dict(challenge=token, seq=seq, linear=linear, angular=angular)

    def test_initial_and_disarmed_motion_is_zero(self):
        token = self.c.issue('a', 10)
        self.assertFalse(self.c.drive('a', self.cmd(token), 10))
        self.assertEqual(self.c.tick(10), (0, 0))

    def test_hold_release_and_watchdog(self):
        token = self.arm()
        self.assertTrue(self.c.drive('a', self.cmd(token), 10.01))
        self.assertEqual(self.c.tick(10.1), (0.5, -0.2))
        self.assertTrue(self.c.drive('a', self.cmd(token, 1, 0, 0), 10.11))
        self.assertTrue(self.c.armed)
        self.assertEqual(self.c.tick(10.2), (0, 0))
        self.assertEqual(self.c.tick(10.4), (0, 0))
        self.assertFalse(self.c.armed)

    def test_late_packet_cannot_revive_expired_lease(self):
        self.arm()
        fresh = self.c.issue('a', 10.3)
        self.assertFalse(self.c.drive('a', self.cmd(fresh), 10.3))
        self.assertFalse(self.c.armed)

    def test_duplicate_arm_stops_instead_of_resetting_sequence(self):
        for fault, supplied in [('', 'valid'), ('bad frame', 'valid'), ('', 'bad')]:
            with self.subTest(fault=fault, supplied=supplied):
                self.c.stop()
                token = self.arm()
                self.assertTrue(self.c.drive('a', self.cmd(token, 3), 10.01))
                challenge = token if supplied == 'valid' else 'wrong'
                self.assertFalse(self.c.arm('a', {'challenge': challenge}, 10.02, fault))
                self.assertFalse(self.c.armed)
                self.assertEqual(self.c.tick(10.03), (0, 0))
                self.assertFalse(self.c.drive('a', self.cmd(token, 1), 10.04))

    def test_late_arm_cannot_revive_expired_lease_with_live_challenge(self):
        token = self.arm()
        self.assertTrue(self.c.drive('a', self.cmd(token, 3), 10.01))
        self.assertFalse(self.c.arm('a', {'challenge': token}, 10.27))
        self.assertFalse(self.c.drive('a', self.cmd(token, 1), 10.28))
        self.assertEqual(self.c.tick(10.29), (0, 0))

    def test_challenge_is_bound_to_client(self):
        token = self.c.issue('a', 10)
        self.assertFalse(self.c.arm('b', {'challenge': token}, 10.01))

    def test_expired_challenge_is_rejected(self):
        token = self.arm()
        self.assertTrue(self.c.drive('a', self.cmd(token), 10.2))
        self.assertFalse(self.c.drive('a', self.cmd(token, 1), 10.31))
        self.assertFalse(self.c.armed)

    def test_stop_invalidates_queued_arm(self):
        token = self.arm()
        self.c.stop()
        self.assertFalse(self.c.arm('a', {'challenge': token}, 10.02))
        self.assertEqual(self.c.tick(10.03), (0, 0))

    def test_other_viewer_cannot_take_control_or_fault_owner(self):
        token = self.arm()
        self.c.drive('a', self.cmd(token), 10.01)
        other = self.c.issue('b', 10.02)
        self.assertFalse(self.c.arm('b', {'challenge': other}, 10.02))
        self.assertFalse(self.c.drive('b', self.cmd(other), 10.03, 'bad video'))
        self.assertEqual(self.c.owner, 'a')
        self.assertTrue(self.c.armed)

    def test_invalid_numbers_and_sequence_stop(self):
        for value in [math.nan, math.inf, -math.inf, True, '0.5', 1.1, None]:
            with self.subTest(value=value):
                self.c.stop()
                token = self.arm()
                self.assertFalse(self.c.drive('a', self.cmd(token, linear=value), 10.01))
                self.assertFalse(self.c.armed)
        token = self.arm()
        self.assertTrue(self.c.drive('a', self.cmd(token, 2), 10.01))
        self.assertFalse(self.c.drive('a', self.cmd(token, 2), 10.02))
        self.assertFalse(self.c.armed)

    def test_camera_arduino_or_client_video_fault_stops(self):
        for fault in ['Камера не передаёт свежие кадры', 'Arduino не готова',
                      'Телефон не подтвердил свежий кадр']:
            token = self.arm()
            self.c.drive('a', self.cmd(token), 10.01)
            self.assertFalse(self.c.drive('a', self.cmd(token, 1), 10.02, fault))
            self.assertEqual(self.c.tick(10.03), (0, 0))


if __name__ == '__main__':
    unittest.main()
