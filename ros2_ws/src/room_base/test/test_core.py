from pathlib import Path
import math
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from room_base.core import (EncoderOdometry, body_velocity, crc16, frame, fresh,
                            parse_frame, signed_delta, uint32, wheel_targets)


class BaseCoreTests(unittest.TestCase):
    def test_wire_crc_and_reject_corruption(self):
        self.assertEqual(crc16(b'123456789'), 0x29B1)
        self.assertEqual(parse_frame(frame('C 12 2 9 -100 100')),
                         ['C', '12', '2', '9', '-100', '100'])
        for bad in (b'C 12 2 9 -101 100*0000\n', b'Q*123\n', b'Q*0000', b'*FFFF\n'):
            with self.assertRaises(ValueError): parse_frame(bad)
        for bad in ('-1', '4294967296', ' 2', '1.0', '１２'):
            with self.assertRaises(ValueError): uint32(bad)

    def test_encoder_odometry_uses_counts_and_wraps(self):
        odo = EncoderOdometry(.2, 1000.)
        self.assertIsNone(odo.update(1000, 0, 0))
        self.assertEqual(odo.update(1100, 0, 0), (0., 0.))
        self.assertEqual(odo.x, 0.)
        v, w = odo.update(1200, 10, 10)
        self.assertAlmostEqual(v, .1); self.assertAlmostEqual(w, 0.)
        self.assertAlmostEqual(odo.x, .01)
        v, w = odo.update(1300, 0, 20)
        self.assertAlmostEqual(v, 0.); self.assertAlmostEqual(w, 1.)
        self.assertAlmostEqual(odo.yaw, .1)
        self.assertEqual(signed_delta(5, 0xfffffffe), 7)
        self.assertEqual(signed_delta(0xfffffffe, 5), -7)
        with self.assertRaises(ValueError): odo.update(1400, 100000, 100000)
        self.assertEqual(odo.previous, (1300, 0, 20))

    def test_exact_arc_and_clock_wrap(self):
        odo = EncoderOdometry(.2, 1000.)
        odo.update(0xfffffff0, 0, 0)
        v, w = odo.update(84, 0, 20)
        self.assertAlmostEqual(v, .1); self.assertAlmostEqual(w, 1.)
        self.assertAlmostEqual(odo.x, .1 * math.sin(.1))
        self.assertAlmostEqual(odo.y, .1 * (1-math.cos(.1)))
        self.assertIsNone(odo.update(84, 0, 20))
        with self.assertRaises(ValueError): odo.update(3000, 1, 21)

    def test_setpoint_bounds_and_freshness(self):
        self.assertEqual(wheel_targets(.1, 0., .16, 150), (100, 100))
        self.assertEqual(wheel_targets(0., 1., .2, 150), (-100, 100))
        self.assertEqual(wheel_targets(10., 10., .2, 150), (123, 150))
        v, w = body_velocity(-100, 100, .2)
        self.assertAlmostEqual(v, 0.); self.assertAlmostEqual(w, 1.)
        with self.assertRaises(ValueError): wheel_targets(float('nan'), 0., .2, 150)
        self.assertTrue(fresh(0., .25) and fresh(.25, .25))
        self.assertFalse(fresh(.251, .25)); self.assertFalse(fresh(-.01, .25))

    def test_firmware_host_harness(self):
        compiler = shutil.which('g++')
        if compiler is None: self.skipTest('g++ unavailable')
        folder = Path(__file__).resolve().parents[4] / 'firmware' / 'room_base_v3' / 'tests'
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / 'firmware_test'
            subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(folder),
                            str(folder/'firmware_test.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__': unittest.main()
