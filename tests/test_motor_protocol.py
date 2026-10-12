"""Pure protocol/mixing/stamped-command tests; firmware has a separate harness."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "robot_web_teleop"
sys.path.insert(0, str(PKG))
from robot_web_teleop.protocol import (BT_ACTIVE, KNOWN_FLAGS, VelocityGate, crc16,
    decode_ack, encode_command, encode_emergency_stop, frame, mix_velocity, sequence_newer)


class ProtocolTests(unittest.TestCase):
    def test_crc_and_ack_bounds(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        ack = decode_ack(frame("A 4294967295 -140 140 0"))
        self.assertEqual((ack.sequence, ack.left, ack.right), (2**32 - 1, -140, 140))
        for data in (b"A 1 0 0 0*0000\n", frame("A 1 141 0 0"),
                     frame("A 4294967296 0 0 0"), frame("A 1 0 0 64"), b"x" * 97):
            with self.assertRaises(ValueError):
                decode_ack(data)
        with self.assertRaises(ValueError):
            encode_command(1, 141, 0)

    def test_mix_and_common_saturation(self):
        self.assertEqual(mix_velocity(.2, 0), (70, 70))
        self.assertEqual(mix_velocity(-.2, 0), (-70, -70))
        self.assertEqual(mix_velocity(.4, 4), (7, 140))
        left, right = mix_velocity(0, 1)
        self.assertEqual(left, -right)
        self.assertLess(left, 0)
        with self.assertRaises(ValueError):
            mix_velocity(float("nan"), 0)

    def test_sequence_wrap(self):
        self.assertTrue(sequence_newer(0, 2**32 - 1))
        self.assertFalse(sequence_newer(10, 10))
        self.assertFalse(sequence_newer(9, 10))

    def gate(self):
        gate = VelocityGate()
        gate.set_ready(True, 1_000_000_000)
        return gate

    def test_timeout_uses_monotonic_time_and_cannot_replay(self):
        gate = self.gate()
        self.assertTrue(gate.accept(.2, 0, 1_100_000_000, 1_110_000_000, 5.0))
        self.assertEqual(gate.target(5.24, 1_120_000_000), (.2, 0))
        self.assertEqual(gate.target(5.25, 1_120_000_000), (0, 0))
        self.assertFalse(gate.accept(.2, 0, 1_100_000_000, 1_130_000_000, 5.26))
        self.assertTrue(gate.accept(.2, 0, 1_140_000_000, 1_145_000_000, 5.27))

    def test_zero_invalid_stamps_nan_and_reconnect(self):
        gate = self.gate()
        for stamp in (0, 900_000_000, 1_500_000_000):
            self.assertFalse(gate.accept(.2, 0, stamp, 1_400_000_000, 1.0))
        self.assertTrue(gate.accept(0, 0, 0, 1_500_000_000, 1.1))
        self.assertFalse(gate.accept(.2, 0, 1_490_000_000, 1_510_000_000, 1.2))
        self.assertTrue(gate.accept(.2, 0, 1_520_000_000, 1_530_000_000, 1.3))
        self.assertFalse(gate.accept(float("nan"), 0, 1_540_000_000, 1_550_000_000, 1.4))
        gate.set_ready(False, 1_560_000_000)
        gate.set_ready(True, 1_600_000_000)
        self.assertFalse(gate.accept(.2, 0, 1_590_000_000, 1_610_000_000, 1.5))

    def test_idle_timer_does_not_advance_stamp_barrier(self):
        gate = self.gate()
        gate.target(1.0, 1_030_000_000)
        self.assertTrue(gate.accept(.2, 0, 1_010_000_000, 1_040_000_000, 1.1))

    def test_queued_command_cannot_skip_expiry_when_callback_beats_timer(self):
        gate = self.gate()
        self.assertTrue(gate.accept(.2, 0, 1_010_000_000, 1_020_000_000, 1.0))
        self.assertFalse(gate.accept(.2, 0, 1_120_000_000, 1_320_000_000, 1.3))
        self.assertEqual(gate.velocity, (0, 0))


    def test_bluetooth_ack_has_distinct_pwm_bound(self):
        for flags in (BT_ACTIVE, KNOWN_FLAGS):
            ack = decode_ack(frame(f"A 3 -255 255 {flags}"))
            self.assertEqual((ack.left, ack.right, ack.flags), (-255, 255, flags))
        for body in ("A 3 255 0 0", "A 3 255 0 16", "A 3 256 0 8", "A 3 0 0 72"):
            with self.assertRaises(ValueError):
                decode_ack(frame(body))

    def test_global_stop_is_separate_from_usb_zero(self):
        self.assertEqual(encode_emergency_stop(42), frame("E 42"))
        self.assertEqual(encode_command(42, 0, 0), frame("M 42 0 0"))
        for seq in (-1, 2**32, True, 1.5):
            with self.assertRaises(ValueError):
                encode_emergency_stop(seq)
        for target in (-255, 141, True, 1.0):
            with self.assertRaises(ValueError):
                encode_command(1, target, 0)


if __name__ == "__main__":
    unittest.main()
