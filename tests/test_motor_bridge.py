"""Actual MotorBridge methods with ROS/serial stubs; no device access or ROS install."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ros2_ws' / 'src' / 'robot_web_teleop'
sys.path.insert(0, str(PKG))
from robot_web_teleop import protocol as wire_protocol


class WriteTimeout(OSError):
    pass


class Wire:
    def __init__(self):
        self.rx, self.writes, self.closed = bytearray(), [], False
        self.short_write, self.write_error, self.read_error = False, None, None

    @property
    def in_waiting(self):
        if self.read_error:
            raise self.read_error
        return len(self.rx)

    def read(self, count):
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def write(self, data):
        self.writes.append(data)
        if self.write_error:
            raise self.write_error
        return 1 if self.short_write else len(data)

    def close(self):
        self.closed = True

    def reset_input_buffer(self):
        self.rx.clear()

    def reset_output_buffer(self):
        pass


class FakeNode:
    overrides = {}

    def __init__(self, _name):
        self.parameters, self.publishers, self.subscriptions = {}, {}, {}

    def declare_parameter(self, key, value):
        self.parameters[key] = self.overrides.get(key, value)

    def get_parameter(self, key):
        return NS(value=self.parameters[key])

    def create_publisher(self, _kind, name, _qos):
        messages = []
        publisher = NS(messages=messages, publish=lambda msg: messages.append(msg.data))
        self.publishers[name] = publisher
        return publisher

    def create_subscription(self, _kind, name, callback, _qos):
        self.subscriptions[name] = callback

    def create_timer(self, *_args, **_kwargs):
        pass

    def get_logger(self):
        return NS(warning=lambda _: None)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.now, self.wire = 1.0, Wire()
        FakeNode.overrides = {}
        namespace = dict(vars(wire_protocol))
        namespace.update(Node=FakeNode, math=math, secrets=NS(randbits=lambda _: 100),
            time=NS(monotonic=lambda: self.now), rclpy=NS(ok=lambda: True),
            serial=NS(Serial=lambda *a, **kw: self.wire, SerialException=OSError,
                      SerialTimeoutException=WriteTimeout),
            Bool=NS, String=NS, Empty=NS, TwistStamped=NS,
            Clock=lambda **kw: None, ClockType=NS(STEADY_TIME=1),
            QoSProfile=lambda **kw: None, ReliabilityPolicy=NS(RELIABLE=1), HistoryPolicy=NS(KEEP_LAST=1))
        tree = ast.parse((PKG / 'robot_web_teleop' / 'motor_bridge.py').read_text())
        tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        exec(compile(tree, 'motor_bridge.py', 'exec'), namespace)
        self.cls = namespace['MotorBridge']
        self.b = self.cls()
        self.b.ros_ns = lambda: round(self.now * 1e9)
        self.b.port = self.wire

    def advance(self, duration):
        self.now += duration
        self.b.tick()

    def ack(self, left=0, right=0, flags=0, delay=.065):
        self.assertIsNotNone(self.b.pending)
        seq = self.b.pending[0]
        self.wire.rx.extend(wire_protocol.frame(f'A {seq} {left} {right} {flags}'))
        self.advance(delay)
        return seq

    def ready(self):
        self.b.tick()
        self.ack()
        self.assertTrue(self.b.gate.ready)

    def command(self, v, w=0, stamp=None):
        self.now += .001
        ns = round(self.now * 1e9) if stamp is None else stamp
        msg = NS(header=NS(stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9)),
                 twist=NS(linear=NS(x=v, y=0, z=0), angular=NS(x=0, y=0, z=w)))
        self.b.on_command(msg)

    def motion_pending(self):
        self.ready()
        self.command(.2)
        self.advance(.02)
        self.assertEqual(self.b.pending[1], (70, 70))

    def simulate_clean_acks(self, duration):
        end, seen, samples = self.now + duration, set(), []
        while self.now < end:
            self.now += .01
            pending = self.b.pending
            if pending and self.now - pending[2] >= .065 and pending[0] not in seen:
                seq, target, _, kind = pending
                self.wire.rx.extend(wire_protocol.frame(f'A {seq} {target[0]} {target[1]} 0'))
                seen.add(seq)
            self.b.tick()
            samples.append((self.now, self.b.gate.ready))
        return samples

    def test_default_baud_and_explicit_global_stop_topic(self):
        self.assertEqual(self.b.baud, 9600)
        self.assertEqual(set(self.b.subscriptions), {'cmd_vel', 'drive/stop'})
        self.assertEqual(set(self.b.publishers), {'drive/ready', 'drive/source'})
        FakeNode.overrides = {'baud': 115200}
        with self.assertRaises(ValueError):
            self.cls()

    def test_only_current_matching_ack_changes_readiness_or_source(self):
        b = self.b
        b.tick()
        self.wire.rx.extend(wire_protocol.frame('A 100 255 255 25'))
        self.advance(.02)
        self.assertFalse(b.gate.ready)
        self.assertEqual(b.source, 'disconnected')
        self.ack(delay=.02)
        stamp = b.last_ack
        self.wire.rx.extend(wire_protocol.frame('A 101 255 255 25'))
        self.advance(.02)
        self.assertTrue(b.gate.ready)
        self.assertEqual((b.last_ack, b.source), (stamp, 'idle'))

    def test_expired_matching_ack_never_becomes_fresh_and_keeps_port_open(self):
        b = self.b
        b.tick()
        self.ack(delay=.181)
        self.assertFalse(b.gate.ready)
        self.assertIsNone(b.last_ack_seq)
        self.assertEqual(b.last_ack, float('-inf'))
        self.assertFalse(self.wire.closed)
        self.assertEqual(b.pending[1], (0, 0))

    def test_zero_preempts_nonzero_and_old_ack_cannot_revive_command(self):
        self.motion_pending()
        old_seq = self.b.pending[0]
        self.command(0)
        self.advance(.001)
        new_seq = self.b.pending[0]
        self.assertNotEqual(new_seq, old_seq)
        self.assertEqual(self.b.pending[1:], ((0, 0), self.now, 'M'))
        self.wire.rx.extend(wire_protocol.frame(f'A {old_seq} 70 70 0'))
        self.ack()
        self.assertEqual(self.b.last_ack_seq, new_seq)
        self.assertEqual(self.b.gate.velocity, (0, 0))
        self.assertFalse(any(p.lstrip().startswith(b'E ') for p in self.wire.writes))

    def test_bt_actual_pwm_overrides_usb_target_without_closing_or_global_stop(self):
        self.motion_pending()
        self.ack(255, -200, 25)
        self.assertEqual(self.b.source, 'bluetooth')
        self.assertFalse(self.b.gate.ready)
        self.assertEqual(self.b.gate.velocity, (0, 0))
        self.assertEqual(self.b.pending[1], (0, 0))
        self.command(0)
        self.command(.2)
        self.ack(255, -200, 25)
        self.assertEqual(self.b.source, 'bluetooth')
        self.assertFalse(self.wire.closed)
        self.assertFalse(any(p.lstrip().startswith(b'E ') for p in self.wire.writes))

    def test_short_bt_handoff_cannot_be_missed_and_needs_post_hold_handshake(self):
        self.ready()
        self.command(.2)
        self.advance(.02)
        # A complete BT session can occur between polls: only sticky 16 is seen.
        self.ack(0, 0, 17)
        recovered_at, hold = self.now, self.b.hold_until
        stale_stamp = round((recovered_at - .001) * 1e9)
        self.assertFalse(self.b.gate.ready)
        samples = self.simulate_clean_acks(.8)
        self.assertTrue(all(not ready for t, ready in samples if t < hold))
        self.assertTrue(self.b.gate.ready)
        first_ready = next(t for t, ready in samples if ready)
        self.assertGreaterEqual(first_ready, hold + .06)
        self.assertGreaterEqual(self.b.ready_pub.messages.count(False), 5)
        self.command(.2, stamp=stale_stamp)
        self.assertEqual(self.b.gate.velocity, (0, 0))
        self.command(.2)
        self.assertEqual(self.b.gate.velocity, (.2, 0))

    def test_each_stop_flag_blocks_and_requires_clean_zero_after_hold(self):
        for flags in (1, 2, 4, 16, 32):
            with self.subTest(flags=flags):
                self.setUp()
                self.b.tick()
                self.ack(flags=flags)
                self.assertFalse(self.b.gate.ready)
                self.assertGreaterEqual(self.b.hold_until, self.now + .5)
                self.simulate_clean_acks(.8)
                self.assertTrue(self.b.gate.ready)
                self.assertFalse(self.wire.closed)

    def test_crc_unknown_flags_and_oversize_recover_with_bounded_rx(self):
        for bad in (b'A 1 0 0 0*0000\n', wire_protocol.frame('A 1 0 0 64'), b'x' * 1000):
            with self.subTest(bad=bad[:20]):
                self.setUp()
                self.motion_pending()
                self.wire.rx.extend(bad)
                for _ in range(5):
                    self.advance(.01)
                    self.assertLessEqual(len(self.b.rx), wire_protocol.MAX_LINE)
                self.assertFalse(self.b.gate.ready)
                self.assertFalse(self.wire.closed)
                self.assertFalse(any(p.lstrip().startswith(b'E ') for p in self.wire.writes))
                self.wire.rx.extend(b'\n')
                self.advance(.01)
                self.simulate_clean_acks(.9)
                self.assertTrue(self.b.gate.ready)

    def test_ack_timeout_and_liveness_recover_without_resetting_usb(self):
        self.motion_pending()
        self.advance(.181)
        self.assertFalse(self.b.gate.ready)
        self.assertEqual(self.b.gate.velocity, (0, 0))
        self.assertEqual(self.b.pending[1], (0, 0))
        self.assertTrue(self.wire.writes[-1].startswith(b'\nM '))
        self.assertFalse(self.wire.closed)
        self.setUp()
        self.ready()
        self.b.pending, self.b.next_send = None, 100
        self.advance(.301)
        self.assertFalse(self.b.gate.ready)
        self.assertEqual(self.b.source, 'disconnected')
        self.assertFalse(self.wire.closed)

    def test_explicit_E_is_retried_and_cannot_be_replaced_by_normal_zero(self):
        self.motion_pending()
        self.ack(255, 255, 25)
        self.b.on_stop(NS())
        self.advance(.001)
        first_seq = self.b.pending[0]
        self.assertEqual(self.b.pending[3], 'E')
        self.command(0)
        self.advance(.05)
        self.assertEqual(self.b.pending[0], first_seq)
        self.assertTrue(self.b.emergency)
        self.advance(.14)
        retry_seq = self.b.pending[0]
        self.assertGreater(retry_seq, first_seq)
        self.assertEqual(self.b.pending[3], 'E')
        self.wire.rx.extend(wire_protocol.frame(f'A {first_seq} 0 0 33'))
        self.advance(.01)
        self.assertTrue(self.b.emergency)
        self.ack(flags=33, delay=.055)
        self.assertFalse(self.b.emergency)
        self.assertFalse(self.b.gate.ready)
        self.assertEqual(self.b.source, 'idle')
        self.assertFalse(self.wire.closed)
        self.simulate_clean_acks(.8)
        self.assertTrue(self.b.gate.ready)
        self.assertEqual(self.b.gate.velocity, (0, 0))

    def test_crc_error_does_not_drop_outstanding_global_stop(self):
        self.b.on_stop(NS())
        self.b.tick()
        first = self.b.pending
        self.wire.rx.extend(b'bad ack\n')
        self.advance(.01)
        self.assertEqual(self.b.pending, first)
        self.assertTrue(self.b.emergency)
        self.advance(.18)
        self.assertEqual(self.b.pending[3], 'E')
        self.assertNotEqual(self.b.pending[0], first[0])

    def test_short_writes_and_write_timeout_do_not_close_or_flood(self):
        for failure in ('short_write', 'write_error'):
            with self.subTest(failure=failure):
                self.setUp()
                setattr(self.wire, failure, True if failure == 'short_write' else WriteTimeout())
                self.b.on_stop(NS())
                self.b.tick()
                self.assertIsNone(self.b.pending)
                self.assertTrue(self.b.emergency)
                for _ in range(6):
                    self.advance(.01)
                self.assertEqual(len(self.wire.writes), 1)
                self.advance(.021)
                self.assertEqual(len(self.wire.writes), 2)
                self.assertTrue(all(p.lstrip().startswith(b'E ') for p in self.wire.writes))
                self.assertFalse(self.wire.closed)

    def test_real_device_error_disconnects_and_clears_source(self):
        self.ready()
        self.wire.read_error = OSError('device removed')
        self.advance(.01)
        self.assertTrue(self.wire.closed)
        self.assertIsNone(self.b.port)
        self.assertFalse(self.b.gate.ready)
        self.assertEqual(self.b.source_pub.messages[-1], 'disconnected')

    def test_normal_requests_are_rate_limited_to_80ms(self):
        self.ready()
        self.assertEqual(len(self.wire.writes), 1)
        self.advance(.014)
        self.assertEqual(len(self.wire.writes), 1)
        self.advance(.002)
        self.assertEqual(len(self.wire.writes), 2)


if __name__ == '__main__':
    unittest.main()
