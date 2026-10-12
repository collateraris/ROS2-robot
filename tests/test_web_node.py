"""Exercise the real ROS gateway class without ROS or a running web server."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "ros2_ws" / "src"
          / "robot_web_teleop" / "robot_web_teleop" / "web_node.py")


class TwistStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))


class Publisher:
    def __init__(self, topic):
        self.topic, self.messages = topic, []

    def publish(self, message):
        self.messages.append(message)


class WebHarness:
    def __init__(self, view_only=False, camera_required=True):
        self.monotonic_now, self.ros_ns = 10.0, 100_000_000_000
        self.ros_ok, self.clock_calls, self.tokens = True, 0, 0
        self.publishers, self.subscriptions = [], []
        harness = self

        class Node:
            def __init__(self, name):
                self.name, self.parameters = name, {}

            def declare_parameter(self, name, default):
                overrides = {'view_only': view_only, 'camera_required': camera_required}
                self.parameters[name] = overrides.get(name, default)

            def get_parameter(self, name):
                return SimpleNamespace(value=self.parameters[name])

            def create_publisher(self, message_type, topic, qos):
                publisher = Publisher(topic)
                harness.publishers.append(publisher)
                return publisher

            def create_subscription(self, message_type, topic, callback, qos):
                subscription = SimpleNamespace(topic=topic, callback=callback)
                harness.subscriptions.append(subscription)
                return subscription

            def get_clock(self):
                return SimpleNamespace(now=harness.clock_now)

        namespace = dict(
            Node=Node, math=math, threading=threading, TwistStamped=TwistStamped,
            Bool=object, Empty=object, String=object,
            CompressedImage=object, QoSProfile=SimpleNamespace,
            ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best_effort"),
            time=SimpleNamespace(monotonic=lambda: self.monotonic_now),
            secrets=SimpleNamespace(token_urlsafe=self.new_token),
            rclpy=SimpleNamespace(ok=lambda: self.ros_ok))
        tree = ast.parse(SOURCE.read_text())
        tree.body = [item for item in tree.body if isinstance(item, ast.ClassDef)]
        exec(compile(tree, str(SOURCE), "exec"), namespace)
        self.node = namespace["WebNode"]()

    def clock_now(self):
        self.clock_calls += 1
        stamp = self.stamp(self.ros_ns)
        return SimpleNamespace(nanoseconds=self.ros_ns, to_msg=lambda: stamp)

    @staticmethod
    def stamp(nanoseconds):
        return SimpleNamespace(sec=nanoseconds // 1_000_000_000,
                               nanosec=nanoseconds % 1_000_000_000)

    def new_token(self, size):
        self.tokens += 1
        return f"frame-{self.tokens}"

    def receive_frame(self, stamp_age=0.0):
        # CameraNode stamps host receipt time; this is not exposure time.
        stamp = self.stamp(self.ros_ns - round(stamp_age * 1e9))
        self.node.on_frame(SimpleNamespace(
            format="bgr8; jpeg compressed bgr8", data=b"\xff\xd8\xff\xd9",
            header=SimpleNamespace(stamp=stamp)))
        return self.node.latest

    def advance(self, seconds):
        self.monotonic_now += seconds
        self.ros_ns += round(seconds * 1e9)


class WebNodeTests(unittest.TestCase):
    def test_view_only_has_preview_subscription_and_cannot_publish_commands(self):
        h = WebHarness(view_only=True)
        n = h.node
        self.assertEqual([s.topic for s in h.subscriptions],
                         ["camera/preview/compressed"])
        self.assertEqual(h.publishers, [])
        self.assertIsNone(n.pub)
        # Even an explicit callback from an external ready=True source cannot
        # change the mode or cause a zero/nonzero command to be published.
        n.on_ready(SimpleNamespace(data=True))
        self.assertTrue(n.ready)
        self.assertFalse(n.health()["drive_ready"])
        n.publish(.5, -.25)
        n.publish(0.0, 0.0)
        n.emergency_stop()
        self.assertEqual(h.publishers, [])
        self.assertEqual(h.clock_calls, 0)

    def test_normal_mode_uses_only_strict_camera_topic_and_command_publisher(self):
        h = WebHarness()
        self.assertEqual([s.topic for s in h.subscriptions],
                         ["drive/ready", "drive/source", "camera/image_raw/compressed"])
        self.assertEqual([p.topic for p in h.publishers], ["cmd_vel", "drive/stop"])
        h.node.on_ready(SimpleNamespace(data=True))
        self.assertTrue(h.node.health()["drive_ready"])
        h.advance(.351)
        self.assertFalse(h.node.health()["drive_ready"])

    def test_direct_sight_uses_preview_and_explicit_stop_has_own_topic(self):
        h = WebHarness(camera_required=False)
        self.assertEqual(h.subscriptions[-1].topic, 'camera/preview/compressed')
        self.assertEqual(h.node.frame_max_age, 5.0)
        h.node.publish(0.0, 0.0)
        self.assertEqual(h.node.stop_pub.messages, [])
        h.node.emergency_stop()
        self.assertEqual(len(h.node.stop_pub.messages), 1)
        self.assertEqual(len(h.node.pub.messages), 1)
        h.ros_ok = False
        h.node.emergency_stop()
        self.assertEqual(len(h.node.stop_pub.messages), 1)

    def test_bluetooth_source_blocks_ready_and_expires_as_disconnected(self):
        h = WebHarness()
        h.node.on_ready(SimpleNamespace(data=True))
        h.node.on_source(SimpleNamespace(data='bluetooth'))
        self.assertEqual(h.node.health()['drive_source'], 'bluetooth')
        self.assertFalse(h.node.health()['drive_ready'])
        h.node.on_source(SimpleNamespace(data='unknown'))
        self.assertEqual(h.node.health()['drive_source'], 'bluetooth')
        h.advance(.351)
        self.assertEqual(h.node.health()['drive_source'], 'disconnected')
        self.assertFalse(h.node.health()['drive_ready'])

    def test_view_frame_ttl_is_five_seconds_but_control_freshness_stays_strict(self):
        h = WebHarness(view_only=True)
        item = h.receive_frame()
        self.assertEqual(h.node.frame(), item)
        self.assertTrue(h.node.health()["camera_fresh"])
        self.assertTrue(h.node.frame_fresh(item[1]))
        h.advance(2.0)
        self.assertEqual(h.node.frame(), item)
        self.assertFalse(h.node.health()["camera_fresh"])
        self.assertFalse(h.node.frame_fresh(item[1]))
        h.advance(3.0)
        self.assertEqual(h.node.frame(), item)
        h.advance(.001)
        self.assertEqual(h.node.frame(), (None, ""))

    def test_normal_frame_ttl_remains_point_seven_seconds(self):
        h = WebHarness()
        item = h.receive_frame()
        h.advance(.699)
        self.assertEqual(h.node.frame(), item)
        self.assertTrue(h.node.frame_fresh(item[1]))
        h.advance(.002)
        self.assertEqual(h.node.frame(), (None, ""))
        self.assertFalse(h.node.health()["camera_fresh"])
        self.assertFalse(h.node.frame_fresh(item[1]))

    def test_frame_requires_both_fresh_ros_stamp_and_fresh_local_receipt(self):
        for view_only, max_age in ((False, .7), (True, 5.0)):
            with self.subTest(view_only=view_only, stale="header"):
                h = WebHarness(view_only=view_only)
                h.receive_frame(stamp_age=max_age + .001)
                self.assertEqual(h.node.frame(), (None, ""))
            with self.subTest(view_only=view_only, stale="receipt"):
                h = WebHarness(view_only=view_only)
                h.receive_frame()
                # Keep ROS time frozen to isolate the monotonic deadline.
                h.monotonic_now += max_age + .001
                self.assertEqual(h.node.frame(), (None, ""))

    def test_receipt_from_future_or_header_over_50ms_ahead_is_rejected(self):
        for view_only in (False, True):
            with self.subTest(view_only=view_only, future="receipt"):
                h = WebHarness(view_only=view_only)
                h.receive_frame()
                h.monotonic_now -= .001
                self.assertEqual(h.node.frame(), (None, ""))
            with self.subTest(view_only=view_only, future="header"):
                h = WebHarness(view_only=view_only)
                h.receive_frame(stamp_age=-.051)
                self.assertEqual(h.node.frame(), (None, ""))

    def test_normal_commands_keep_current_stamp_frame_and_velocity_scaling(self):
        h = WebHarness()
        h.node.publish(.5, -.25)
        message = h.node.pub.messages[0]
        self.assertEqual(message.header.frame_id, "base_link")
        self.assertEqual(message.header.stamp.sec * 1_000_000_000
                         + message.header.stamp.nanosec, h.ros_ns)
        self.assertAlmostEqual(message.twist.linear.x, .125)
        self.assertAlmostEqual(message.twist.angular.z, -.45)
        self.assertEqual((message.twist.linear.y, message.twist.linear.z,
                          message.twist.angular.x, message.twist.angular.y),
                         (0.0, 0.0, 0.0, 0.0))
        h.advance(.125)
        h.node.publish(0.0, 0.0)
        stop = h.node.pub.messages[-1]
        self.assertEqual((stop.twist.linear.x, stop.twist.angular.z), (0.0, 0.0))
        self.assertEqual(stop.header.stamp.sec * 1_000_000_000
                         + stop.header.stamp.nanosec, h.ros_ns)
        self.assertEqual(len(h.node.pub.messages), 2)
        h.ros_ok = False
        h.node.publish(1.0, 1.0)
        self.assertEqual(len(h.node.pub.messages), 2)


if __name__ == "__main__":
    unittest.main()
