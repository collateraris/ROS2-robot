"""ROS 2 <-> phone gateway. All ROS topics are relative to the launch namespace."""
import asyncio
import math
import os
import secrets
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from aiohttp import web

from .web_server import RobotServer


class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        defaults = {'bind_host': '0.0.0.0', 'port': 8080, 'view_only': False,
                    'max_linear_mps': 0.25, 'max_angular_rps': 1.8}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        if not 1 <= self.get_parameter('port').value <= 65535:
            raise ValueError('HTTP port must be between 1 and 65535')
        for key in ('max_linear_mps', 'max_angular_rps'):
            value = self.get_parameter(key).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.lock = threading.Lock()
        # Snapshot this mode at startup. Viewing slow frames must never create
        # a command publisher, even if a separate drive/ready publisher exists.
        self.view_only = bool(self.get_parameter('view_only').value)
        self.frame_max_age = 5.0 if self.view_only else 0.7
        self.ready, self.ready_at = False, float('-inf')
        self.latest = None
        self.frames = {}
        self.pub = None
        if not self.view_only:
            self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
            self.create_subscription(Bool, 'drive/ready', self.on_ready, 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = ('camera/preview/compressed' if self.view_only
                       else 'camera/image_raw/compressed')
        self.create_subscription(CompressedImage, image_topic, self.on_frame, qos)

    def on_ready(self, msg):
        with self.lock:
            self.ready, self.ready_at = msg.data, time.monotonic()

    def on_frame(self, msg):
        if 'jpeg' not in msg.format.lower() or not 4 <= len(msg.data) <= 2_000_000:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now, token = time.monotonic(), secrets.token_urlsafe(12)
        with self.lock:
            self.frames = {k: v for k, v in self.frames.items()
                           if now - v[0] <= self.frame_max_age}
            self.frames[token] = (now, stamp)
            self.latest = (bytes(msg.data), token)

    def _fresh(self, item, max_age=0.7):
        if item is None:
            return False
        monotonic_stamp, ros_stamp = item
        age = (self.get_clock().now().nanoseconds - ros_stamp) / 1e9
        return (ros_stamp > 0 and -0.05 <= age <= max_age
                and 0 <= time.monotonic() - monotonic_stamp <= max_age)

    def health(self):
        with self.lock:
            frame = self.frames.get(self.latest[1]) if self.latest else None
            return {'drive_ready': bool(not self.view_only and self.ready
                                       and time.monotonic() - self.ready_at <= 0.35),
                    'camera_fresh': self._fresh(frame)}

    def frame(self):
        with self.lock:
            if self.latest and self._fresh(self.frames.get(self.latest[1]),
                                            self.frame_max_age):
                return self.latest
            return None, ''

    def frame_fresh(self, token):
        with self.lock:
            return isinstance(token, str) and self._fresh(self.frames.get(token))

    def publish(self, linear, angular):
        if self.view_only or self.pub is None or not rclpy.ok():
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear * self.get_parameter('max_linear_mps').value
        msg.twist.angular.z = angular * self.get_parameter('max_angular_rps').value
        self.pub.publish(msg)


async def serve(node):
    filename = 'camera_view.html' if node.view_only else 'index.html'
    page = Path(get_package_share_directory('robot_web_teleop')) / 'web' / filename
    server = RobotServer(node, page, os.environ.get('ROBOT_PASSWORD', ''),
                         view_only=node.view_only)
    runner = web.AppRunner(server.app, access_log=None)
    await runner.setup()
    try:
        port = node.get_parameter('port').value
        await web.TCPSite(runner, node.get_parameter('bind_host').value, port).start()
        node.get_logger().info(f'Phone UI: http://<Raspberry-Pi-LAN-IP>:{port}')
        if node.view_only:
            node.get_logger().info('VIEW ONLY: no cmd_vel publisher; motor bridge is not needed')
        while rclpy.ok():
            if server.ticker_task.done():
                await server.ticker_task  # Surface its original exception.
                raise RuntimeError('Control ticker stopped unexpectedly')
            await asyncio.sleep(0.1)
    finally:
        await runner.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        asyncio.run(serve(node))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.publish(0.0, 0.0)
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
