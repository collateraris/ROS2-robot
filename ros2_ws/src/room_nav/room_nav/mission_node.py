"""Explicit missions only: exact waypoint grammar, fresh metric goals, Nav2 action."""
from math import sin, cos, isfinite
from pathlib import Path
import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
from .core import parse_command, validate_waypoints


class MissionNode(Node):
    def __init__(self):
        super().__init__('room_mission')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('enable_waypoints', False)
        self.declare_parameter('enable_external_goals', False)
        self.declare_parameter('max_external_goal_age_s', 1.0)
        path = self.get_parameter('waypoints_file').value
        self.waypoints = {}
        if path:
            with Path(path).open(encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
            if data.get('frame_id', 'map') != 'map':
                raise ValueError('Waypoint coordinates must use map frame')
            self.waypoints = validate_waypoints(data)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._generation = 0
        self._pending = False
        self._goal_handle = None
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.estop = self.create_publisher(Bool, '/room_base/estop', latch)
        self.status = self.create_publisher(String, '/room_nav/status', 10)
        self.instruction = self.create_publisher(String, '/room_nav/instruction', latch)
        self.demo_target = self.create_publisher(String, '/room_nav/demo_target', latch)
        self.demo_context = self.create_publisher(String, '/room_nav/demo_context', latch)
        self.context('', '')
        self.create_subscription(String, '/room_nav/text_command', self.command, 10)
        self.create_subscription(PoseStamped, '/room_nav/goal', self.external_goal, 1)
        self.say('READY: motors require independent deadman + /room_base/arm; STOP always available')

    def say(self, message):
        self.status.publish(String(data=message))
        self.get_logger().info(message)

    def context(self, instruction, target):
        # Atomic context is authoritative for recordings; independent topics are convenience only.
        self.demo_context.publish(String(data=json.dumps(
            {'instruction': instruction, 'target': target, 'generation': self._generation},
            ensure_ascii=False)))
        self.demo_target.publish(String(data=target))
        self.instruction.publish(String(data=instruction))

    def clear_label(self):
        self.context('', '')

    def stop(self):
        self._generation += 1
        # Publish immediate latched hardware-layer stop before waiting for action cancellation.
        self.estop.publish(Bool(data=True))
        self.context('стоп', 'STOP')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        # A pending accepted goal is cancelled by _accepted using the generation token.
        self.say('STOP: estop latched; navigation canceled; explicit physical/operator re-arm required')

    def command(self, message):
        kind, target = parse_command(message.data, self.waypoints)
        if kind == 'stop':
            self.stop()
            return
        if kind != 'waypoint':
            self.say('REJECTED: unknown waypoint/command. Use a configured alias or стоп. Person adapter is separate.')
            return
        if not self.get_parameter('enable_waypoints').value:
            self.say('REJECTED: waypoint navigation disabled; replace example coordinates then enable_waypoints')
            return
        x, y, yaw = self.waypoints[target]['pose']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = sin(yaw / 2.0), cos(yaw / 2.0)
        self.send(pose, message.data, target)

    def external_goal(self, pose):
        if not self.get_parameter('enable_external_goals').value:
            self.say('REJECTED: external goals disabled')
            return
        stamp_ns = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        p, q = pose.pose.position, pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        norm = sum(v*v for v in [q.x, q.y, q.z, q.w])
        if (pose.header.frame_id != 'map' or stamp_ns <= 0 or age_s < -0.1
                or age_s > self.get_parameter('max_external_goal_age_s').value
                or not all(isfinite(v) for v in values) or abs(p.z) > 0.05
                or abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or abs(norm - 1.0) > 0.01):
            self.say('REJECTED: external goal needs fresh map-frame finite planar pose and unit quaternion')
            return
        # External perception needs its own grounded metric localization and user-intent gate.
        self.send(pose, 'external metric goal', '')

    def send(self, pose, instruction, target):
        if self._pending or self._goal_handle is not None:
            self.say('REJECTED: navigation busy; cancel current mission with стоп first')
            return
        if not self.nav.wait_for_server(timeout_sec=0.0):
            self.say('REJECTED: NavigateToPose action server not ready; check lifecycle and initial pose')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._generation += 1
        generation = self._generation
        self._pending = True
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._accepted(f, generation, instruction, target))

    def _accepted(self, future, generation, instruction, target):
        self._pending = False
        try:
            handle = future.result()
        except Exception as exc:
            if generation == self._generation:
                self.clear_label()
            self.say(f'ERROR sending goal: {exc}')
            return
        if not handle.accepted:
            if generation == self._generation:
                self.clear_label()
            self.say('REJECTED by Nav2')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._result(f, handle, generation))
        if generation != self._generation:
            handle.cancel_goal_async()
            return
        self.context(instruction, target)
        self.say(f'NAVIGATING: {target or "external metric goal"}')

    def _result(self, future, handle, generation):
        if self._goal_handle is handle:
            self._goal_handle = None
        try:
            status = future.result().status
            self.say(f'NAVIGATION FINISHED: action status={status} (4=success, 5=canceled, 6=aborted)')
        except Exception as exc:
            self.say(f'NAVIGATION ERROR: {exc}')
        if generation == self._generation:
            self.clear_label()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
