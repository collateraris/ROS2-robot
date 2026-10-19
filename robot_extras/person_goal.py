"""Trusted metric person pose -> one Nav2 goal. No detection/depth fabrication.

Only a stationary, unambiguous selected person is supported. A moved/lost person
latches the base stop; restarting requires a new task and explicit base arm.
"""
import math
import time
from .geometry import standoff_goal


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    class PersonGoal(Node):
        def __init__(self):
            super().__init__('person_goal_adapter')
            for name, value in [('trusted_metric_source', False), ('standoff_m', 1.0),
                                ('max_age_s', .5), ('target_jump_m', .3), ('max_task_s', 15.0)]:
                self.declare_parameter(name, value)
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.goal_pub = self.create_publisher(PoseStamped, '/room_nav/goal', 1)
            latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.stop_pub = self.create_publisher(Bool, '/room_base/estop', latch)
            self.status = self.create_publisher(String, '/room_nav/person_status', 1)
            self.create_subscription(String, '/room_nav/text_command', self.command, 1)
            self.create_subscription(PoseStamped, '/room_nav/person_pose', self.person, 1)
            self.create_timer(.1, self.tick)
            self.active = False
            self.sent = False
            self.target = None
            self.received = -math.inf
            self.target_stamp = -math.inf
            self.started = -math.inf
            self.initial = None

        def p(self, name):
            return self.get_parameter(name).value

        def report(self, text):
            self.status.publish(String(data=text))

        def stop(self, reason):
            self.active = False
            self.stop_pub.publish(Bool(data=True))
            self.report(reason)

        def command(self, msg):
            text = ' '.join(msg.data.lower().strip().split())
            if text in ('стоп', 'stop', 'остановись'):
                self.active = False
                return
            if text not in ('к человеку', 'подъедь к человеку'):
                return
            if not self.p('trusted_metric_source'):
                self.report('No trusted metric person source configured')
                return
            self.active, self.sent = True, False
            self.initial = None
            self.started = time.monotonic()

        def person(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            now = self.get_clock().now().nanoseconds*1e-9
            xy = (msg.pose.position.x, msg.pose.position.y)
            if (msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in xy)
                    or not 0 <= now-stamp <= self.p('max_age_s') or stamp <= self.target_stamp):
                if self.active:
                    self.stop('Invalid/stale/ambiguous person observation')
                return
            if self.active and self.initial is not None:
                if math.dist(xy, self.initial) > self.p('target_jump_m'):
                    self.stop('Person moved: stationary-person task cancelled')
                    return
            self.target, self.target_stamp = xy, stamp
            self.received = time.monotonic()

        def tick(self):
            if not self.active:
                return
            now = time.monotonic()
            rosnow = self.get_clock().now().nanoseconds*1e-9
            if now-self.started > self.p('max_task_s'):
                self.stop('Person task deadline')
                return
            if self.target is None or now-self.received > self.p('max_age_s'):
                if now-self.started > self.p('max_age_s'):
                    self.stop('Person lost')
                return
            if not 0 <= rosnow-self.target_stamp <= self.p('max_age_s'):
                self.stop('Person pose timestamp expired')
                return
            if self.sent:
                return
            try:
                # Current robot pose for goal construction; person pose is already
                # grounded in map at its own acquisition timestamp by its producer.
                tr = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                t = tr.header.stamp.sec + tr.header.stamp.nanosec*1e-9
                if not 0 <= rosnow-t <= .5:
                    raise ValueError('Stale robot localization')
                goal = standoff_goal((tr.transform.translation.x, tr.transform.translation.y),
                                     self.target, self.p('standoff_m'))
                if goal is None:
                    self.stop('Already at standoff')
                    return
                out = PoseStamped()
                out.header.frame_id = 'map'
                out.header.stamp = self.get_clock().now().to_msg()
                out.pose.position.x, out.pose.position.y = goal[:2]
                out.pose.orientation.z = math.sin(goal[2]/2)
                out.pose.orientation.w = math.cos(goal[2]/2)
                self.goal_pub.publish(out)
                self.sent, self.initial = True, self.target
                self.report('One-shot person standoff goal sent through Nav2')
            except Exception as exc:
                self.stop('Person goal rejected: '+str(exc))

    rclpy.init()
    node = PersonGoal()
    try:
        rclpy.spin(node)
    finally:
        if node.active:
            node.stop('Person adapter shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
