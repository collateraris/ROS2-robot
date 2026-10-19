"""Simulation-only velocity gate. No serial import, port or physical PWM output exists."""
import math
import time
from .safety import SimulationDriveGate


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from room_base.odom_core import usable_scan

    class SimDrive(Node):
        def __init__(self):
            super().__init__('room_sim_drive')
            if not self.get_parameter('use_sim_time').value:
                raise ValueError('room_sim_drive requires use_sim_time:=true; it is simulation-only')
            defaults = dict(simulation_autonomy=False, command_timeout=.25, sensor_timeout=.35,
                            health_timeout=.20, deadman_timeout=.25, max_linear=.10,
                            max_angular=.40, first_command_wait=10.)
            params = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            self.gate = SimulationDriveGate(**params)
            transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.out = self.create_publisher(Twist, '/sim/cmd_vel', 1)
            self.applied = self.create_publisher(TwistStamped, '/room_base/applied_cmd', 10)
            self.applied_alias = self.create_publisher(TwistStamped, '/room_base/applied_cmd_vel', 10)
            self.armed = self.create_publisher(Bool, '/room_base/armed', transient)
            self.estop = self.create_publisher(Bool, '/room_base/estop_latched', transient)
            self.status = self.create_publisher(String, '/room_base/status', 1)
            self.action_source = self.create_publisher(String, '/room_base/action_source', 1)
            self.marker = self.create_publisher(Bool, '/room_sim/is_simulation', 1)
            self.mode = self.create_publisher(String, '/room_sim/output_mode', transient)
            self.mode.publish(String(data='simulation'))
            self.create_subscription(Twist, '/cmd_vel_safe', self.on_command, 1)
            self.create_subscription(Bool, '/room_base/deadman',
                lambda msg: self.gate.on_deadman(msg.data, time.monotonic()), 1)
            self.create_subscription(Bool, '/room_base/estop',
                lambda msg: self.gate.on_estop(msg.data), transient)
            self.create_subscription(Bool, '/room_base/odom_healthy',
                lambda msg: self.gate.on_health(msg.data, time.monotonic()), 1)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_service(Trigger, '/room_base/arm', self.on_arm)
            self.create_service(Trigger, '/room_base/stop', self.on_stop)
            self.create_service(Trigger, '/room_base/reset_estop', self.on_reset)
            self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info('Gazebo-only drive adapter: output /sim/cmd_vel; explicit ARM required')
            if params['simulation_autonomy']:
                self.get_logger().warning('Simulation test autonomy enabled: human deadman is bypassed after explicit ARM')

        def ros_now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        @staticmethod
        def stamp(msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        def on_scan(self, msg):
            valid = usable_scan(msg.ranges, msg.range_min, msg.range_max, msg.angle_increment,
                                minimum=60, sectors=16, minimum_sectors=8)
            if not valid:
                self.gate.scan.received = -math.inf
                self.gate.disarm('invalid laser scan')
                return
            self.gate.scan.accept(self.stamp(msg), time.monotonic(), self.ros_now(), self.gate.sensor_timeout)

        def on_odom(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w,msg.twist.twist.linear.x,msg.twist.twist.angular.z)
            if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or
                not all(math.isfinite(v) for v in values) or abs(p.z)>.02 or
                abs(q.x)>.01 or abs(q.y)>.01 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
                self.gate.odom.received = -math.inf
                self.gate.disarm('invalid validated RF2O odometry')
                return
            self.gate.odom.accept(self.stamp(msg), time.monotonic(), self.ros_now(), self.gate.sensor_timeout)

        def on_command(self, msg):
            unused = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
            supported = all(math.isfinite(v) and abs(v)<1e-9 for v in unused)
            self.gate.on_command(msg.linear.x, msg.angular.z, time.monotonic(), supported)
            # A collision-monitor zero takes effect immediately, including between ticks.
            if not self.gate.armed or (msg.linear.x == 0 and msg.angular.z == 0):
                self.publish_velocity(0.,0.)

        def on_arm(self, request, response):
            del request
            response.success, response.message = self.gate.arm(time.monotonic(), self.ros_now())
            return response

        def on_stop(self, request, response):
            del request
            self.gate.disarm('operator stop; explicit rearm required')
            self.publish_velocity(0.,0.)
            response.success, response.message = True, self.gate.status
            return response

        def on_reset(self, request, response):
            del request
            response.success, response.message = self.gate.reset_estop()
            return response

        def publish_velocity(self, linear, angular):
            msg = Twist()
            msg.linear.x, msg.angular.z = linear, angular
            self.out.publish(msg)
            applied = TwistStamped()
            applied.header.stamp = self.get_clock().now().to_msg()
            applied.header.frame_id = 'base_link'
            applied.twist = msg
            self.applied.publish(applied)
            self.applied_alias.publish(applied)

        def tick(self):
            self.publish_velocity(*self.gate.output(time.monotonic(), self.ros_now()))
            self.armed.publish(Bool(data=self.gate.armed))
            self.estop.publish(Bool(data=self.gate.estop))
            self.status.publish(String(data=self.gate.status))
            self.marker.publish(Bool(data=True))
            self.action_source.publish(String(data='simulator_accepted_velocity_setpoint'))

    rclpy.init(args=args)
    node = SimDrive()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.gate.disarm('simulation adapter shutdown')
            node.publish_velocity(0.,0.)
            node.marker.publish(Bool(data=False))
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
