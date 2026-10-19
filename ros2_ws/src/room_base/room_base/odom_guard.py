"""Only validated scan-stamped RF2O poses publish /odom and odom -> base_link."""
import math
import time
from collections import deque
from .odom_core import OdomGate, PoseSample, usable_scan


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.clock import Clock, ClockType
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import Bool, String
    from tf2_ros import TransformBroadcaster

    class OdomGuard(Node):
        def __init__(self):
            super().__init__('room_odom_guard')
            defaults = dict(raw_odom_topic='/odom_rf2o', scan_topic='/scan',
                            odom_topic='/odom', odom_frame='odom', base_frame='base_link',
                            scan_timeout=.35, odom_timeout=.35, max_gap=.40,
                            max_linear=.40, max_angular=1.20, position_slack=.015,
                            angle_slack=.04, good_frames=3, stamp_tolerance=.002,
                            future_tolerance=.03, minimum_returns=60,
                            sectors=16, minimum_sectors=8, publish_tf=True)
            for key, value in defaults.items():
                self.declare_parameter(key, value)
            self.p = {key:self.get_parameter(key).value for key in defaults}
            gate_keys = ('scan_timeout','odom_timeout','max_gap','max_linear','max_angular',
                         'position_slack','angle_slack','good_frames','stamp_tolerance',
                         'future_tolerance','odom_frame','base_frame')
            self.gate = OdomGate(**{k:self.p[k] for k in gate_keys})
            self.pending = deque(maxlen=8)
            self.output = self.create_publisher(Odometry, self.p['odom_topic'], 10)
            self.health = self.create_publisher(Bool, '/room_base/odom_healthy', 1)
            self.status = self.create_publisher(String, '/room_base/odom_status', 1)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, self.p['raw_odom_topic'], self.on_odom, qos_profile_sensor_data)
            self.create_timer(.05, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.report()

        @staticmethod
        def seconds(stamp):
            return stamp.sec+stamp.nanosec*1e-9

        def now_ros(self):
            return self.get_clock().now().nanoseconds*1e-9

        def report(self):
            self.drain_pending()
            healthy = self.gate.healthy(time.monotonic(), self.now_ros())
            self.health.publish(Bool(data=healthy))
            self.status.publish(String(data=self.gate.latched_fault or self.gate.reason))

        def on_scan(self, msg):
            self.gate.add_scan(self.seconds(msg.header.stamp), time.monotonic(),
                usable_scan(msg.ranges, msg.range_min, msg.range_max, msg.angle_increment,
                            self.p['minimum_returns'], self.p['sectors'], self.p['minimum_sectors']))
            self.report()

        def on_odom(self, msg):
            # DDS does not guarantee ordering between /scan and /odom_rf2o.
            # Allow a bounded wait for the matching scan without replacing its source timestamp.
            if len(self.pending) == self.pending.maxlen:
                self.pending.popleft()
                self.gate.reject('pending odometry queue overflow')
            self.pending.append((msg,time.monotonic()))
            self.report()

        def drain_pending(self):
            while self.pending:
                msg, received = self.pending[0]
                stamp = self.seconds(msg.header.stamp)
                matches = any(abs(s.stamp-stamp)<=self.p['stamp_tolerance'] for s in self.gate.scans)
                if not matches:
                    if time.monotonic()-received <= .10:
                        return
                    self.pending.popleft()
                    self.gate.reject('matching laser scan did not arrive within 100 ms')
                    continue
                self.pending.popleft()
                self.accept_message(msg,received)

        def accept_message(self, msg, received):
            q, pos = msg.pose.pose.orientation, msg.pose.pose.position
            values = (q.x,q.y,q.z,q.w,pos.x,pos.y,pos.z)
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[:4])-1.)>.01 or abs(pos.z)>.02 or abs(q.x)>.01 or abs(q.y)>.01:
                self.gate.reject('nonplanar or invalid quaternion/position', latch=True)
                return
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            sample = PoseSample(self.seconds(msg.header.stamp), pos.x, pos.y, yaw,
                                msg.header.frame_id, msg.child_frame_id)
            result = self.gate.accept(sample, received, self.now_ros(), monotonic_now=time.monotonic())
            if result is not False:
                # Preserve acquisition stamp. Zero upstream covariance is NOT treated as exact data.
                msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = result
                msg.twist.twist.linear.z = msg.twist.twist.angular.x = msg.twist.twist.angular.y = 0.
                msg.pose.covariance = [0.]*36
                msg.twist.covariance = [0.]*36
                for index in (14,21,28):
                    msg.pose.covariance[index] = msg.twist.covariance[index] = 1e6
                for index,value in ((0,.02),(7,.02),(35,.04)):
                    msg.pose.covariance[index] = value
                for index,value in ((0,.04),(7,.04),(35,.09)):
                    msg.twist.covariance[index] = value
                self.output.publish(msg)
                if self.p['publish_tf']:
                    tf = TransformStamped()
                    tf.header, tf.child_frame_id = msg.header, msg.child_frame_id
                    tf.transform.translation.x, tf.transform.translation.y = pos.x,pos.y
                    tf.transform.rotation = q
                    self.tf.sendTransform(tf)

    rclpy.init(args=args)
    node = OdomGuard()
    try:
        rclpy.spin(node)
    finally:
        node.health.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
