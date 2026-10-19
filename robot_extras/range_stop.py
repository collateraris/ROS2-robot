"""Optional single-beam Range stop. Never synthesizes a LaserScan."""
import time
from .geometry import range_clear


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Range
    from std_msgs.msg import Bool

    class RangeStop(Node):
        def __init__(self):
            super().__init__('range_stop')
            self.declare_parameter('topic', '/range')
            self.declare_parameter('stop_distance_m', .4)
            self.declare_parameter('timeout_s', .3)
            self.declare_parameter('positive_infinity_means_clear', False)
            latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(Bool, '/room_base/estop', latch)
            self.create_subscription(Range, self.get_parameter('topic').value,
                                     self.observe, qos_profile_sensor_data)
            self.received, self.good = time.monotonic(), False
            self.create_timer(.05, self.tick)

        def observe(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9 - stamp
            self.received = time.monotonic()
            self.good = (0 <= age <= self.get_parameter('timeout_s').value and
                         range_clear(msg.range, msg.min_range, msg.max_range,
                                     self.get_parameter('stop_distance_m').value,
                                     self.get_parameter('positive_infinity_means_clear').value))

        def tick(self):
            if time.monotonic()-self.received > self.get_parameter('timeout_s').value:
                self.good = False
            if not self.good:
                # False/clear never releases an existing latched stop.
                self.pub.publish(Bool(data=True))

    rclpy.init()
    node = RangeStop()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Bool(data=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
