"""ROS 2 Jazzy bridge for protocol v4 (firmware room_base_v3).

Actuation is opt-in; counts are measured, accepted setpoints are not odometry.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy)
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .core import (EncoderOdometry, body_velocity, frame, fresh, parse_frame,
                   uint32, wheel_targets)


class BaseBridge(Node):
    def __init__(self):
        super().__init__('room_base')
        defaults = dict(allow_actuation=False, calibration_confirmed=False,
                        serial_port='/dev/serial/by-id/REPLACE_ME', serial_baud=115200,
                        wheel_separation_m=.160, ticks_per_meter=1000.,
                        max_wheel_mm_s=150, max_linear_m_s=.10, max_angular_rad_s=.50,
                        command_timeout_s=.25, deadman_timeout_s=.25,
                        scan_timeout_s=.25, feedback_timeout_s=.20, ack_timeout_s=.15,
                        odom_frame='odom', base_frame='base_link', publish_tf=True)
        self.p = {}
        for name, value in defaults.items():
            self.p[name] = self.declare_parameter(name, value).value
        if self.p['wheel_separation_m'] <= 0 or self.p['ticks_per_meter'] <= 0:
            raise ValueError('wheel geometry must be positive')
        self.odo = EncoderOdometry(self.p['wheel_separation_m'], self.p['ticks_per_meter'])
        self.ser = None
        self.rx = bytearray()
        self.boot = None
        self.nonce = None
        self.mcu_nonce = 0
        self.pending_arm = None
        self.pending = None
        self.seq = 0
        self.armed = False
        self.estop = False
        self.deadman = False
        self.deadman_at = self.scan_at = self.feedback_at = -math.inf
        self.scan_stamp_ns = None
        self.command = None
        self.command_at = -math.inf
        self.last_send = self.last_query = self.last_open = -math.inf
        self.status_text = 'actuation disabled by default'
        self.applied_pub = self.create_publisher(TwistStamped, '/room_base/applied_cmd', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        latch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.armed_pub = self.create_publisher(Bool, '/room_base/armed', latch_qos)
        self.estop_pub = self.create_publisher(Bool, '/room_base/estop_latched', latch_qos)
        self.status_pub = self.create_publisher(String, '/room_base/status', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_command, 1)
        self.create_subscription(Bool, '/room_base/deadman', self.on_deadman, 1)
        self.create_subscription(Bool, '/room_base/estop', self.on_estop, latch_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/room_base/arm', self.on_arm)
        self.create_service(Trigger, '/room_base/stop', self.on_stop)
        self.create_service(Trigger, '/room_base/reset_estop', self.on_reset_estop)
        self.create_timer(.01, self.tick)
        self.create_timer(.05, self.publish_state)
        if not self.p['allow_actuation']:
            self.get_logger().warning('Dry run: serial port is NOT opened. Set allow_actuation only after hardware checks.')

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(x) for x in values) or any(abs(x) > 1e-9 for x in
                (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            self.disarm('nonfinite or unsupported Twist component')
            return
        if not self.armed:
            return  # no command cached across arming
        self.command = (max(-self.p['max_linear_m_s'], min(self.p['max_linear_m_s'], msg.linear.x)),
                        max(-self.p['max_angular_rad_s'], min(self.p['max_angular_rad_s'], msg.angular.z)))
        self.command_at = time.monotonic()

    def on_deadman(self, msg):
        self.deadman = msg.data
        self.deadman_at = time.monotonic()
        if not msg.data:
            self.disarm('deadman released')

    def on_estop(self, msg):
        if msg.data:
            self.estop = True
            self.disarm('emergency stop latched')

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if (stamp <= 0 or not fresh(age, self.p['scan_timeout_s']) or
                not msg.ranges or not any(math.isfinite(x) and msg.range_min <= x <= msg.range_max
                                          for x in msg.ranges)):
            self.scan_at = -math.inf
            return
        self.scan_at = time.monotonic()
        self.scan_stamp_ns = stamp

    def safety_reason(self, now):
        if not self.p['allow_actuation']: return 'allow_actuation is false'
        if not self.p['calibration_confirmed']: return 'calibration_confirmed is false'
        if self.estop: return 'emergency stop latched'
        if self.ser is None or self.boot is None: return 'new firmware handshake unavailable'
        if not self.deadman or not fresh(now - self.deadman_at, self.p['deadman_timeout_s']):
            return 'deadman missing or stale'
        if not fresh(now - self.scan_at, self.p['scan_timeout_s']): return 'scan receive timeout'
        if self.scan_stamp_ns is None or not fresh(
                (self.get_clock().now().nanoseconds - self.scan_stamp_ns) / 1e9,
                self.p['scan_timeout_s']): return 'scan acquisition timestamp stale'
        if not fresh(now - self.feedback_at, self.p['feedback_timeout_s']):
            return 'measured encoder feedback stale'
        return ''

    def on_arm(self, request, response):
        del request
        reason = self.safety_reason(time.monotonic())
        if self.armed or self.pending_arm:
            response.success = False
            response.message = 'already armed or handshake pending'
        elif reason:
            response.success = False
            response.message = reason
        else:
            if self.mcu_nonce >= 0xffffffff:
                response.success = False
                response.message = 'MCU session counter exhausted; controlled power cycle required'
                return response
            self.nonce = self.mcu_nonce + 1
            self.pending_arm = time.monotonic()
            self.command = None
            self.command_at = -math.inf
            self.seq = 0
            if self.send(f'H {self.boot} {self.nonce}'):
                response.success = True
                response.message = 'arm requested; wait for /room_base/armed=true; then send NEW cmd_vel_safe'
            else:
                response.success = False
                response.message = 'serial write failed'
        return response

    def on_stop(self, request, response):
        del request
        self.disarm('operator stop')
        response.success, response.message = True, 'stop sent; rearm required; verify physical standstill'
        return response

    def on_reset_estop(self, request, response):
        del request
        if self.armed or self.pending_arm:
            response.success, response.message = False, 'stop before resetting latch'
        else:
            self.estop = False
            response.success, response.message = True, 'latch reset; robot remains disarmed'
        return response

    def disarm(self, reason):
        was_live = self.armed or self.pending_arm is not None or self.pending is not None
        self.armed = False
        self.pending_arm = self.pending = None
        self.command = None
        self.status_text = reason
        # A STOP does not manufacture an acknowledged zero setpoint.
        if was_live and self.ser:
            self.send('S')
        self.nonce = None

    def close_serial(self, reason):
        self.disarm(reason)
        if self.ser:
            try: self.ser.close()
            except Exception: pass
        self.ser = None
        self.boot = None
        self.rx.clear()
        self.odo.previous = None
        self.feedback_at = -math.inf

    def send(self, body):
        if not self.ser: return False
        try:
            data = frame(body)
            if self.ser.write(data) != len(data):
                raise OSError('partial serial write')
            return True
        except Exception as exc:
            # Avoid recursively trying STOP on an already failed connection.
            self.armed = False
            self.pending_arm = self.pending = None
            self.nonce = None
            self.command = None
            self.status_text = f'serial write failed: {exc}'
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            self.boot = None
            self.feedback_at = -math.inf
            self.odo.previous = None
            return False

    def tick(self):
        now = time.monotonic()
        if not self.p['allow_actuation']: return
        if self.ser is None:
            if now - self.last_open < 2.: return
            self.last_open = now
            try:
                import serial
                self.ser = serial.Serial(self.p['serial_port'], self.p['serial_baud'],
                                         timeout=0, write_timeout=.02, exclusive=True)
                self.rx.clear()
                self.status_text = 'connected; disarmed; waiting for protocol v4'
                self.last_query = -math.inf
            except Exception as exc:
                self.status_text = f'cannot open configured serial port: {exc}'
                return
        try:
            self.rx.extend(self.ser.read(min(self.ser.in_waiting, 4096)))
            if len(self.rx) > 4096: raise ValueError('serial backlog')
            for _ in range(32):
                if b'\n' not in self.rx: break
                raw, _, rest = self.rx.partition(b'\n')
                self.rx = bytearray(rest)
                self.handle(parse_frame(bytes(raw) + b'\n'), now)
            if len(self.rx) > 160: raise ValueError('oversize serial line')
        except Exception as exc:
            self.close_serial(f'protocol or serial fault: {exc}')
            return
        if self.ser is None: return
        if self.boot is None and now - self.last_query >= .5:
            self.last_query = now
            self.send('Q')
        if self.armed or self.pending_arm:
            reason = self.safety_reason(now)
            if reason:
                self.disarm(reason)
                return
        if self.pending_arm and now - self.pending_arm > self.p['ack_timeout_s']:
            self.disarm('arm acknowledgement timeout')
            return
        if not self.armed: return
        if not fresh(now - self.command_at, self.p['command_timeout_s']):
            # A short grace after R allows the controller to observe the armed state.
            if now - self.armed_at > self.p['command_timeout_s']:
                self.disarm('command timeout; explicit rearm required')
            return
        if self.pending:
            if now - self.pending[3] > self.p['ack_timeout_s']:
                self.disarm('wheel command acknowledgement timeout')
            return
        if now - self.last_send >= .05:
            if self.seq >= 0xffffffff:
                self.disarm('sequence exhausted; rearm required')
                return
            self.seq += 1
            left, right = wheel_targets(*self.command, self.p['wheel_separation_m'], self.p['max_wheel_mm_s'])
            self.pending = (self.seq, left, right, now)
            self.last_send = now
            self.send(f'C {self.boot} {self.nonce} {self.seq} {left} {right}')

    def handle(self, fields, now):
        kind = fields[0]
        if kind == 'V' and len(fields) == 6:
            version, boot, limit, watchdog, ticks = map(uint32, fields[1:])
            if version != 4 or not boot or limit < self.p['max_wheel_mm_s'] or watchdog != 250:
                raise ValueError('wrong firmware protocol/limits')
            if abs(ticks - self.p['ticks_per_meter']) > 1e-6:
                raise ValueError('MCU/host ticks_per_meter mismatch')
            if self.boot != boot:
                self.disarm('MCU boot identity changed; rearm required')
                self.odo.previous = None
                self.feedback_at = -math.inf
                self.mcu_nonce = 0
            self.boot = boot
        elif kind == 'R' and len(fields) == 3:
            boot, nonce = map(uint32, fields[1:])
            if not self.pending_arm or boot != self.boot or nonce != self.nonce:
                raise ValueError('unexpected arm acknowledgement')
            self.pending_arm = None
            self.mcu_nonce = max(self.mcu_nonce, nonce)
            self.armed = True
            self.armed_at = now
            self.status_text = 'armed; requires fresh command, scan, deadman and encoders'
            self.publish_state()
        elif kind == 'A' and len(fields) == 7:
            boot, nonce, seq = map(uint32, fields[1:4])
            left, right = int(fields[4]), int(fields[5])
            uint32(fields[6])  # MCU acceptance tick, not a ROS timestamp
            if (not self.armed or not self.pending or boot != self.boot or nonce != self.nonce or
                    (seq, left, right) != self.pending[:3]):
                raise ValueError('unknown/mismatched wheel acknowledgement')
            self.pending = None
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p['base_frame']
            msg.twist.linear.x, msg.twist.angular.z = body_velocity(left, right, self.p['wheel_separation_m'])
            self.applied_pub.publish(msg)
        elif kind == 'F' and len(fields) == 7:
            boot, nonce, tick, left, right, active = map(uint32, fields[1:])
            if self.boot is None: return  # wait for version and boot identity first
            if boot != self.boot: raise ValueError('MCU reset during session')
            self.mcu_nonce = max(self.mcu_nonce, nonce)
            if active not in (0, 1): raise ValueError('bad MCU active flag')
            if self.armed and (nonce != self.nonce or active != 1):
                self.disarm('MCU watchdog/stop disarmed base')
            before = self.odo.previous
            velocity = self.odo.update(tick, left, right)
            if before is None or velocity is not None:
                self.feedback_at = now
            if velocity is not None:
                self.publish_odometry(*velocity)
        elif kind == 'D' and len(fields) == 1:
            self.disarm('MCU reported disarmed')
        elif kind == 'E' and len(fields) == 1:
            raise ValueError('MCU rejected command')
        else:
            raise ValueError('unknown or wrong-length firmware response')

    def publish_odometry(self, linear, angular):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = self.p['odom_frame'], self.p['base_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.odo.x, self.odo.y
        msg.pose.pose.orientation.z = math.sin(.5 * self.odo.yaw)
        msg.pose.pose.orientation.w = math.cos(.5 * self.odo.yaw)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = linear, angular
        # Conservative starting values; measure/tune them on the actual floor.
        for index, value in ((0,.02),(7,.02),(14,1e6),(21,1e6),(28,1e6),(35,.05)):
            msg.pose.covariance[index] = value
            msg.twist.covariance[index] = value
        self.odom_pub.publish(msg)
        if self.p['publish_tf']:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = msg.header, msg.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = self.odo.x, self.odo.y
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(tf)

    def publish_state(self):
        self.armed_pub.publish(Bool(data=self.armed))
        self.estop_pub.publish(Bool(data=self.estop))
        self.status_pub.publish(String(data=self.status_text))


def main(args=None):
    rclpy.init(args=args)
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial('node shutdown')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
