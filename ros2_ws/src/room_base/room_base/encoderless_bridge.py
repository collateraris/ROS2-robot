"""ROS 2 Jazzy encoderless bridge, protocol v5 (room_base_rf2o firmware).

RF2O + the independent guard publish all odometry/TF. PWM calibration is open
loop. Applied commands are ACK-accepted NOMINAL speeds, never measurements.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy)
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .core import frame, fresh, parse_frame, uint32
from .encoderless_core import OpenLoopMapping, SpeedCurve, SafetyState


class EncoderlessBridge(Node):
    def __init__(self):
        super().__init__('room_base')
        defaults = dict(allow_actuation=False, calibration_confirmed=False,
                        serial_port='/dev/serial/by-id/REPLACE_ME', serial_baud=115200,
                        wheel_separation_m=.160,
                        max_wheel_mm_s=150, max_linear_m_s=.10, max_angular_rad_s=.50,
                        command_timeout_s=.25, deadman_timeout_s=.25,
                        scan_timeout_s=.30, odom_timeout_s=.35, health_timeout_s=.20,
                        feedback_timeout_s=.20, ack_timeout_s=.15,
                        odom_frame='odom', base_frame='base_link')
        for side in ('left', 'right'):
            for direction in ('forward', 'reverse'):
                for key, value in dict(min_mm_s=20., max_mm_s=150., min_pwm=60, max_pwm=120).items():
                    defaults[f'{side}_{direction}_{key}'] = value
        self.p = {}
        for name, value in defaults.items():
            self.p[name] = self.declare_parameter(name, value).value
        curves = [SpeedCurve(**{key: self.p[f'{side}_{direction}_{key}']
                                  for key in ('min_mm_s', 'max_mm_s', 'min_pwm', 'max_pwm')})
                  for side in ('left', 'right') for direction in ('forward', 'reverse')]
        self.mapping = OpenLoopMapping(*curves, self.p['wheel_separation_m'],
                                       self.p['max_wheel_mm_s'])
        for key in ('max_linear_m_s', 'max_angular_rad_s', 'command_timeout_s',
                    'deadman_timeout_s', 'scan_timeout_s', 'odom_timeout_s',
                    'health_timeout_s', 'feedback_timeout_s', 'ack_timeout_s'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.health = SafetyState()
        self.odom_at = self.health_at = -math.inf
        self.odom_healthy = False
        self.odom_stamp_ns = None
        self.ser = None
        self.rx = bytearray()
        self.boot = None
        self.nonce = None
        self.mcu_nonce = 0
        self.pending_arm = None
        self.pending = {}
        self.last_output = None
        self.seq = 0
        self.armed = False
        self.estop = False
        self.deadman = False
        self.deadman_at = self.scan_at = self.feedback_at = -math.inf
        self.feedback_tick = None
        self.scan_stamp_ns = None
        self.command = None
        self.command_at = -math.inf
        self.last_send = self.last_query = self.last_open = -math.inf
        self.status_text = 'actuation disabled by default'
        self.applied_pub = self.create_publisher(TwistStamped, '/room_base/applied_cmd', 10)
        # Compatibility topic and descriptive alias contain accepted nominal setpoints.
        self.applied_vel_pub = self.create_publisher(TwistStamped, '/room_base/applied_cmd_vel', 10)
        latch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.armed_pub = self.create_publisher(Bool, '/room_base/armed', latch_qos)
        self.estop_pub = self.create_publisher(Bool, '/room_base/estop_latched', latch_qos)
        self.status_pub = self.create_publisher(String, '/room_base/status', 1)
        self.source_pub = self.create_publisher(String, '/room_base/action_source', 1)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_command, 1)
        self.create_subscription(Bool, '/room_base/deadman', self.on_deadman, 1)
        self.create_subscription(Bool, '/room_base/estop', self.on_estop, latch_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/room_base/odom_healthy', self.on_odom_health, 1)
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
        if self.command == (0., 0.) and self.last_output != (0, 0):
            # Zero from Collision Monitor supersedes one nonzero command without
            # waiting for its ACK. Normal zero keeps the explicit ARM session.
            if len(self.pending) >= 2:
                self.disarm('too many commands awaiting acknowledgement')
            else:
                self.send_setpoint(self.command_at, self.command)

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
        if self.scan_stamp_ns is not None and stamp <= self.scan_stamp_ns:
            return  # duplicates cannot refresh the safety clock
        self.scan_at = time.monotonic()
        self.scan_stamp_ns = stamp

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        q = msg.pose.pose.orientation
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                  q.x, q.y, q.z, q.w, msg.twist.twist.linear.x,
                  msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if (stamp <= 0 or not fresh(age, self.p['odom_timeout_s']) or
                msg.header.frame_id != self.p['odom_frame'] or
                msg.child_frame_id != self.p['base_frame'] or
                not all(math.isfinite(x) for x in values) or
                not .95 <= sum(x*x for x in (q.x, q.y, q.z, q.w)) <= 1.05):
            self.odom_at = -math.inf
            self.disarm('invalid RF2O guarded odometry')
            return
        if self.odom_stamp_ns is not None and stamp <= self.odom_stamp_ns:
            return
        self.odom_at = time.monotonic()
        self.odom_stamp_ns = stamp

    def on_odom_health(self, msg):
        self.odom_healthy = bool(msg.data)
        self.health_at = time.monotonic()
        if not self.odom_healthy:
            self.disarm('RF2O odometry health lost; explicit rearm required')

    def safety_reason(self, now):
        ros_ns = self.get_clock().now().nanoseconds
        self.health = SafetyState(
            allow_actuation=self.p['allow_actuation'],
            calibration_confirmed=self.p['calibration_confirmed'],
            estop=self.estop, connected=self.ser is not None, boot_known=self.boot is not None,
            deadman=self.deadman, odom_healthy=self.odom_healthy,
            deadman_at=self.deadman_at, scan_at=self.scan_at,
            scan_age=(ros_ns-self.scan_stamp_ns)/1e9 if self.scan_stamp_ns else math.inf,
            odom_at=self.odom_at,
            odom_age=(ros_ns-self.odom_stamp_ns)/1e9 if self.odom_stamp_ns else math.inf,
            health_at=self.health_at, feedback_at=self.feedback_at)
        return self.health.reason(now, self.p)

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
        was_live = self.armed or self.pending_arm is not None or bool(self.pending)
        self.armed = False
        self.pending_arm = None
        self.pending = {}
        self.last_output = None
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
        self.feedback_at = -math.inf
        self.feedback_tick = None

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
            self.pending_arm = None
            self.pending = {}
            self.last_output = None
            self.nonce = None
            self.command = None
            self.status_text = f'serial write failed: {exc}'
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            self.boot = None
            self.feedback_at = -math.inf
            self.feedback_tick = None
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
                self.status_text = 'connected; disarmed; waiting for encoderless protocol v5'
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
            if any(now-item['sent_at'] > self.p['ack_timeout_s'] for item in self.pending.values()):
                self.disarm('PWM command acknowledgement timeout')
            return
        if now - self.last_send >= .05:
            self.send_setpoint(now, self.command)

    def send_setpoint(self, now, command):
        if self.seq >= 0xffffffff:
            self.disarm('sequence exhausted; rearm required')
            return
        self.seq += 1
        left, right, nominal_v, nominal_w = self.mapping.convert(*command)
        self.pending[self.seq] = dict(left=left, right=right, sent_at=now,
                                     nominal=(nominal_v, nominal_w))
        self.last_output = (left, right)
        self.last_send = now
        self.send(f'P {self.boot} {self.nonce} {self.seq} {left} {right}')

    def handle(self, fields, now):
        kind = fields[0]
        if kind == 'V' and len(fields) == 5:
            version, boot, limit, watchdog = map(uint32, fields[1:])
            if version != 5 or not boot or limit < self.mapping.max_pwm or watchdog != 250:
                raise ValueError('wrong encoderless firmware protocol/PWM limits')
            if self.boot != boot:
                self.disarm('MCU boot identity changed; rearm required')
                self.feedback_at = -math.inf
                self.feedback_tick = None
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
            self.status_text = 'armed; requires fresh command, scan, deadman and healthy RF2O odometry'
            self.publish_state()
        elif kind == 'A' and len(fields) == 7:
            boot, nonce, seq = map(uint32, fields[1:4])
            left, right = int(fields[4]), int(fields[5])
            uint32(fields[6])  # MCU acceptance tick, not a ROS timestamp
            item = self.pending.get(seq)
            if (not self.armed or item is None or boot != self.boot or nonce != self.nonce or
                    (left, right) != (item['left'], item['right'])):
                raise ValueError('unknown/mismatched PWM acknowledgement')
            nominal_v, nominal_w = item['nominal']
            del self.pending[seq]
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p['base_frame']
            msg.twist.linear.x, msg.twist.angular.z = nominal_v, nominal_w
            self.applied_pub.publish(msg)
            self.applied_vel_pub.publish(msg)
        elif kind == 'M' and len(fields) == 7:
            boot, nonce, tick = map(uint32, fields[1:4])
            left, right, active = int(fields[4]), int(fields[5]), uint32(fields[6])
            if self.boot is None: return  # version identity must arrive first
            if boot != self.boot: raise ValueError('MCU reset during session')
            if active not in (0, 1) or max(abs(left), abs(right)) > 255:
                raise ValueError('invalid MCU output heartbeat')
            self.mcu_nonce = max(self.mcu_nonce, nonce)
            if self.armed and (nonce != self.nonce or active != 1):
                self.disarm('MCU watchdog/stop disarmed base')
            if not active and (left or right):
                raise ValueError('disarmed MCU reports nonzero output')
            if self.feedback_tick is not None:
                elapsed = (tick - self.feedback_tick) & 0xffffffff
                if elapsed == 0:
                    return  # duplicates cannot keep the MCU liveness gate open
                if elapsed >= 0x80000000:
                    raise ValueError('MCU heartbeat tick went backwards')
            self.feedback_tick = tick
            # PWM outputs confirm firmware liveness only, not physical motion.
            self.feedback_at = now
        elif kind == 'D' and len(fields) == 1:
            self.disarm('MCU reported disarmed')
        elif kind == 'E' and len(fields) == 1:
            raise ValueError('MCU rejected command')
        else:
            raise ValueError('unknown or wrong-length firmware response')

    def publish_state(self):
        self.source_pub.publish(String(data="mcu_acknowledged_nominal_velocity_setpoint"))
        self.armed_pub.publish(Bool(data=self.armed))
        self.estop_pub.publish(Bool(data=self.estop))
        self.status_pub.publish(String(data=self.status_text))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderlessBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial('node shutdown')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
