"""ROS 2 stamped velocity -> acknowledged Arduino PWM; no fake odometry."""
import math
import secrets
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
import serial

from .protocol import (BOOT_LOCK, MAX_LINE, VelocityGate, decode_ack,
                       encode_command, mix_velocity, sequence_newer)


class MotorBridge(Node):
    def __init__(self):
        super().__init__("motor_bridge")
        defaults = dict(serial_port="/dev/ttyACM0", baud=115200, usb_reset_wait_s=2.0,
                        wheel_track_m=.18, max_wheel_speed_mps=.40, pwm_limit=140,
                        command_timeout_s=.25, stamp_max_age_s=.25)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.port_name, self.baud = p["serial_port"], p["baud"]
        self.reset_wait = p["usb_reset_wait_s"]
        if self.baud != 115200 or not math.isfinite(self.reset_wait) or not 0 <= self.reset_wait <= 10:
            raise ValueError("Firmware uses 115200 baud; USB reset wait must be 0..10 seconds")
        self.mixing = (p["wheel_track_m"], p["max_wheel_speed_mps"], p["pwm_limit"])
        mix_velocity(0, 0, *self.mixing)  # Validate before opening a motor connection.
        self.gate = VelocityGate(p["command_timeout_s"], p["stamp_max_age_s"])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.ready_pub = self.create_publisher(Bool, "drive/ready", qos)
        self.create_subscription(TwistStamped, "cmd_vel", self.on_command, qos)
        self.port, self.pending, self.last_ack_seq = None, None, None
        self.rx, self.sequence, self.last_requested = bytearray(), secrets.randbits(32), (0, 0)
        self.open_after = self.next_send = self.next_connect = self.last_ack = 0.0
        self.next_status = self.last_warning = 0.0
        # Pausing/jumping ROS simulation time must not pause either host watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.01, self.tick, clock=self.steady_clock)

    def ros_ns(self):
        return self.get_clock().now().nanoseconds

    def status(self):
        self.ready_pub.publish(Bool(data=self.gate.ready))

    def warn(self, message):
        if time.monotonic() - self.last_warning > 1.0:
            self.get_logger().warning(message)
            self.last_warning = time.monotonic()

    def on_command(self, msg):
        # Callback never writes serial; the steady timer owns the serial port.
        t, stamp = msg.twist, msg.header.stamp
        values = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        now_ns = self.ros_ns()
        if (not all(math.isfinite(x) for x in values)
                or any(x != 0 for x in (t.linear.y, t.linear.z, t.angular.x, t.angular.y))):
            self.gate.clear(now_ns)
            self.warn("Invalid/unsupported Twist component: selecting zero")
            return
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if not 0 <= stamp.nanosec < 1_000_000_000:
            stamp_ns = 0
        if not self.gate.accept(t.linear.x, t.angular.z, stamp_ns, now_ns, time.monotonic()):
            self.warn("Rejected unstamped, stale, reordered or not-ready motion command")

    def disconnect(self, reason=""):
        self.gate.set_ready(False, self.ros_ns())
        if self.port is not None:
            try:
                self.sequence = (self.sequence + 1) % 2**32
                # Newline terminates any interrupted frame before the stop packet.
                self.port.write(b"\n" + encode_command(self.sequence, 0, 0))
            except (OSError, serial.SerialException):
                pass  # Unconfirmed stop: MCU's independent watchdog remains active.
            try:
                self.port.close()
            except (OSError, serial.SerialException):
                pass
        self.port, self.pending = None, None
        self.rx.clear()
        self.last_requested = (0, 0)
        self.last_ack_seq = None
        self.next_connect = time.monotonic() + 1.0
        if rclpy.ok():
            self.status()
        if reason:
            self.warn(f"Drive not ready: {reason}")

    def open_port(self, now):
        self.gate.set_ready(False, self.ros_ns())
        self.port = serial.Serial(self.port_name, self.baud, timeout=0,
                                  write_timeout=0, exclusive=True)
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        self.open_after, self.next_send = now + self.reset_wait, now + self.reset_wait
        self.pending, self.last_requested = None, (0, 0)
        self.rx.clear()

    def send(self, target, now):
        self.sequence = (self.sequence + 1) % 2**32
        packet = encode_command(self.sequence, *target)
        if self.port.write(packet) != len(packet):
            raise OSError("Partial nonblocking serial write")
        self.pending = (self.sequence, target, now)
        self.last_requested, self.next_send = target, now + .05

    def receive(self, now):
        for byte in self.port.read(min(256, self.port.in_waiting)):
            self.rx.append(byte)
            if len(self.rx) > MAX_LINE:
                raise ValueError("Oversized/unterminated ACK")
            if byte != 10:
                continue
            ack = decode_ack(bytes(self.rx))
            self.rx.clear()
            if (self.pending is None or ack.sequence != self.pending[0]
                    or not sequence_newer(ack.sequence, self.last_ack_seq)):
                continue  # An old ACK never refreshes readiness or a deadline.
            if now - self.pending[2] > .100:
                raise TimeoutError("Received a matching but expired ACK")
            expected = (0, 0) if ack.flags & BOOT_LOCK else self.pending[1]
            if (ack.left, ack.right) != expected or ack.flags:
                raise ValueError(f"Arduino target mismatch/stop event: flags={ack.flags}")
            self.last_ack, self.last_ack_seq, self.pending = now, ack.sequence, None
            if not self.gate.ready:
                self.gate.set_ready(True, self.ros_ns())  # Fresh zero handshake succeeded.

    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None:
                if now >= self.next_connect:
                    self.open_port(now)
            elif now >= self.open_after:
                if self.gate.ready and now - self.last_ack > .200:
                    raise TimeoutError("No fresh matching ACK for 200 ms")
                self.receive(now)
                if self.pending is not None and now - self.pending[2] > .100:
                    raise TimeoutError("Motor ACK deadline expired")
                target = mix_velocity(*self.gate.target(now, self.ros_ns()), *self.mixing)
                urgent_zero = target == (0, 0) and self.last_requested != (0, 0)
                if urgent_zero or (self.pending is None and
                                   (now >= self.next_send or target != self.last_requested)):
                    self.send(target, now)
        except (OSError, ValueError, serial.SerialException) as exc:
            self.disconnect(str(exc))
        if now >= self.next_status:
            self.status()
            self.next_status = now + .1

    def close(self):
        self.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorBridge()
        rclpy.spin(node)  # Single-threaded executor; no concurrent serial access.
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
