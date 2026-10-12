"""Wi-Fi PWM with Bluetooth priority; CRC/ACK recovery keeps USB open."""
import math
import secrets
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Empty, String
import serial

from .protocol import (ACK_DEADLINE, ACK_FRESH, BOOT_LOCK, BT_ACTIVE,
                       MAX_LINE, RECOVERY_HOLD, SEND_INTERVAL, VelocityGate,
                       decode_ack, encode_command, encode_emergency_stop,
                       mix_velocity, sequence_newer)


class MotorBridge(Node):
    def __init__(self):
        super().__init__("motor_bridge")
        defaults = dict(serial_port="/dev/ttyACM0", baud=9600, usb_reset_wait_s=2.0,
                        wheel_track_m=.18, max_wheel_speed_mps=.40, pwm_limit=140,
                        command_timeout_s=.25, stamp_max_age_s=.25)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.port_name, self.baud = p["serial_port"], p["baud"]
        self.reset_wait = p["usb_reset_wait_s"]
        if self.baud != 9600 or not math.isfinite(self.reset_wait) or not 0 <= self.reset_wait <= 10:
            raise ValueError("Firmware uses 9600 baud; USB reset wait must be 0..10 seconds")
        self.mixing = (p["wheel_track_m"], p["max_wheel_speed_mps"], p["pwm_limit"])
        mix_velocity(0, 0, *self.mixing)
        self.gate = VelocityGate(p["command_timeout_s"], p["stamp_max_age_s"])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.ready_pub = self.create_publisher(Bool, "drive/ready", qos)
        self.source_pub = self.create_publisher(String, "drive/source", qos)
        self.create_subscription(TwistStamped, "cmd_vel", self.on_command, qos)
        self.create_subscription(Empty, "drive/stop", self.on_stop, qos)
        self.port, self.pending, self.last_ack_seq = None, None, None
        self.rx, self.sequence, self.last_requested = bytearray(), secrets.randbits(32), (0, 0)
        self.discard_rx = self.resync = self.emergency = self.emergency_sent = False
        self.source, self.last_ack = "disconnected", float('-inf')
        self.open_after = self.next_send = self.next_connect = self.hold_until = 0.0
        self.next_status = self.last_warning = 0.0
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.01, self.tick, clock=self.steady_clock)

    def ros_ns(self):
        return self.get_clock().now().nanoseconds

    def status(self):
        self.ready_pub.publish(Bool(data=self.gate.ready))
        self.source_pub.publish(String(data=self.source))

    def warn(self, message):
        if time.monotonic() - self.last_warning > 1.0:
            self.get_logger().warning(message)
            self.last_warning = time.monotonic()

    def on_command(self, msg):
        # A normal zero releases only Wi-Fi; it must never turn into a global E.
        t, stamp = msg.twist, msg.header.stamp
        values = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        now_ns = self.ros_ns()
        if (not all(math.isfinite(x) for x in values)
                or any(x != 0 for x in (t.linear.y, t.linear.z, t.angular.x, t.angular.y))):
            self.gate.clear(now_ns)
            self.warn("Invalid/unsupported Twist component: selecting USB zero")
            return
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if not 0 <= stamp.nanosec < 1_000_000_000:
            stamp_ns = 0
        if not self.gate.accept(t.linear.x, t.angular.z, stamp_ns, now_ns, time.monotonic()):
            self.warn("Rejected stale, reordered or not-ready Wi-Fi motion command")

    def on_stop(self, _msg):
        if not self.emergency:
            self.emergency, self.emergency_sent = True, False
        self.recover(time.monotonic(), "Explicit global STOP", resync=False, source=self.source)

    def recover(self, now, reason, resync=True, source="disconnected"):
        """Block old Wi-Fi commands without closing USB or stopping Bluetooth."""
        self.gate.set_ready(False, self.ros_ns())
        self.hold_until = max(self.hold_until, now + RECOVERY_HOLD)
        self.source, self.resync = source, self.resync or resync
        if self.pending is not None and self.pending[3] != "E":
            self.pending = None
        self.next_status = min(self.next_status, now)
        self.warn(reason)

    def disconnect(self, reason=""):
        self.gate.set_ready(False, self.ros_ns())
        if self.port is not None:
            try:
                self.port.close()
            except (OSError, serial.SerialException):
                pass
        self.port, self.pending, self.last_ack_seq = None, None, None
        self.rx.clear()
        self.discard_rx = False
        self.last_requested, self.source, self.last_ack = (0, 0), "disconnected", float('-inf')
        self.next_connect = time.monotonic() + 1.0
        if rclpy.ok():
            self.status()
        if reason:
            self.warn(f"USB disconnected: {reason}")

    def open_port(self, now):
        self.gate.set_ready(False, self.ros_ns())
        self.port = serial.Serial(self.port_name, self.baud, timeout=0,
                                  write_timeout=0, exclusive=True)
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        self.open_after = self.next_send = now + self.reset_wait
        self.hold_until = max(self.hold_until, now + RECOVERY_HOLD)
        self.pending, self.last_requested, self.last_ack_seq = None, (0, 0), None
        self.source, self.last_ack = "disconnected", float('-inf')
        self.discard_rx = self.emergency_sent = False
        self.resync = True
        self.rx.clear()

    def send(self, target, now, emergency=False):
        self.sequence = (self.sequence + 1) % 2**32
        packet = (encode_emergency_stop(self.sequence) if emergency
                  else encode_command(self.sequence, *target))
        packet = (b"\n" if self.resync else b"") + packet
        # Rate-limit failed attempts too; a short write must not flood 9600 baud.
        self.last_requested, self.next_send = target, now + SEND_INTERVAL
        if emergency:
            self.emergency_sent = True
        try:
            if self.port.write(packet) != len(packet):
                self.recover(now, "Partial serial write: resynchronizing USB")
                return
        except serial.SerialTimeoutException:
            self.recover(now, "Serial write timeout: keeping USB open")
            return
        self.resync = False
        self.pending = (self.sequence, target, now, "E" if emergency else "M")

    def receive(self, now):
        for byte in self.port.read(min(256, self.port.in_waiting)):
            if self.discard_rx:
                if byte == 10:
                    self.discard_rx = False
                continue
            self.rx.append(byte)
            if len(self.rx) > MAX_LINE:
                self.rx.clear()
                self.discard_rx = byte != 10
                self.recover(now, "Oversized ACK: discarding until newline")
                continue
            if byte != 10:
                continue
            line = bytes(self.rx)
            self.rx.clear()
            try:
                ack = decode_ack(line)
            except ValueError:
                self.recover(now, "Malformed ACK/CRC: resynchronizing USB")
                continue
            if (self.pending is None or ack.sequence != self.pending[0]
                    or not sequence_newer(ack.sequence, self.last_ack_seq)):
                continue  # Old ACKs cannot refresh either deadline or source.
            _, target, sent_at, kind = self.pending
            if not 0 <= now - sent_at <= ACK_DEADLINE:
                self.recover(now, "Expired matching ACK")
                continue
            expected = (0, 0) if kind == "E" or ack.flags & BOOT_LOCK else target
            bluetooth = bool(ack.flags & BT_ACTIVE)
            if ((bluetooth and kind == "E")
                    or (not bluetooth and (ack.left, ack.right) != expected)):
                self.recover(now, "Arduino applied target does not match USB request")
                continue
            self.last_ack, self.last_ack_seq, self.pending = now, ack.sequence, None
            self.source = "bluetooth" if bluetooth else ("wifi" if ack.left or ack.right else "idle")
            if kind == "E":
                self.emergency = self.emergency_sent = False
            if ack.flags or kind == "E":
                self.recover(now, f"Arduino arbitration/stop flags={ack.flags}",
                             resync=False, source=self.source)
            elif (not self.gate.ready and target == (0, 0)
                  and sent_at >= self.hold_until and now >= self.hold_until):
                self.gate.set_ready(True, self.ros_ns())
                self.next_status = min(self.next_status, now)

    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None:
                if now >= self.next_connect:
                    self.open_port(now)
            elif now >= self.open_after:
                self.receive(now)
                if self.pending is not None and now - self.pending[2] > ACK_DEADLINE:
                    self.recover(now, "Motor ACK deadline expired")
                    self.pending = None  # E is immediately retried below; never replaced by M0.
                if self.source != "disconnected" and now - self.last_ack > ACK_FRESH:
                    self.recover(now, "No fresh matching ACK for 300 ms")
                target = mix_velocity(*self.gate.target(now, self.ros_ns()), *self.mixing)
                if self.emergency:
                    if self.pending is None and (not self.emergency_sent or now >= self.next_send):
                        self.send((0, 0), now, emergency=True)
                else:
                    urgent_zero = target == (0, 0) and self.last_requested != (0, 0)
                    if urgent_zero or (self.pending is None and now >= self.next_send):
                        self.send(target, now)
        except serial.SerialTimeoutException:
            self.recover(now, "Serial timeout: keeping USB open")
        except (OSError, serial.SerialException) as exc:
            self.disconnect(str(exc))  # Real device/OS failure only; reopening may reset the MCU.
        except ValueError as exc:
            self.recover(now, str(exc))
        if now >= self.next_status:
            self.status()
            self.next_status = now + .1

    def close(self):
        if self.port is not None:
            try:
                self.sequence = (self.sequence + 1) % 2**32
                packet = (encode_emergency_stop(self.sequence) if self.emergency
                          else encode_command(self.sequence, 0, 0))
                self.port.write(b"\n" + packet)
            except (OSError, serial.SerialException):
                pass  # No confirmed ACK: firmware watchdog remains independent.
        self.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorBridge()
        rclpy.spin(node)
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
