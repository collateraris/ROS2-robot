"""Pure Python and optional g++ host tests; no ROS or physical hardware required."""
import ast
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "robot_web_teleop"
sys.path.insert(0, str(PKG))
from robot_web_teleop.protocol import (BOOT_LOCK, MAX_LINE, VelocityGate, crc16,
    decode_ack, encode_command, frame, mix_velocity, sequence_newer)


class ProtocolTests(unittest.TestCase):
    def test_crc_and_ack_bounds(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        ack = decode_ack(frame("A 4294967295 -140 140 0"))
        self.assertEqual((ack.sequence, ack.left, ack.right), (2**32 - 1, -140, 140))
        for data in (b"A 1 0 0 0*0000\n", frame("A 1 141 0 0"),
                     frame("A 4294967296 0 0 0"), frame("A 1 0 0 8"), b"x" * 97):
            with self.assertRaises(ValueError):
                decode_ack(data)
        with self.assertRaises(ValueError):
            encode_command(1, 141, 0)

    def test_mix_and_common_saturation(self):
        self.assertEqual(mix_velocity(.2, 0), (70, 70))
        self.assertEqual(mix_velocity(-.2, 0), (-70, -70))
        self.assertEqual(mix_velocity(.4, 4), (7, 140))
        left, right = mix_velocity(0, 1)
        self.assertEqual(left, -right)
        self.assertLess(left, 0)
        with self.assertRaises(ValueError):
            mix_velocity(float("nan"), 0)

    def test_sequence_wrap(self):
        self.assertTrue(sequence_newer(0, 2**32 - 1))
        self.assertFalse(sequence_newer(10, 10))
        self.assertFalse(sequence_newer(9, 10))

    def gate(self):
        gate = VelocityGate()
        gate.set_ready(True, 1_000_000_000)
        return gate

    def test_timeout_uses_monotonic_time_and_cannot_replay(self):
        gate = self.gate()
        self.assertTrue(gate.accept(.2, 0, 1_100_000_000, 1_110_000_000, 5.0))
        self.assertEqual(gate.target(5.24, 1_120_000_000), (.2, 0))
        self.assertEqual(gate.target(5.25, 1_120_000_000), (0, 0))
        self.assertFalse(gate.accept(.2, 0, 1_100_000_000, 1_130_000_000, 5.26))
        self.assertTrue(gate.accept(.2, 0, 1_140_000_000, 1_145_000_000, 5.27))

    def test_zero_invalid_stamps_nan_and_reconnect(self):
        gate = self.gate()
        for stamp in (0, 900_000_000, 1_500_000_000):
            self.assertFalse(gate.accept(.2, 0, stamp, 1_400_000_000, 1.0))
        self.assertTrue(gate.accept(0, 0, 0, 1_500_000_000, 1.1))
        self.assertFalse(gate.accept(.2, 0, 1_490_000_000, 1_510_000_000, 1.2))
        self.assertTrue(gate.accept(.2, 0, 1_520_000_000, 1_530_000_000, 1.3))
        self.assertFalse(gate.accept(float("nan"), 0, 1_540_000_000, 1_550_000_000, 1.4))
        gate.set_ready(False, 1_560_000_000)
        gate.set_ready(True, 1_600_000_000)
        self.assertFalse(gate.accept(.2, 0, 1_590_000_000, 1_610_000_000, 1.5))

    def test_idle_timer_does_not_advance_stamp_barrier(self):
        gate = self.gate()
        gate.target(1.0, 1_030_000_000)
        self.assertTrue(gate.accept(.2, 0, 1_010_000_000, 1_040_000_000, 1.1))

    def test_queued_command_cannot_skip_expiry_when_callback_beats_timer(self):
        gate = self.gate()
        self.assertTrue(gate.accept(.2, 0, 1_010_000_000, 1_020_000_000, 1.0))
        self.assertFalse(gate.accept(.2, 0, 1_120_000_000, 1_320_000_000, 1.3))
        self.assertEqual(gate.velocity, (0, 0))


class Wire:
    def __init__(self):
        self.rx, self.writes, self.closed = bytearray(), [], False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.0
        namespace = dict(Node=object, time=SimpleNamespace(monotonic=lambda: self.now),
            rclpy=SimpleNamespace(ok=lambda: True), serial=SimpleNamespace(SerialException=OSError),
            BOOT_LOCK=BOOT_LOCK, MAX_LINE=MAX_LINE, VelocityGate=VelocityGate,
            decode_ack=decode_ack, encode_command=encode_command,
            mix_velocity=mix_velocity, sequence_newer=sequence_newer)
        tree = ast.parse((PKG / "robot_web_teleop" / "motor_bridge.py").read_text())
        tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        exec(compile(tree, "motor_bridge.py", "exec"), namespace)
        cls = namespace["MotorBridge"]
        self.b = b = cls.__new__(cls)
        b.gate, b.port, b.rx = VelocityGate(), Wire(), bytearray()
        b.sequence, b.last_ack_seq, b.pending, b.last_requested = 100, None, None, (0, 0)
        b.open_after, b.last_ack, b.next_status, b.next_send = 0, self.now, 100, 0
        b.mixing = (.18, .40, 140)
        b.ros_ns = lambda: round(self.now * 1e9)
        b.status = lambda: None
        b.warn = lambda message: None

    def test_only_current_fresh_ack_can_make_ready(self):
        b = self.b
        b.send((0, 0), self.now)
        b.port.rx.extend(frame("A 100 0 0 0"))
        b.receive(self.now + .01)
        self.assertFalse(b.gate.ready)
        b.port.rx.extend(frame("A 101 0 0 0"))
        b.receive(self.now + .02)
        self.assertTrue(b.gate.ready)
        self.assertEqual(b.last_ack_seq, 101)

    def test_late_matching_ack_is_not_fresh(self):
        b = self.b
        b.send((0, 0), self.now)
        b.port.rx.extend(frame("A 101 0 0 0"))
        with self.assertRaises(TimeoutError):
            b.receive(self.now + .101)
        self.assertFalse(b.gate.ready)

    def test_zero_preempts_pending_and_old_ack_does_not_revive_motion(self):
        b = self.b
        b.gate.set_ready(True, 0)
        b.send((70, 70), self.now)
        b.gate.clear(b.ros_ns())
        self.now += .01
        b.tick()
        self.assertEqual(b.pending[:2], (102, (0, 0)))
        b.port.rx.extend(frame("A 101 70 70 0") + frame("A 102 0 0 0"))
        self.now += .01
        b.tick()
        self.assertEqual(b.last_ack_seq, 102)
        self.assertEqual(b.last_requested, (0, 0))

    def test_ack_timeout_and_200ms_liveness_clear_cached_command(self):
        for stale_time in (.101, .201):
            self.setUp()
            b, wire = self.b, self.b.port
            b.gate.set_ready(True, 0)
            b.gate.accept(.2, 0, 999_000_000, 1_000_000_000, self.now)
            b.send((70, 70), self.now)
            self.now += stale_time
            b.tick()
            self.assertFalse(b.gate.ready)
            self.assertEqual(b.gate.velocity, (0, 0))
            self.assertTrue(wire.closed)
            self.assertIn(b" 0 0*", wire.writes[-1])


ARDUINO_STUB = r'''
#pragma once
#include <cstdint>
#include <deque>
#include <string>
constexpr int LOW=0,HIGH=1,OUTPUT=1;
inline unsigned long ticks=0;
inline int pins[20]={0},pwm[20]={0};
inline unsigned long millis(){return ticks;}
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){pins[p]=v;}
inline void analogWrite(int p,int v){pwm[p]=v;}
struct SerialStub{
 std::deque<char> in; std::string out;
 void begin(int){}
 int available(){return (int)in.size();}
 char read(){char c=in.front();in.pop_front();return c;}
 size_t write(const uint8_t* p,size_t n){out.append((const char*)p,n);return n;}
};
inline SerialStub Serial;
'''
FIRMWARE_TEST = r'''
#include <cassert>
#include <iostream>
#include "FIRMWARE_PATH"
std::string packet(const std::string& s){
 char suffix[8];snprintf(suffix,sizeof(suffix),"*%04X\n",(unsigned)crc16(s.data(),s.size()));
 return s+suffix;
}
std::string send(const std::string& s){
 Serial.out.clear();for(char c:s)Serial.in.push_back(c);
 while(Serial.available())loop();
 return Serial.out;
}
int main(){
 setup();assert(crc16("123456789",9)==0x29B1);
 assert(send(packet("M 1 70 70"))==packet("A 1 0 0 1"));
 assert(locked && actualLeft==0 && pins[STBY]==LOW);
 assert(send(packet("M 2 0 0"))==packet("A 2 0 0 0"));
 assert(send(packet("M 3 140 -140"))==packet("A 3 140 -140 0"));
 assert(pwm[PWM_LEFT]==140 && pins[LEFT_1]==HIGH && pins[RIGHT_2]==HIGH);
 assert(pins[STBY]==HIGH);
 assert(send("M 4 10 10*0000\n").empty());
 assert(locked && actualLeft==0);
 assert(send(packet("M 5 50 50"))==packet("A 5 0 0 5"));
 send(packet("M 6 0 0"));send(packet("M 7 70 70"));
 send("M 8 1");ticks=350;loop();
 assert(locked && actualLeft==0 && (events&2));
 send("\n");send(packet("M 9 0 0"));
 send(packet("M 10 70 70"));send(std::string(120,'x'));
 assert(locked && actualLeft==0);
 send("\n");send(packet("M 11 0 0"));
 assert(send(packet("M 4294967296 0 0")).empty());
 assert(locked);
 send(packet("M 12 0 0"));
 std::string injected=packet("M 13 50 50");injected.insert(3,1,'\r');
 assert(send(injected).empty());assert(locked);
 send(packet("M 14 0 0"));
 std::string crlf=packet("M 15 70 70");crlf.insert(crlf.size()-1,1,'\r');
 assert(send(crlf)==packet("A 15 70 70 0"));
 std::cout<<"firmware CRC/parser/boot/watchdog host tests passed\n";
}
'''


class FirmwareTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "g++ needed for optional host simulation")
    def test_real_firmware_with_host_stub(self):
        with tempfile.TemporaryDirectory(prefix="teleop_firmware_") as directory:
            folder = Path(directory)
            (folder / "Arduino.h").write_text(ARDUINO_STUB)
            source = ROOT / "arduino" / "robot_drive" / "robot_drive.ino"
            (folder / "test.cpp").write_text(FIRMWARE_TEST.replace("FIRMWARE_PATH", str(source)))
            exe = folder / "test"
            subprocess.run([shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra",
                "-I", str(folder), str(folder / "test.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
