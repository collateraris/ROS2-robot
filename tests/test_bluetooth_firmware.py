"""Compile and exercise the actual Bluetooth/USB sketch with host I/O stubs.

These tests validate parsing, arbitration and pin writes, not AVR timing,
SoftwareSerial electrical behaviour or a physical motor driver.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "arduino" / "blu_drive" / "blu_drive.ino"

ARDUINO_STUB = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
constexpr int LOW=0, HIGH=1, INPUT=0, OUTPUT=1, INPUT_PULLUP=2, A0=14;
inline uint32_t ticks=0;
inline int pins[32]={0}, pwm[32]={0}, modes[32]={0};
inline unsigned long millis(){return ticks;}
inline unsigned long micros(){return static_cast<uint32_t>(ticks*1000);}
inline void pinMode(int pin,int mode){modes[pin]=mode;}
inline void digitalWrite(int pin,int value){pins[pin]=value;}
inline int digitalRead(int pin){return pins[pin];}
inline void analogWrite(int pin,int value){pwm[pin]=value;}
inline void delay(unsigned long amount){ticks+=amount;}
template <typename T> inline T constrain(T value,T low,T high){return value<low?low:(value>high?high:value);}
struct SerialStub{
 std::deque<char> in;
 std::string out;
 unsigned long baud=0;
 void begin(unsigned long value){baud=value;}
 int available(){return static_cast<int>(in.size());}
 int availableForWrite(){return 64;}
 int read(){if(in.empty())return -1;int c=static_cast<unsigned char>(in.front());in.pop_front();return c;}
 size_t write(const uint8_t* data,size_t length){out.append(reinterpret_cast<const char*>(data),length);return length;}
 size_t write(uint8_t value){out.push_back(static_cast<char>(value));return 1;}
 void print(const char* value){out+=value;}
 void println(const char* value){out+=value;out+='\n';}
};
inline SerialStub Serial;
#define F(value) value
'''

SOFTWARE_SERIAL_STUB = r'''
#pragma once
#include "Arduino.h"
class SoftwareSerial: public SerialStub{
 public:
 int rx,tx;
 SoftwareSerial(int receive,int transmit):rx(receive),tx(transmit){}
 bool listen(){return true;}
};
'''

FIRMWARE_TEST = r'''
#include <cassert>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include "Arduino.h"
#include "FIRMWARE_PATH"

uint16_t reference_crc(const std::string& body){
 uint16_t result=0xffff;
 for(unsigned char c:body){
  result^=static_cast<uint16_t>(c)<<8;
  for(int i=0;i<8;++i)result=(result&0x8000)?(result<<1)^0x1021:result<<1;
 }
 return result;
}
std::string packet(const std::string& body){
 char ending[8];
 std::snprintf(ending,sizeof(ending),"*%04X\n",reference_crc(body));
 return body+ending;
}
void drain(){
 unsigned int turns=0;
 while(Serial.available() || BTSerial.available()){
  loop();
  if(++turns>10000)throw std::runtime_error("Firmware did not drain its input");
 }
}
std::string usb_bytes(const std::string& wire){
 Serial.out.clear();
 for(char c:wire)Serial.in.push_back(c);
 drain();
 return Serial.out;
}
void bluetooth(const std::string& wire){
 for(char c:wire)BTSerial.in.push_back(c);
 drain();
}
struct Ack{uint32_t sequence;int left,right,flags;};
Ack usb(const std::string& body){
 auto response=usb_bytes(packet(body));
 auto separator=response.find('*');
 if(separator==std::string::npos || response.back()!='\n')
  throw std::runtime_error("Missing ACK for "+body+": "+response);
 std::string payload=response.substr(0,separator);
 if(response!=packet(payload))throw std::runtime_error("Invalid ACK CRC or extra bytes: "+response);
 std::istringstream stream(payload);
 char kind=0;uint64_t sequence=0;Ack ack{};std::string extra;
 if(!(stream>>kind>>sequence>>ack.left>>ack.right>>ack.flags)
    || kind!='A' || sequence>UINT32_MAX || (stream>>extra))
  throw std::runtime_error("Malformed ACK: "+response);
 ack.sequence=static_cast<uint32_t>(sequence);
 std::istringstream request(body);
 uint64_t requested=0;request>>kind>>requested;
 assert(ack.sequence==requested);
 return ack;
}
int electrical(int pin1,int pin2,int enable){
 assert(pwm[enable]>=0 && pwm[enable]<=255);
 if(pwm[enable]==0 || (pins[pin1]==LOW && pins[pin2]==LOW))return 0;
 assert(pins[pin1]!=pins[pin2]);
 return pins[pin1]==HIGH?pwm[enable]:-pwm[enable];
}
void motors(int left,int right){
 int actualLeft=electrical(L1,L2,ENA),actualRight=electrical(R1,R2,ENB);
 if(actualLeft!=left || actualRight!=right){
  std::ostringstream message;
  message<<"Expected electrical motors "<<left<<','<<right<<"; got "
         <<actualLeft<<','<<actualRight;
  throw std::runtime_error(message.str());
 }
}
void flags(const Ack& ack,int required,int forbidden=0){
 assert((ack.flags&required)==required);
 assert((ack.flags&forbidden)==0);
}
void logical(const Ack& ack,int left,int right){assert(ack.left==left && ack.right==right);}
void usb_start(uint32_t sequence=1){
 auto ack=usb("M "+std::to_string(sequence)+" 0 0");
 logical(ack,0,0);flags(ack,0,1|8);
}
void mapping(){
 assert(Serial.baud==9600 && BTSerial.baud==9600);
 assert(BTSerial.rx==10 && BTSerial.tx==11);
 motors(0,0);
 const std::vector<std::pair<char,std::pair<int,int>>> cases={
  {'F',{-140,-140}}, {'B',{140,140}}, {'L',{-140,140}},
  {'R',{140,-140}}, {'G',{0,140}}, {'I',{140,0}},
  {'H',{140,-140}}, {'J',{-140,140}}};
 for(const auto& entry:cases){
  bluetooth(std::string(1,entry.first));
  motors(entry.second.first,entry.second.second);
  bluetooth("Xx");motors(entry.second.first,entry.second.second);
  bluetooth("S");motors(0,0);
 }
 bluetooth("F");bluetooth("D");motors(0,0);
}
void speed(){
 bluetooth("9qXx");motors(0,0);
 usb_start();logical(usb("M 2 70 -60"),70,-60);motors(-70,60);
 bluetooth("5");motors(-70,60); // Speed alone cannot acquire Bluetooth control.
 bluetooth("L");motors(-127,127);
 for(int digit=0;digit<=9;++digit){
  bluetooth(std::string(1,static_cast<char>('0'+digit)));
  int level=digit*255/10;motors(-level,level);
 }
 bluetooth("q");motors(-255,255);
 auto ack=usb("M 3 0 0");logical(ack,-255,255);flags(ack,1|8|16);
 bluetooth("S9q");motors(0,0);
 bluetooth("F");motors(-255,-255);
 bluetooth("D4");motors(0,0);
 bluetooth("B");motors(102,102);
}
void handshake_and_replay(){
 auto ack=usb("M 1 70 -60");logical(ack,0,0);flags(ack,1);motors(0,0);
 usb_start(2);
 logical(usb("M 3 70 -60"),70,-60);motors(-70,60);
 auto duplicate=usb("M 3 -100 100");logical(duplicate,0,0);flags(duplicate,1|4);motors(0,0);
 usb_start(4);usb("M 5 70 -60");motors(-70,60);
 auto stale=usb("M 4 -100 100");logical(stale,0,0);flags(stale,1|4);motors(0,0);
 usb_start(6);
 logical(usb("M 7 -140 140"),-140,140);motors(140,-140);
}
void sequence_wrap(){
 usb_start(4294967294U);
 logical(usb("M 4294967295 40 50"),40,50);motors(-40,-50);
 logical(usb("M 0 45 55"),45,55);motors(-45,-55);
 auto stale=usb("M 4294967295 -100 -100");logical(stale,0,0);flags(stale,1|4);motors(0,0);
 usb_start(0);
 auto half=usb("M 2147483648 -100 -100");logical(half,0,0);flags(half,1|4);motors(0,0);
 usb_start(0);
 logical(usb("M 1 60 70"),60,70);motors(-60,-70);
}
void watchdog(){
 usb_start();usb("M 2 70 70");
 ticks=399;loop();motors(-70,-70);
 ticks=400;loop();motors(0,0);
 auto ack=usb("M 3 70 70");logical(ack,0,0);flags(ack,1|2);
 usb_start(4);usb("M 5 60 60");motors(-60,-60);
}
void partial_timeout(){
 usb_start();usb("M 2 70 70");
 assert(usb_bytes("M 3 80").empty());
 ticks=149;loop();motors(-70,-70);
 ticks=150;loop();motors(0,0);
 usb_bytes("\n");
 auto ack=usb("M 4 70 70");logical(ack,0,0);flags(ack,1|4);
}
void malformed(const std::string& kind){
 usb_start();usb("M 2 70 70");
 std::string bad;
 if(kind=="bad_crc")bad="M 3 70 70*0000\n";
 else if(kind=="oversized")bad=std::string(97,'x');
 else if(kind=="out_of_range")bad=packet("M 3 141 0");
 else if(kind=="sequence_overflow")bad=packet("M 4294967296 0 0");
 else if(kind=="embedded_cr")bad=packet("M 3 70 70"),bad.insert(3,1,'\r');
 else if(kind=="nul")bad=std::string("M 3 \0 70\n",9);
 else throw std::runtime_error("Unknown bad-frame case");
 assert(usb_bytes(bad).empty());motors(0,0);
 usb_bytes("\n");
 auto ack=usb("M 4 70 70");logical(ack,0,0);flags(ack,1|4);
 usb_start(5);usb("M 6 50 60");motors(-50,-60);
}
void bt_priority(){
 usb_start();usb("M 2 70 60");motors(-70,-60);
 bluetooth("F");motors(-140,-140);
 auto zero=usb("M 3 0 0");logical(zero,-140,-140);flags(zero,1|8|16);motors(-140,-140);
 auto move=usb("M 4 80 -80");logical(move,-140,-140);flags(move,1|8|16);motors(-140,-140);
 usb_bytes("M 5 70 70*0000\n");motors(-140,-140);
 usb_bytes(std::string(120,'x')+"\n");motors(-140,-140);
 usb_bytes("M 6 80");ticks=151;loop();motors(-140,-140);usb_bytes("\n");
 ticks=1000;loop();motors(-140,-140); // Default Bluetooth timeout is disabled.
 auto again=usb("M 7 0 0");logical(again,-140,-140);flags(again,1|8|16);
}
void handoff(){
 bluetooth("F");
 flags(usb("M 1 0 0"),1|8|16);
 flags(usb("M 2 0 0"),1|8|16);
 bluetooth("S");motors(0,0);
 auto blocked=usb("M 3 70 70");logical(blocked,0,0);flags(blocked,1|16,8);
 auto release=usb("M 4 0 0");logical(release,0,0);flags(release,16,1|8);
 auto clean=usb("M 5 0 0");logical(clean,0,0);assert(clean.flags==0);
 logical(usb("M 6 70 60"),70,60);motors(-70,-60);
}
void emergency_stop(){
 bluetooth("F");
 auto stop=usb("E 1");logical(stop,0,0);flags(stop,1|16|32,8);motors(0,0);
 bluetooth("FF9qF");motors(0,0);
 usb("M 2 0 0");
 logical(usb("M 3 70 60"),70,60);motors(-70,-60);
 bluetooth("F");motors(-70,-60); // USB recovery does not release the BT lock.
 auto repeated=usb("E 1");logical(repeated,0,0);flags(repeated,1|32);motors(0,0);
 bluetooth("F");motors(0,0);
 bluetooth("SqF");motors(-255,-255);
 usb("E 4");bluetooth("DqF");motors(-255,-255);
}
void bt_timeout(){
 bluetooth("F");ticks=80;
 auto ack=usb("M 1 0 0");flags(ack,1|8|16);motors(-140,-140);
 usb_bytes("broken\n");bluetooth("Xx");
 ticks=99;loop();motors(-140,-140);
 ticks=100;loop();motors(0,0);
 auto expired=usb("M 2 70 70");logical(expired,0,0);flags(expired,1|16,8);
}
void bt_state(){
 bluetooth("F");motors(-140,-140);
 pins[8]=LOW;loop();motors(0,0);
 bluetooth("F");motors(0,0);
 usb("M 1 0 0");usb("M 2 70 60");motors(-70,-60);
 pins[8]=HIGH;loop();motors(-70,-60);
 bluetooth("SF");motors(-140,-140);
}
int main(int argc,char** argv){
 try{
  assert(argc==2);
  pins[8]=HIGH;
  setup();
  assert(reference_crc("123456789")==0x29b1);
  std::string which=argv[1];
  if(which=="mapping")mapping();
  else if(which=="speed")speed();
  else if(which=="handshake")handshake_and_replay();
  else if(which=="wrap")sequence_wrap();
  else if(which=="watchdog")watchdog();
  else if(which=="partial")partial_timeout();
  else if(which=="priority")bt_priority();
  else if(which=="handoff")handoff();
  else if(which=="estop")emergency_stop();
  else if(which=="bt_timeout")bt_timeout();
  else if(which=="bt_state")bt_state();
  else malformed(which);
  std::cout<<which<<" passed\n";
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
'''


@unittest.skipUnless(shutil.which("g++"), "g++ is required for host firmware tests")
class BluetoothFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="bluetooth_firmware_")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.folder = Path(cls.temp.name)
        (cls.folder / "Arduino.h").write_text(ARDUINO_STUB)
        (cls.folder / "SoftwareSerial.h").write_text(SOFTWARE_SERIAL_STUB)
        (cls.folder / "test.cpp").write_text(
            FIRMWARE_TEST.replace("FIRMWARE_PATH", str(SKETCH)))
        cls.executables = {}
        cls.compile("default")

    @classmethod
    def compile(cls, variant, defines=()):
        executable = cls.folder / variant
        command = [shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra",
                   "-DARDUINO_AVR_UNO", "-D__AVR_ATmega328P__",
                   *defines, "-I", str(cls.folder), str(cls.folder / "test.cpp"),
                   "-o", str(executable)]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(f"Firmware compile failed ({variant}):\n{result.stderr}")
        cls.executables[variant] = executable

    def run_case(self, case, variant="default"):
        result = subprocess.run([str(self.executables[variant]), case],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0,
                         f"{case} ({variant}):\n{result.stdout}{result.stderr}")

    def test_original_bluetooth_mappings_default_pwm_and_stop(self):
        self.run_case("mapping")

    def test_speed_preserves_direction_without_acquiring_or_reviving(self):
        self.run_case("speed")

    def test_usb_zero_handshake_and_duplicate_rejection(self):
        self.run_case("handshake")

    def test_usb_sequence_wrap_and_half_range_rejection(self):
        self.run_case("wrap")

    def test_usb_watchdog_at_400ms(self):
        self.run_case("watchdog")

    def test_partial_line_expires_at_150ms(self):
        self.run_case("partial")

    def test_bad_usb_frames_stop_and_require_handshake(self):
        for case in ("bad_crc", "oversized", "out_of_range", "sequence_overflow",
                     "embedded_cr", "nul"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_bluetooth_preempts_usb_and_survives_usb_traffic(self):
        self.run_case("priority")

    def test_handoff_flag_persists_until_inactive_zero_handshake(self):
        self.run_case("handoff")

    def test_global_estop_and_independent_bluetooth_release(self):
        self.run_case("estop")

    def test_optional_bluetooth_timeout_is_not_refreshed_by_usb(self):
        self.compile("timeout", ("-DBT_COMMAND_TIMEOUT_MS=100",))
        self.run_case("bt_timeout", "timeout")

    def test_optional_bluetooth_state_pin_and_usb_recovery(self):
        self.compile("state", ("-DBT_STATE_PIN=8",))
        self.run_case("bt_state", "state")


if __name__ == "__main__":
    unittest.main()
