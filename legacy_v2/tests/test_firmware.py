"""Compile the actual sketch in a host Arduino stub and inject faults.

This tests C++ parser/state/cutoff/EEPROM-journal logic. It does not compile AVR
machine code, measure interrupt jitter, validate bootloader/watchdog behavior,
or assert anything about wiring, braking, battery or physical robot safety.
"""
from pathlib import Path
import shutil
import subprocess

import pytest

ARDUINO_H = r'''
#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <deque>
#include <string>
constexpr int HIGH=1, LOW=0, OUTPUT=1, A0=14;
inline uint32_t fakeNow=1000;
inline int fakePWM[32]={}, fakeDigital[32]={};
inline uint32_t millis() { return fakeNow; }
inline void pinMode(uint8_t,int) {}
inline void digitalWrite(uint8_t p,int v) { fakeDigital[p]=v; }
inline void analogWrite(uint8_t p,int v) { fakePWM[p]=v; }
struct SerialStub {
  std::deque<char> rx; std::string tx; int writable=63;
  void begin(int) {}
  int available() { return (int)rx.size(); }
  int availableForWrite() { return writable; }
  int read() { char c=rx.front(); rx.pop_front(); return (unsigned char)c; }
  size_t write(const uint8_t* s,size_t n) {
    assert(n<=(size_t)writable); writable-=(int)n; tx.append((const char*)s,n); return n;
  }
  void input(const std::string& s) { for(char c:s) rx.push_back(c); }
};
inline SerialStub Serial;
'''
SOFTWARE_SERIAL_H = r'''
#pragma once
#include "Arduino.h"
struct SoftwareSerial: SerialStub { SoftwareSerial(uint8_t,uint8_t){} };
'''
EEPROM_H = r'''
#pragma once
#include <array>
#include <cstdint>
struct EEPROMStub {
  std::array<uint8_t,1024> data;
  bool dropWrites=false;
  EEPROMStub() { data.fill(0xFF); }
  uint8_t read(int a) { return data.at(a); }
  void update(int a,uint8_t b) { if(!dropWrites) data.at(a)=b; }
};
inline EEPROMStub EEPROM;
'''
HARNESS = r'''
#include <cassert>
#include <iostream>
#include <string>
#include "SKETCH_PATH"

std::string owner() { return std::to_string(bootId)+" 123"; }
void raw(const std::string& s) {
  Serial.input(s);
  for(int i=0;Serial.available() && i<1000;i++) loop();
  loop();
}
void body(const std::string& b) {
  char checksum[8]; snprintf(checksum,sizeof(checksum),"*%04X\n",crc16(b.data(),b.size()));
  raw(b+checksum);
}
void advance(unsigned ms,bool mainLoop=true,bool txProgress=true) {
  // UART capacity recovers at ~9600 baud (10 bits/byte). Deliberately bounded;
  // pumpTx() cannot write more than currently available capacity.
  static unsigned fraction=0;
  for(unsigned i=0;i<ms;i++) {
    fakeNow++; fraction+=960;
    if(fraction>=1000) { fraction-=1000; if(txProgress) Serial.writable=std::min(63,Serial.writable+1); }
    timerTick();
    if(mainLoop) loop();
  }
}
void unlock() { advance(100); body("Z "+owner()); assert(!locked && !active); }
void start(const std::string& duration="80") {
  advance(100);
  body("P "+owner()+" 1 120 90 "+duration);
  assert(active && !locked && fakePWM[ENA]==120 && fakePWM[ENB]==90);
  assert(fakeDigital[L1]==LOW && fakeDigital[L2]==HIGH);
}
void zero() { assert(!active && fakePWM[ENA]==0 && fakePWM[ENB]==0); }
int main(int argc,char**argv) {
  assert(argc==2); setup(); zero(); assert(locked && bootId==1);
  std::string c=argv[1];
  if(c=="deadline") {
    unlock(); start("100"); uint32_t begin=fakeNow;
    advance(99); assert(active); advance(1); zero();
    assert(pulseStop-pulseStart==100 && stopReason==COMPLETE);
    advance(300);
    assert(Serial.tx.find("C 1 123 1 "+std::to_string(begin)+"*")!=std::string::npos);
    assert(Serial.tx.find("A 1 123 1 "+std::to_string(begin)+" 120 90*")!=std::string::npos);
    assert(Serial.tx.find("D 1 123 1 "+std::to_string(begin+100)+" 0*")!=std::string::npos);
  } else if(c=="repeated") {
    unlock(); start(); advance(20); body("P "+owner()+" 1 120 90 80");
    zero(); assert(locked && pulseStop-pulseStart<=20);
    advance(500); body("Z "+owner()); assert(locked);
    body("P "+owner()+" 2 120 90 80"); zero(); assert(locked);
    advance(500); body("Z 1 456"); assert(!locked); advance(100);
    body("P 1 456 1 120 90 80"); assert(active);
  } else if(c=="fresh_seq_overlap") {
    unlock(); start(); advance(20); body("P "+owner()+" 2 120 90 100");
    zero(); assert(locked && pulseStop-pulseStart<=20);
  } else if(c=="locked") {
    body("P "+owner()+" 1 120 90 80"); zero(); assert(locked);
    body("Q"); zero(); assert(locked);
  } else if(c=="corruption") {
    unlock(); start(); advance(10); raw("P "+owner()+" 2 120 90 80*0000\n");
    zero(); assert(locked);
  } else if(c=="overflow_fields") {
    unlock(); body("P "+owner()+" 1 120 90 80 1 2 3"); zero(); assert(locked);
  } else if(c=="integer_overflow") {
    unlock(); body("P "+owner()+" 1 4294967296 90 80"); zero(); assert(locked);
  } else if(c=="signed") {
    unlock(); body("P "+owner()+" 1 -1 90 80"); zero(); assert(locked);
  } else if(c=="suffix") {
    unlock(); body("P "+owner()+" 1 120x 90 80"); zero(); assert(locked);
  } else if(c=="duration") {
    unlock(); body("P "+owner()+" 1 120 90 101"); zero(); assert(locked);
  } else if(c=="overspeed") {
    unlock(); body("P "+owner()+" 1 141 90 80"); zero(); assert(locked);
  } else if(c=="wrong_nonce") {
    unlock(); body("P 1 456 1 120 90 80"); zero(); assert(locked);
  } else if(c=="wrong_boot") {
    unlock(); body("P 2 123 1 120 90 80"); zero(); assert(locked);
  } else if(c=="partial_line") {
    unlock(); raw("P "+owner()+" 1 "); advance(201); zero(); assert(locked);
  } else if(c=="receive_overflow") {
    unlock(); raw(std::string(150,'x')+"\n"); zero(); assert(locked);
  } else if(c=="bt") {
    unlock(); bt.input("FBRL"); loop(); zero(); assert(!locked);
    start(); advance(7); bt.input("S"); loop(); zero(); assert(locked && stopReason==BT_STOP);
    assert(pulseStop-pulseStart==7);
  } else if(c=="tx_blocked") {
    unlock(); advance(100); Serial.writable=0;
    body("P "+owner()+" 1 120 90 80"); zero(); assert(locked);
  } else if(c=="tx_stalls_during_pulse") {
    unlock(); start("100"); Serial.writable=0;
    advance(100,true,false); zero(); assert(pulseStop-pulseStart==100);
    assert(stopReason==COMPLETE); advance(600);
    assert(Serial.tx.find("D 1 123 1 ")!=std::string::npos);
  } else if(c=="watchdog_main_stall") {
    unlock(); start(); advance(39,false); assert(active);
    advance(1,false); zero(); assert(locked && stopReason==WATCHDOG);
    assert(pulseStop-pulseStart==40); advance(400);
    assert(Serial.tx.find("D 1 123 1 "+std::to_string(pulseStop)+" 4*")!=std::string::npos);
  } else if(c=="wrap") {
    unlock(); advance(100); fakeNow=0xFFFFFFF0u;
    body("P "+owner()+" 1 120 90 100"); assert(active);
    advance(99); assert(active); advance(1); zero();
    assert((uint32_t)(pulseStop-pulseStart)==100);
  } else if(c=="rx_starvation") {
    unlock(); start("100"); Serial.input(std::string(1000,'X'));
    advance(100); zero(); assert(pulseStop-pulseStart<=100);
  } else if(c=="bt_starvation") {
    unlock(); start("100"); bt.input(std::string(1000,'F'));
    advance(100); zero(); assert(pulseStop-pulseStart==100);
  } else if(c=="stop") {
    unlock(); start(); advance(5); body("S 999 999"); zero(); assert(locked);
    assert(pulseStop-pulseStart==5 && stopReason==HOST_STOP);
  } else if(c=="ack_order_abort") {
    unlock(); start(); advance(10); body("S 999 999"); advance(500);
    auto a=Serial.tx.find("C 1 123 1 "), b=Serial.tx.find("A 1 123 1 "), d=Serial.tx.find("D 1 123 1 ");
    assert(a!=std::string::npos && a<b && b<d);
  } else if(c=="boot_counter") {
    for(uint32_t i=2;i<=100;i++) assert(nextBoot()==i); // wraps journal slots, never boot id
    EEPROM.dropWrites=true; assert(nextBoot()==0); // never use uncommitted identity
  } else if(c=="boot_corrupt") {
    EEPROM.data.fill(0); assert(nextBoot()==0);
  } else if(c=="boot_corrupt_committed") {
    assert(nextBoot()==2); EEPROM.data[8+5]^=1;
    assert(nextBoot()==0); // cannot fall back to boot 1 and reuse boot 2
  } else if(c=="boot_interrupted_write") {
    EEPROM.data[8]=0; // next journal slot was invalidated before power loss
    assert(nextBoot()==2); // old committed boot 1 was preserved
  } else if(c=="boot_interrupted_first_write") {
    EEPROM.data.fill(0xFF); EEPROM.data[0]=0;
    assert(nextBoot()==1); // no prior identity was ever committed or used
  } else if(c=="boot_exhaustion") {
    EEPROM.data.fill(0xFF); uint32_t n=UINT32_MAX; char b[4];
    for(int i=0;i<4;i++) { b[i]=(char)(n>>(8*i)); EEPROM.data[1+i]=(uint8_t)b[i]; }
    uint16_t crc=crc16(b,4); EEPROM.data[0]=0xA5; EEPROM.data[7]=0x5A;
    EEPROM.data[5]=(uint8_t)crc; EEPROM.data[6]=(uint8_t)(crc>>8);
    assert(nextBoot()==0);
  } else if(c=="heartbeat") {
    unlock(); advance(100); body("T "+owner()+" 9"); advance(100);
    assert(Serial.tx.find("T 1 123 9 ")!=std::string::npos); zero();
  } else if(c=="strict_spacing") {
    unlock(); body("P  1 123 1 120 90 80"); zero(); assert(locked);
  } else if(c=="max_uint_packets") {
    unlock(); advance(100); bootId=nonce=0xFFFFFFFFu; seq=0xFFFFFFFEu; haveSeq=false;
    body("P 4294967295 4294967295 4294967294 140 140 100"); assert(active);
    advance(600); zero();
    assert(Serial.tx.find("A 4294967295 4294967295 4294967294 ")!=std::string::npos);
    assert(Serial.tx.find("D 4294967295 4294967295 4294967294 ")!=std::string::npos);
  } else { assert(false); }
  std::cout << "PASS " << c << "\n";
}
'''


@pytest.fixture(scope="module")
def firmware_binary(tmp_path_factory):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ is needed for the host-only Arduino logic harness")
    build = tmp_path_factory.mktemp("arduino_stub")
    (build / "Arduino.h").write_text(ARDUINO_H)
    (build / "SoftwareSerial.h").write_text(SOFTWARE_SERIAL_H)
    (build / "EEPROM.h").write_text(EEPROM_H)
    sketch = Path(__file__).parents[1] / "firmware" / "roomwm_drive" / "roomwm_drive.ino"
    source = build / "main.cpp"
    source.write_text(HARNESS.replace("SKETCH_PATH", str(sketch)))
    binary = build / "firmware_test"
    result = subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-D__AVR_ATmega328P__", "-fsanitize=undefined", "-fno-sanitize-recover=all",
        "-I", str(build), str(source), "-o", str(binary)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return binary


@pytest.mark.parametrize("case", [
    "deadline", "repeated", "fresh_seq_overlap", "locked", "corruption", "overflow_fields",
    "integer_overflow", "signed", "suffix", "duration", "overspeed", "wrong_nonce", "wrong_boot",
    "partial_line", "receive_overflow", "bt", "tx_blocked", "tx_stalls_during_pulse",
    "watchdog_main_stall", "wrap", "rx_starvation", "bt_starvation", "stop", "ack_order_abort",
    "boot_counter", "boot_corrupt", "boot_corrupt_committed", "boot_interrupted_write",
    "boot_interrupted_first_write", "boot_exhaustion", "heartbeat", "strict_spacing", "max_uint_packets",
])
def test_actual_firmware_logic(firmware_binary, case):
    result = subprocess.run([str(firmware_binary), case], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.strip() == f"PASS {case}"
