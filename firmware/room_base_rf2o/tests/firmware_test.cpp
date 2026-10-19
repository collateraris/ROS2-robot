#define ROOM_BASE_HOST_TEST 1
#include "Arduino.h"
#include "EEPROM.h"
#include <cassert>
#include <cstdio>
#include <iostream>
uint32_t fakeNow=0;
int pinValues[32]={},pwmValues[32]={};
SerialMock Serial;
EEPROMMock EEPROM;
#include "../room_base_rf2o.ino"

void drain(){while(txCount)pumpTx();}
void command(const char* body){
  drain();
  used=(uint8_t)snprintf(rx,sizeof(rx),"%s*%04X",body,(unsigned)crc16(body,strlen(body)));
  process();used=0;drain();
}
void reports(){reportStop();drain();}
void arm(uint32_t session){
  reports();char body[64];snprintf(body,sizeof(body),"H %lu %lu",(unsigned long)bootId,(unsigned long)session);
  command(body);assert(armed);assert(leftPWM==0&&rightPWM==0);
}
int main(){
  setup();assert(bootId==1&&!armed);assert(nextBoot()==2);
  assert(crc16("123456789",9)==0x29b1);
  command("Q");assert(Serial.output.find("V 5 1 120 250*")!=std::string::npos);
  arm(1);command("P 1 1 1 -60 120");
  assert(armed&&leftPWM==-60&&rightPWM==120);
  assert(pwmValues[ENA]==60&&pwmValues[ENB]==120);
  assert(pinValues[L1]==HIGH&&pinValues[L2]==LOW);
  assert(pinValues[R1]==LOW&&pinValues[R2]==HIGH);
  assert(Serial.output.find("A 1 1 1 -60 120 0*")!=std::string::npos);
  // Continuous output: no scheduled pulse or minimum-impulse cutoff.
  fakeNow=200;loopHeartbeat=fakeNow;timerTick();
  assert(armed&&pwmValues[ENA]==60&&pwmValues[ENB]==120);
  feedback(fakeNow);drain();
  assert(Serial.output.find("M 1 1 200 -60 120 1*")!=std::string::npos);
  // Exact zero command cuts both outputs at acceptance, still in same session.
  command("P 1 1 2 0 0");assert(armed&&pwmValues[ENA]==0&&pwmValues[ENB]==0);
  command("P 1 1 2 100 100");assert(!armed);reports(); // duplicate rejects
  arm(2);command("P 1 2 1 120 120");assert(armed);
  fakeNow=450;loopHeartbeat=fakeNow;timerTick(); // command watchdog
  assert(!armed&&pwmValues[ENA]==0&&pwmValues[ENB]==0);reports();
  command("H 1 1");assert(!armed);reports(); // old nonce cannot rearm
  command("P 1 2 2 120 120");assert(!armed);reports(); // no spontaneous resume
  arm(3);fakeNow=501;timerTick();assert(!armed);reports(); // main loop watchdog
  arm(4);command("P 1 4 1 121 0");assert(!armed);reports(); // PWM bound
  arm(5);command("C 1 5 1 100 100");assert(!armed);reports(); // v4 rejected
  arm(6);strcpy(rx,"P 1 6 1 100 100*0000");used=(uint8_t)strlen(rx);
  process();assert(!armed);reports(); // CRC cuts immediately
  arm(7);command("P 1 7 1 100 -100");command("S");
  assert(!armed&&pwmValues[ENA]==0&&pwmValues[ENB]==0);reports();
  // Corrupt committed boot records lock output rather than replay an identity.
  EEPROM.data[5]^=1;assert(nextBoot()==0);
  std::cout<<"encoderless firmware CRC/session/PWM/watchdog tests passed; physical timing NOT emulated\n";
}
