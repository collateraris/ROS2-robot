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
#include "../room_base_v3.ino"

void drain(){while(txCount)pumpTx();}
void command(const char* body){
  drain();
  used=(uint8_t)snprintf(rx,sizeof(rx),"%s*%04X",body,(unsigned)crc16(body,strlen(body)));
  process();used=0;drain();
}
void reports(){reportStop();drain();}
void arm(uint32_t session){
  reports();char body[64];snprintf(body,sizeof(body),"H %lu %lu",(unsigned long)bootId,(unsigned long)session);
  command(body);assert(armed);assert(leftTarget==0&&rightTarget==0);
}
int main(){
  setup();assert(bootId==1&&!armed);assert(nextBoot()==2);
  assert(crc16("123456789",9)==0x29b1);
  command("Q");assert(Serial.output.find("V 4 1 150 250 1000*")!=std::string::npos);
  arm(1);command("C 1 1 1 -100 100");assert(armed&&leftTarget==-100&&rightTarget==100);
  assert(Serial.output.find("A 1 1 1 -100 100 0*")!=std::string::npos);
  // PI uses measured ticks, not only a target. A positive error yields output.
  fakeNow=20;control(fakeNow);assert(pwmValues[ENA]>0&&pwmValues[ENB]>0);
  command("C 1 1 1 100 100");assert(!armed); // repeated sequence locks
  assert(pwmValues[ENA]==0&&pwmValues[ENB]==0);reports();
  arm(2);command("C 1 2 2 150 150");assert(armed);
  fakeNow=270;loopHeartbeat=fakeNow;timerTick();assert(!armed); // MCU command watchdog
  reports();command("H 1 1");assert(!armed);reports(); // replayed old session rejected
  arm(3);fakeNow=321;timerTick();assert(!armed);reports(); // main-loop stall watchdog
  arm(4);command("C 1 4 1 151 0");assert(!armed);reports(); // speed bound
  arm(5);command("P 1 5 1 100 100");assert(!armed);reports(); // no pulse fallback
  arm(6);strcpy(rx,"C 1 6 1 100 100*0000");used=(uint8_t)strlen(rx);process();assert(!armed);reports();
  arm(7);command("S");assert(!armed);reports();
  // Physical A/B ISR logic generates signed counts and unsigned wrapping.
  pinValues[LEFT_A]=pinValues[LEFT_B]=1;leftEncoder();assert(leftTicks==1);
  pinValues[LEFT_B]=0;leftEncoder();assert(leftTicks==0);
  leftEncoder();assert(leftTicks==0xffffffffUL);
  std::cout<<"firmware host state/CRC/watchdog tests passed; hardware timing NOT emulated\n";
}
