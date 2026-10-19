// Pulse autonomy firmware, serial protocol 3 / action schema 2.
// Uno R3 / classic Nano ATmega328P; USB 9600 8N1. NO legacy M fallback.
// Pin mapping is inherited; verify direction, A0 purpose and driver pull-downs.
// TIMER2 IS RESERVED: 1 kHz independent pulse cutoff / loop-liveness watchdog.
// Timer0 (millis) and Timer1 (PWM pin 9) are preserved; ENB pin 6 uses Timer0.
// Bluetooth F/B/R/L are ignored; S/D cut PWM and latch. PWM OFF != physical rest.
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if !defined(__AVR_ATmega328P__)
#error "Use Uno R3 / classic Nano ATmega328P"
#endif
#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
uint8_t resetFlags __attribute__((section(".noinit")));
void captureResetFlags(void) __attribute__((naked, used, section(".init3")));
void captureResetFlags(void) { resetFlags=MCUSR; MCUSR=0; wdt_disable(); }
#define MCU_ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
// Host test invokes the same timerTick() helper explicitly. It does not emulate
// AVR interrupt scheduling, electrical outputs or real watchdog reset behavior.
uint8_t resetFlags=0;
#define MCU_ATOMIC for(bool once=true;once;once=false)
#endif

constexpr uint8_t L1=3, L2=2, R1=4, R2=5, ENB=6, ENA=9;
constexpr uint8_t BT_RX=10, BT_TX=11;
constexpr int LEFT_SIGN=-1, RIGHT_SIGN=-1;
constexpr uint16_t MAX_PULSE_MS=100, MAX_PWM=140, LOOP_WATCHDOG_MS=40;
constexpr uint8_t COMPLETE=0, HOST_STOP=1, BT_STOP=2, PROTOCOL_FAULT=3, WATCHDOG=4, ACK_BACKPRESSURE=5;
SoftwareSerial bt(BT_RX, BT_TX);
volatile bool locked=true, active=false, donePending=false;
bool haveSeq=false;
uint32_t bootId=0, nonce=0, seq=0;
volatile uint32_t pulseStart=0, pulseStop=0, loopHeartbeat=0;
volatile uint16_t pulseDuration=0;
volatile uint8_t stopReason=COMPLETE;
char rx[96]; uint8_t used=0; uint32_t lineStart=0;

// Bounded application queue. No Serial.print/flush and no wait for UART space.
// Three frames are reserved before PWM starts: ACCEPT, START, eventual STOP.
constexpr uint8_t TX_SLOTS=6, TX_SIZE=84;
char tx[TX_SLOTS][TX_SIZE]; uint8_t txLength[TX_SLOTS]={};
uint8_t txHead=0, txCount=0, txOffset=0;
bool faultPending=false; uint8_t faultReason=PROTOCOL_FAULT;

uint16_t crc16(const char* p, size_t n) {
  uint16_t crc=0xFFFF;
  while(n--) {
    crc ^= (uint16_t)(uint8_t)*p++ << 8;
    for(uint8_t i=0;i<8;i++) crc=(crc&0x8000) ? (uint16_t)((crc<<1)^0x1021) : (uint16_t)(crc<<1);
  }
  return crc;
}
bool queueBody(const char* body) {
  if(txCount==TX_SLOTS) return false;
  uint8_t slot=(txHead+txCount)%TX_SLOTS;
  int n=snprintf(tx[slot],TX_SIZE,"%s*%04X\n",body,(unsigned)crc16(body,strlen(body)));
  if(n<=0 || n>=TX_SIZE) return false;
  txLength[slot]=(uint8_t)n; txCount++; return true;
}
void pumpTx() {
  if(!txCount) return;
  int available=Serial.availableForWrite();
  if(available<=0) return;
  uint8_t remaining=txLength[txHead]-txOffset;
  uint8_t n=remaining<(unsigned)available ? remaining:(uint8_t)available;
  size_t written=Serial.write((const uint8_t*)tx[txHead]+txOffset,n);
  if(written>n) written=n;
  txOffset+=(uint8_t)written;
  if(txOffset==txLength[txHead]) { txOffset=0; txHead=(txHead+1)%TX_SLOTS; txCount--; }
}
void wheel(uint8_t a,uint8_t b,uint8_t en,int value) {
  analogWrite(en,0);
  digitalWrite(a,value>0 ? HIGH:LOW); digitalWrite(b,value<0 ? HIGH:LOW);
  analogWrite(en,value<0 ? -value:value);
}
void drive(int left,int right) {
  wheel(L1,L2,ENA,left*LEFT_SIGN); wheel(R1,R2,ENB,right*RIGHT_SIGN);
}
// Called inside an atomic block or Timer2 ISR; only bounded I/O/state writes.
void cutPulse(uint8_t reason,uint32_t now) {
  drive(0,0);
  if(active) { pulseStop=now; stopReason=reason; active=false; donePending=true; }
  if(reason!=COMPLETE) locked=true;
}
void endPulse(uint8_t reason=HOST_STOP) {
  MCU_ATOMIC { cutPulse(reason,millis()); }
}
void timerTick() {
  uint32_t now=millis();
  if(active) {
    // A lost main loop cuts the pulse early and reports its actual elapsed ticks.
    if((uint32_t)(now-loopHeartbeat)>=LOOP_WATCHDOG_MS) cutPulse(WATCHDOG,now);
    else if((uint32_t)(now-pulseStart)>=pulseDuration) cutPulse(COMPLETE,now);
  }
}
#if defined(__AVR__)
ISR(TIMER2_COMPA_vect) { timerTick(); }
#endif
void fail(uint8_t reason=PROTOCOL_FAULT) {
  endPulse(reason); locked=true; used=0; faultReason=reason; faultPending=true;
}
void reportPending() {
  // Formatting/CRC/UART only in the main loop, never in the cutoff interrupt.
  uint32_t stop=0; uint8_t reason=0; bool pending=false;
  MCU_ATOMIC { pending=donePending; stop=pulseStop; reason=stopReason; }
  if(pending) {
    char out[72]; snprintf(out,sizeof(out),"D %lu %lu %lu %lu %u",(unsigned long)bootId,
        (unsigned long)nonce,(unsigned long)seq,(unsigned long)stop,(unsigned)reason);
    if(queueBody(out)) { MCU_ATOMIC { donePending=false; } }
  }
  if(faultPending && !donePending) {
    char out[72]; snprintf(out,sizeof(out),"E %lu %lu %lu %u",(unsigned long)bootId,
        (unsigned long)nonce,(unsigned long)seq,(unsigned)faultReason);
    if(queueBody(out)) faultPending=false;
  }
}

// Wear-levelled EEPROM boot counter: 32 records * 8 bytes, CRC plus commit bytes.
// A new identity is used only after read-back verifies the committed record.
// Completely corrupt storage or uint32 exhaustion locks the firmware, never
// silently returns to boot 1. EEPROM contents are identity metadata, not secrets.
constexpr uint8_t BOOT_SLOTS=32, BOOT_BYTES=8;
bool bootRecord(uint8_t slot,uint32_t& value) {
  int a=slot*BOOT_BYTES;
  if(EEPROM.read(a)!=0xA5 || EEPROM.read(a+7)!=0x5A) return false;
  char bytes[4]; value=0;
  for(uint8_t i=0;i<4;i++) { bytes[i]=(char)EEPROM.read(a+1+i); value|=(uint32_t)(uint8_t)bytes[i]<<(8*i); }
  uint16_t crc=(uint16_t)EEPROM.read(a+5)|((uint16_t)EEPROM.read(a+6)<<8);
  return value && crc==crc16(bytes,4);
}
uint32_t nextBoot() {
  uint32_t highest=0; uint8_t previous=BOOT_SLOTS-1; bool corrupt=false;
  uint8_t interruptedSlots=0,interrupted=0;
  for(uint8_t slot=0;slot<BOOT_SLOTS;slot++) {
    bool blank=true;
    for(uint8_t i=0;i<BOOT_BYTES;i++) if(EEPROM.read(slot*BOOT_BYTES+i)!=0xFF) blank=false;
    uint32_t value=0;
    bool valid=bootRecord(slot,value);
    if(valid && value>highest) { highest=value; previous=slot; }
    // A zero first marker is an interrupted, never-published write. A broken
    // committed record must lock rather than roll back and reuse its identity.
    if(!valid && !blank) {
      if(EEPROM.read(slot*BOOT_BYTES)!=0) corrupt=true;
      else { interruptedSlots++; interrupted=slot; }
    }
  }
  if(corrupt || highest==UINT32_MAX || interruptedSlots>1
      || (interruptedSlots && interrupted!=(previous+1)%BOOT_SLOTS)) return 0;
  uint32_t value=highest+1; uint8_t slot=(previous+1)%BOOT_SLOTS; int a=slot*BOOT_BYTES;
  EEPROM.update(a,0); EEPROM.update(a+7,0);
  char bytes[4];
  for(uint8_t i=0;i<4;i++) { bytes[i]=(char)(value>>(8*i)); EEPROM.update(a+1+i,(uint8_t)bytes[i]); }
  uint16_t crc=crc16(bytes,4);
  EEPROM.update(a+5,(uint8_t)crc); EEPROM.update(a+6,(uint8_t)(crc>>8));
  EEPROM.update(a+7,0x5A); EEPROM.update(a,0xA5);
  uint32_t verify=0; return bootRecord(slot,verify) && verify==value ? value:0;
}

bool number(const char* s,uint32_t& out) {
  if(!s || !*s || strlen(s)>10) return false;
  uint32_t n=0;
  for(;*s;s++) {
    if(*s<'0'||*s>'9') return false;
    uint8_t d=*s-'0'; if(n>(UINT32_MAX-d)/10) return false;
    n=n*10+d;
  }
  out=n; return true;
}
void process() {
  rx[used]=0;
  char* star=strrchr(rx,'*');
  if(!star || strlen(star+1)!=4) { fail(); return; }
  uint16_t supplied=0;
  for(uint8_t i=1;i<=4;i++) {
    char c=star[i]; uint8_t d;
    if(c>='0'&&c<='9') d=c-'0';
    else if(c>='A'&&c<='F') d=c-'A'+10;
    else if(c>='a'&&c<='f') d=c-'a'+10;
    else { fail(); return; }
    supplied=(supplied<<4)|d;
  }
  if(crc16(rx,star-rx)!=supplied) { fail(); return; }
  *star=0;
  if(!*rx || *rx==' ' || star[-1]==' ' || strstr(rx,"  ")) { fail(); return; }
  // P has exactly seven fields. Reject extras before conversion.
  char* tokens[7]; uint8_t count=0; char* save=nullptr;
  for(char* s=strtok_r(rx," ",&save);s;s=strtok_r(nullptr," ",&save)) {
    if(count==7) { fail(); return; } tokens[count++]=s;
  }
  if(count==1 && !strcmp(tokens[0],"Q")) {
    char out[64]; snprintf(out,sizeof(out),"V 3 %lu 100 140 40 250 %u",(unsigned long)bootId,(unsigned)resetFlags);
    queueBody(out); return;
  }
  uint32_t values[6];
  for(uint8_t i=1;i<count;i++) if(!number(tokens[i],values[i-1])) { fail(); return; }
  if(count==3 && !strcmp(tokens[0],"S")) { endPulse(HOST_STOP); locked=true; return; }
  if(count==3 && !strcmp(tokens[0],"Z")) {
    if(active || donePending || faultPending || !bootId || values[0]!=bootId || !values[1]
        || values[1]==nonce) { fail(); return; }
    // Drain every old ACK before resetting its session; never drop a STOP.
    if(txCount) return;
    nonce=values[1]; seq=0; haveSeq=false; locked=false;
    char out[64]; snprintf(out,sizeof(out),"R %lu %lu %lu",(unsigned long)bootId,
        (unsigned long)nonce,(unsigned long)millis());
    if(!queueBody(out)) fail(ACK_BACKPRESSURE);
    return;
  }
  if(count==4 && !strcmp(tokens[0],"T")) {
    if(!bootId || values[0]!=bootId || values[1]!=nonce || !values[2]) { fail(); return; }
    char out[72]; snprintf(out,sizeof(out),"T %lu %lu %lu %lu",(unsigned long)bootId,
        (unsigned long)nonce,(unsigned long)values[2],(unsigned long)millis());
    queueBody(out); return;
  }
  if(count!=7 || strcmp(tokens[0],"P")) { fail(); return; }
  uint32_t b=values[0],n=values[1],s=values[2],l=values[3],r=values[4],duration=values[5];
  if(locked || active || donePending || faultPending || !bootId || b!=bootId || n!=nonce
      || !s || (haveSeq && s<=seq) || l>MAX_PWM || r>MAX_PWM || !(l||r)
      || !duration || duration>MAX_PULSE_MS) { fail(); return; }
  // Reserve queue slots and require UART progress before enabling either wheel.
  // A subsequently blocked UART cannot postpone the independent Timer2 cutoff.
  if(txCount>TX_SLOTS-3 || Serial.availableForWrite()<=0) { fail(ACK_BACKPRESSURE); return; }
  seq=s; haveSeq=true;
  uint32_t accept=millis();
  char out[76]; snprintf(out,sizeof(out),"C %lu %lu %lu %lu",(unsigned long)bootId,
      (unsigned long)nonce,(unsigned long)seq,(unsigned long)accept);
  if(!queueBody(out)) { fail(ACK_BACKPRESSURE); return; }
  MCU_ATOMIC {
    pulseDuration=(uint16_t)duration; pulseStart=millis(); loopHeartbeat=pulseStart;
    stopReason=COMPLETE; active=true; drive((int)l,(int)r);
  }
  snprintf(out,sizeof(out),"A %lu %lu %lu %lu %lu %lu",(unsigned long)bootId,
      (unsigned long)nonce,(unsigned long)seq,(unsigned long)pulseStart,(unsigned long)l,(unsigned long)r);
  if(!queueBody(out)) fail(ACK_BACKPRESSURE);
}
void setup() {
  const uint8_t pins[] = {L1,L2,R1,R2,ENA,ENB};
  for(uint8_t i=0;i<sizeof(pins);i++) pinMode(pins[i],OUTPUT);
  drive(0,0); locked=true; active=false; donePending=false;
  pinMode(A0,OUTPUT); digitalWrite(A0,HIGH);
  bootId=nextBoot(); nonce=seq=0; haveSeq=false; loopHeartbeat=millis();
  Serial.begin(9600); bt.begin(9600);
#if defined(__AVR__)
  // CTC at 1 kHz on 16 MHz Uno/Nano. Timer2 is not used for either motor enable.
  static_assert(F_CPU==16000000UL,"Timer2 configuration requires 16 MHz MCU");
  MCU_ATOMIC { TCCR2A=_BV(WGM21); TCCR2B=_BV(CS22); TCNT2=0; OCR2A=249; TIMSK2=_BV(OCIE2A); }
  wdt_enable(WDTO_250MS);
#endif
}
void loop() {
  // Check before advancing liveness. Host harness uses this same helper when it
  // models a timer tick during a main-loop stall.
  MCU_ATOMIC { timerTick(); loopHeartbeat=millis(); }
  reportPending(); pumpTx();
  uint32_t now=millis();
  for(uint8_t budget=0;budget<8 && bt.available();budget++) {
    char c=bt.read(); if(c=='S'||c=='D') endPulse(BT_STOP);
  }
  for(uint8_t budget=0;budget<24 && Serial.available();budget++) {
    char c=Serial.read();
    if(c=='\n') { if(used) process(); used=0; }
    else if(c!='\r') {
      if(c<32 || c>126) { fail(); break; }
      if(!used) lineStart=now;
      if(used>=sizeof(rx)-1) { fail(); break; }
      rx[used++]=c;
    }
    MCU_ATOMIC { timerTick(); }
  }
  if(used && (uint32_t)(millis()-lineStart)>200) fail();
  reportPending(); pumpTx();
#if defined(__AVR__)
  wdt_reset();
#endif
}
