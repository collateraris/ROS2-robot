// Room robot V3 base firmware, SERIAL PROTOCOL V4. INCOMPATIBLE with pulse V2.
// Arduino Uno / classic Nano ATmega328P, USB 115200. Rewire motor pins FIRST.
// External quadrature encoders required. No commanded/PWM-based odometry.
#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#if !defined(__AVR_ATmega328P__) && !defined(ROOM_BASE_HOST_TEST)
#error "This firmware targets Uno/classic Nano ATmega328P"
#endif
#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
void clearWatchdog(void) __attribute__((naked,used,section(".init3")));
void clearWatchdog(void) { MCUSR=0; wdt_disable(); }
#define ATOMIC MCU_ATOMIC_BLOCK
#define MCU_ATOMIC_BLOCK ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
#define ATOMIC for(bool once=true;once;once=false)
#endif

// NEW wiring: D2 and D3 are reserved for encoder interrupts.
constexpr uint8_t L1=7,L2=8,R1=4,R2=5,ENA=9,ENB=6;
constexpr uint8_t LEFT_A=2,RIGHT_A=3,LEFT_B=A1,RIGHT_B=A2;
// Set signs with wheels raised; positive target AND positive count = forward.
constexpr int LEFT_MOTOR_SIGN=-1,RIGHT_MOTOR_SIGN=-1;
constexpr int LEFT_ENCODER_SIGN=1,RIGHT_ENCODER_SIGN=1;
// PLACEHOLDERS: calibrate counts/metre for x2 decoding (A CHANGE, B read).
// Host ticks_per_meter must match this constant; must not be guessed in use.
constexpr uint32_t TICKS_PER_METER=1000;
constexpr int MAX_MM_S=150,MAX_PWM=120;
constexpr uint16_t COMMAND_WATCHDOG_MS=250,LOOP_WATCHDOG_MS=50;
constexpr float KP=.35f,KI=.8f; // conservative starting gains, tune raised first

volatile uint32_t leftTicks=0,rightTicks=0;
volatile bool armed=false,stopPending=false;
volatile int16_t leftTarget=0,rightTarget=0;
volatile uint32_t lastCommand=0,loopHeartbeat=0;
uint32_t bootId=0,nonce=0,lastSeq=0;
uint32_t prevControl=0,prevFeedback=0,prevLeft=0,prevRight=0;
float leftIntegral=0,rightIntegral=0;
char rx[128]; uint8_t used=0; uint32_t lineStart=0;
constexpr uint8_t TX_SLOTS=4,TX_BYTES=100;
char tx[TX_SLOTS][TX_BYTES];uint8_t txLength[TX_SLOTS]={};
uint8_t txHead=0,txCount=0,txOffset=0;
bool errorPending=false;

uint16_t crc16(const char* p,size_t n) {
  uint16_t crc=0xffff;
  while(n--) {
    crc^=(uint16_t)(uint8_t)*p++<<8;
    for(uint8_t i=0;i<8;i++) crc=(crc&0x8000)?(uint16_t)((crc<<1)^0x1021):(uint16_t)(crc<<1);
  }
  return crc;
}
void wheel(uint8_t a,uint8_t b,uint8_t en,int pwm) {
  analogWrite(en,0);
  digitalWrite(a,pwm>0?HIGH:LOW);digitalWrite(b,pwm<0?HIGH:LOW);
  analogWrite(en,pwm<0?-pwm:pwm);
}
void drive(int left,int right) {
  wheel(L1,L2,ENA,left*LEFT_MOTOR_SIGN);wheel(R1,R2,ENB,right*RIGHT_MOTOR_SIGN);
}
void cut() { drive(0,0);armed=false;leftTarget=rightTarget=0;stopPending=true; }
void stop() { ATOMIC { cut(); } leftIntegral=rightIntegral=0; }
void fail() { stop();errorPending=true;used=0; }
void timerTick() {
  uint32_t now=millis();
  if(armed && ((uint32_t)(now-lastCommand)>=COMMAND_WATCHDOG_MS ||
               (uint32_t)(now-loopHeartbeat)>=LOOP_WATCHDOG_MS)) cut();
}
#if defined(__AVR__)
ISR(TIMER2_COMPA_vect) { timerTick(); }
#endif
void leftEncoder() {
  int direction=(digitalRead(LEFT_A)==digitalRead(LEFT_B)?1:-1)*LEFT_ENCODER_SIGN;
  if(direction>0) leftTicks++;else leftTicks--;
}
void rightEncoder() {
  int direction=(digitalRead(RIGHT_A)==digitalRead(RIGHT_B)?1:-1)*RIGHT_ENCODER_SIGN;
  if(direction>0) rightTicks++;else rightTicks--;
}
bool queueBody(const char* body) {
  if(txCount==TX_SLOTS) return false;
  uint8_t slot=(txHead+txCount)%TX_SLOTS;
  int n=snprintf(tx[slot],TX_BYTES,"%s*%04X\n",body,(unsigned)crc16(body,strlen(body)));
  if(n<=0||n>=TX_BYTES) return false;
  txLength[slot]=(uint8_t)n;txCount++;return true;
}
void pumpTx() {
  if(!txCount) return;
  int avail=Serial.availableForWrite();if(avail<=0)return;
  uint8_t remaining=txLength[txHead]-txOffset;
  uint8_t n=remaining<(unsigned)avail?remaining:(uint8_t)avail;
  size_t written=Serial.write((const uint8_t*)tx[txHead]+txOffset,n);
  if(written>n)written=n;
  txOffset+=(uint8_t)written;
  if(txOffset==txLength[txHead]){txOffset=0;txHead=(txHead+1)%TX_SLOTS;txCount--;}
}

// Wear-levelled monotonic boot identity. Corrupt committed records lock output;
// an interrupted unpublished write can be retried. Never silently reuse boot 1.
constexpr uint8_t BOOT_SLOTS=32,BOOT_BYTES=8;
bool bootRecord(uint8_t slot,uint32_t& value) {
  int a=slot*BOOT_BYTES;
  if(EEPROM.read(a)!=0xa5||EEPROM.read(a+7)!=0x5a)return false;
  char bytes[4];value=0;
  for(uint8_t i=0;i<4;i++){bytes[i]=(char)EEPROM.read(a+1+i);value|=(uint32_t)(uint8_t)bytes[i]<<(8*i);}
  uint16_t crc=(uint16_t)EEPROM.read(a+5)|((uint16_t)EEPROM.read(a+6)<<8);
  return value&&crc==crc16(bytes,4);
}
uint32_t nextBoot() {
  uint32_t highest=0;uint8_t previous=BOOT_SLOTS-1,interrupted=0,interruptedSlots=0;
  bool corrupt=false;
  for(uint8_t slot=0;slot<BOOT_SLOTS;slot++) {
    bool blank=true;
    for(uint8_t i=0;i<BOOT_BYTES;i++)if(EEPROM.read(slot*BOOT_BYTES+i)!=0xff)blank=false;
    uint32_t value=0;bool valid=bootRecord(slot,value);
    if(valid&&value>highest){highest=value;previous=slot;}
    if(!valid&&!blank){if(EEPROM.read(slot*BOOT_BYTES)!=0)corrupt=true;
      else{interruptedSlots++;interrupted=slot;}}
  }
  if(corrupt||highest==UINT32_MAX||interruptedSlots>1||
      (interruptedSlots&&interrupted!=(previous+1)%BOOT_SLOTS))return 0;
  uint32_t value=highest+1;uint8_t slot=(previous+1)%BOOT_SLOTS;int a=slot*BOOT_BYTES;
  EEPROM.update(a,0);EEPROM.update(a+7,0);char bytes[4];
  for(uint8_t i=0;i<4;i++){bytes[i]=(char)(value>>(8*i));EEPROM.update(a+1+i,(uint8_t)bytes[i]);}
  uint16_t crc=crc16(bytes,4);EEPROM.update(a+5,(uint8_t)crc);EEPROM.update(a+6,(uint8_t)(crc>>8));
  EEPROM.update(a+7,0x5a);EEPROM.update(a,0xa5);
  uint32_t verify=0;return bootRecord(slot,verify)&&verify==value?value:0;
}
bool number(const char* s,uint32_t& out) {
  if(!s||!*s||strlen(s)>10)return false;
  uint32_t n=0;
  for(;*s;s++){if(*s<'0'||*s>'9')return false;uint8_t d=*s-'0';
    if(n>(UINT32_MAX-d)/10)return false;
    n=n*10+d;}
  out=n;return true;
}
bool signedSpeed(const char* s,int& out) {
  bool neg=*s=='-';if(neg)s++;uint32_t n=0;
  if(!number(s,n)||n>MAX_MM_S)return false;
  out=neg?-(int)n:(int)n;return true;
}
void process() {
  rx[used]=0;char* star=strrchr(rx,'*');
  if(!star||star==rx||strlen(star+1)!=4){fail();return;}
  uint16_t supplied=0;
  for(uint8_t i=1;i<=4;i++){
    char c=star[i];uint8_t d;
    if(c>='0'&&c<='9')d=c-'0';else if(c>='A'&&c<='F')d=c-'A'+10;
    else if(c>='a'&&c<='f')d=c-'a'+10;else{fail();return;}
    supplied=(supplied<<4)|d;
  }
  if(crc16(rx,star-rx)!=supplied){fail();return;}*star=0;
  if(*rx==' '||star[-1]==' '||strstr(rx,"  ")){fail();return;}
  char* tokens[6];uint8_t count=0;char* save=nullptr;
  for(char* s=strtok_r(rx," ",&save);s;s=strtok_r(nullptr," ",&save)){
    if(count==6){fail();return;}tokens[count++]=s;
  }
  char out[84];
  if(count==1&&!strcmp(tokens[0],"S")){stop();return;}
  if(count==1&&!strcmp(tokens[0],"Q")){
    snprintf(out,sizeof(out),"V 4 %lu %u 250 %lu",(unsigned long)bootId,(unsigned)MAX_MM_S,(unsigned long)TICKS_PER_METER);
    if(!queueBody(out))fail();
    return;
  }
  uint32_t b=0,n=0,s=0;
  if(count<3||!number(tokens[1],b)||!number(tokens[2],n)||!bootId||b!=bootId||!n){fail();return;}
  if(count==3&&!strcmp(tokens[0],"H")){
    // New nonce requires an explicit disarmed handshake; no auto-rearm.
    if(armed||n<=nonce||txCount){fail();return;}
    nonce=n;lastSeq=0;leftIntegral=rightIntegral=0;
    snprintf(out,sizeof(out),"R %lu %lu",(unsigned long)bootId,(unsigned long)nonce);
    if(!queueBody(out)){fail();return;}
    ATOMIC{leftTarget=rightTarget=0;lastCommand=millis();armed=true;}
    return;
  }
  int left=0,right=0;
  if(count!=6||strcmp(tokens[0],"C")||!armed||n!=nonce||!number(tokens[3],s)||
      !s||s<=lastSeq||!signedSpeed(tokens[4],left)||!signedSpeed(tokens[5],right)||
      txCount>=TX_SLOTS-1||Serial.availableForWrite()<=0){fail();return;}
  lastSeq=s;uint32_t now=millis();
  snprintf(out,sizeof(out),"A %lu %lu %lu %d %d %lu",(unsigned long)bootId,
      (unsigned long)nonce,(unsigned long)s,left,right,(unsigned long)now);
  if(!queueBody(out)){fail();return;}
  ATOMIC{leftTarget=left;rightTarget=right;lastCommand=now;}
}

int controlWheel(int target,float measured,float dt,float& integral) {
  if(!target){integral=0;return 0;}
  float error=target-measured,candidate=integral+KI*error*dt;
  float value=KP*error+candidate;
  // Conditional integration: no winding up against output saturation.
  if((value<MAX_PWM&&value>-MAX_PWM)||(value>=MAX_PWM&&error<0)||(value<=-MAX_PWM&&error>0))integral=candidate;
  value=KP*error+integral;
  if(value>MAX_PWM)value=MAX_PWM;
  if(value<-MAX_PWM)value=-MAX_PWM;
  return (int)value;
}
void control(uint32_t now) {
  uint32_t elapsed=now-prevControl;if(elapsed<20)return;
  uint32_t l,r;int lt,rt;bool live;
  ATOMIC{l=leftTicks;r=rightTicks;lt=leftTarget;rt=rightTarget;live=armed;}
  float dt=elapsed*.001f;
  float ls=(int32_t)(l-prevLeft)*1000.f/(TICKS_PER_METER*dt);
  float rs=(int32_t)(r-prevRight)*1000.f/(TICKS_PER_METER*dt);
  prevControl=now;prevLeft=l;prevRight=r;
  if(!live){leftIntegral=rightIntegral=0;return;}
  int lp=controlWheel(lt,ls,dt,leftIntegral),rp=controlWheel(rt,rs,dt,rightIntegral);
  // Check under the same lock as output: ISR cutoff cannot be undone by PI.
  ATOMIC{if(armed)drive(lp,rp);else drive(0,0);}
}
void feedback(uint32_t now) {
  if(now-prevFeedback<50)return;
  prevFeedback=now;
  if(txCount>=TX_SLOTS-1){fail();return;}
  uint32_t l,r;bool live;ATOMIC{l=leftTicks;r=rightTicks;live=armed;}
  char out[84];snprintf(out,sizeof(out),"F %lu %lu %lu %lu %lu %u",(unsigned long)bootId,
      (unsigned long)nonce,(unsigned long)now,(unsigned long)l,(unsigned long)r,(unsigned)live);
  if(!queueBody(out))fail();
}
void reportStop() {
  bool pending;ATOMIC{pending=stopPending;}
  if(pending&&queueBody("D")){ATOMIC{stopPending=false;}}
  if(errorPending&&queueBody("E"))errorPending=false;
}
void setup() {
  const uint8_t pins[]={L1,L2,R1,R2,ENA,ENB};
  for(uint8_t i=0;i<sizeof(pins);i++)pinMode(pins[i],OUTPUT);
  drive(0,0);armed=false;
  pinMode(LEFT_A,INPUT_PULLUP);pinMode(RIGHT_A,INPUT_PULLUP);
  pinMode(LEFT_B,INPUT_PULLUP);pinMode(RIGHT_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_A),leftEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A),rightEncoder,CHANGE);
  bootId=nextBoot();Serial.begin(115200);
  prevControl=prevFeedback=loopHeartbeat=millis();
#if defined(__AVR__)
  static_assert(F_CPU==16000000UL,"requires 16 MHz Uno/Nano");
  ATOMIC{TCCR2A=_BV(WGM21);TCCR2B=_BV(CS22);TCNT2=0;OCR2A=249;TIMSK2=_BV(OCIE2A);}
  wdt_enable(WDTO_250MS);
#endif
}
void loop() {
  ATOMIC{timerTick();loopHeartbeat=millis();}
  reportStop();pumpTx();uint32_t now=millis();
  // Bound parsing work; never block waiting for newline or UART output.
  for(uint8_t budget=0;budget<32&&Serial.available();budget++){
    char c=Serial.read();
    if(c=='\n'){if(used)process();used=0;}
    else if(c!='\r'){
      if(c<32||c>126||used>=sizeof(rx)-1){fail();break;}
      if(!used)lineStart=now;
      rx[used++]=c;
    }
  }
  if(used&&(uint32_t)(now-lineStart)>100)fail();
  control(now);feedback(now);reportStop();pumpTx();
#if defined(__AVR__)
  wdt_reset();
#endif
}
