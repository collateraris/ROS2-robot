/* UNO/Nano ATmega328P, four-wheel skid steer: two LEFT + two RIGHT DC motors.
 * PWM left/right=D5/D6; LEFT DIR=D7/D8, RIGHT DIR=D9/D10.
 * TB6612 STBY=D4; for L298N leave D4 unconnected and remove ENA/ENB jumpers.
 * Check the combined stall current of TWO motors on each driver channel.
 * Motor supply is separate from logic, with common GND. No motors from Pi GPIO.
 * USB Serial uses D0/D1: disconnect an HC-05 there. 115200 baud, 8N1.
 * Verify forward signs with wheels lifted; +PWM means logical forward.
 * No encoder feedback, physical stop input, odometry or obstacle sensing here.
 * M <uint32 sequence> <left PWM> <right PWM>*<4 HEX CRC>\n
 * A <same sequence> <applied left PWM> <right PWM> <flags>*<CRC>\n
 * CRC16-CCITT-FALSE over ASCII before '*': poly1021/initFFFF/no final XOR.
 * Flags:1=locked,2=watchdog event,4=bad-frame event (events clear after ACK).
 * Boot/watchdog/bad input lock drive until a valid ZERO command. Zero coasts.
 */
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PWM_LEFT=5, PWM_RIGHT=6, LEFT_1=7, LEFT_2=8, RIGHT_1=9, RIGHT_2=10;
const uint8_t STBY=4;
const int MOTOR_LEFT_SIGN=1, MOTOR_RIGHT_SIGN=1;
static_assert(MOTOR_LEFT_SIGN==1 || MOTOR_LEFT_SIGN==-1,"LEFT sign must be +/-1");
static_assert(MOTOR_RIGHT_SIGN==1 || MOTOR_RIGHT_SIGN==-1,"RIGHT sign must be +/-1");
const int PWM_LIMIT=140;
const unsigned long WATCHDOG_MS=350;
char input[96];
uint8_t used=0;
bool dropping=false, locked=true;
uint8_t events=0;
int actualLeft=0, actualRight=0;
unsigned long lastCommand=0;

uint16_t crc16(const char* data, size_t length) {
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<length;++i) {
    crc^=(uint16_t)(uint8_t)data[i]<<8;
    for(uint8_t b=0;b<8;++b) crc=(crc&0x8000) ? (crc<<1)^0x1021 : crc<<1;
  }
  return crc;
}
void side(uint8_t pwm,uint8_t in1,uint8_t in2,int value,int sign) {
  value*=sign;
  analogWrite(pwm,0); // Remove PWM before reversing direction.
  digitalWrite(in1,value>0 ? HIGH : LOW);
  digitalWrite(in2,value<0 ? HIGH : LOW);
  analogWrite(pwm,abs(value));
}
void motors(int left,int right) {
  actualLeft=left; actualRight=right;
  if(!left && !right) digitalWrite(STBY,LOW);
  side(PWM_LEFT,LEFT_1,LEFT_2,left,MOTOR_LEFT_SIGN);
  side(PWM_RIGHT,RIGHT_1,RIGHT_2,right,MOTOR_RIGHT_SIGN);
  if(left || right) digitalWrite(STBY,HIGH);
}
void stopAndLock(uint8_t event) { motors(0,0); locked=true; events|=event; }
void watchdog() {
  if(!locked && (unsigned long)(millis()-lastCommand)>=WATCHDOG_MS) stopAndLock(2);
}
bool parseSequence(const char* text,uint32_t& seq) {
  if(!*text) return false;
  seq=0;
  for(;*text;++text) {
    if(*text<'0' || *text>'9') return false;
    uint8_t d=*text-'0';
    if(seq>(0xFFFFFFFFUL-d)/10) return false;
    seq=seq*10+d;
  }
  return true;
}
bool parsePWM(const char* text,int& pwm) {
  char* end;
  long value=strtol(text,&end,10);
  if(end==text || *end || value < -PWM_LIMIT || value>PWM_LIMIT) return false;
  pwm=(int)value;
  return true;
}
bool checkCRC(char*& star) {
  star=strchr(input,'*');
  if(!star || strlen(star+1)!=4) return false;
  uint16_t supplied=0;
  for(uint8_t i=1;i<=4;++i) {
    char c=star[i];
    int value=c>='0' && c<='9' ? c-'0' :
              c>='A' && c<='F' ? c-'A'+10 : c>='a' && c<='f' ? c-'a'+10 : -1;
    if(value<0) return false;
    supplied=(supplied<<4)|value;
  }
  return supplied==crc16(input,star-input);
}
void handleLine() {
  char* star;
  if(!checkCRC(star)) { stopAndLock(4); return; }
  *star='\0';
  char* state;
  char* type=strtok_r(input," ",&state);
  char* sequence=strtok_r(NULL," ",&state);
  char* leftText=strtok_r(NULL," ",&state);
  char* rightText=strtok_r(NULL," ",&state);
  char* extra=strtok_r(NULL," ",&state);
  uint32_t seq; int left,right;
  if(!type || strcmp(type,"M") || !sequence || !leftText || !rightText || extra
      || !parseSequence(sequence,seq) || !parsePWM(leftText,left) || !parsePWM(rightText,right)) {
    stopAndLock(4); return;
  }
  watchdog();
  if(!left && !right) locked=false;
  motors(locked ? 0 : left,locked ? 0 : right);
  lastCommand=millis();
  uint8_t flags=(locked?1:0)|events;
  char body[48], reply[56];
  snprintf(body,sizeof(body),"A %lu %d %d %u",(unsigned long)seq,actualLeft,actualRight,(unsigned)flags);
  snprintf(reply,sizeof(reply),"%s*%04X\n",body,(unsigned)crc16(body,strlen(body)));
  Serial.write((const uint8_t*)reply,strlen(reply));
  events=0;
}
void setup() {
  pinMode(PWM_LEFT,OUTPUT); pinMode(PWM_RIGHT,OUTPUT);
  pinMode(LEFT_1,OUTPUT); pinMode(LEFT_2,OUTPUT);
  pinMode(RIGHT_1,OUTPUT); pinMode(RIGHT_2,OUTPUT); pinMode(STBY,OUTPUT);
  motors(0,0); lastCommand=millis(); Serial.begin(115200);
}
void loop() {
  watchdog();
  for(uint8_t n=0;n<64 && Serial.available();++n) {
    char c=Serial.read();
    if(c=='\n') {
      if(used && input[used-1]=='\r') --used; // Allow terminal CRLF, not embedded CR.
      if(!dropping && used) { input[used]='\0'; handleLine(); }
      used=0; dropping=false;
    } else if(!dropping) {
      if(c=='\0' || used>=sizeof(input)-1) { stopAndLock(4); dropping=true; }
      else input[used++]=c;
    }
    watchdog(); // Partial or excessive input never blocks the watchdog.
  }
}
