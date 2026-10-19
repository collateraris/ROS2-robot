#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define A1 15
#define A2 16
extern uint32_t fakeNow;
extern int pinValues[32],pwmValues[32];
inline uint32_t millis(){return fakeNow;}
inline void pinMode(uint8_t,int){}
inline void digitalWrite(uint8_t pin,int value){pinValues[pin]=value;}
inline int digitalRead(uint8_t pin){return pinValues[pin];}
inline void analogWrite(uint8_t pin,int value){pwmValues[pin]=value;}
inline int digitalPinToInterrupt(int pin){return pin;}
inline void attachInterrupt(int,void(*)(),int){}
struct SerialMock {
  std::string input,output;
  void begin(int){}
  int available(){return (int)input.size();}
  int read(){char c=input.front();input.erase(0,1);return c;}
  int availableForWrite(){return 64;}
  size_t write(const uint8_t* p,size_t n){output.append((const char*)p,n);return n;}
};
extern SerialMock Serial;
