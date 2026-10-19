#pragma once
#include <stdint.h>
#include <algorithm>
struct EEPROMMock {
  uint8_t data[1024];
  EEPROMMock(){std::fill(data,data+1024,0xff);}
  uint8_t read(int at){return data[at];}
  void update(int at,uint8_t value){data[at]=value;}
};
extern EEPROMMock EEPROM;
