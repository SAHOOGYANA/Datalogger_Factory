#ifndef RTCLOCK_H
#define RTCLOCK_H

//#define SDA 20
//#define SCL 21

#include<Arduino.h>
#include<RTClib.h>
#include<EEPROM.h>
#include <LiquidCrystal_I2C.h>

extern int year, month, day, hour, minute, second;
extern char _fileName[15];         // Buffer for file name
extern char fileName[15]; 

void RTC_Init();
void RTC_ADJUST(long int , long int);
void RTC_GIVEDate();
void RTC_GIVETime();
void generateFileasDate();
void setFilename();




#endif //RTCLOCK