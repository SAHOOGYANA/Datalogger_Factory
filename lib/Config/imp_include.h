#ifndef IMP_INCLUDE_H
#define IMP_INCLUDE_H


#include<Arduino.h>

uint8_t numofConfigs = 6;
uint8_t configNum    = 0;
int configAddress    = 14;

//Speed parameters and PID parameters
float sp_kp     = 0.08;
float sp_ki     = 0.1;
float sp_kd     = 0;
int pwm_max_out = 600;
int pwm_min_out = 160;


float currenttoSave;
int rpmtoSave;
int tempmotortoSave;

String stringData1;
String stringData2;
String stringTime;

int faultNum;
char faultCases[6][13] = {"LOW VOLTAGE ", "HIGHPCURRENT","HIGHBCURRENT", "HIGH MOTTEMP", "HIGH C TEMP ", "SPEEDNRISING"};
char taskState[6][10]= {"READY    ", "CONFIGURE", "FAULT    " , "TESTING  ", "TESTED   ", "STOPPED  "};

struct motorConfig{
  char config[9];
  int current1;
  int current2;
  int rpm;
  int voltage;
  int motorTemp1;
  int motorTemp2;
};

struct motorConfig motorConfigs[6]=
{
   {"config1", 60,40, 3000, 48,35,30},
   {"config2", 60,40, 2500, 48,35,30},
   {"config3", 60,40, 2100, 48,35,30},
   {"config4", 60,40, 1400, 48,35,30},
   {"config5", 60,40, 1000, 48,35,30},
   {"config6", 60,40, 800 , 48,35,30}
};



#endif