
#ifndef ADC_H
#define ADC_H

#include<Arduino.h>

#define _I_P 100
#define _I_B 50
#define _V_B 50
#define _Tm  20
#define _Tc  20
#define _Td  10


#define SERIES_RESISTOR     10000  // 10kΩ pull-down resistor
#define NOMINAL_RESISTANCE  10000  // 10kΩ at 25°C
#define NOMINAL_TEMPERATURE 25     // 25°C
#define B_COEFFICIENT       3950   // Thermistor Beta Coefficient
#define VREF                5.0    // Reference voltage (in volts)

#define ToC 2

const int total_samples = _I_P + _I_B + _V_B + _Tm + _Tc + _Td;

extern int ADC_counter;

extern long ADC_VALUE ;
extern float Phase_current;
extern float Batt_current;
extern float Batt_Voltage;
extern int t_m ;
extern int t_c ;
extern int t_d ;
extern int RPM ;


uint16_t ADC_read();
void ADC_setup();
void ADC_Scedule();
void ADC_readStart(uint8_t channel);
void ADC_calculations();


/*
class ADCs{
   public:
     ADCs();
     
     void ADC_setup(uint8_t I_P, uint8_t I_B, uint8_t V_B, uint8_t Tm, uint8_t Tc);
     void ADC_Scedule();
     void ADC_readStart(uint8_t channel);

   private:
    uint8_t _I_P;
    uint8_t _I_B;
    uint8_t _V_B;
    uint8_t _Tm;
    uint8_t _Tc;
    long ADC_VALUE = 0;


};
*/

#endif //ADC_H