#include "ADC.h"
//#include <imp_include.h>

// My variables
int ADC_counter = 0;
uint8_t measuring_state = 0;



//ADC channels in use
uint8_t phasecurrentChannel   = 2;
uint8_t batterycurrentChannel = 0;
uint8_t batteryvoltageChannel = 6;
uint8_t motortempChannel      = 5;
uint8_t controllertempChannel = 3;
uint8_t dynotempChannel       = 7;


uint8_t low_byte;
uint8_t high_byte;



bool adcCompleteFlag = false;

long ADC_VALUE = 0;
float Phase_current = 0;
float Batt_current = 0;
float Batt_Voltage = 0;
int t_m = 0;
int t_c = 0;
int t_d = 0;
uint8_t channel = 0;
int _ADC_VALUE = 0;

float resistance;
int tempKelvin;

void ADC_setup()
{

  ADMUX = (1 << REFS0);                                // For reference set at Avcc
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // For prescalar 128 with clk of 16 MHz

  ADCSRA |= (1 << ADEN); // ADC enable
  ADCSRA |= (1 << ADIE); // ADC interupt set
}

void ADC_Scedule()
{

  if (ADC_counter == 100)
  {
    Phase_current = 0.436*(sqrt(ADC_VALUE / _I_P));
    channel = batterycurrentChannel;
    ADC_VALUE = 0;
  }
  if (ADC_counter == 151)
  {
    // Serial.println(ADMUX);
    Batt_current = (ADC_VALUE / _I_B)*0.436;
    channel = batteryvoltageChannel;
    ADC_VALUE = 0;
  }
  if (ADC_counter == 202)
  {
    //Serial.println(ADMUX);
    Batt_Voltage =  (ADC_VALUE/_V_B)/6.7;
    channel = motortempChannel;
    //Serial.println(ADC_VALUE);
    ADC_VALUE = 0;
  }
  if (ADC_counter == 223)
  {
    //Serial.println(ADMUX);
    resistance = ((1023.000*_Tc - ADC_VALUE ) / ADC_VALUE);
    tempKelvin = 3950 /(10.93 + log(resistance));
    t_m = tempKelvin - 273;
    if(t_m < 0) t_m = 0;
    channel = controllertempChannel;
    ADC_VALUE = 0;
  }
  if(ADC_counter == 244)
  {
    resistance = (1023.000*_Tc - ADC_VALUE ) / ADC_VALUE;
    tempKelvin = 3950 /(10.93 + log(resistance));
    t_c = tempKelvin- 273;
     if(t_c < 0) t_c = 0;
    channel = dynotempChannel;
    ADC_VALUE = 0;    
  }

  if (ADC_counter == 255)
  {
    resistance = (1023.000*_Tc - ADC_VALUE ) / ADC_VALUE;
    tempKelvin = 3950 /(10.93 + log(resistance));
    t_d = tempKelvin - 273;
     if(t_d < 0) t_d = 0;
    channel = phasecurrentChannel;
    ADC_VALUE = 0;
   
  }


if(ADC_counter < total_samples)
{
  ADC_readStart(channel);
}
}

// ADC for !_p, I_B, V_B, Tc, Tm
void ADC_readStart(uint8_t channel)
{ 

  ADMUX = (ADMUX & 0x40) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
}

uint16_t ADC_read()
{
  low_byte = ADCL;
  high_byte = ADCH;
  return (high_byte << 8) | low_byte;
}

void ADC_calculations()

{
    //

  if (ADC_counter < 100)
    ADC_VALUE = ADC_VALUE + pow((_ADC_VALUE - 509),2);
  else if ((ADC_counter < 151) & (ADC_counter > 100))
    ADC_VALUE = ADC_VALUE + (_ADC_VALUE - 509);
  else if ((ADC_counter < 202) & (ADC_counter > 151))
    ADC_VALUE = ADC_VALUE + _ADC_VALUE;
  else if ((ADC_counter < 223) & (ADC_counter > 202))
    ADC_VALUE = ADC_VALUE + _ADC_VALUE;
  else if ((ADC_counter < 244) & (ADC_counter > 223))
    ADC_VALUE = ADC_VALUE + _ADC_VALUE;
  else if ((ADC_counter < 255) & (ADC_counter > 244))
    ADC_VALUE = ADC_VALUE + _ADC_VALUE;

    //////
}


ISR(ADC_vect)
{
  _ADC_VALUE = ADC_read(); // read adcvalue
  ADC_calculations();
}
