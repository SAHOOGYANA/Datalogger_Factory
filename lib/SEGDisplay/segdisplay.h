#ifndef SEGDISPLAY_H
#define SEGDISPLAY_H

// Define pins for custom SPI
#define DATA_PIN 25 // DIN pin of MAX7219
#define CLK_PIN 23  // CLK pin of MAX7219
#define CS_PIN 27   // CS pin of MAX7219

#include <Arduino.h>
#include <LedControl.h>
#include <SPI.h>



void segDisplayInit();
void display(int, int, int, int );
void displayafterTest(char*, char*, char*, char*);
/*
class SegDisplay{
    public:
      SegDisplay(int dataPin, int clkPin, int csPin, int numDevices);
      void segDisplayInit();
      void display(int current, int voltage, int rpm, int temperature);

    private:
      int _dataPin, _clkPin, _csPin, _numDevices;
      LedControl _lc ;
};
*/

#endif //SEGDISPLAY_H