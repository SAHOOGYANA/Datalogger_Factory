#ifndef MYKEYPAD_H
#define MYKEYPAD_H




#include <Arduino.h>
#define scan_prescalar 50

// For Key board
// Colomn
#define C1 10
#define C2 9
#define C3 8
#define C4 7
// Row
#define R1 15
#define R2 14
#define R3 13
#define R4 12
#define R5 11


extern char key ;

void Keypad_Init();
void key_scan();

/*
class Keypad{
    public:
      Keypad(uint8_t rowPins[5], uint8_t colPins[4], const char Keys[5][4]);
      void Keypad_Init();
      void key_scan();


    private:
      uint8_t _rowPins[5];
      uint8_t _colPins[4];
      uint8_t _scan_counter;
      uint8_t _scan_prescalar;


};
*/

#endif // MYKEYPAD_H