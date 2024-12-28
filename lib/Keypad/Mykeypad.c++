#include "Mykeypad.h"

const byte ROWS = 5;
const byte COLS = 4;

// Define the symbols on the buttons of the keypad
char _Keys[ROWS][COLS] = {
    {'N', 'C', '#', '*'},
    {'1', '2', '3', 'U'},
    {'4', '5', '6', 'D'},
    {'7', '8', '9', 'O'},
    {'<', '0', '>', 'E'}};

uint8_t rowPins[ROWS] = {9, 8, 7, 6, 5};  // Rows
uint8_t colPins[COLS] = {14, 12, 11, 10}; // Columns

char key = '\0';
int scan_counter = 0;
uint8_t _count = 0;


void Keypad_Init()
{
    // Row pins initialisation
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);

    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);

    // Col pins initialisation

    // pin direction
    DDRB &= ~(1 << PB4);
    DDRB &= ~(1 << PB5);
    DDRB &= ~(1 << PB6);
    DDRJ &= ~(1 << PJ1);

    // Pin pullup
    PORTB |= (1 << PB4);
    PORTB |= (1 << PB5);
    PORTB |= (1 << PB6);
    PORTJ |= (1 << PJ1);

    // pinchange interupt port define
    PCICR |= (1 << PCIE0);
    PCICR |= (1 << PCIE1);

    // Pinchange interupt masking
    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK1 |= (1 << PCINT10);
}

void key_scan()
{

    // Increment the row count cyclically
    if (scan_counter < scan_prescalar)
    {
        scan_counter++;
    }

    else
    {
        scan_counter = 0;
        if (_count < 4)
            _count++;
        else
            _count = 0;

        // Serial.println(count);

        for (int i = 0; i < 5; i++)
        {
            if (i == _count)
                digitalWrite(rowPins[i], LOW);
            else
                digitalWrite(rowPins[i], HIGH);
        }
    }
}


ISR(PCINT0_vect)
{
  if (!(PINB & (1 << PB4)))
  {
    key = _Keys[4-_count][3];
  } // Serial.print(key);  }
  else if (!(PINB & (1 << PB5)))
  {
    key = _Keys[4-_count][2];
  } // Serial.print(key); }
  else if (!(PINB & (1 << PB6)))
  {
    key = _Keys[4-_count][1];
  } // Serial.print(key); }
}

ISR(PCINT1_vect)
{
  if(!(PINJ & (1 << PJ1))) 
  {
  key = _Keys[4-_count][0]; //
  //Serial.print("4");
  }
}
 //

