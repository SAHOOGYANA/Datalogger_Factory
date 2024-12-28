#include "segdisplay.h"
LedControl lc(DATA_PIN, CLK_PIN, CS_PIN,2);

bool dp = false;

void segDisplayInit()
{
  for (int i = 0; i < 2; i++)
  {
    lc.shutdown(i, false); // Turn off power-saving mode
    lc.setIntensity(i, 2); // Set brightness (0-15)
    lc.clearDisplay(0);    // Clear the display
  }
}

void display(int current1, int voltage, int rpm, int current2)
{

  for (int i = 0; i < 4; i++)
  { 
    if(i == 2) dp = true;
    else dp = false;

    lc.setDigit(0, i, current1 % 10, dp);
    current1 = current1 / 10;
  }

  for (int i = 0; i < 4; i++)
  {
    if(i == 1) dp = true;
    else dp = false;

    lc.setDigit(0, i + 4, voltage % 10, dp);
    voltage = voltage / 10;
  }

  for (int i = 0; i < 4; i++)
  {
    lc.setDigit(1, i, rpm % 10, false);
    rpm = rpm / 10;
  }

  for (int i = 0; i < 4; i++)
  {
    if(i == 2) dp = true;
    else dp = false;

    lc.setDigit(1, i + 4, current2 % 10, dp);
    current2 = current2 / 10;
  }
}

void displayafterTest(char* sign1, char* sign2, char* sign3, char* sign4)
{

  for (int i = 0; i < 4; i++)
  {
    lc.setChar(0, i, sign1[i], false);
  }

  for (int i = 0; i < 4; i++)
  {
    lc.setChar(0, i + 4, sign2[i], false);
  }

  for (int i = 0; i < 4; i++)
  {
    lc.setChar(1, i, sign3[i], false);
  }

  for (int i = 0; i < 4; i++)
  {
    lc.setChar(1, i + 4, sign4[i], false);
  }

}