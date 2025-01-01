#include <LCD.h>
#include <RTClock.h>
#include <EEPROM.h>
// #include<imp_include.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool toggleflassFlag = false;
bool toggleblinkFlag = false;

bool testresult1Flag  = false;
bool testresult2Flag  = false;

bool passFlag1        = false;
bool passFlag2        = false;
uint8_t flassCounter = 0;

void lcd_Init()
{

    lcd.init();
    // Turn on the backlight
    lcd.backlight();
    lcd.clear();
    // Print text on the first line
    lcd.setCursor(0, 0); // Set cursor to column 0, line 0
    lcd.print("VoltWorks");
    // EEPROM.get(configAddress,configNum);
    // lcd.setCursor(0,1);
    // lcd.print(motorConfigs[configNum].config);
    delay(500);
}

void lcd_date()
{

    lcd.setCursor(0, 0);
    lcd.print(year);
    lcd.setCursor(4, 0);
    lcd.print(":");
    if (month < 10)
        lcd.print(0);
    lcd.print(month);
    lcd.print(":");
    if (day < 10)
        lcd.print(0);
    lcd.print(day);
    lcd.print("  ");
    // lcd.print(":");
}

// Prints time at
void lcd_time()
{

    lcd.setCursor(12, 0);
    if (hour < 10)
    {
        lcd.print(0);
    }
    lcd.print(hour);
    lcd.setCursor(14, 0);
    lcd.print(":");
    if (minute < 10)
    {
        lcd.print(0);
    }
    lcd.print(minute);
    lcd.setCursor(17, 0);
    lcd.print(":");
    if (second < 10)
    {
        lcd.print(0);
    }
    lcd.print(second);
}

// Flass between two messages
void lcd_flash(char *message, char *message2, bool faultFlag)
{
    if (flassCounter == 4)
    {
        flassCounter = 0;
        // Serial.println(message);
        toggleflassFlag = toggleflassFlag ^ true;
        if (toggleflassFlag)
        {  
            lcd_date();
            if(testresult1Flag)
            {
             lcd.setCursor(0,2);
             if(passFlag1) lcd.print("test1 passed");
             else lcd.print("test1 failed");
            }
            
            if(testresult2Flag)
            {
             lcd.setCursor(0,2);          
             if(passFlag2) lcd.print("test2 passed");
             else lcd.print("test2 failed");
            }
            if(faultFlag)
            {
              lcd.setCursor(0,2);
              lcd.print(message2);
            }
        }
        
        else
        {
            lcd.setCursor(0, 0);
            lcd.print(message);

            if(testresult2Flag | testresult1Flag | faultFlag)
            {
              lcd.setCursor(0,2);
              lcd.print("            ");
            }
        }
    }
    else
        flassCounter++;
}

void lcd_blink(String symbole, int row, int col)
{
    toggleblinkFlag = toggleblinkFlag ^ true;

    if (toggleblinkFlag)
    {
        lcd.setCursor(col, row);
        lcd.print(symbole);
    }
    else
    {
        lcd.setCursor(col, row);
        lcd.print("  ");
    }
}

void lcd_cprint(char *msg, int col, int row)
{
    lcd.setCursor(col, row);
    lcd.print(msg);
}

void lcd_c2print(char msg, int col, int row)
{
    lcd.setCursor(col, row);
    lcd.print(msg);
}

void lcd_iprint(int msg, int col, int row)
{
    lcd.setCursor(col, row);
    lcd.print(msg);
}

void lcd_clear()
{
    lcd.setCursor(0,2);
    lcd.print("            ");
}
