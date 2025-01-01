#ifndef LCD_H
#define LCD_H

#include<Arduino.h>
#include <LiquidCrystal_I2C.h>

extern bool testresult1Flag ;
extern bool testresult2Flag ;

extern bool passFlag1;
extern bool passFlag2;

void lcd_Init();
void lcd_date();
void lcd_time();
void lcd_flash(char* , char*, bool);
void lcd_blink(String , int, int);
void lcd_cprint(char* , int, int);
void lcd_iprint(int , int, int);
void lcd_c2print(char , int, int);
void lcd_clear();



#endif //LCD
