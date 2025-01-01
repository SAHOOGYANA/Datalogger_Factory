
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <segdisplay.h>
#include <Mykeypad.h>
#include <ADC.h>
#include <SDcard.h>
#include <RTClock.h>
#include <LCD.h>
#include <imp_include.h>
#include <EEPROM.h>
#include <PID.h>

// Defines for Samplings of channels

#define READY     0
#define CONFIGURE 1
#define FAULT     2
#define TESTING   3
#define TESTED    4
#define STOPPED   5


int refRPM ;
float refCurrent1 ;
float refCurrent2 ;
int refTemp1;
int refTemp2;
int load_rpm;
float refVoltage;

// RGB LED
#define RED   27
#define GREEN 29
#define BLUE  31

void retrieveConfig();

void ExternalInterupt_Init();                                // Interupt for RPM
void speedSet();

PID spdpid(sp_kp, sp_ki, sp_kd, pwm_max_out, pwm_min_out);   // PID for Speed

void RGBLed_Init();
void PushBtnIntrpt_Init();                                   // Interupt pins for push button
void Timer1_Init();                                          // Interupt for Timer
void Timer5_Init();                                          // For PWM

void RGB_set();
void RGB_flash(bool);

void TaskSheduler();
void fault_cheak();
void fault_clear_cheak();
void passfail_cheak();
void setup_includes();
void loop_includes();

int testingCounter1   = 0;
int testingCounter2   = 0;
int testingCounter3   = 0;
int testedCounter1    = 0;
int passfailCounter   = 0;

uint8_t lcdCounter1   = 0;

bool flashCounter = false;
bool rFlag = false;
bool gFlag = false;
bool bFlag = false;


// Button Flags
bool sttFLbtnFlag     = false;
bool sttNLbtnFlag     = false;
bool stpbtnFlag       = false;

// Task Flags
bool lcdWriteFlag     = false;
bool adcFlag          = false;
bool segDisplayFlag   = false;

bool updateStateFlag  = true;

// key type flags
bool commandFlag      = false;
bool charFlag         = false;
bool StringFlag       = false;
bool intFlag          = false;

// Stage flags;
bool configStage1Flag = false;
bool configStage2Flag = false;
bool configupgoFlag   = false;
bool configdowngoFlag = false;
bool configsetFlag    = false;

// Flags used during Testing
bool test1Flag        = false;
bool test2Flag        = false;
bool testedFlag       = false;
bool testingFlag      = false;
bool faultFlag        = false;

int task_state        = READY;
int prev_state        = READY;
int max_count         = 4 * total_samples;
int _counter          = 0;

uint8_t sec_counter = 0;

// For RPM
volatile uint32_t lastInterruptTime = 0;
int RPM               = 0;
int  rotation         = 0;
int rpmPwm            = 0;
uint8_t rpmCounter    = 0;
uint8_t rpmPrescalar  = 200;

int a = 0;
int ct = 0;
char _key = '\0';

void setup()
{

  Serial.begin(115200);
  setup_includes();
}

void loop()
{
  loop_includes();
}

// put function definitions here:
void GPIO_set()       // My pin Sets
{
  DDRA |= (1 << PA2); // D24 -> PA2 Using for pulse duaration measurement
}


void Timer1_Init()  // Timer1
{
  TCCR1A = 0x00;                                                   // Normal mode
  TCCR1B = (1 << WGM12) | (0 << CS10) | (1 << CS11) | (0 << CS12); //| (1 << CS10); // Prescaler = 8;
  OCR1A = 500 - 1;
  TCNT1 = 0;                                                       // Set Timer1 initial value
  // TIMSK1 = (1 << TOIE1);                                        // Enable Timer1 overflow
  TIMSK1 = (1 << OCIE1A);                                          // Enable Timer1 Compare A Match interrupt
}

ISR(TIMER1_COMPA_vect)
{

  speedSet() ;                                                    // Speed set

  if (ADC_counter == max_count)                                   // ADC counter update
  {
    ADC_counter = 0;
  }
  else ADC_counter++;


  if (ADC_counter == 0)                                          // adcFlag on for adc calculations
  {
    adcFlag = true;
    segDisplayFlag = false;
    lcdWriteFlag = false;
    updateStateFlag = false;
   
  }
  else if (ADC_counter == total_samples + 6)                    // displayFlag on for data displaying
  {
    adcFlag = false;
    segDisplayFlag = true;
    lcdWriteFlag = true; 
    updateStateFlag = true;
  }

  if ((updateStateFlag == true))
  {
   TaskSheduler();                                           // running task scheduler   
     if(ADC_counter == total_samples + 8)
     {
      updateStateFlag = false;
     }
  }

  if ((segDisplayFlag == true) & (ADC_counter == total_samples + 14))
  {
    if (sec_counter == 4)                                     // sec_counter for counting 1 second once
      sec_counter = 0;
    else
      sec_counter++;

    display( Batt_current*100, Batt_Voltage*10 , RPM, Phase_current*100);

    
  }

  if (adcFlag)                                              // if adcFlag is high
  {
    ADC_Scedule();
    key_scan();
  }
}

void Timer5_Init()
{
  // Set PL5 (D44) as output
  DDRL |= (1 << PL5);

  // Configure Timer5 in Fast PWM mode with ICR5 as TOP
  TCCR5A = (1 << COM5C1) | (1 << WGM51);                      // (1 << WGM50); // Clear OC5C on compare match
  TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS50);         // Prescaler = 8, Fast PWM

  // Set TOP value for 5kHz PWM
  ICR5 = 799;                                                 // TOP = 399 for 5kHz PWM frequency
  
  // Set initial duty cycle to 0%
  OCR5C = 0;                                                  // Duty cycle = (OCR5C / ICR5) * 100 = (200 / 399) * 100 = ~50%
}

// Push button interupt init
void PushBtnIntrpt_Init()
{
  DDRK &= ~((1 << PK2) | (1 << PK4) | (1 << PK6));            // Set as input
  PORTK |= (1 << PK2) | (1 << PK4) | (1 << PK6);              // Set as pullup
  PCICR |= (1 << PCIE2);                                      // Pinchange interupt pin define
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT20) | (1 << PCINT22); // pinchange interupt
                                                              // GIFR |= (1 << PCIF2);
}

ISR(PCINT2_vect)
{
  if (!(PINK & (1 << PK2)))                                   // Yellow button is pressed
  {
    //Serial.print("pressed");
    if ((task_state == TESTED)||(task_state == READY))
    {
      sttNLbtnFlag = true;
      test1Flag = false;
      test2Flag = true;
    }
  }
  else if (!(PINK & (1 << PK4)))                              // Green button is pressed
  {
    if ((task_state == READY)||((task_state == TESTED)))
    {
      sttFLbtnFlag = true;
      test1Flag = true;
      test2Flag = false;      
    }
  }
  else if (!(PINK & (1 << PK6)))                              // Black button is pressed
  {
    if (task_state == TESTING)
    {
      //stpbtnFlag = true;
      task_state = STOPPED;
      test1Flag = false;
      test2Flag = false;
      testingCounter1 = 0;
      testingCounter2 = 0;
      testresult1Flag = false;
      testresult2Flag = false;
    }
  }
}


void RGBLed_Init()
{
  DDRA |= (1 << PA4)|(1 << PA6);
  DDRC |= (1 << PC7);

  PORTA &= ~(1 << PA4);
  PORTA &= ~(1 << PA6);
  PORTC &= ~(1 << PC7);
}

void RGB_set()
{
  if((task_state == READY) || (task_state == CONFIGURE)) // BLUE
  {
    rFlag = false;
    gFlag = false;
    bFlag = true ;
  }
  else if((task_state == FAULT) || (task_state == STOPPED))
  {
    rFlag = true;
    gFlag = false;
    bFlag = false;
  } 
  else if((task_state == TESTED))
  {
    rFlag = false;
    gFlag = true;
    bFlag = false;
  } 
  else if((task_state == TESTING) && (test1Flag == true))
  {
    rFlag = false;
    gFlag = true;
    bFlag = false;
  } 

  else if((task_state == TESTING) && (test2Flag == true))
  {
    rFlag = false;
    gFlag = true;
    bFlag = true;
  }


 
  
}

void RGB_flash(bool flash)
{
// if flash is true then led flash
if(flash)
{
  flashCounter = flashCounter^true ;
  if(flashCounter)
  {
   if(rFlag) PORTA |= (1 << PA4); else PORTA &= ~(1 << PA4);
   if(gFlag) PORTA |= (1 << PA6); else PORTA &= ~(1 << PA6);
   if(bFlag) PORTC |= (1 << PC7); else PORTC &= ~(1 << PA7);
  }
  else 
  {
    PORTA &= ~(1 << PA4);
    PORTA &= ~(1 << PA6);
    PORTC &= ~(1 << PC7);
  }
  
}
else 
{
   if(rFlag) PORTA |= (1 << PA4); else PORTA &= ~(1 << PA4);
   if(gFlag) PORTA |= (1 << PA6); else PORTA &= ~(1 << PA6);
   if(bFlag) PORTC |= (1 << PC7); else PORTC &= ~(1 << PA7);
}

}


void retrieveConfig()              // Configuration retrieve
{
  EEPROM.get(configAddress, configNum);
}

void TaskSheduler()
{
  switch (task_state)
  {
  case READY:
    // if F2 is pressed then -> CONFIGURE
    // if green buttn is pressed Testing start with load
    // if yellow buttn is pressed Testing start with no load
    if(ADC_counter == total_samples + 8)
    {
    RGB_flash(false);
    if (key == 'C')
    {
      configStage1Flag = true;
      _key = key;                     // _key as key to use in loop for display
      key = '\0';                     // reseting key to overcome repeated false key
      task_state = CONFIGURE;         // enter to configure state
    }

    if ( sttFLbtnFlag )  // any one of the green or yellow pressed
    {
      spdpid.reset();
      task_state = TESTING;
      sttNLbtnFlag = false;
      sttFLbtnFlag = false;
      test1Flag = true;
    }

    if ( sttNLbtnFlag )  // any one of the green or yellow pressed
    {
      spdpid.reset();
      task_state = TESTING;
      sttNLbtnFlag = false;
      sttFLbtnFlag = false;
      test2Flag = true;
    }   
    }
    break;

  case FAULT:
  if(ADC_counter == total_samples + 6)
  {
     RGB_flash(true);
     OCR5C = 0;
     if(key == 'O')
     {
      task_state = READY;
      faultFlag = false;
      key = '\0';
     }
  }
  if(ADC_counter == total_samples + 7)
  {
    fault_clear_cheak();
  }
    break;  

  case CONFIGURE:
    // if F1 pressed -> NORMAL
    // can set current speed date and time
    // Serial.print(key);
   if(ADC_counter == total_samples + 6)
   {
    RGB_flash(true);

    if (key == 'O')         // Escape key
    {
      _key = key;
      key = '\0';
      task_state = READY;
    }

    if (key == 'U')         // Up key
    {
      _key = key;
      key = '\0';
      configupgoFlag = true;
    }

    if (key == 'D')         // Down key
    {
      _key = key;
      key = '\0';
      configdowngoFlag = true;
    }

    if (key == 'E')         // Enter key
    {
      _key = key;
      key = '\0';
      configsetFlag = true;
    }
   }
    break;

  case TESTING:
    // if Btn3 pressed or time limit excced -> TESTED
    // data showing on 7 seg display
    if(ADC_counter == total_samples + 6)
    {

    testingCounter2++;                                //Used for fault detection
    fault_cheak();

    }
    
    if(ADC_counter == total_samples + 7)
    {
    if(key =='O') key = '\0';                         // This is to avoid "escape" latch
  
    if (test1Flag == true & test2Flag == false)       //Test1 tasks
    {
      if (((RPM - refRPM) < 20) & (RPM - refRPM) > -20)
      { 
        passfail_cheak();
        if (testingCounter1 == 27)
        {
          if(passFlag1)
          {
          stringData1 = String(currenttoSave) + ", " + String(rpmtoSave) + ", " + String(tempmotortoSave) +"- PASSED" ;
          }
          else
          {
           stringData1 = String(currenttoSave) + ", " + String(rpmtoSave) + ", " + String(tempmotortoSave) +"- FAILED" ;
          }
          testingCounter1 = 0;
          task_state = TESTED;
          test1Flag = false;
        }

        else if(testingCounter1 == 26)
        {
          // take 5 data and average them then store in sd card
          currenttoSave = currenttoSave/5;
          rpmtoSave = rpmtoSave/5;
          tempmotortoSave = tempmotortoSave/5;

          currenttoSave = 0;
          rpmtoSave = 0;
          tempmotortoSave = 0;

          testingCounter1++;
        }

        else if(testingCounter1 < 26 & testingCounter1 > 20)
        {

          currenttoSave +=  Batt_current;
          rpmtoSave += RPM ;
          tempmotortoSave += t_m ;

          testingCounter1++;

        }

        else if(testingCounter1 == 20)
        {
          stringTime = String(hour) + ":"+ String(minute) + ":" + String(second);
          currenttoSave = 0;
          rpmtoSave = 0;
          tempmotortoSave = 0;
          testresult1Flag = true;
          testingCounter1++;
        }
        else
          testingCounter1++;
      }

      if((testingCounter2 == 200 ) & (testingCounter1 < 20))
      {
        task_state = FAULT;
        faultNum = 5;
        faultFlag = true;
        testingCounter1 = 0;
        testingCounter2 = 0;
        test1Flag = false;
        test2Flag = false;
      }
    }

    else if (test1Flag == false & test2Flag == false) //Test pause
    {
      RGB_flash(true);
       if(key == 'O')
       {
        key = '\0';
        task_state = READY;
        testresult1Flag = false;
       }
      //   if(sttNLbtnFlag)
      //  {
      //   sttFLbtnFlag = false;
      //   sttNLbtnFlag = false;
      //   testresult1Flag = false;
      //   testresult2Flag = false;        
      //   test2Flag = true;
      //   RGB_set();
      //   RGB_flash(false);
      //  }
      //   if(sttFLbtnFlag)
      //  {
      //   sttFLbtnFlag = false;
      //   sttNLbtnFlag = false;
      //   testresult1Flag = false;        
      //   testresult2Flag = false;
      //   test1Flag = true;
      //   RGB_set();
      //   RGB_flash(false);
      //  }       
    }

    else if (test1Flag == false & test2Flag == true)  //Test2 tasks
    {
      if (rpmPwm == pwm_max_out)
      {
       passfail_cheak();

       if (testingCounter1 == 40)
        {
          testingCounter1 = 0;
          task_state = TESTED;
          stpbtnFlag = false;
          test2Flag = false;
        }

        else if(testingCounter1 == 16)
        {
          currenttoSave = currenttoSave/5;
          rpmtoSave = rpmtoSave/5;
          tempmotortoSave = tempmotortoSave/5;
          currenttoSave = 0;
          rpmtoSave = 0;
          tempmotortoSave = 0;

          testingCounter1++;
        }

        else if(testingCounter1 == 17)
        {
          if(passFlag2)
          {
          stringData1 = String(currenttoSave) + ", " + String(rpmtoSave) + ", " + String(tempmotortoSave) +"- PASSED" ;
          }
          else
          {
           stringData1 = String(currenttoSave) + ", " + String(rpmtoSave) + ", " + String(tempmotortoSave) +"- FAILED" ;
          }
          testresult2Flag  = true;
          testingCounter1++;
        }
        else if(testingCounter1 > 10 & testingCounter1 < 16)
        {
          
          currenttoSave +=  Batt_current;
          rpmtoSave += RPM ;
          tempmotortoSave += t_m ;

          testingCounter1++;
        }

       else if(testingCounter1 == 10) 
       {
          currenttoSave = 0;
          rpmtoSave = 0;
          tempmotortoSave = 0;
         testingCounter1++;
       }
        else
          testingCounter1++;
      }
    }
    }

    break;

  case TESTED:
    // Stop data showing on 7 seg display
    // data write to sd card
    // data upload
    //  state -> READY
    if(ADC_counter == total_samples + 6)
    {
    spdpid.reset();
    RGB_flash(true);

    if (key == 'O')
    {
      task_state = READY;
      testresult1Flag = false;
      testresult2Flag  = false;
      key = '\0';

    }

    if(key == 'E')
    {
      sdcard_write(_fileName, stringTime);
      sdcard_write(_fileName, stringData1);  // Writing in the sd card
      sdcard_write(_fileName, stringData2);  // Writing in the sd card
      task_state = READY;
      testresult1Flag = false;
      testresult2Flag  = false;
    }

    if(sttFLbtnFlag || sttNLbtnFlag)
    {
      RGB_set();
      RGB_flash(false);
      testingCounter1 = 0;
      testingCounter2 = 0;
      task_state = TESTING;
      testresult1Flag = false;
      testresult2Flag  = false;
      sttNLbtnFlag = false;
      sttFLbtnFlag = false;
    }

    }
    break;

  case STOPPED:
  if(ADC_counter == total_samples + 6)
  {
     RGB_flash(false);
     OCR5C =0;
      if(key == 'O')
     {
      task_state = READY;
      key = '\0';
      testingCounter1=0;
     }

     if(testingCounter1 == 10)
     {
      testingCounter1 = 0;
      task_state = READY;
     }
     else testingCounter1++;
  }
    break;  

  default:
    // State -> READY

    break;
  }
}

// Rpm

void interruptHandler()
{
  //uint32_t currentTime = millis(); // Get current time (requires a timer for millis())
 // if (currentTime - lastInterruptTime > 2)
 // { // 50ms debounce interval
    rotation++;
 //   lastInterruptTime = currentTime;
 //}
 
}

void ExternalInterupt_Init()       // External interupt init for rotation count
{                           /*
                              DDRD &= ~(1 << PD3);  // PD3 as Input
                              PORTD |= (1 << PD3);  // PullUp  pin PD3
                          
                              // For Falling edge trigger set
                              EICRA |= (1 << ISC01); // ISC01 = 1
                              EICRA &= ~(1 << ISC00); // ISC00 = 0
                          
                              EIMSK |= (1 << INT1); // Enable INT0 interupt
                          
                              sei(); // Enable global interupt
                              */
  pinMode(3, INPUT_PULLUP); // Set D3 as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(3), interruptHandler, FALLING);
}

void speedSet()                    //PWM set for speed control
{
 // Serial.println(rpmPwm);

  if(rpmCounter == rpmPrescalar) 
  {
  RPM = rotation * 6;
  rotation =0;
  //Serial.println(RPM);
  if(task_state == TESTING)
  {
  if (test1Flag)
  {
    OCR5C = spdpid.compute(RPM, refRPM);
    // Serial.print(RPM);
    // Serial.print(",");
    // Serial.print(motorConfigs[configNum].rpm);
    // Serial.print(",");
    // Serial.println(OCR5C);
  }

  if((test1Flag == false) & (test2Flag == false))
  {
    OCR5C = 0;
  }

  if (test2Flag)
  {
    if (rpmPwm < pwm_max_out)
      rpmPwm += 10;
    else
      rpmPwm = pwm_max_out;

    OCR5C = rpmPwm;
  }
    rpmCounter = 0;                  // rpm_counter reset
  }
  else
  {
  if((test1Flag == false) & (test2Flag == false))
  {
    OCR5C = 0;
  }
  }
  }
  else rpmCounter++;
}

void fault_cheak()
{
     if(t_m > 100)
   {
    task_state = FAULT;
    faultFlag = true;
    faultNum = 3;
   }

    if(t_c > 100)
   {
    task_state = FAULT;
    faultFlag = true;
    faultNum = 4;
   }

   if(Batt_current > 50)
   {
    task_state = FAULT;
    faultFlag = true;
    faultNum = 2;
   }
    if(Phase_current > 50)
   {
    task_state = FAULT;
    faultFlag = true;
    faultNum = 1;
   }

   if(Batt_Voltage < refVoltage - 2)
   {
    task_state = FAULT;
    faultFlag = true;
    faultNum = 0;
   }

   if((testingCounter2 == 200 ) & (testingCounter1 < 20))
   {
    task_state = FAULT;
    faultNum = 5;
    faultFlag = true;

    test1Flag = false;
    test2Flag = false;
   }
}

void fault_clear_cheak()
{
  if(faultNum == 0)
  {
    if(Batt_Voltage > refVoltage - 2)
   {
    task_state = READY;
    faultFlag = false;
   }
  }
  // else if(faultNum == 1)
  // {

  // }
  // else if(faultNum == 2)
  // {

  // }
  else if(faultNum == 3)
  {
    if(t_m < 90)
   {
    task_state = READY;
    faultFlag = false;
   }
  }
  else if(faultNum == 4)
  {
    if(t_c < 90)
   {
    task_state = READY;
    faultFlag = false;
   }
  }

}

void passfail_cheak()
{
  if(test1Flag)
  {
  if(Batt_current < refCurrent1)
  {
      passfailCounter++;
  }

  if(testingCounter1 == 20)
  {
   if(passfailCounter > 15)
   {
    //Serial.println(passfailCounter);
    passFlag1 = true;
   }
   else
   {
    passFlag1 = false;
   }
    passfailCounter = 0;
  }
  }

  if(test2Flag)
  {
  if(Batt_current < refCurrent2)
  {
      passfailCounter++;
  }
  if(testingCounter1 == 14)
  {
   if(passfailCounter > 10)
   {
    passFlag2 = true;
   }
   else
   {
   passFlag2 = false;
   }
   passfailCounter = 0;
  }

  }

}

void setup_includes()             //All setup initialisation functions
{

  sdcard_Init();
  ADC_setup();
  Timer5_Init();
  Keypad_Init();
  // EEPROM.put(configAddress,2);
  RTC_Init();
  //RTC_ADJUST(20241224, 162530);
  RTC_GIVEDate();
  lcd_Init();
  EEPROM.get(configAddress, configNum);
  setFilename();
  Serial.print(_fileName);
  Timer1_Init();
  lcd_cprint(motorConfigs[configNum].config, 0, 1);
  refRPM = motorConfigs[configNum].rpm;
  refCurrent1 = motorConfigs[configNum].current1;
  refCurrent2 = motorConfigs[configNum].current2;
  refTemp1 = motorConfigs[configNum].motorTemp1;
  refTemp2 = motorConfigs[configNum].motorTemp2;
  refVoltage = motorConfigs[configNum].voltage;
  lcd_cprint(taskState[task_state], 0, 3);
  lcd_cprint("Tm:", 14, 2);
  lcd_cprint("Tc:", 14, 3);
  segDisplayInit();
  task_state = READY;
  RGBLed_Init();
  RGB_set();
  PushBtnIntrpt_Init();
  ExternalInterupt_Init();

}


void loop_includes()             //All the works to be done in void loop
{
  if ((lcdWriteFlag == true))
  {
    if(lcdCounter1 == 4)
    {
      lcdCounter1 = 0;
    if (t_m < 10)
    {
      lcd_cprint("  ", 17, 2);
      lcd_iprint(t_m, 19, 2);
    }
    else if (t_m >= 10 & t_m < 100)
    {
      lcd_cprint(" ", 17, 2);
      lcd_iprint(t_m, 18, 2);
    }
    else
    {
      lcd_iprint(t_m, 17, 3);
    }

    if (t_c < 10)
    {
      lcd_cprint("  ", 17, 3);
      lcd_iprint(t_c, 19, 3);
    }
    else if (t_c >= 10 & t_c < 100)
    {
      lcd_cprint(" ", 17, 3);
      lcd_iprint(t_c, 18, 3);
    }

    else
    {
      lcd_iprint(t_c, 17, 3);
    }
    }
    else lcdCounter1++;
    lcd_flash(_fileName, faultCases[faultNum], faultFlag);
    RTC_GIVETime();
    lcd_time();
    if (_key)
    {
      lcd_c2print(_key, 18, 1);
      _key = '\0';
    }

    if (task_state == CONFIGURE)
    {
      lcd_blink("<<", 1, 10);

      if (configupgoFlag == true)                          // upper Configuration mode 
      {
        configupgoFlag = false;
        if (configNum > 0)
          configNum--;

        lcd_cprint(motorConfigs[configNum].config, 0, 1);
      }

      if (configdowngoFlag == true)                        // lower configuration mode 
      {
        configdowngoFlag = false;
        if (configNum < numofConfigs-1)
          configNum++;

        lcd_cprint(motorConfigs[configNum].config, 0, 1);
      }

      if (configsetFlag == true)                           // configuration mode set
      {
        configsetFlag = false;
        EEPROM.put(configAddress, configNum);
        refRPM = motorConfigs[configNum].rpm;
        refCurrent1 = motorConfigs[configNum].current1;
        refCurrent2 = motorConfigs[configNum].current2;
        refTemp1 = motorConfigs[configNum].motorTemp1;
        refTemp2 = motorConfigs[configNum].motorTemp2;
        refVoltage = motorConfigs[configNum].voltage;
      }
    }

    if (task_state != prev_state)
    {
      RGB_set();
      lcd_clear();
      prev_state = task_state;
      lcd_cprint(taskState[task_state], 0, 3);
      EEPROM.get(configAddress, configNum);
      lcd_cprint(motorConfigs[configNum].config, 0, 1);
     
      if (testedFlag == true)
      {
        testedFlag = false;
      }
    }

    delay(250);
    lcd_c2print(' ', 18, 1);
  }
}