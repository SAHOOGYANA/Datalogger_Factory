#include <RTClock.h>

RTC_DS3231 rtc;
int numofmondayAddress = 0; // address in EEPROM for ndayAddres file name
int FileAddress        = 1; // address for file name
char _fileName[15];         // Buffer for file name
char fileName[15];          // Buffer for file name
const char *daysOfTheWeek[] = {"Sun", "Mon", "Tues", "Wed", "Thur", "Fri", "Sat"};

int year=0, month=0, day=0, hour=0, minute=0, second=0;

void RTC_Init()
{
    rtc.begin();
}

void RTC_ADJUST(long int DATE, long int TIME)
{
    long int receivedInteger = DATE;
    //String inputString = "";

    Serial.println("Adjust the date in the format->  YYYYMMDD ");
/*
    while (true)
    {
        if (Serial.available() > 0)
        {
            receivedInteger = Serial.parseInt(); // Read the integer from Serial input

            // If the input is valid (non-zero)
            if (receivedInteger != 0)
            {
                Serial.println(receivedInteger);
                break; // Exit the loop after receiving the integer
            }
        }
    }
    */

    int day   = receivedInteger % 100;  // Day
    receivedInteger = receivedInteger / 100;
    int month = receivedInteger % 100;  // Month
    receivedInteger = receivedInteger / 100;
    int year  = receivedInteger;        // Year

    Serial.println("Adjust the date in the format->  HHMMSS ");
/*
    while (true)
    {
        if (Serial.available() > 0)
        {
            receivedInteger = Serial.parseInt(); // Read the integer from Serial input

            // If the input is valid (non-zero)
            if (receivedInteger != 0)
            {
                Serial.println(receivedInteger);
                break; // Exit the loop after receiving the integer
            }
        }
    }
    */

    receivedInteger = TIME;
    Serial.println(receivedInteger);
    int second = receivedInteger % 100; // Second
    receivedInteger = receivedInteger / 100;
    int minute = receivedInteger % 100; // Minute
    receivedInteger = receivedInteger / 100;
    int hour   = receivedInteger % 100; // Hour
    receivedInteger = receivedInteger / 100;

    Serial.print(year);
    Serial.println(":");
    Serial.print(month);
    Serial.println(":");
    Serial.print(day);
    Serial.println(":");
    Serial.print(hour);
    Serial.println(":");
    Serial.print(minute);
    Serial.println(":");
    Serial.print(second);
    Serial.println(":");
    rtc.adjust(DateTime(year, month, day, hour, minute, second));
}
void RTC_GIVEDate()
{

    DateTime now = rtc.now();

    year = now.year();
    month=now.month();
    day = now.day();
    hour = now.hour();
    minute = now.minute();
    second = now.second();


    // Get the day of the week
    int dayOfWeek = now.dayOfTheWeek(); // Returns 0 = Sunday, 1 = Monday, etc.
    //Serial.print("Day of the Week: ");
    //Serial.println(daysOfTheWeek[dayOfWeek]);

}
void RTC_GIVETime()
{
    DateTime now = rtc.now();
    hour = now.hour();
    minute = now.minute();
    second = now.second();
}

void generateFileasDate()
{
    DateTime now = rtc.now();
    // Generate file name from date
    snprintf(fileName, sizeof(fileName), "%04d%02d%02d.TXT", now.year(), now.month(), now.day());

    Serial.print("Generated file name: ");
    Serial.println(fileName);
}

// get file name for storing everyday data
void setFilename()
{
    generateFileasDate();
    DateTime now = rtc.now();
    int dayOfWeek = now.dayOfTheWeek();
    
    if (dayOfWeek == 1)
    {
        EEPROM.get(FileAddress, _fileName);
        if (!(_fileName == fileName) || _fileName[0] == '\0')
        {
            snprintf(_fileName, sizeof(_fileName), fileName);
            EEPROM.put(FileAddress, _fileName);
        }
    }

    else
    {
        EEPROM.get(FileAddress, _fileName);
        Serial.println(_fileName);
        if (_fileName[3] != '4')
        {
            snprintf(_fileName, sizeof(_fileName), fileName);
            EEPROM.put(FileAddress, _fileName);
        }
    }
    
           //EEPROM.get(FileAddress, _fileName);

           /*
        if (!(_fileName == fileName) || _fileName[0] == '\0')
        {
            snprintf(_fileName, sizeof(_fileName), fileName);
            EEPROM.put(FileAddress, _fileName);
        }
        */

    Serial.print(_fileName);
}
