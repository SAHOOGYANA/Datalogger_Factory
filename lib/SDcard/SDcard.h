#ifndef SDCARD_H
#define SDCARD_H

#define _cs_pin 53
#define MAX_FILES 100 // Maximum number of files to list

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>


void sdcard_Init();
void sdcard_write(const char *File_name, String data);
void sdcard_read(const char *File_name);
void list_files(File dir);
void show_files();
/*
class SDCARD{
    public:
       SDCARD(const int cs_pin);
       void sdcard_Init();
       void sdcard_write(const char *File_name, String data);
       void sdcard_read(const char *File_name);
       void list_files(File dir);


    private:
       const int _cs_pin;



};
*/

#endif // SDCARD_H
