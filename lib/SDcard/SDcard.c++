#include "SDcard.h"


String fileList[MAX_FILES]; // Array to store filenames
int fileCount = 0;          // Counter for the number of files
File _dir;

// SD card initialisation
void sdcard_Init()
{
  Serial.println("Initializing SD card...");
  SPI.begin();

  if (!SD.begin(_cs_pin))
  {
    Serial.println("SD card initialization failed.");
    return;
  }
  Serial.println("SD card initialized successfully.");
}

// SD card write
void sdcard_write(const char *File_name, String data)
{

  File dataFile = SD.open(File_name, FILE_WRITE);

  if (dataFile)
  {
    dataFile.println(data);
    dataFile.close();
  }

  else
    Serial.println("File is unable to write");
}

// SD card read a file
void sdcard_read(const char *File_name)
{
  Serial.print("Reading from file: ");
  Serial.println(File_name);
  File dataFile = SD.open(File_name);

  if (dataFile)
  {
    while (dataFile.available())
    {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }

  else
    Serial.println("File is unable to be read");
}

// SD card list all files
void list_files(File dir)
{
  while (true)
  {
    File entry = dir.openNextFile();
    if (!entry)
    {
      // No more files
      break;
    }

    // Only add files (not directories)
    if (!entry.isDirectory() && fileCount < MAX_FILES)
    {
      fileList[fileCount++] = entry.name(); // Store the filename
    }

    entry.close();
  }
}

// Show all file names
void show_files()
{
  Serial.println("\nStored filenames:");
  for (int i = 0; i < fileCount; i++)
  {
    Serial.println(fileList[i]);
  }
}
