#include <Arduino.h>
#include "FS.h"
#include <flash.h>
#include <queue>

/* You only need to format LittleFS the first time you run a
   test or else use the LITTLEFS plugin to create a partition
   https://github.com/lorol/arduino-esp32littlefs-plugin

   If you test two partitions, you need to use a custom
   partition.csv file, see in the sketch folder */

// #define TWOPART

#define FORMAT_LITTLEFS_IF_FAILED true

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("- failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.path(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char *path)
{
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path))
    {
        Serial.println("Dir removed");
    }
    else
    {
        Serial.println("rmdir failed");
    }
}

std::queue<String> readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("- failed to open file for reading");
        return std::queue<String>();
    }

    Serial.println("- read from file:");
    // Serial.println(file.read());
    std::queue<String> FileQueue;
    String line;
    while (file.available())
    {
        char data = file.read();
        if (data == '\n')
        {
            Serial.println(line);
            FileQueue.push(line);
            line = "";
        }
        else
        {
            line += data;
        }
    }
    if (!line.isEmpty())
    {
        Serial.println(line);
        FileQueue.push(line);
    }
    file.close();
    file = fs.open(path, FILE_WRITE);
    file.close();
    return FileQueue;
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("- file written");
        Serial.println(message);
    }
    else
    {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("- failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("- message appended");
        Serial.println(message);
    }
    else
    {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2))
    {
        Serial.println("- file renamed");
    }
    else
    {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\r\n", path);
    if (fs.remove(path))
    {
        Serial.println("- file deleted");
    }
    else
    {
        Serial.println("- delete failed");
    }
}
char read(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\r\n", path);
    char temps;
    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("− failed to open file for reading");
        return temps;
    }

    Serial.println("− read from file:");
    while (file.available())
    {
        temps = file.read();
        Serial.write(temps);
    }
    return temps;
}
