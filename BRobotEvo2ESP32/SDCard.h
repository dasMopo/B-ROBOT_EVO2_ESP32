/*
 * SDCard.h
 *
 *  Created on: 30.06.2018
 *      Author: dasMopo
 */

#ifndef SDCARD_H_
#define SDCARD_H_

// SPI speed for SD card
// #define SDSPEED 1000000

// #include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"


void appendFile(fs::FS &fs, const char * path, const char * message);
void createDir(fs::FS &fs, const char * path);
void deleteFile(fs::FS &fs, const char * path);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void readFile(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void testFileIO(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);

bool sdcardTest(uint8_t ssPin);

#endif /* SDCARD_H_ */


