//
// Created by robos on 03/05/2022.
//

#ifndef SDCARDDATALOG_H
#define SDCARDDATALOG_H

#include <Arduino.h>
#include "SdFat.h"
#include "RingBuf.h"

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
//#define LOG_INTERVAL_USEC 40

#define LOG_FILE_SIZE 10*25000*600  // 150,000,000 bytes. // TODO: determine the minumum file size for our rocket

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512

#define LOG_FILENAME "SdioLogger10mychanges.csv" // csv is far easier than other formats.

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

struct dataStruct;

#endif //SDCARDDATALOG_H
