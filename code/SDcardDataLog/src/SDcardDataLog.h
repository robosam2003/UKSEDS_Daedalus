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

#define LOG_FILE_SIZE 256*100*60*40  // 61,440,000 bytes - 61MB - This gives us over 40 minutes of data logging

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512

// We are now generating a unique filename for each log.
//#define LOG_FILENAME "daedalusDataLog2.csv" // csv is far easier than other formats.

extern SdFs sd;
extern FsFile file;

// RingBuf for File type FsFile.
extern RingBuf<FsFile, RING_BUF_CAPACITY> rb;



struct SDDataLogStruct { // This is the structure that will be written to each line of the file.
    byte logCode;
    u_int64_t timeStamp;
    /** BNO055 **/
    // Raw, (NOT bias corrected) unfiltered sensor data especially for Tom.
    double BNO055_acc_x;
    double BNO055_acc_y;
    double BNO055_acc_z;
    double BNO055_gyr_x;
    double BNO055_gyr_y;
    double BNO055_gyr_z;

    // Filtered, bias corrected sensor data that is used in dead reckoning
    double BNO055_acc_x_filt;
    double BNO055_acc_y_filt;
    double BNO055_acc_z_filt;
    double BNO055_gyr_x_filt;
    double BNO055_gyr_y_filt;
    double BNO055_gyr_z_filt;

    /** ADXL377 **/
    //Ffiltered data.
    double ADXL_acc_x;
    double ADXL_acc_y;
    double ADXL_acc_z;

    /** BMP280 **/
    double BMP280_temp;
    double BMP280_pres;
    double BMP280_alt;

    /** NEO6M **/
    double GPS_lat;
    double GPS_lon;
    double GPS_alt;
    double GPS_tow;
    double GPS_hacc;
    double GPS_vacc;

    /** Dead reckoning **/
    double DR_pos_x;
    double DR_pos_y;
    double DR_pos_z;

    double DR_vel_x;
    double DR_vel_y;
    double DR_vel_z;

    double DR_ori_x; // orientation data - will be nice to know how my algorithm works out afterwards
    double DR_ori_y;
    double DR_ori_z;
};

extern SDDataLogStruct SDDataLog;


void sdSetup(unsigned long long timeStamp);

void logData();

#endif //SDCARDDATALOG_H
