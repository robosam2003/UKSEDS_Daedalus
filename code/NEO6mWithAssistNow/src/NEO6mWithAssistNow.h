//
// Created by robos on 04/05/2022.
//

#ifndef NEO6MWITHASSISTNOW_H
#define NEO6MWITHASSISTNOW_H

// Created by Samuel scott (robosam2003) on 30/04/2022
// This program sets up the neo6m gps and performs assistNow loading.
/// Datasheet url: https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#page=118&zoom=100,0,0
/// Also see: https://content.u-blox.com/sites/default/files/products/documents/MultiGNSS-Assistance_UserGuide_%28UBX-13004360%29.pdf
/// little endian format /:)

// This program reads the mgaonline.ubx file from the sd card and then writes it to the NEO6M GPS


#include <Arduino.h>
#include <SD.h>
#include <TimeLib.h> // : https://github.com/PaulStoffregen/Time
#include "assistNow.h"
#include <ctime>



#define transistorPin 32
#define ubxHeader1 0xB5
#define ubxHeader2 0x62
#define gpsBaudRate 115200 // running at faster than default to avoid serial reads of over 10ms (which was common at 9600 BAUD)

/// Change this what Serial port the GPS is connected to
#define gpsSerial Serial2 // Hardware serial port for GPS


struct GPSDataStruct {
    unsigned int towMs; // time of week in milliseconds
    double lat; // latitude in degrees
    double lon; // longitude in degrees
    double alt; // HSML (height above mean sea level) in meters
    signed int hAcc; // horizontal accuracy estimate in mm
    signed int vAcc; // vertical accuracy estimate in mm
};
extern GPSDataStruct GPSdata;

void NEO6mSetup();

void getGPSData();

void getGPSlatLongAlt(double GPSdata[3]);

uint64_t getTimestampMillis();

time_t getTeensyTime();

void rtcSetup();

time_t TimeFromYMD(int year, int month, int day);

unsigned short GPSweek();

unsigned int actualTimeOfWeekms();

void serialClear(); // gps serial clear hehe

void sendUbx(byte ubxClassID, byte messageID, short payloadLength, const byte payload[]);

void getUbx(byte ubxClassID, byte messageID, short payloadLength, byte payload[]);

int getUbxFromFile(File fptr, byte ubxClassID, byte messageID, short payloadLength, byte payload[]);

void performOnlineAssist();

void NEO6mConfig();








#endif //NEO6MWITHASSISTNOW_H
