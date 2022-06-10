
#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define BMP_CS   10


struct bmp280DataStruct {
    double temperature;
    double pressure;
    double altitude;
};

extern bmp280DataStruct bmpData;

extern Adafruit_BMP280 bmp;

void BMP280Setup();

void getBMP280Data(double seaLevelHPA);

#endif //BMP280_H
