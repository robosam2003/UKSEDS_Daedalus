
#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define BMP_CS   10

extern Adafruit_BMP280 bmp;

void BMP280Setup();

void getBMP280Data(double returnVect[]);

#endif //BMP280_H
