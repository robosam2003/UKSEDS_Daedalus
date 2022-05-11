#include "BMP280.h"


/// datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf



Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

void BMP280Setup() {
    unsigned int status = bmp.begin();

}

void getBMP280Data(double returnVect[]) {
    float a = bmp.readTemperature();
    float b = bmp.readPressure();
    float c = bmp.readAltitude(1013.25); // Adjust to local forecast!
    returnVect[0] = a;
    returnVect[1] = b;
    returnVect[2] = c;
}