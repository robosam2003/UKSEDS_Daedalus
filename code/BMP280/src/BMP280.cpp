#include "BMP280.h"


/// datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

struct bmp280DataStruct bmpData;


Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

void BMP280Setup() {
    unsigned int status = bmp.begin();
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1 );
}

void getBMP280Data(double seaLevelHPA) {
    bmpData.temperature = static_cast<double>( bmp.readTemperature() );
    bmpData.pressure =    static_cast<double>( bmp.readPressure() );
    bmpData.altitude =    static_cast<double>( bmp.readAltitude(seaLevelHPA) ); // in meters
}