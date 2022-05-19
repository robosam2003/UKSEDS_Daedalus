#include <ADXL377.h>
#include "SPI.h"


void setup() {
    ADXL377Setup();
}

void loop() {
    Vector<double>rawADXLacc = getADXL377Acc();
    updateADXL377Filters(rawADXLacc);
    Serial.printf("\n%lf, %lf, %lf", rawADXLacc[0], rawADXLacc[1], rawADXLacc[2]);
    //Serial.printf("\n%lf, %lf, %lf\n", filteredADXL[0], filteredADXL[1], filteredADXL[2]);
    delayMicroseconds(25000);
}

