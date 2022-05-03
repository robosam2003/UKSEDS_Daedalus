//
// Created by robosam2003 on 03/05/2022.
//

#include "ADXL377.h"


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void ADXL377Setup () {
    pinMode(ADXL377_XPin, INPUT);
    pinMode(ADXL377_YPin, INPUT);
    pinMode(ADXL377_ZPin, INPUT);
}


void getADXL377Acc (Vector<double> returnVect) {
    // Get raw accelerometer data for each axis
    int rawX = analogRead(ADXL377_XPin);
    int rawY = analogRead(ADXL377_YPin);
    int rawZ = analogRead(ADXL377_ZPin);

    returnVect[0] = mapf(rawX, 0, 1023, -ADXLscale, ADXLscale) - 9.579712; // Between 0 and 1023 on 3.3V processor
    returnVect[1] = mapf(rawY, 0, 1023, -ADXLscale, ADXLscale) - 5.747803;
    returnVect[2] = mapf(rawZ, 0, 1023, -ADXLscale, ADXLscale)- 21.075317 + 9.81;

}

void updateADXL377Filters(Vector<double> ADXLacc) {
    filteredADXL[0] = filteredAccADXL377_X.updateEstimate(static_cast<float>(ADXLacc[0]));
    filteredADXL[1] = filteredAccADXL377_Y.updateEstimate(static_cast<float>(ADXLacc[1]));
    filteredADXL[2] = filteredAccADXL377_Z.updateEstimate(static_cast<float>(ADXLacc[2]));
}