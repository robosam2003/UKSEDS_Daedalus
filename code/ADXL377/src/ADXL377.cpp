//
// Created by robosam2003 on 03/05/2022.
//

#include "ADXL377.h"


float ADXLscale = 200*9.8; //  (±200g) for ADXL377
int a = 2;
SimpleKalmanFilter filteredAccADXL377_X = SimpleKalmanFilter(a, a, 0.01);
SimpleKalmanFilter filteredAccADXL377_Y = SimpleKalmanFilter(a, a, 0.01);
SimpleKalmanFilter filteredAccADXL377_Z = SimpleKalmanFilter(a, a, 0.01);
Vector<double> filteredADXL;


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void ADXL377Setup () {
    pinMode(ADXL377_XPin, INPUT);
    pinMode(ADXL377_YPin, INPUT);
    pinMode(ADXL377_ZPin, INPUT);
}


Vector<double> getADXL377Acc () {
    Vector<double> acc;
    // Get raw accelerometer data for each axis
    int rawX = analogRead(ADXL377_XPin);
    int rawY = analogRead(ADXL377_YPin);
    int rawZ = analogRead(ADXL377_ZPin);

    acc[0] = mapf(rawX, 0, 1023, -ADXLscale, ADXLscale); // Between 0 and 1023 on 3.3V processor
    acc[1] = mapf(rawY, 0, 1023, -ADXLscale, ADXLscale);
    acc[2] = mapf(rawZ, 0, 1023, -ADXLscale, ADXLscale);
    return acc;
}

void updateADXL377Filters(Vector<double> ADXLacc) {
    filteredADXL[0] = filteredAccADXL377_X.updateEstimate(static_cast<float>(ADXLacc[0]));
    filteredADXL[1] = filteredAccADXL377_Y.updateEstimate(static_cast<float>(ADXLacc[1]));
    filteredADXL[2] = filteredAccADXL377_Z.updateEstimate(static_cast<float>(ADXLacc[2]));
}