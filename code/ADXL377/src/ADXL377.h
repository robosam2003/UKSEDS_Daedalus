//
// Created by robosam2003 on 03/05/2022.
//

#ifndef ADXL377_H
#define ADXL377_H

// #includes
#include <Arduino.h>
#include <BNO055.h>
#include <SimpleKalmanFilter.h>


#define ADXL377_XPin 21
#define ADXL377_YPin 22
#define ADXL377_ZPin 23

extern float ADXLscale; //  (±200g) for ADXL377

extern SimpleKalmanFilter filteredAccADXL377_X;
extern SimpleKalmanFilter filteredAccADXL377_Y;
extern SimpleKalmanFilter filteredAccADXL377_Z;
extern Vector<double> filteredADXL;


float mapf(float x, float in_min, float in_max, float out_min, float out_max);

void ADXL377Setup ();

Vector<double> getADXL377Acc();

void updateADXL377Filters(Vector<double> ADXLacc);


#endif //ADXL377_H
