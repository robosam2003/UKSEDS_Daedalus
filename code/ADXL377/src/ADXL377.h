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

float ADXLscale = 200*9.8; //  (±200g) for ADXL377

SimpleKalmanFilter filteredAccADXL377_X = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccADXL377_Y = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccADXL377_Z = SimpleKalmanFilter(0.05, 0.05, 0.01);
Vector<double> filteredADXL;


float mapf(float x, float in_min, float in_max, float out_min, float out_max);

void ADXL377Setup ();

void getADXL377Acc (Vector<double> returnVect);

void updateADXL377Filters(Vector<double> ADXLacc);


#endif //ADXL377_H
