//
// Created by robosam2003 on 03/05/2022.
//

#ifndef BNO055DAEDALUS_H
#define BNO055DAEDALUS_H

#include <Arduino.h>
#include "BNO055.h"
#include <Wire.h>
#include "SimpleKalmanFilter.h"


/// \b GLOBAL \b VARIABLES
const byte BNO055_I2C_ADDRESS = 0x28;
BNO055 sensor(BNO055_I2C_ADDRESS, &Wire);

Vector<double> acc_biases = {0,0,0}; // biases for the accelerometers and gyros
Vector<double> gyr_biases = {0,0,0};


const int numDR = 50; // number of data points to average
double prevVect[numDR][9] = {}; /// Holds previous values of (true acceleration), velocity and omega (from gyro)
Vector<double> omegaAverage = {0,0,0}; // live bias calculations
Vector<double> accAverage = {0,0,0};
double biasAverageOmegaThreshold = 0.3;
double biasAverageAccThreshold = 0.3;

Vector<double> pos = {0}; // For dead reckoning purposes.
Vector<double> vel = {0};
Vector<double> ori = {0};
int DRcounter = 0;

SimpleKalmanFilter filteredAccBNO055X = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccBNO055Y = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccBNO055Z = SimpleKalmanFilter(0.05, 0.05, 0.01);

SimpleKalmanFilter filteredGyroX = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredGyroY = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredGyroZ = SimpleKalmanFilter(0.05, 0.05, 0.01);

Vector<double> filteredAccBNO055;
Vector<double> filteredGyro;
Vector<double> trueAccVect;

#define cycleTimeus 10000

/// \b Function \b Prototypes
// TODO: ADD function prototypes at end
void updateFilters(Vector<double> gyro, Vector<double> acc);

bno055_calib_stat_t calibrate();

double magnitude(Vector<double> vect);

void calcRotationVect(Vector<double> acc_meas, Vector<double> ori, Vector<double>& returnVect);

Vector<double> find_acc_biases();
Vector<double> find_gyr_biases();
void remove_biases(Vector<double> acc_biases, Vector<double> gyrOffsets);

void getInitialOrientation();

void deadReckoning(Vector<double> acc, Vector<double> omega, int updateTimeUs, double prevValues[numDR][9]);

void interruptCallback();

void BNO055Setup();



#endif //BNO055DAEDALUS_H
