//
// Created by robosam2003 on 03/05/2022.
//

#ifndef BNO055DAEDALUS_H
#define BNO055DAEDALUS_H

#include <Arduino.h>
#include "BNO055.h"
#include <Wire.h>
#include "SimpleKalmanFilter.h"

#define numDR 50
#define cycleTimeus 10000

/// \b GLOBAL \b VARIABLES
extern const byte BNO055_I2C_ADDRESS;
extern BNO055 sensor;

extern Vector<double> acc_biases; // biases for the accelerometers and gyros
extern Vector<double> gyr_biases;



extern double prevVect[numDR][9]; /// Holds previous values of (true acceleration), velocity and omega (from gyro)
extern Vector<double> omegaAverage; // live bias calculations
extern Vector<double> accAverage;
extern double biasAverageOmegaThreshold;
extern double biasAverageAccThreshold;

extern Vector<double> pos; // For dead reckoning purposes.
extern Vector<double> vel;
extern Vector<double> ori;
extern int DRcounter;

extern SimpleKalmanFilter filteredAccBNO055X;
extern SimpleKalmanFilter filteredAccBNO055Y;
extern SimpleKalmanFilter filteredAccBNO055Z;

extern SimpleKalmanFilter filteredGyroX;
extern SimpleKalmanFilter filteredGyroY;
extern SimpleKalmanFilter filteredGyroZ;

extern Vector<double> filteredAccBNO055;
extern Vector<double> filteredGyro;
extern Vector<double> trueAccVect;

extern volatile bool launchInterrupt;



///  Function Prototypes
void updateBNOFilters(Vector<double> gyro, Vector<double> acc);

void enterToContinue();

bno055_calib_stat_t calibrate();

double magnitude(Vector<double> vect);

void calcRotationVect(Vector<double> acc_meas, Vector<double> ori, Vector<double>& returnVect);

Vector<double> find_acc_biases();
Vector<double> find_gyr_biases();
void remove_biases(Vector<double> acc_biases, Vector<double> gyrOffsets);

void getInitialOrientation();

void deadReckoning(Vector<double> acc, Vector<double> omega, int updateTimeUs);

void interruptCallback();

void BNO055Setup();



#endif //BNO055DAEDALUS_H
