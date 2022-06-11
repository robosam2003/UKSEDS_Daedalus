// Created by robosam2003

#include "BNO055daedalus.h"


void setup() {
    BNO055Setup();
}



void loop() {
    uint32_t startOfLoop = micros();
    /// Data Acquisition
    bno055_burst_t data = sensor.getAllData();
    Vector<double> BNO055accRaw = data.accel;
    Vector<double> magRaw = data.mag;
    Vector<double> gyroRaw = data.gyro;

    // TESTING HEADING PITCH AND ROLL
    /// Removing biases for internal integration of gyroscopes and accelerometers. \n Results in higher precision of calculation than setting bias/offset registers
    for(int i=0;i<3;i++) { BNO055accRaw[i] -= acc_biases[i];  gyroRaw[i] -= gyr_biases[i]; }

    //eul[0] = 360-eul[0];   eul[1] = -eul[1];  // necessary to have all the angles going anticlockise-> increasing. - for the reference frame conversion in fusion mode

    /// Kalman filtering
    updateBNOFilters(gyroRaw, BNO055accRaw);


    /// Calculations
    //deadReckoning(filteredAccBNO055, filteredGyro, cycleTimeus);

    trueAccVect = {0,0,0};
    calcRotationVect(filteredAccBNO055, ori, trueAccVect);

    double radToDeg = 180/PI;
    double x = filteredAccBNO055[0];
    double y = filteredAccBNO055[1];
    double z = filteredAccBNO055[2];

    ori[0] = atan2(y, (sqrt(z*z + x*x)))*radToDeg;
    ori[1] = atan2(-x, sqrt(y*y+z*z))*radToDeg;
    ori[2] = 0;


    // ori[0] = atan2(filteredAccBNO055[1], filteredAccBNO055[2]) * (radToDeg); // roll
    // ori[1] = atan2(-filteredAccBNO055[0], (sqrt(filteredAccBNO055[0]))) * (radToDeg); // pitch
    // ori[2] = 0; // yaw/heading - doesn't matter what this is essentially. 
    // SHOULD USE FILTERED VALUES
             
    Serial.printf("ACC: %lf, %lf, %lf     -    ORI(from acc) : %lf, %lf, %lf     \n     -  TAV: %lf, %lf, %lf", 
    filteredAccBNO055[0], filteredAccBNO055[1], filteredAccBNO055[2],
    ori[0], ori[1], ori[2],
    trueAccVect[0], trueAccVect[1], trueAccVect[2]);
    




    uint32_t endOfLoop = micros();
    //Serial.println(endOfLoop - startOfLoop);
    delayMicroseconds( ((endOfLoop-startOfLoop) < cycleTimeus ) ? (cycleTimeus- (endOfLoop - startOfLoop) ) : 0 );

}