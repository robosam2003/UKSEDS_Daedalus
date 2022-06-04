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
    deadReckoning(filteredAccBNO055, filteredGyro, cycleTimeus);

//    Serial.printf("TAV: %lf, %lf, %lf  |  ORI: %lf, %lf, %lf  |  ",
//                  trueAccVect[0], trueAccVect[1], trueAccVect[2],
//                 //  vel[0], vel[1], vel[2],
//                 //  pos[0], pos[1], pos[2],
//                  ori[0], ori[1], ori[2]);

//    Serial.printf("GYRO: %lf, %lf, %lf     ", filteredGyro[0], filteredGyro[1], filteredGyro[2]);

    // double radToDeg = 180/PI;
    // ori[0] = atan2(filteredAccBNO055[1], filteredAccBNO055[2]) * (radToDeg); // roll
    // ori[1] = atan2(-filteredAccBNO055[0], filteredAccBNO055[2]) * (radToDeg); // pitch
    // ori[2] = 0; // yaw/heading - doesn't matter what this is essentially

    Serial.printf("TAV : %lf, %lf, %lf  |  ORI (GYR): %lf, %lf, %lf  | POS: %lf, %lf, %lf\n",
            trueAccVect[0], trueAccVect[1], trueAccVect[2],
            ori[0], ori[1], ori[2],
            pos[0], pos[1], pos[2]);
             
//    Serial.printf("filt ACC : %lf, %lf, %lf     \n", filteredAccBNO055[0], filteredAccBNO055[1], filteredAccBNO055[2]);
//    Serial.printf("POS: %lf, %lf, %lf\n", pos[0], pos[1], pos[2]);



    uint32_t endOfLoop = micros();
    //Serial.println(endOfLoop - startOfLoop);
    delayMicroseconds( ((endOfLoop-startOfLoop) < cycleTimeus ) ? (cycleTimeus- (endOfLoop - startOfLoop) ) : 0 );

}