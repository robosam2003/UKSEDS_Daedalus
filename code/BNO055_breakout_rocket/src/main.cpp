//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include <Arduino.h>
#include "BNO055.h"
#include <Wire.h>
#include <SPI.h>

elapsedMillis t1;


const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

void setup() {
    Wire.begin();
    sensor.begin();
    sensor.setOperationMode(NDOF);
}


void loop() {
    unsigned long a = t1;
    Vector<double> acc = sensor.getRawAcceleration();
    Serial.println((String)acc.getX() + "," + (String)acc.getY() + ","
                   + (String)acc.getZ() + ",");
    /*
    double acc[3] = {data.accel.getX(),
                          data.accel.getY(),
                          data.accel.getZ()} ;

    double grav[3] = {data.gravityVector.getX(),
                           data.gravityVector.getY(),
                           data.gravityVector.getZ()} ;

    double ori[3] = {data.euler.getX(),
                          data.euler.getY(),
                          data.euler.getZ()} ;


    Serial.printf("%lf, %lf, %lf,   ", acc[0], acc[1], acc[2]);
    Serial.printf("%lf, %lf, %lf,   ", grav[0], grav[1], grav[2]);
    Serial.printf("%lf, %lf, %lf,   \n", ori[0], ori[1], ori[2]);
    */
    // TODO: add offset configuration code.


    delay(10);
}
