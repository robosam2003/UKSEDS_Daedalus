#include <Arduino.h>
#include "BNO055.h"
#include "I2CDevice.h"
#include "Wire.h"
#include "SimpleKalmanFilter.h"

const byte BNO055_I2C_ADDRESS = 0x28;


elapsedMillis milliTimer;
elapsedMicros microTimer;
BNO055 sensor(BNO055_I2C_ADDRESS, &Wire);


void init() {
    delay(2000);
    sensor.setPowerMode(NORMAL);
    sensor.setOperationMode(CONFIGMODE); // registers must be configured in config mode
    sensor.setAccelerometerConfig(0b00000011); //16G

    sensor.writeRegister(BNO055_UNIT_SEL, 0b00000110); // Celsius, degrees, dps, m/s^2

    sensor.writeRegister(BNO055_AXIS_MAP_CONFIG, 0b00100100); // TODO: Check that this will be correct for our pcb
    sensor.writeRegister(BNO055_AXIS_MAP_SIGN, 0b00000000);

    sensor.setOperationMode(NDOF);


}

void setup() {
    Wire.begin();
    Wire.setClock(1000000);  // i2c seems to work great at 1Mhz, but may need to run on 400kHz or even 100Khz if we have issues.
    sensor.begin();
    init();

}



void loop() {
    unsigned long a = microTimer;
    Vector<double> acc = sensor.getRawAcceleration();
    Vector<double> mag = sensor.getRawMagnetometer();
    Vector<double> gyro = sensor.getRawGyro();
    Vector<double> lia = sensor.getLinearAcceleration();
    Vector<double> grav = sensor.getGravity();
    Vector<double> eul = sensor.getEuler();



    //Serial.printf("%lf, lf, lf%c\n", acc.getX(), acc.getY(), acc.getZ());

    //Serial.print((String)data.mag.getX() + ", " + (String)data.mag.getY() + ", " + (String)data.mag.getZ() + ",   ");
    //Serial.println((String)data.gyro.getX() + ", " + (String)data.gyro.getY() + ", " + (String)data.gyro.getZ() + ",   ");
    unsigned long b = microTimer;
    Serial.println(b-a);
}