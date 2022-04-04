#include <Arduino.h>
#include "BNO055.h"
#include "i2c_driver_wire.h"
#include "SimpleKalmanFilter.h"

const byte BNO055_I2C_ADDRESS = 0x28;
BNO055 sensor(BNO055_I2C_ADDRESS, &Wire);

Vector<double> acc_offsets;
Vector<double> gyr_offsets;

SimpleKalmanFilter filteredAccX = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccY = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccZ = SimpleKalmanFilter(0.05, 0.05, 0.01);

SimpleKalmanFilter filteredGyroX = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredGyroY = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredGyroZ = SimpleKalmanFilter(0.05, 0.05, 0.01);

Vector<double> filteredAcc;
Vector<double> filteredGyro;


void updateFilters(Vector<double> gyro, Vector<double> acc){ // TODO: Add any other data points you want filtered
    filteredAcc[0] = filteredAccX.updateEstimate(acc[0]);
    filteredAcc[1] = filteredAccY.updateEstimate(acc[1]);
    filteredAcc[2] = filteredAccZ.updateEstimate(acc[2]);
    filteredGyro[0] = filteredGyroX.updateEstimate(gyro[0]);
    filteredGyro[1] = filteredGyroY.updateEstimate(gyro[1]);
    filteredGyro[2] = filteredGyroZ.updateEstimate(gyro[2]);
}

bno055_calib_stat_t calibrate(){
    uint32_t timer = millis();
    int timeout = 20000; // milliseconds
    sensor.setOperationMode(NDOF);
    bno055_calib_stat_t calstat = sensor.getCalibrationStatus();
    while ( ((calstat.accel < 3) || (calstat.gyro < 3) || (calstat.mag < 3) || (calstat.sys < 2) ) && ((millis() - timer) < timeout) ) {
        calstat = sensor.getCalibrationStatus();
        (Serial) ? Serial.printf("ACC: %d,   GYR: %d,   MAG: %d,   SYS: %d\n", calstat.accel, calstat.gyro, calstat.mag, calstat.sys) : 0;
        delay(1); // This small delay is needed so that the sensor has a chance to write to the register.
    }
    delay(3000);
    return calstat;

}

double magnitude(Vector<double> vect){
    return sqrt( sq(vect[0]) + sq(vect[1]) + sq(vect[2]) );
}

void calcRotationVect(Vector<double> acc_meas, Vector<double> ori, Vector<double>& returnVect) { // returnVect needs to be passes by reference
    /// calculates the absolute acceleration (relative to north and flat) based on acceleration data and orientation data

    double degToRad = (2*pi)/360; // the ori returns data in degrees.
    double heading = ori[0]*degToRad; // in radians
    double pitch = ori[1]*degToRad; // in radians
    double roll = ori[2]*degToRad; // in radians

    double rotMatX[3][3] = { {1, 0,          0         },
                             {0, cos(roll),  -sin(roll) },
                             {0, sin(roll), cos(roll) } };

    double rotMatY[3][3] = { { cos(pitch),  0, sin(pitch) },
                             { 0,           1, 0          },
                             { -sin(pitch), 0, cos(pitch) } };

    double rotMatZ[3][3] = { { cos(heading), -sin(heading), 0 },
                             { sin(heading), cos(heading),  0 },
                             { 0,            0,             1} };
    double vec1[3] = {};
    double vec2[3] = {};
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            vec1[i] += rotMatX[i][j]*acc_meas[j];
        }
    }
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            vec2[i] += rotMatY[i][j]*vec1[j];
        }
    }
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            returnVect[i] += rotMatZ[i][j]*vec2[j];
        }
    }

}

Vector<double> find_acc_offsets() {
    /// Should be done on a calibrated sensor
    bool offsets_calibrated = false;
    int counter = 0;

    const int num = 127;
    double acc_biases_x[num] = {0};
    double acc_biases_y[num] = {0};
    double acc_biases_z[num] = {0};

    sensor.setOperationMode(CONFIGMODE);  // set sensor to config mode to reset offsets - // TODO: IS THIS NECESSARY???
    for (auto x : {ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB, ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB, ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB}) {
        sensor.writeRegister(x, 0);
    }
    sensor.setOperationMode(NDOF); // NDOF mode for reading

    double avg_x = 0, avg_y = 0, avg_z = 0;

    // ACC calibration
    while (!offsets_calibrated) {
        Vector<double> acc_offset = sensor.getRawAcceleration();
        Vector<double> ori = sensor.getEuler();
        ori[0] = 360-ori[0];   ori[1] = -ori[1];
        Vector<double> trueAccVect = {0,0,0};
        calcRotationVect(acc_offset, ori, trueAccVect);

        acc_biases_x[counter % num] = trueAccVect[0];
        acc_biases_y[counter % num] = trueAccVect[1];
        acc_biases_z[counter % num] = trueAccVect[2];
        bool within_range_x = true;
        bool within_range_y = true;
        bool within_range_z = true;
        if (counter > (2 * num)) {

            for (auto ax: acc_biases_x) { avg_x += ax; }
            for (auto ay: acc_biases_y) { avg_y += ay; }
            for (auto az: acc_biases_z) { avg_z += az; }

            avg_x /= num;
            avg_y /= num;
            avg_z /= num;

            double thres = 0.05;
            for (auto a: acc_biases_x) { if (abs((a - avg_x) > thres)) { within_range_x = false; }}
            for (auto b: acc_biases_y) { if (abs((b - avg_y) > thres)) { within_range_y = false; }}
            for (auto c: acc_biases_z) { if (abs((c - avg_z) > thres)) { within_range_z = false; }}
            if ((within_range_x & within_range_y) & within_range_z) {
                /*double lambda = 9.80 / sqrt(sq(avg_x) + sq(avg_y) + sq(avg_z));
                avg_x *= lambda;
                avg_y *= lambda; // scaling to make the total equal to 9.81
                avg_z *= lambda;*/ /// not sure if this but works or is even necessary tbh.

                Serial.println("calculated averages");
                avg_z -= 9.80; // for if you are using acc data
                offsets_calibrated = true;
            }
        }
        counter++;

        if (Serial) {
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", trueAccVect[i]); }
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", ori[i]); }
            Serial.printf("%lf, %lf, %lf       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter > num), within_range_x, within_range_y, within_range_z);
        }
        delay(10);

    }
    Vector <double> offsets = {avg_x, avg_y, avg_z};
    return offsets;
}

Vector<double> find_gyr_offsets() {
    /// Should be done on a calibrated sensor
    bool offsets_calibrated = false;
    int counter = 0;

    const int num = 512;
    double gyr_biases_x[num] = {0};
    double gyr_biases_y[num] = {0};
    double gyr_biases_z[num] = {0};

    sensor.setOperationMode(CONFIGMODE);  // set sensor to config mode to reset offsets - // TODO: IS THIS NECESSARY???
    for (auto x : {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB, GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB, GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB}) {
        sensor.writeRegister(x, 0);
    }
    sensor.setOperationMode(AMG); // NDOF mode for reading

    double avg_x = 0, avg_y = 0, avg_z = 0;

    // ACC calibration
    while (!offsets_calibrated) {
        Vector<double> gyrVect = sensor.getRawGyro();
        updateFilters(gyrVect, {0,0,0});
        gyr_biases_x[counter % num] = filteredGyro[0];
        gyr_biases_y[counter % num] = filteredGyro[1];
        gyr_biases_z[counter % num] = filteredGyro[2];
        bool within_range_x = true;
        bool within_range_y = true;
        bool within_range_z = true;
        if (counter > (num)) {
            for (auto ax: gyr_biases_x) { avg_x += ax; }
            for (auto ay: gyr_biases_y) { avg_y += ay; }
            for (auto az: gyr_biases_z) { avg_z += az; }
            avg_x /= num;
            avg_y /= num;
            avg_z /= num;

            double thres = 0.05;
            for (auto a: gyr_biases_x) { if (abs((a - avg_x) > thres)) { within_range_x = false; }}
            for (auto b: gyr_biases_y) { if (abs((b - avg_y) > thres)) { within_range_y = false; }}
            for (auto c: gyr_biases_z) { if (abs((c - avg_z) > thres)) { within_range_z = false; }}
            if ((within_range_x & within_range_y) & within_range_z) {
                Serial.println("calculated averages");
                offsets_calibrated = true;
            }
        }
        counter++;

        if (Serial) {
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", filteredGyro[i]); }

            Serial.printf("%lf, %lf, %lf       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter > num), within_range_x, within_range_y, within_range_z);
        }
        delay(10);

    }
    Vector <double> offsets = {avg_x, avg_y, avg_z};
    return offsets;
}

void remove_offsets(Vector<double> acc_offsets, Vector<double> gyrOffsets){

    signed short addresses[3][9] = {{ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB,
                                     ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB,
                                     ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB,
                                     static_cast<short>(acc_offsets[0]*100), static_cast<short>(acc_offsets[1]*100), static_cast<short>(acc_offsets[2]*100) },
                                    {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB,
                                     MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB,
                                     MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB,
                                     0, 0, 0},
                                    {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB,
                                     GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB,
                                     GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB,
                                     static_cast<short>(gyrOffsets[0]*16), static_cast<short>(gyrOffsets[1]*16), static_cast<short>(gyrOffsets[2]*16)}};

    for (int i = 0; i<3; i++) {
        signed short x = addresses[i][6];
        signed short y = addresses[i][7];
        signed short z = addresses[i][8];

        byte datalow_x = x & 0xFF;
        byte datahigh_x = (x >> 8) & 0xFF;
        byte datalow_y = y & 0xFF;
        byte datahigh_y = (y >> 8) & 0xFF;
        byte datalow_z = z & 0xFF;
        byte datahigh_z = (z >> 8) & 0xFF;

        sensor.setOperationMode(CONFIGMODE);
        delay(20);
        sensor.writeRegister(addresses[i][0], datalow_x);
        sensor.writeRegister(addresses[i][1], datahigh_x);
        delay(10);
        sensor.writeRegister(addresses[i][2], datalow_y);
        sensor.writeRegister(addresses[i][3], datahigh_y);
        delay(10);
        sensor.writeRegister(addresses[i][4], datalow_z);
        sensor.writeRegister(addresses[i][5], datahigh_z);
        sensor.setOperationMode(NDOF);

        Serial.printf("x: %d, y: %d, z: %d  \n", x, y, z);
        Serial.println("ALL OFFSETS CALIBRATED");
        delay(1000);
    }
}

void BNO055Setup() {
    sensor.begin();
    sensor.setPowerMode(NORMAL);
    sensor.setOperationMode(CONFIGMODE); // registers must be configured in config mode

    //calibrate();
    //
    acc_offsets = find_acc_offsets();
    gyr_offsets = find_gyr_offsets();
    //remove_offsets(acc_offsets, gyr_offsets);
    sensor.setOperationMode(AMG);

    sensor.writeRegister(BNO055_UNIT_SEL, 0b00000000); // Celsius, degrees, dps, m/s^2
    sensor.setAccelerometerConfig(0b00011011); //Normal mode, 500Hz, 16G
    sensor.setGyroscopeConfig(0b00000000); // 523 Hz, 2000dps
    sensor.setGyroscopeOperationMode(0b00000000); // Normal mode : 0bxxxxx000
    sensor.setMagnetometerConfig(0b00011111);

    //sensor.writeRegister(BNO055_AXIS_MAP_CONFIG, 0x24); // TODO: Check that this will be correct for our pcb (Axis remap);
    sensor.writeRegister(BNO055_AXIS_MAP_SIGN, 0b00000000);

}

void setup() {
    Wire.setClock(1000000);  // i2c seems to work great at 1Mhz
    Wire.begin();

    BNO055Setup();

}

void deadReckoning(Vector<double> acc, Vector<double> gyr, int updateTimeUs, double prevValues[6], double returnVect[9] ) {
    for (int i=0;i<3;i++) {

        returnVect[6+i] += (updateTimeUs*0.000001)*0.5*(gyr[2-i] + prevValues[5-i]); /// integration of angular velocity, first order hold.

        // Do reference frame conversion before this step. and also minus gravity,
        returnVect[3+i] += acc[i]*(updateTimeUs*0.000001); /// Velocity calculation, zero order hold.
        returnVect[i]   += returnVect[3+i]*(updateTimeUs*0.000001); /// position calculation, zero order hold
    }
}

double returnVect[9] = {0,0,0, 0,0,0, 0,0,0}; // posx, posy, posz, velx, vely, velz, heading, pitch, roll // initially starts at zero.
double prevVect[6] = {0,0,0, 0,0,0}; // previous values of acceleration, and gyro
// should start at ori from NDFOF mode.
void loop() {
    uint32_t startOfLoop = micros();
    /// Data Aquisition
    bno055_burst_t data = sensor.getAllData();
    Vector<double> BNO055acc = data.accel;
    Vector<double> mag = data.mag;
    Vector<double> gyro = data.gyro;

    /// removing offsets, for internal gyroscope integration. Higher precision of calculation this way.
    for(int i=0;i<3;i++) { BNO055acc[i] -= acc_offsets[0]; gyro[i] -= gyr_offsets[i]; }

    //eul[0] = 360-eul[0];   eul[1] = -eul[1];  // necessary to have all the angles going anticlockise-> increasing. - for the reference frame conversion.

/// --------------------------------------------------------------------------------------

/// Kalman filtering

    updateFilters(gyro, BNO055acc);


    /// Calculations
    deadReckoning(filteredAcc, filteredGyro, 10000, prevVect, returnVect);
    for(int i=0;i<3;i++) { prevVect[i] = BNO055acc[i]; prevVect[3+i] = gyro[i]; }



    Serial.printf("%lf,  %lf,  %lf  \n", returnVect[6], returnVect[7], returnVect[8]);



    uint32_t endOfLoop = micros();

    delayMicroseconds(((endOfLoop-startOfLoop) < 10000 ) ? (10000- (endOfLoop - startOfLoop) ) : 0);
}