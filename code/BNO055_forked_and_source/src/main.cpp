#include <Arduino.h>
#include "BNO055.h"
#include "Wire.h"
#include "SimpleKalmanFilter.h"

const byte BNO055_I2C_ADDRESS = 0x28;
BNO055 sensor(BNO055_I2C_ADDRESS, &Wire);

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

void calcRotationVect(Vector<double> acc_meas, Vector<double> ori, double returnVect[3]) {
    /// calculates the absolute acceleration (relative to north and flat) based on acceleration data and orientation data
    double degToRad = (2*pi)/360; // the ori returns data in degrees.
    double roll = ori[2]*degToRad; // in radians
    double pitch = ori[1]*degToRad; // in radians
    double heading = ori[0]*degToRad; // in radians
    double rotMatX[3][3] = { {1, 0,          0         },
                             {0, cos(roll),  -sin(roll) },
                             {0, sin(roll), cos(roll) } };
    double rotMatY[3][3] = { { cos(pitch),  0, sin(pitch) },
                             { 0,           1, 0          },
                             { -sin(pitch), 0, cos(pitch) } };
    double rotMatZ[3][3] = { { cos(heading), -sin(heading), 0 },
                             { sin(heading), cos(heading),  0 },
                             { 0,            0,             1} };
    double vec1[3] = {}, vec2[3] = {};
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
            returnVect[i] += rotMatZ[i][j]*vec2[j]; // TODO: Can i make this absolute value equal to 9.8 when on the ground? fixes needed
        }
    }
}

Vector<double> find_offsets() {
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

    while (!offsets_calibrated) {
        Vector<double> acc_offset = sensor.getRawAcceleration();
        Vector<double> ori = sensor.getEuler();

        double trueAccVect[3] = {0,0,0};
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

void remove_offsets(Vector<double> offsets){
    short accOffsetX = (offsets[0]*100),
          accOffsetY = (offsets[1]*100),
          accOffsetZ = (offsets[2]*100);
    signed short addresses[3][9] = {{ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB,
                                     ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB,
                                     ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB,
                                     static_cast<short>(accOffsetX), static_cast<short>(accOffsetY), static_cast<short>(accOffsetZ) },
                                    {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB,
                                     MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB,
                                     MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB, // TODO: Are there offsets for mag and gyro?
                                     0, 0, 0},
                                    {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB,
                                     GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB,
                                     GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB,
                                     0, 0 ,0}};
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

void BNO055Init() {
    delay(2000);
    sensor.setPowerMode(NORMAL);
    sensor.setOperationMode(CONFIGMODE); // registers must be configured in config mode
    sensor.setAccelerometerConfig(0b00000011); //16G
    sensor.writeRegister(BNO055_UNIT_SEL, 0b00000110); // Celsius, degrees, dps, m/s^2
    sensor.writeRegister(BNO055_AXIS_MAP_CONFIG, 0b00100100); // TODO: Check that this will be correct for our pcb
    sensor.writeRegister(BNO055_AXIS_MAP_SIGN, 0b00000000);

    //calibrate();
    Vector<double> offsets = find_offsets();
    remove_offsets(offsets);
    sensor.setOperationMode(NDOF);

}


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Wire.begin();
    Wire.setClock(1000000);  // i2c seems to work great at 1Mhz, but may need to run on 400kHz or even 100Khz if we have issues.
    delay(1000);
    sensor.begin();
    BNO055Init();

}


void loop() {
    unsigned long a = micros();

    /// Data Aquisition
    Vector<double> BNO055acc = sensor.getRawAcceleration();
    Vector<double> mag = sensor.getRawMagnetometer();
    Vector<double> gyro = sensor.getRawGyro();
    Vector<double> lia = sensor.getLinearAcceleration();
    Vector<double> grav = sensor.getGravity();
    Vector<double> eul = sensor.getEuler();



       /// Kalman filtering

    updateFilters(gyro, BNO055acc);

    Serial.printf("%lf,  %lf,  %lf      %lf,  %lf,  %lf \n", BNO055acc[0], BNO055acc[1], BNO055acc[2], eul[0], eul[1], eul[2]);


    unsigned long b = micros();

    delayMicroseconds(10000);
}