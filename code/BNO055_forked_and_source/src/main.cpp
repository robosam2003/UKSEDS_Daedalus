#include <Arduino.h>
#include "BNO055.h"
#include "I2CDevice.h"
#include "Wire.h"
#include "SimpleKalmanFilter.h"

const byte BNO055_I2C_ADDRESS = 0x28;


elapsedMillis milliTimer;
elapsedMicros microTimer;
BNO055 sensor(BNO055_I2C_ADDRESS, &Wire);

SimpleKalmanFilter filteredAccX = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccY = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccZ = SimpleKalmanFilter(0.05, 0.05, 0.01);

SimpleKalmanFilter filteredGyroX = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredGyroY = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredGyroZ = SimpleKalmanFilter(0.05, 0.05, 0.01);

Vector<double> filteredAcc;
Vector<double> filteredGyro;


void updateFilters(Vector<double> gyro, Vector<double> acc){

    filteredAcc[0] = filteredAccX.updateEstimate(acc[0]);
    filteredAcc[1] = filteredAccY.updateEstimate(acc[1]);
    filteredAcc[2] = filteredAccZ.updateEstimate(acc[2]);

    filteredGyro[0] = filteredGyroX.updateEstimate(gyro[0]);
    filteredGyro[1] = filteredGyroY.updateEstimate(gyro[1]);
    filteredGyro[2] = filteredGyroZ.updateEstimate(gyro[2]);

}

bno055_calib_stat_t calibrate(){
    sensor.setOperationMode(NDOF);
    bno055_calib_stat_t calstat = sensor.getCalibrationStatus();                       /// This delay can be changed -_
    while ( ((calstat.accel < 3) || (calstat.gyro < 3) || (calstat.mag < 3) || (calstat.sys < 2) ) && (milliTimer < 10000) ) {
        calstat = sensor.getCalibrationStatus();
        (Serial) ? Serial.printf("ACC: %d,   GYR: %d,   MAG: %d,   SYS: %d\n", calstat.accel, calstat.gyro, calstat.mag, calstat.sys) : 0;
        delay(1); // This small delay is needed so that the sensor has a chance to write to the register.
    }
    delay(3000);
    return calstat;

}


void find_offsets() {
    bool offsets_calibrated = false;
    int counter = 0;

    const int num = 256;
    double acc_biases_x[num] = {0};
    double acc_biases_y[num] = {0};
    double acc_biases_z[num] = {0};

    REGSET(OPR_MODE, 0b00000000); //config mode
    delay(20);
    REGSET(ACC_OFFSET_X_LSB, 0);
    REGSET(ACC_OFFSET_X_MSB, 0);
    REGSET(ACC_OFFSET_Y_LSB, 0);
    REGSET(ACC_OFFSET_Y_MSB, 0);
    REGSET(ACC_OFFSET_Z_LSB, 0);
    REGSET(ACC_OFFSET_Z_MSB, 0);

    REGSET(OPR_MODE, 0b00001100); // NDOF for reading

    double avg_x=-29, avg_y=8, avg_z=-30;


    while (!offsets_calibrated) {
        signed short acc_offset_VECT[3] = {};
        READVECT(ACC_DATA_X, 3, 2, acc_offset_VECT);
        signed short ori_VECT[3] = {};
        READVECT(EUL_HEADING, 3, 2, ori_VECT);
        for (auto i : ori_VECT) { i /= 900; } // radians
        double trueAccVect[3] = {};
        calcRotationVect(reinterpret_cast<double *>(acc_offset_VECT), reinterpret_cast<double *>(ori_VECT), trueAccVect);

        acc_biases_x[counter % num] = trueAccVect[0];
        acc_biases_y[counter % num] = trueAccVect[1];
        acc_biases_z[counter % num] = trueAccVect[2];
        bool within_range_x = true;
        bool within_range_y = true;
        bool within_range_z = true;
        if (counter > (2*num)) {

            for (auto ax: acc_biases_x) { avg_x += ax; }
            for (auto ay: acc_biases_y) { avg_y += ay; }
            for (auto az: acc_biases_z) { avg_z += az; }

            avg_x /= num;
            avg_y /= num;
            avg_z /= num;

            int thres = 5;
            for (auto a: acc_biases_x) { if (abs((a - avg_x) > thres)){ within_range_x = false; } }
            for (auto b: acc_biases_y) { if (abs((b - avg_y) > thres)){ within_range_y = false; } }
            for (auto c: acc_biases_z) { if (abs((c - avg_z) > thres)){ within_range_z = false; } }
            if ((within_range_x & within_range_y) & within_range_z) {

                double lambda = 980/ (sq(avg_x) + sq(avg_y)+ sq(avg_z));
                avg_x *= lambda;
                avg_y *= lambda;
                avg_z *= lambda;

                avg_z -= 980; // for if you are using acc data
                //avg_z = -avg_z;
                offsets_calibrated = true;

            }
        }
        counter++;

        if (Serial) { for (auto x:acc_offset_VECT) { Serial.printf("%d,     ", x ) ;} Serial.printf("%lf, %lf, %lf       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter>num), within_range_x, within_range_y, within_range_z); }
        delay(10);

    }



    //byte acc_offset_addresses[6] = {ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB, ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB, ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB};
    //byte mag_offset_addresses[6] = {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB, MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB, MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB};
    //byte gyr_offset_addresses[6] = {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB, GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB, GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB};

    signed short addresses[3][9] = {{ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB,
                                            ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB,
                                            ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB,
                                            static_cast<short>(avg_x), static_cast<short>(avg_y), static_cast<short>(avg_z)},
                                    {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB,
                                            MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB,
                                            MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB,
                                            0, 0, 0},
                                    {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB,
                                            GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB,
                                            GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB,
                                            0, 0 ,0}};


    // Sets offsets for Accelerometer, Gyroscope, and Magnetometer
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

        REGSET(OPR_MODE, 0b00000000);  // config mode for writing regs
        delay(20);

        REGSET(addresses[i][0], datalow_x);
        REGSET(addresses[i][1], datahigh_x);
        delay(10);
        REGSET(addresses[i][2], datalow_y);
        REGSET(addresses[i][3], datahigh_y);
        delay(10);
        REGSET(addresses[i][4], datalow_z);
        REGSET(addresses[i][5], datahigh_z);


        REGSET(OPR_MODE, 0b00001100);  // NDOF mode for reading

        //Serial.printf("%d,  %d,  %d  \n", acc_biases_x[0], acc_biases_x[1], acc_biases_x[2]);
        //Serial.printf("%d,  %d,  %d  \n", acc_biases_y[0], acc_biases_y[1], acc_biases_y[2]);
        //Serial.printf("%d,  %d,  %d  \n", acc_biases_z[0], acc_biases_z[1], acc_biases_z[2]);
        Serial.printf("x: %d, y: %d, z: %d  \n", x, y, z);

        Serial.println("ALL OFFSETS CALIBRATED");
        delay(1000);
    }





    //delay(2000);

    //REGSET(OPR_MODE, 0b00001100);

}

void remove_offsets(){
    short accOffsetX = -29,
            accOffsetY = 8,
            accOffsetZ = -30;
    signed short addresses[3][9] = {{ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB,
                                            ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB,
                                            ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB,
                                            accOffsetX, accOffsetY, accOffsetZ},
                                    {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB,
                                            MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB,
                                            MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB,
                                            0, 0, 0},
                                    {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB,
                                            GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB,
                                            GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB,
                                            0, 0 ,0}};
    for (int i = 0; i<3; i++) {
        short x = addresses[i][6];
        short y = addresses[i][7];
        short z = addresses[i][8];

        byte datalow_x = x & 0xFF;
        byte datahigh_x = (x >> 8) & 0xFF;
        byte datalow_y = y & 0xFF;
        byte datahigh_y = (y >> 8) & 0xFF;
        byte datalow_z = z & 0xFF;
        byte datahigh_z = (z >> 8) & 0xFF;

        sensor.setOperationMode(CONFIGMODE);


        sensor.writeRegister(addresses[i][0], datalow_x);


        sensor.writeRegister(addresses[i][1], datahigh_x);
        delay(10);
        sensor.writeRegister(addresses[i][2], datalow_y);
        sensor.writeRegister(addresses[i][3], datahigh_y);
        delay(10);
        sensor.writeRegister(addresses[i][4], datalow_z);
        sensor.writeRegister(addresses[i][5], datahigh_z);

        sensor.setOperationMode(NDOF);

        //Serial.printf("%d,  %d,  %d  \n", acc_biases_x[0], acc_biases_x[1], acc_biases_x[2]);
        //Serial.printf("%d,  %d,  %d  \n", acc_biases_y[0], acc_biases_y[1], acc_biases_y[2]);
        //Serial.printf("%d,  %d,  %d  \n", acc_biases_z[0], acc_biases_z[1], acc_biases_z[2]);
        Serial.printf("x: %d, y: %d, z: %d  \n", x, y, z);

        Serial.println("ALL OFFSETS CALIBRATED");
        delay(1000);
    }
}


void init() {
    delay(2000);
    sensor.setPowerMode(NORMAL);
    sensor.setOperationMode(CONFIGMODE); // registers must be configured in config mode
    sensor.setAccelerometerConfig(0b00000011); //16G

    sensor.writeRegister(BNO055_UNIT_SEL, 0b00000110); // Celsius, degrees, dps, m/s^2

    sensor.writeRegister(BNO055_AXIS_MAP_CONFIG, 0b00100100); // TODO: Check that this will be correct for our pcb
    sensor.writeRegister(BNO055_AXIS_MAP_SIGN, 0b00000000);


    remove_offsets();
    sensor.setOperationMode(NDOF);
    //calibrate();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Wire.begin();
    Wire.setClock(400000);  // i2c seems to work great at 1Mhz, but may need to run on 400kHz or even 100Khz if we have issues.
    delay(1000);
    sensor.begin();
    init();

}

double magnitude(Vector<double> vect){
    return sqrt( sq(vect[0]) + sq(vect[1]) + sq(vect[2]) );
}


void loop() {
    unsigned long a = microTimer;

    /// Data Aquisition
    Vector<double> acc = sensor.getRawAcceleration();
    Vector<double> mag = sensor.getRawMagnetometer();
    Vector<double> gyro = sensor.getRawGyro();
    Vector<double> lia = sensor.getLinearAcceleration();
    Vector<double> grav = sensor.getGravity();
    Vector<double> eul = sensor.getEuler();


       /// Kalman filtering

    //(gyro, acc);
    Serial.printf("%lf, %lf, %lf,    %lf,  9.81\n", acc[0], acc[1], acc[2], magnitude(acc));

    //Serial.print((String)data.mag.getX() + ", " + (String)data.mag.getY() + ", " + (String)data.mag.getZ() + ",   ");
    //Serial.println((String)data.gyro.getX() + ", " + (String)data.gyro.getY() + ", " + (String)data.gyro.getZ() + ",   ");
    unsigned long b = microTimer;
    //Serial.println(b-a);
    delayMicroseconds(10000);
}