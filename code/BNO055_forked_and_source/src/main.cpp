/// \b INCLUDES
#include <Arduino.h>
#include "BNO055.h"
#include "i2c_driver_wire.h"
#include "SimpleKalmanFilter.h"


/// \b GLOBAL \b VARIABLES
const byte BNO055_I2C_ADDRESS = 0x28;
BNO055 sensor(BNO055_I2C_ADDRESS, &Wire);

Vector<double> acc_biases = {0,0,0}; // biases for the accelerometers and gyros
Vector<double> gyr_biases = {0,0,0};


const int numDR = 49;
double prevVect[numDR][9] = {}; /// Holds previous values of (true acceleration), velocity and omega (from gyro)

Vector<double> omegaAverage = {0,0,0}; // live bias calculations
Vector<double> accAverage = {0,0,0};
double biasAverageOmegaThreshold = 0;
double biasAverageAccThreshold = 0;

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

bno055_calib_stat_t calibrate();
void getInitialOrientation();


void updateFilters(Vector<double> gyro, Vector<double> acc){ // TODO: Add any other data points you want filtered
    filteredAccBNO055[0] = filteredAccBNO055X.updateEstimate( static_cast<float>(acc[0]) );
    filteredAccBNO055[1] = filteredAccBNO055Y.updateEstimate( static_cast<float>(acc[1]) );
    filteredAccBNO055[2] = filteredAccBNO055Z.updateEstimate( static_cast<float>(acc[2]) );

    filteredGyro[0] = filteredGyroX.updateEstimate( static_cast<float>(gyro[0]) );
    filteredGyro[1] = filteredGyroY.updateEstimate( static_cast<float>(gyro[1]) );
    filteredGyro[2] = filteredGyroZ.updateEstimate( static_cast<float>(gyro[2]) );


}

bno055_calib_stat_t calibrate(){
    sensor.setOperationMode(NDOF);
    for (int i=0;i<300;i++) {
        bno055_burst_t data = sensor.getAllData();
        delay(10);
    }
    uint32_t timer = millis();
    int timeout = 10000; // milliseconds

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
    double heading = ori[0] * degToRad; // in radians
    double pitch = ori[1] * degToRad; // in radians
    double roll = ori[2] * degToRad; // in radians

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

Vector<double> find_acc_biases() {
    uint32_t start = 0;
    uint32_t end = 0;
    /// Should be done on a calibrated sensor
    bool biases_calibrated = false;
    int counter = 0;

    const int num = 512;
    double acc_biases_x[num] = {0};
    double acc_biases_y[num] = {0};
    double acc_biases_z[num] = {0};

    sensor.setOperationMode(CONFIGMODE);  // set sensor to config mode to reset biases
    for (auto x : {ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB, ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB, ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB}) {
        sensor.writeRegister(x, 0x00);
    }
    //sensor.setOperationMode(NDOF); // NDOF mode getting orientation

    double avg_x = 0, avg_y = 0, avg_z = 0;
    getInitialOrientation();

    ori[0] = 360-ori[0];   ori[1] = -ori[1];

    sensor.setOperationMode(AMG); // Setting to AMG mode for getting acceleration, because I fear acceleration values are different in fusion mode???
    // ACC calibration
    while (!biases_calibrated) {
        start = micros();
        bno055_burst_t data = sensor.getAllData();
        Vector<double> acc_bias= data.accel;
        Vector<double> magRaw = data.mag;
        Vector<double> gyroRaw = data.gyro; // getting all the data to simulate the exact same conditions as the main loop.


        Vector<double> tav = {0,0,0};
        calcRotationVect(acc_bias, ori, tav);

        acc_biases_x[counter % num] = tav[0];
        acc_biases_y[counter % num] = tav[1];
        acc_biases_z[counter % num] = tav[2];
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
                avg_z -= 9.81; // for if you are using acc data
                biases_calibrated = true;
            }
        }
        counter++;

        if (Serial) {
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", tav[i]); }
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", ori[i]); }
            Serial.printf("%lf, %lf, %lf       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter > num), within_range_x, within_range_y, within_range_z);
        }
        end = micros();
        delayMicroseconds( ((end-start) < cycleTimeus ) ? (cycleTimeus- (end - start) ) : 0 );


    }
    Vector <double> biases = {avg_x, avg_y, avg_z};
    return biases;
}

Vector<double> find_gyr_biases() {
    /// Should be done on a calibrated sensor
    bool biases_calibrated = false;
    int counter = 0;

    const int num = 512;
    double gyr_biases_x[num] = {0};
    double gyr_biases_y[num] = {0};
    double gyr_biases_z[num] = {0};

    sensor.setOperationMode(CONFIGMODE);  // set sensor to config mode to reset biases
    for (auto x : {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB, GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB, GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB}) {
        sensor.writeRegister(x, 0);
    }
    sensor.setOperationMode(AMG); // AMG mode for reading

    double avg_x = 0, avg_y = 0, avg_z = 0;

    // GYR calibration
    while (!biases_calibrated) {
        uint32_t start = micros();

        bno055_burst_t data = sensor.getAllData(); // so that the conditions are the exact same.
        Vector<double> BNO055accRaw = data.accel;
        Vector<double> magRaw = data.mag;
        Vector<double> gyrVect = data.gyro;
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
                biases_calibrated = true;
            }
        }
        counter++;

        if (Serial) {
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", filteredGyro[i]); }

            Serial.printf("%lf, %lf, %lf       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter > num), within_range_x, within_range_y, within_range_z);
        }
        uint32_t end = micros();
        delayMicroseconds( ((end-start) < cycleTimeus ) ? (cycleTimeus- (end - start) ) : 0 );

    }
    Vector <double> biases = {avg_x, avg_y, avg_z};
    return biases;
}

void remove_biases(Vector<double> acc_biases, Vector<double> gyrOffsets){

    signed short addresses[3][9] = {{ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB,
                                     ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB,
                                     ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB,
                                     static_cast<short>(acc_biases[0]*100), static_cast<short>(acc_biases[1]*100), static_cast<short>(acc_biases[2]*100) },
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

void getInitialOrientation() {
    sensor.setOperationMode(NDOF);
    for (int i=0; i<40;i++) {
        ori = sensor.getEuler(); // seems to need a few tries before it works properly ???
        delay(10);
    }
    ori = sensor.getEuler(); // gives data in degrees
    Serial.printf("INITIAL ORIENTATION: %lf, %lf, %lf\n", ori[0], ori[1], ori[2]);
    delay(2000);

}

void deadReckoning(Vector<double> acc, Vector<double> omega, int updateTimeUs, double prevValues[numDR][9]) {
    /// omega (angular velocity) is pulled directly from a filtered gyro estimate.\n
    /// acc is the acceleration pulled straight from the accelerometers, conversion is done internally.

    for (int i=0;i<3;i++) {
        /// Integration of angular velocity, to get orientationFirst order hold (trapezium rule)\n
        /// If the filtered gyro value is within the range of +- 0.5 dps (due to noise) it is deemed insignificant
        (abs(omega[2-i]) > 0.5) ? ori[i] += (updateTimeUs * 0.000001) * 0.5 * (omega[2 - i] + prevValues[DRcounter % numDR][8 - i]) : ori[i] += 0;
        prevValues[DRcounter % numDR][8-i] = omega[2-i]; // setting previous value of omega for next time.
    }
    trueAccVect = {0,0,0};
    calcRotationVect(acc, ori, trueAccVect);
    trueAccVect[2] -= 9.81;
    for(int i=0;i<3;i++) {
        /// Velocity calculation, First order hold.\n
        /// If the true acceleration value is within the range of +- 0.1 (due to noise) it is deemed insignificant
        if (abs(trueAccVect[i]) > 0.1) {
            vel[i] += (updateTimeUs*0.000001)*0.5*(trueAccVect[i] + prevValues[DRcounter % numDR][i]);
        }
        prevValues[DRcounter % numDR][i] = trueAccVect[i]; // setting previous value of acceleration for next time
        /// Position calculation, First order hold
        /// If the value is within the range of +- 0.1 (due to noise) it is deemed insignificant
        if ( abs(prevValues[DRcounter % numDR][3+i]) != abs(vel[i]) ) {
            pos[i] +=  (updateTimeUs*0.000001)*0.5*(vel[i] + prevValues[DRcounter % numDR][3+i]);
        }
        prevValues[DRcounter % numDR][3+i] = static_cast<double>(vel[i]); // setting previous value of velocity for next time.
    }

    /// Live Accelerometer and gyroscope bias estimation\n

    if (DRcounter < numDR) {
        for (int j=0;j<3;j++) {
            omegaAverage[j] = (omegaAverage[j]*DRcounter + prevValues[DRcounter][8-j]) / (DRcounter+1);
            accAverage[j] = (accAverage[j]*DRcounter + prevValues[DRcounter][j]) / (DRcounter + 1);
        }
    }
    else {
        for (int j=0;j<3;j++) {
            omegaAverage[j] = (omegaAverage[j]*numDR + prevValues[DRcounter % numDR][8-j]) / (numDR+1);
            accAverage[j] = (accAverage[j]*DRcounter + prevValues[DRcounter][j]) / (DRcounter + 1);
        }
        bool withinRangeOmega[3] = {true, true, true};
        bool withinRangeAcc[3] = {true, true, true};

        for (int i=0;i<numDR;i++) { // checking if the average values are within a specified range - i.e, the data is all noise, not actual acceleration.
            for (int j = 0; j < 3; j++) {
                if (abs(prevValues[i][8 - j] - omegaAverage[j]) > biasAverageOmegaThreshold) { withinRangeOmega[j] = false; }
                if (abs(prevValues[i][8 - j] - accAverage[i]) > biasAverageAccThreshold) { withinRangeAcc[j] = true; }
            }
        }
        if (withinRangeOmega[0] && withinRangeOmega[1] && withinRangeOmega[2]) {
            for (int i=0;i<3;i++) {
                gyr_biases[i] += omegaAverage[i];
            }
        }
        if (withinRangeAcc[0] && withinRangeAcc[1] && withinRangeAcc[2]) {
            for (int i=0;i<3;i++) {
                acc_biases[i] += accAverage[i];
            }
        }

    }
    DRcounter++;
}




void BNO055Setup() {
    sensor.begin();
    sensor.setPowerMode(NORMAL);
    sensor.setOperationMode(CONFIGMODE); // registers must be configured in config mode


    sensor.writeRegister(BNO055_UNIT_SEL, 0b00000000); // Celsius, degrees, dps, m/s^2
    sensor.setGyroscopeOperationMode(0b00000000); // Normal mode : 0bxxxxx000
    sensor.setMagnetometerConfig(0b00011111);
    switch (cycleTimeus) {
        /// 100Hz
        case 10000:
            sensor.setAccelerometerConfig(0b00010011); //Normal mode, 125Hz, 16G
            sensor.setGyroscopeConfig(0b00010010); // 116 Hz, 500dps
            break;
        /// 200Hz
        case 5000:
            sensor.setAccelerometerConfig(0b00010111); //Normal mode, 250Hz, 16G
            sensor.setGyroscopeConfig(0b00001010); // 230 Hz, 500dps
            break;
        /// 400Hz
        case 2500:
            sensor.setAccelerometerConfig(0b00011011); //Normal mode, 500Hz, 16G
            sensor.setGyroscopeConfig(0b00000010); // 523 Hz, 500dps
            break;
        default: /// Defaults to 400Hz config
            sensor.setAccelerometerConfig(0b00011011); //Normal mode, 500Hz, 16G
            sensor.setGyroscopeConfig(0b00000010); // 523 Hz, 500dps
            break;
    }
    sensor.setAccelerometerConfig(0b00010011); //Normal mode, 125Hz, 16G
    sensor.setGyroscopeConfig(0b00010010); // 116 Hz, 500dps



    //sensor.writeRegister(BNO055_AXIS_MAP_CONFIG, 0x24); // TODO: Check that this will be correct for our pcb (Axis remap);
    sensor.writeRegister(BNO055_AXIS_MAP_SIGN, 0b00000000);

    calibrate();
    //acc_biases = find_acc_biases();
    //gyr_biases = find_gyr_biases();

    //getInitialOrientation();

    sensor.setOperationMode(AMG);
}


void setup() {
    Wire.setClock(1000000);  // i2c seems to work great at 1Mhz
    Wire.begin();

    BNO055Setup();

}




void loop() {
    uint32_t startOfLoop = micros();

    /// Data Aquisition
    bno055_burst_t data = sensor.getAllData();
    Vector<double> BNO055accRaw = data.accel;
    Vector<double> magRaw = data.mag;
    Vector<double> gyroRaw = data.gyro;

    /// Removing biases for internal integration of gyroscopes and accelerometers. \n Results in higher precision of calculation than setting bias/offset registers
    for(int i=0;i<3;i++) { BNO055accRaw[i] -= acc_biases[i]; gyroRaw[i] -= gyr_biases[i]; }

    //eul[0] = 360-eul[0];   eul[1] = -eul[1];  // necessary to have all the angles going anticlockise-> increasing. - for the reference frame conversion in fusion mode

/// Kalman filtering
    updateFilters(gyroRaw, BNO055accRaw);


    /// Calculations
    deadReckoning(filteredAccBNO055, filteredGyro, 5000, prevVect);

//    Serial.printf("TAV: %lf, %lf, %lf  |  VEL: %lf, %lf, %lf  |  POS: %lf, %lf, %lf  |  ORI: %lf, %lf, %lf\n",
//                  trueAccVect[0], trueAccVect[1], trueAccVect[2],
//                  vel[0], vel[1], vel[2],
//                  pos[0], pos[1], pos[2],
//                  ori[0], ori[1], ori[2]);
    Serial.printf("GYRO: %lf, %lf, %lf     ", filteredGyro[0], filteredGyro[1], filteredGyro[2]);
    Serial.printf("ACC : %lf, %lf, %lf     ", filteredAccBNO055[0], filteredAccBNO055[1], filteredAccBNO055[2]);
    Serial.printf("POS: %lf, %lf, %lf\n", pos[0], pos[1], pos[2]);



    uint32_t endOfLoop = micros();
    //Serial.println(endOfLoop - startOfLoop);
    delayMicroseconds( ((endOfLoop-startOfLoop) < cycleTimeus ) ? (cycleTimeus- (endOfLoop - startOfLoop) ) : 0 );
}