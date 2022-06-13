//
// Created by robosam2003 on 03/05/2022.
//

#include "ADXL377.h"


float ADXLscale = 200*9.8; //  (±200g) for ADXL377
int a = 1;
SimpleKalmanFilter filteredAccADXL377_X = SimpleKalmanFilter(a, a, 0.01);
SimpleKalmanFilter filteredAccADXL377_Y = SimpleKalmanFilter(a, a, 0.01);
SimpleKalmanFilter filteredAccADXL377_Z = SimpleKalmanFilter(a, a, 0.01);
Vector<double> filteredADXL;

Vector<double> ADXL377_biases = {0,0,0};



double mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return static_cast<double>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


void ADXL377Setup () {
    pinMode(ADXL377_XPin, INPUT);
    pinMode(ADXL377_YPin, INPUT);
    pinMode(ADXL377_ZPin, INPUT);

    Serial.println("Finding ADXL377 biases...");
    delay(500);
    ADXL377_biases = find_ADXL_biases();
    Serial.println("ADXL377 biases found.");
    enterToContinue();
}


Vector<double> getADXL377Acc () {
    Vector<double> acc;
    // Get raw accelerometer data for each axis
    int rawX = analogRead(ADXL377_XPin);
    int rawY = analogRead(ADXL377_YPin);
    int rawZ = analogRead(ADXL377_ZPin);

    acc[0] = -mapf(rawY, 0, 1023, -ADXLscale, ADXLscale) - ADXL377_biases[0]; // Between 0 and 1023 on 3.3V processor // TODO add offsets
    acc[1] = mapf(rawX, 0, 1023, -ADXLscale, ADXLscale) - ADXL377_biases[1]; // x -> y,    -y -> x,   z -> z  - this is done because the axis are in a different orientation to the BNO055
    acc[2] = mapf(rawZ, 0, 1023, -ADXLscale, ADXLscale) - ADXL377_biases[2]; 
    return acc;
}

void updateADXL377Filters(Vector<double> ADXLacc) {
    filteredADXL[0] = filteredAccADXL377_X.updateEstimate(static_cast<float>(ADXLacc[0]));
    filteredADXL[1] = filteredAccADXL377_Y.updateEstimate(static_cast<float>(ADXLacc[1]));
    filteredADXL[2] = filteredAccADXL377_Z.updateEstimate(static_cast<float>(ADXLacc[2]));
}

Vector<double> find_ADXL_biases() {
    uint32_t start = 0;
    uint32_t end = 0;
    /// Should be done on a calibrated sensor
    bool biases_calibrated = false;
    int counter = 0;

    const int num = 512;
    double acc_biases_x[num] = {0};
    double acc_biases_y[num] = {0};
    double acc_biases_z[num] = {0};

    double avg_x = 0, avg_y = 0, avg_z = 0;
    for (int i=0;i<100;i++) {
        Vector<double> acc = getADXL377Acc();
        delay(10); // get the sensor to settle
    } 
    // ACC calibration
    while (!biases_calibrated) {
        start = micros();
        Vector<double> acc_bias = getADXL377Acc();
        updateADXL377Filters(acc_bias);


        // Vector<double> tav = {0,0,0};
        // calcRotationVect(acc_bias, ori, tav); // we are now just assuming it is straight up, and in the rig i made

        acc_biases_x[counter % num] = filteredADXL[0];
        acc_biases_y[counter % num] = filteredADXL[1];
        acc_biases_z[counter % num] = filteredADXL[2];
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

            double thres = 1;
            for (auto a: acc_biases_x) { if (abs((a - avg_x) > thres)) { within_range_x = false; }}
            for (auto b: acc_biases_y) { if (abs((b - avg_y) > thres)) { within_range_y = false; }}
            for (auto c: acc_biases_z) { if (abs((c - avg_z) > thres)) { within_range_z = false; }}
            if ((within_range_x & within_range_y) & within_range_z) {
                /*double lambda = 9.80 / sqrt(sq(avg_x) + sq(avg_y) + sq(avg_z));
                avg_x *= lambda;
                avg_y *= lambda; // scaling to make the total equal to 9.81
                avg_z *= lambda;*/ /// not sure if this but works or is even necessary tbh.

                Serial.println("calculated Offsets");
                avg_x -= 9.81; // THIS SHOULD BE DONE STRAIGHT UP GROUND in the rig i made
                biases_calibrated = true;
                Serial.printf("AVGX: %lf, AVGY: %lf, AVGZ: %lf\n", avg_x, avg_y, avg_z);
            }
        }
        counter++;

        if (Serial) {
            for (int i=0;i<3;i++) { Serial.printf("%lf,     ", filteredADXL[i]); }
            Serial.printf("%lf, %lf, %lf       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter > num), within_range_x, within_range_y, within_range_z);
        }
        end = micros();
        delayMicroseconds( ((end-start) < cycleTimeus ) ? (cycleTimeus- (end - start) ) : 0 );


    }
    Vector <double> biases = {avg_x, avg_y, avg_z};
    return biases;
}