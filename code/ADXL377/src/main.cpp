#include <Arduino.h>
#include <BNO055.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>


#define ADXL377_XPin 21
#define ADXL377_YPin 22
#define ADXL377_ZPin 23

SimpleKalmanFilter filteredAccADXL377_X = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccADXL377_Y = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter filteredAccADXL377_Z = SimpleKalmanFilter(0.05, 0.05, 0.01);
Vector<double> filteredADXL;

float ADXLscale = 200*9.8; //  (±200g) for ADXL377

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ADXL377Setup () {
    pinMode(ADXL377_XPin, INPUT);
    pinMode(ADXL377_YPin, INPUT);
    pinMode(ADXL377_ZPin, INPUT);
}

void getADXL377Acc (Vector<double> returnVect) {
    // Get raw accelerometer data for each axis
    int rawX = analogRead(ADXL377_XPin);
    int rawY = analogRead(ADXL377_YPin);
    int rawZ = analogRead(ADXL377_ZPin);

    returnVect[0] = mapf(rawX, 0, 1023, -ADXLscale, ADXLscale) - 9.579712; // Between 0 and 1023 on 3.3V processor
    returnVect[1] = mapf(rawY, 0, 1023, -ADXLscale, ADXLscale) - 5.747803;
    returnVect[2] = mapf(rawZ, 0, 1023, -ADXLscale, ADXLscale)- 21.075317 + 9.81;

}

void setup() {
    // Initialize serial communication at 115200 baud
    Serial.begin(115200);
    ADXL377Setup();

}

// Read, scale, and print accelerometer data
void loop() {
    // TODO: filterting of ADXL
    Vector<double> ADXLacc = {};
    getADXL377Acc(ADXLacc);
    delayMicroseconds(100);


}


