#include <Arduino.h>
#include <BNO055.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>


// ADXL377 pins
#define ADXL377_XPin 21
#define ADXL377_YPin 22
#define ADXL377_ZPin 23

SimpleKalmanFilter FilteredAccADXL377_X = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter FilteredAccADXL377_Y = SimpleKalmanFilter(0.05, 0.05, 0.01);
SimpleKalmanFilter FilteredAccADXL377_Z = SimpleKalmanFilter(0.05, 0.05, 0.01);
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377

void setup()
{
    // Initialize serial communication at 115200 baud
    Serial.begin(115200);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    pinMode(ADXL377_XPin, INPUT);
    pinMode(ADXL377_YPin, INPUT);
    pinMode(ADXL377_ZPin, INPUT);
    //analogReadRes(12);
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Read, scale, and print accelerometer data
void loop()
{
    // Get raw accelerometer data for each axis
    int rawX = analogRead(ADXL377_XPin);
    int rawY = analogRead(ADXL377_YPin);
    int rawZ = analogRead(ADXL377_ZPin);


    // Scale accelerometer ADC readings into common units
    // Scale map depends on if using a 5V or 3.3V microcontroller
    float scaledX, scaledY, scaledZ; // Scaled values for each axis

    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);

    // Print out raw X,Y,Z accelerometer readings
    //Serial.print("X: "); Serial.println(rawX);
    //Serial.print("Y: "); Serial.println(rawY);
    //Serial.print("Z: "); Serial.println(rawZ);
    //Serial.println();

    // Print out scaled X,Y,Z accelerometer readings
    Serial.print("X: "); Serial.print(scaledX); Serial.println(" g");
    Serial.print("Y: "); Serial.print(scaledY); Serial.println(" g");
    Serial.print("Z: "); Serial.print(scaledZ); Serial.println(" g");
    Serial.println();

    delay(100); // Minimum delay of 2 milliseconds between sensor reads (500 Hz)
}

// Same functionality as Arduino's standard map function, except using floats

