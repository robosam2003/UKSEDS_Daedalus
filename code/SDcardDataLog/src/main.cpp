// code to test SD card capabilities of the Teensy 4.1
// BUILTIN_SDCARD (254 (serial)) is the pin for the built in SD card
// Samuel Scott 12/01/2022

#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"


SdFat32 sd;
File32 file;
const int chipSelect = 254;
elapsedMicros timer1;

const size_t BUF_DIM = 512;
uint8_t buf[BUF_DIM];

struct dataStruct {
    uint16_t acc_rawx;
    uint16_t acc_rawy;
    uint16_t acc_rawz;
};


void setup() {
// write your initialization code here
    delay(3000);
    Serial.begin(9600);
    while (!Serial);
    // SD card initialisation
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("Could not mount SD card");
        while (true);
    }
    Serial.println("Card initialised");
}


void logData(byte arr[512]) {  /// SDFat has a 512 byte buffer, so we only need to flush after 512 bytes have been written to the buffer
    if (!file.write((const uint8_t*)&arr, 512)) {
        Serial.println("Error: write halted");
    }
}

void logData(struct dataStruct data) {  /// SDFat has a 512 byte buffer, so we only need to flush after 512 bytes have been written to the buffer
    if (!file.write((const uint8_t*)&data, sizeof(data))) {
        Serial.println("Error: write halted");
    }
}

void loop() {
    if (!file.open("dataLog1.txt", O_CREAT | O_WRITE)) {
        Serial.println("Could not open file!");
    }
    byte byteArr[512];
    for (int a=0;a<512;a++){ byteArr[a] = a;}
    for(int j=0;j<10;j++) {
        unsigned long a = micros();
        logData(byteArr); // 512 bytes
        unsigned long b = micros();
        Serial.println(b - a);

        unsigned long beforeFlush = micros();
        file.flush();
        unsigned long afterFlush = micros();
        Serial.printf("To flush 512 bytes it takes: %d\n", afterFlush - beforeFlush);
        Serial.printf("Total time %d\n", afterFlush - a);
    }
    file.close();
    delay(3000);




}