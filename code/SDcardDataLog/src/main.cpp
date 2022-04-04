// code to test SD card capabilities of the Teensy 4.1
// BUILTIN_SDCARD (254 (serial)) is the pin for the built in SD card
// Samuel Scott 12/01/2022

#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"


#define LOG_FILE_SIZE 10*25000*600 // TODO: What size file do we need????

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
    byte arr[464];
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

    if (!file.open("dataLog19.bin", O_CREAT | O_WRITE)) {
        Serial.println("Could not open file!");
    }
    // File must be pre-allocated to avoid huge
    // delays searching for free clusters.
    if (!file.preAllocate(LOG_FILE_SIZE)) { // if i dont preallocate, there;s an extra 3-5ms delay in writing
        Serial.println("preAllocate failed\n");
        file.close();
        return;
    }
    Serial.println("Card initialised");
}


void logData(byte arr[512]) {  /// SDFat has a 512 byte buffer, so we only need to flush after 512 bytes have been written to the buffer
    /*if (!file) {
        Serial.println("Error: write halted");
    }
    for(int i=0;i<512;i++) { file.printf("%d,",arr[i]); }
    file.printf("\n");*/
    if (!file.write(arr, 512)) {
        Serial.print("Error: could not write buffer");
    }
}

void logData(struct dataStruct data) {  /// SDFat has a 512 byte buffer, so we only need to flush after 512 bytes have been written to the buffer
    if (!file.printf("%d,%d,%d,", data.acc_rawx, data.acc_rawy, data.acc_rawz)) {

        Serial.println("Error: write halted");
    }
    for (int i=0;i<464;i++) { file.printf("%d,", data.arr[i]); } // TODO: How do we write a struct to a page of flash like joeyB said???? - That would be mega fast.
    file.printf("\n");
}

void loop() {

    //byte byteArr[512] = {1, 2, 3, 5, 6, 7, 8, 9, 255, 13};
    //for (int a=0;a<512;a++){ byteArr[a] = a;}
    struct dataStruct mydata = {1,2,32, {3, 32, 123}};
    while(1){
        unsigned long a = micros(); // IS SERIAL WRITE SLOWING IT DOWN????
        logData(mydata); // 512 bytes
        unsigned long b = micros();
        Serial.println(b - a);

        unsigned long beforeFlush = micros();
        file.flush();
        unsigned long afterFlush = micros();
        Serial.printf("To flush 512 bytes it takes: %d\n", afterFlush - beforeFlush);
        Serial.printf("Total time %d\n", afterFlush - a);
        //delay(1);
    }
    //file.close();


}