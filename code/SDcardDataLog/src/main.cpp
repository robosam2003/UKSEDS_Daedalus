// Test Teensy SDIO with write busy in a data logger demo.
//
// The driver writes to the uSDHC controller's FIFO then returns
// while the controller writes the data to the SD.  The first sector
// puts the controller in write mode and takes about 11 usec on a
// Teensy 4.1. About 5 usec is required to write a sector when the
// controller is in write mode.

#include "SdFat.h"
#include "RingBuf.h"

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 40

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE 10*25000*600  // 150,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512
#define LOG_FILENAME "SdioLogger10mychanges.csv"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

struct dataStruct { // This can be of any size, up to 512 bytes.
    uint16_t acc_rawx;
    uint16_t acc_rawy;
    uint16_t acc_rawz;
    byte arr[464];
};

void sdSetup() {
    // Initialize the SD.
    if (!sd.begin(SD_CONFIG)) {
        sd.initErrorHalt(&Serial);
    }
    // Open or create file - truncate existing file.
    if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.println("open failed\n");
        return;
    }
    // File must be pre-allocated to avoid huge
    // delays searching for free clusters.
    if (!file.preAllocate(LOG_FILE_SIZE)) {
        Serial.println("preAllocate failed\n");
        file.close();
        return;
    }
    // initialize the RingBuf.
    rb.begin(&file);
}
void logData(struct dataStruct data) {
    // Max RingBuf used bytes. Useful to understand RingBuf overrun.
    size_t maxUsed = 0;

    // Min spare micros in loop.
    int32_t minSpareMicros = INT32_MAX;

    // Start time.
    uint32_t logTime = micros();
    // Log data until Serial input or file full.

    // Amount of data in ringBuf.
    size_t n = rb.bytesUsed();
    if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
        Serial.println("File full - quitting.");
        file.close();
        while(1);
    }
    if (n > maxUsed) {
        maxUsed = n;
    }
    if (n >= 512 && !file.isBusy()) {
        // Not busy only allows one sector before possible busy wait.
        // Write one sector from RingBuf to file.

        if (512 != rb.writeOut(512)) {
            Serial.println("writeOut failed");
            while(1);
        }

    }


    rb.printf("%d,%d,%d,",data.acc_rawx, data.acc_rawy, data.acc_rawz);
    for (int i=0;i<100;i++) { rb.printf("%d,",data.arr[i]); }
    rb.println();
    // Print adc into RingBuf.
    if (rb.getWriteError()) {
        // Error caused by too few free bytes in RingBuf.
        Serial.println("WriteError");
        while(1);
    }


    // Write any RingBuf data to file.

    /*rb.sync();


    file.truncate();
    file.rewind();
    // Print first twenty lines of file.
    Serial.println("spareMicros,ADC0");
    for (uint8_t n = 0; n < 20 && file.available();) {
        int c = file.read();
        if (c < 0) {
            break;
        }
        Serial.write(c);
        if (c == '\n') n++;
    }
    Serial.print("fileSize: ");
    Serial.println((uint32_t)file.fileSize());
    Serial.print("maxBytesUsed: ");
    Serial.println(maxUsed);
    Serial.print("minSpareMicros: ");
    Serial.println(minSpareMicros);
    file.close();*/
}

void setup() {
    Serial.begin(9600);
    sdSetup();


}

void loop() {
    struct dataStruct mydata = {0,1,2, {1,2,3,4,66,77,88,112,123,255}};
    uint32_t beforeLogData = micros();
    logData(mydata);
    uint32_t afterLogData = micros();
    Serial.println(afterLogData-beforeLogData);
    delay(10);
}




// my old code
/*
// code to test SD card capabilities of the Teensy 4.1
// BUILTIN_SDCARD (254 (serial)) is the pin for the built in SD card
// Samuel Scott 12/01/2022

#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"

#define FLIGHT_TIME_MINS 5
#define LOG_FILE_SIZE 128*100*60*FLIGHT_TIME_MINS // TODO: What size file do we need????

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

    if (!file.open("dataLog35pos.csv", O_CREAT | O_WRITE)) {
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
//    if (!file) {
//        Serial.println("Error: write halted");
//    }
//    for(int i=0;i<512;i++) { file.printf("%d,",arr[i]); }
//    file.printf("\n");
    if (!file.write(arr, 512)) {
        Serial.print("Error: could not write buffer");
    }
}

void logData(struct dataStruct data) {  /// SDFat has a 512 byte buffer, so we only need to flush after 512 bytes have been written to the buffer
    if (!file.printf("%d,%d,%d,", data.acc_rawx, data.acc_rawy, data.acc_rawz)) {

        Serial.println("Error: write halted");
    }
    for (int i=0;i<464;i++) { file.printf("%d,", data.arr[i]); }
    file.printf("\n");
}

void loop() {
    int pos = 0;
    struct dataStruct mydata = {1,2,32, {1,2,3,5,68,43, 24, 42}};
    while(1){
        unsigned long a = micros();
        logData(mydata); // 512 bytes
        unsigned long b = micros();
        //Serial.println(b - a);

        unsigned long beforeFlush = micros();
        file.sync();
        unsigned long afterFlush = micros();
        Serial.printf("To flush 512 bytes it takes: %d\n", afterFlush - beforeFlush);
        Serial.println(afterFlush - a);
        delay(20);
    }
    file.truncate(0);
    //file.close();


}*/
