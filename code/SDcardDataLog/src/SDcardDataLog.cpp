//
// Created by robos on 03/05/2022.
//

#include "SDcardDataLog.h"


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
    if (!file.preAllocate(LOG_FILE_SIZE)) { // if i dont preallocate, it takes much longer.
        Serial.println("preAllocate failed\n");
        file.close();
        return;
    }
    // initialize the RingBuf.
    rb.begin(&file);
}

void logData(byte arr[256]) { // can log up to 512 bytes
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


    rb.printf("%d,%d,%d,", arr[0], arr[1], arr[2]);
    for (int i=0;i<100;i++) { rb.printf("%d,", arr[i]); }
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