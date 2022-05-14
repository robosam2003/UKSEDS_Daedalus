//
// Created by robos on 03/05/2022.
//

#include "SDcardDataLog.h"

SdFs sd;
FsFile file;
RingBuf<FsFile, RING_BUF_CAPACITY> rb;
SDDataLogStruct SDDataLog;


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
        file.close();
        while(1) {
            Serial.println("PREALLOCATE FAILED");
            delay(10);
        }
    }
    // initialize the RingBuf.
    rb.begin(&file);

}

void logData() { // can log up to 512 bytes
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
    // 31 data points per line. - all doubles.
    rb.printf("%llu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
              SDDataLog.timeStamp,
              SDDataLog.BNO055_acc_x, SDDataLog.BNO055_acc_y, SDDataLog.BNO055_acc_z,
              SDDataLog.BNO055_gyr_x, SDDataLog.BNO055_gyr_y, SDDataLog.BNO055_gyr_z,
              SDDataLog.BNO055_acc_x_filt, SDDataLog.BNO055_acc_y_filt, SDDataLog.BNO055_acc_z_filt,
              SDDataLog.BNO055_gyr_x_filt, SDDataLog.BNO055_gyr_y_filt, SDDataLog.BNO055_gyr_z_filt,
              SDDataLog.ADXL_acc_x, SDDataLog.ADXL_acc_y, SDDataLog.ADXL_acc_z,
              SDDataLog.BMP280_temp, SDDataLog.BMP280_pres, SDDataLog.BMP280_alt,
              SDDataLog.GPS_lat, SDDataLog.GPS_lon, SDDataLog.GPS_alt, SDDataLog.GPS_tow, SDDataLog.GPS_hacc, SDDataLog.GPS_vacc,
              SDDataLog.DR_pos_x, SDDataLog.DR_pos_y, SDDataLog.DR_pos_z, SDDataLog.DR_vel_x, SDDataLog.DR_vel_y, SDDataLog.DR_vel_z);
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