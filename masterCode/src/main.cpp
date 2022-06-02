#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\ADXL377\src\ADXL377.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\BNO055_forked_and_source\src\BNO055daedalus.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\SDcardDataLog\src\SDcardDataLog.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\NEO6mWithAssistNow\src\NEO6mWithAssistNow.h"

// I have absolutely no idea why just I cant just include the .h file JUST for the transmitter, but hey-ho - quick fix, it works
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Transmitter_LoRa\src\RFM96WtransmitLORA.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Transmitter_LoRa\src\RFM96WtransmitLORA.cpp"

#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\BMP280\src\BMP280.h"

/** Global variables defined in other files:
 * - BNO055daedalus.h
 *   - BNO055 sensor(BNO055_I2C_ADDRESS, &Wire) 
 *   - Vector<double> acc_biases
 *   - Vector<double> gyr_biases
 *   - double prevVect[numDR][9]
 *   - Vector<double> omegaAverage
 *   - Vector<double> accAverage
 *   - double biasAverageOmegaThreshold
 *   - double biasAverageAccThreshold
 *   - Vector<double> pos
 *   - Vector<double> vel
 *   - Vector<double> ori
 *   - int DRcounter
 *   - SimpleKalmanFilter filteredAccBNO055X
 *   - SimpleKalmanFilter filteredAccBNO055Y
 *   - SimpleKalmanFilter filteredAccBNO055Z
 *   - SimpleKalmanFilter filteredGyroX
 *   - SimpleKalmanFilter filteredGyroY
 *   - SimpleKalmanFilter filteredGyroZ
 *   - Vector<double> filteredAccBNO055
 *   - Vector<double> filteredGyro
 *   - Vector<double> trueAccVect
 * - ADXL377.h
 *   - Vector<double> filteredAccADXL377_X
 *   - Vector<double> filteredAccADXL377_Y
 *   - Vector<double> filteredAccADXL377_Z
 *   - Vector<double> filteredADXL
 *   - float ADXLscale
 *   -
 *
 *
 *
 *
 * - NEO6mWithAssistNow.h
 *   - GPSDataStruct GPSData
 *   -
 *
 *
 *
 * - SDcardDataLog.h
 *
 *
 */

#define RTC_SYNC_BYTE 0x99
byte statusCode = 0b00000000; // This can be altered to show various stages of the launch.
#define LAUNCH_DETECTION_MASK 0b00000001
#define GPS_TURN_ON_MASK 0b00000010


void enterToContinue(){
    Serial.printf("Press Enter to continue");
    while (!Serial.available());
    Serial.println();
    Serial.clear();
}

void fullSystemTest() {
    // Tests each subsystem for several seconds. 
    // Do this just before launch, Use HTERM for terminal input.
    Serial.printf("---------- FULL SYSTEM TEST ----------\n\nPress enter to continue");

    Serial.printf("Timestamp test...\n");
    delay(500);
    for (int i=0; i<50; i++) {
        uint64_t ts = getTimestampMillis();
        Serial.printf("UNIX TIME: %llu\n", ts);
        delay(100);
    }
    Serial.printf("Timestamp test complete.\n");
    enterToContinue();

    Serial.printf("Testing BNO055 for 15 Seconds... \n");
    delay(500);
    for (int i=0; i<15*100; i++) {
        bno055_burst_t burst = sensor.getAllData();
        Vector<double>rawBNO055Acc = burst.accel;
        Vector<double>rawBNO055Gyro = burst.gyro;
        updateBNOFilters(rawBNO055Gyro, rawBNO055Acc);
        Serial.printf("BNO055 RAW - acc: (x, y, z) %lf, %lf, %lf     gyro : (x, y, z) %lf, %lf, %lf\n",
                      rawBNO055Acc[0], rawBNO055Acc[1], rawBNO055Acc[2],
                      rawBNO055Gyro[0], rawBNO055Gyro[1], rawBNO055Gyro[2]);
        delay(10);
    }
    Serial.printf("BNO055 test complete.\n");
    enterToContinue();

    Serial.printf("Testing ADXL377 for 15 Seconds... \n");
    delay(500);
    for (int i=0; i<15*100; i++) {
        Vector<double> rawADXLacc = getADXL377Acc();
        updateADXL377Filters(rawADXLacc);
        Serial.printf("ADXL377 FILTERED - acc: (x, y, z) %lf, %lf, %lf\n",
                      filteredADXL[0], filteredADXL[1], filteredADXL[2]);
        delay(10);
    }
}

void groundRTCSync() {
    u_int64_t unixTime = getTimestampMillis();
    Serial.printf("UNIX TIME: %llu\n", unixTime);
    byte arr[22];
    arr[0] = RTC_SYNC_BYTE; // sync byte
    for (int i=0; i<8; i++) {
        arr[i+1] = (unixTime >> (7 * 8 - (i * 8))) & 0xFF; // little endian
    }

    transmitData(arr);
    Serial.printf("Successfully transmitted UNIX TIME to ground station.\n");



}

void setup() {
    Serial.begin(115200);
    delay(500);

    enterToContinue();

    Serial.printf("-------------INITIALIZING SENSORS------------\n");

/** Initialise sensors:
     *  - BNO055
     *  - ADXL377
     *  - BMP280
     *  - GPS  */
    BNO055Setup();
    ADXL377Setup();
    BMP280Setup();
    NEO6mSetup();

/** Initialise SD card and transmitter */
    sdSetup();
    RFM96WtransmitSetup();
    Serial.printf("SUCCESS - All systems initialised.\n");

/** Send Teensy unix time to ground station over telemetry to sync. */
    Serial.printf("SYNCING ground station RTC\n");
    groundRTCSync();

    enterToContinue();
    

/** Full system test before launch */
    fullSystemTest();



/** Calibrate sensors (BNO055) and calculate offsets (BNO055)*/


/** Wait for "Launch commit" mesasnge from ground station */


/** Save current GPS coords to SD card */



/** Shut down GPS ready for launch */


/// We now enter the "ready for launch loop": loop()
} 

void loop() {

    
/// Data acquisition - BNO055, ADXL377, BMP280
    // Get timestamp
    uint64_t timestamp = getTimestampMillis();

    Vector<double>rawADXLacc = getADXL377Acc();
    bno055_burst_t burst = sensor.getAllData();
    Vector<double>rawBNO055Acc = burst.accel;
    Vector<double>rawBNO055Gyro = burst.gyro;

    double bmpData[3];
    getBMP280Data(bmpData);

    




/// Filter updates
    updateBNOFilters(rawBNO055Gyro, rawBNO055Acc);
    updateADXL377Filters(rawADXLacc);


/// Launch detection 
    // TODO: test launch detection interrupt. 
    if (!launchInterrupt && (magnitude(filteredAccBNO055) < 2*9.81)) { // TODO: detemine magnitude threshold
        launchInterrupt = true;
    }
/// Calculations


/// Data logging
    SDDataLog.timeStamp = timestamp;
    SDDataLog.BNO055_acc_x = rawBNO055Acc[0];
    SDDataLog.BNO055_acc_y = rawBNO055Acc[1];
    SDDataLog.BNO055_acc_z = rawBNO055Acc[2];
    SDDataLog.BNO055_gyr_x = rawBNO055Gyro[0];
    SDDataLog.BNO055_gyr_y = rawBNO055Gyro[1];
    SDDataLog.BNO055_gyr_z = rawBNO055Gyro[2];
    SDDataLog.ADXL_acc_x = rawADXLacc[0];
    SDDataLog.ADXL_acc_y = rawADXLacc[1];
    SDDataLog.ADXL_acc_z = rawADXLacc[2];
    SDDataLog.BMP280_temp = bmpData[0];
    SDDataLog.BMP280_pres = bmpData[1];
    SDDataLog.BMP280_alt = bmpData[2];

    // TODO: Add GPS data under conditions


    // TODO: Add Dead reckoning data. - include orientation!



/// Data transmission every third cycle


/// Ensuring cycles are 10ms long

}