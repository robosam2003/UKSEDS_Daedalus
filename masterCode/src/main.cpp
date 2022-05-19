#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\ADXL377\src\ADXL377.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\BNO055_forked_and_source\include\BNO055daedalus.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\SDcardDataLog\src\SDcardDataLog.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\NEO6mWithAssistNow\src\NEO6mWithAssistNow.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Transmitter_LoRa\src\RFM96WtransmitLORA.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\BMP280\src\BMP280.h"


/*#include "Wire.h"
#include "SPI.h"
#include "SdFat.h"
#include "SD.h"
#include "TimeLib.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_I2CDevice.h"*/


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

void fullSystemTest() {
    // Tests each subsystem for several seconds.
    Serial.printf("Starting full system test...\n\n");

    Serial.printf("Timestamp test...\n");
    for (int i=0; i<100; i++) {
        uint64_t ts = getTimestampMillis();
        Serial.printf("UNIX TIME: %llu\n", ts);
        delay(100);
    }
    // press enter to continue
    while (!Serial.available());
    Serial.printf("YOU PRESSED ENTER\n\n");

    Serial.printf("Testing BNO055 for 15 Seconds... \n");
    for (int i=0; i<15*100; i++) {
        bno055_burst_t burst = sensor.getAllData();
        Serial.printf("BNO055 acc x: %lf\n", burst.accel.getX());
        delay(10);
    }
}


void setup() {
    delay(500);
    Serial.begin(115200);
/** Initialise sensors:
     *  - BNO055
     *  - ADXL377
     *  - BMP280
     *  - GPS  */
    BNO055Setup();

    //ADXL377Setup();

    //BMP280Setup();

    //NEO6mSetup();

    // * Initialise SD card and transmitter
    //sdSetup();

    //RFM96WtransmitSetup();

    Serial.printf("SUCCESS - All systems initialised.\n");
    /** Full system test before launch */
    fullSystemTest();
}

void loop() {
/*/// Data acquisition - BNO055, ADXL377, BMP280
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


    // TODO: Add Dead reckoning data.



/// Data transmission every third cycle


/// Ensuring cycles are 10ms long*/

}