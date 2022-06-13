#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\ADXL377\src\ADXL377.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\BNO055_forked_and_source\src\BNO055daedalus.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\SDcardDataLog\src\SDcardDataLog.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\NEO6mWithAssistNow\src\NEO6mWithAssistNow.h"

// I have absolutely no idea why just I cant just include the .h file JUST for the transmitter, but hey-ho - quick fix, it works
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Transmitter_LoRa\src\RFM96WtransmitLORA.h"
#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Transmitter_LoRa\src\RFM96WtransmitLORA.cpp"
//#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Reciever_LoRa\src\RFM96WrecieveLORA.h"
//#include "C:\Users\robos\CLionProjects\UKSEDS_Daedalus\code\RFM96W_Reciever_LoRa\src\RFM96WrecieveLORA.cpp"

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
byte statusCode = 0b00000000; // This can be altered to show various stages of the launch FOR SD CARD

double SEA_LEVEL_HPA = 1010; // CHANGE TO LOCAL FORECAST

// Transmittsion/reception codes - FOR TRANSMISSION
#define RTC_SYNC_BYTE 0x99
#define TEST_CODE 0xAA
#define LAUNCH_COMMIT_CODE 0xBB
#define LAUNCH_COMMIT_CONFIRM 0xCC
#define DATA_CODE 0xDD
unsigned long long cycleCounter = 0;
bool launched = false;

double launchThreshold = 2*9.81;

#define LAUNCH_DETECT_MASK 0b00000001   

#define buzzerPin 24

void fullSystemTest() {
    // Tests each subsystem for several seconds. 
    // Do this just before launch, Use HTERM for terminal input.
    Serial.printf("---------- FULL SYSTEM TEST ----------\n\n");

    Serial.printf("Timestamp test...\n");
    delay(500);
    for (int i=0; i<10; i++) {
        uint64_t ts = getTimestampMillis();
        Serial.printf("UNIX TIME: %llu\n", ts);
        delay(100);
    }
    Serial.printf("Timestamp test complete.\n");
    enterToContinue();

    Serial.printf("Testing BNO055 for 10 Seconds... \n");
    delay(500);
    for (int i=0; i<10*100; i++) {
        bno055_burst_t burst = sensor.getAllData();
        Vector<double>rawBNO055Acc = burst.accel;
        Vector<double>rawBNO055Gyro = burst.gyro;
        for(int i=0;i<3;i++) { rawBNO055Acc[i] -= acc_biases[i];  rawBNO055Gyro[i] -= gyr_biases[i]; }

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
        uint32_t start = micros();
        Vector<double> rawADXLacc = getADXL377Acc();
        updateADXL377Filters(rawADXLacc);
        Serial.printf("ADXL377 filtered - acc: (x, y, z) %lf, %lf, %lf\n",
                      filteredADXL[0], filteredADXL[1], filteredADXL[2]);
        uint32_t end = micros();
        delayMicroseconds( ((end-start) < cycleTimeus ) ? (cycleTimeus- (end - start) ) : 0 );
    }
    
    Serial.printf("ADXL377 test complete.\n");
    enterToContinue();

    Serial.printf("Testing RFM96W transmission for 15 Seconds... \n");
    //delay(500);
    for (int i=0; i<200; i++) {
        byte data[23] = {};
        data[0] = 0xAA;
        while(!transmittedFlag);
        transmitData(data);
        Serial.printf("Successfully transmitted data.\n");
        delay(30);
    }

    Serial.printf("Testing NEO6m...\n");
    enterToContinue();
    Serial.printf("Testing NEO6m. Press enter to exit loop (when locked)\n");
    delay(500);
    Serial.clear();
    while(1) {
        if (Serial.available()){
            Serial.printf("Entering main loop.\n");
            break;
        }
        sendUbx(NAV, NAV_POSLLH, 0, nullptr);
        delay(200);
        getGPSData();
        Serial.printf("\nTOW:        %d\n", GPSdata.towMs);
        Serial.printf("Actual TOW: %d\n", actualTimeOfWeekms());
        Serial.printf("Time %d:%d:%d\n", hour(), minute(), second());
        Serial.printf("LONG: %lf\n", GPSdata.lon);
        Serial.printf("LAT: %lf\n", GPSdata.lat);
        Serial.printf("HMSL(m): %lf\n", GPSdata.alt);
        Serial.printf("HACC(m): %d\n", GPSdata.hAcc / 1000);
        Serial.printf("VACC(m): %d\n", GPSdata.vAcc / 1000);
        
    }
    Serial.clear();




}

void groundRTCSync() {
    //radio.setDio0Action(setFlag);
    u_int64_t unixTime = getTimestampMillis();
    Serial.printf("UNIX TIME: %llu\n", unixTime);
    byte arr[lenTransmissionBytes] = {RTC_SYNC_BYTE};
    for (int i=0; i<8; i++) {
        arr[i+1] = (unixTime >> (7 * 8 - (i * 8))) & 0xFF; // little endian
    }
    while(!transmittedFlag); // transmittedFlag should be true here.
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
    BMP280Setup();
    NEO6mSetup();
    BNO055Setup();
    ADXL377Setup();


/** Initialise SD card and transmitter */
    unsigned long long timestampSDFileName = getTimestampMillis();
    sdSetup(timestampSDFileName);
    
    RFM96WtransmitSetup();
    Serial.printf("SUCCESS - All systems initialised.\n");

/** Send Teensy unix time to ground station over telemetry to sync. */
    Serial.printf("SYNCING ground station RTC\n");
    groundRTCSync();

    enterToContinue();
    

/** Full system test before launch */
    fullSystemTest();




/** Wait for "Launch commit" message from ground station */
    // Set up RFM96W as receiver
    radio.setDio0Action(setFlagRecieve);

    // Wait for Launch commit message
    bool launchCommit = false;
    while (!launchCommit) {
        Serial.println("Waiting for launch commit code from ground station...");
        int state = radio.startReceive();
        while(!receivedFlag); // wait for packet
        state = radio.readData(byteArr, lenTransmissionBytes);
        //RFM96WrecieveBytesLORA();
        Serial.println("Received message");
        if (byteArr[0] == LAUNCH_COMMIT_CODE) {
            launchCommit = true;
            Serial.println("Launch commit message received");
        }
        else {
            Serial.println("That wasnt the launch commit code");
        }
    }
    delay(500);
    // Set up RFM96W as transmitter
    radio.setDio0Action(setFlag);
    // Send confirmation message to ground station
    char launchCommitMessasge[23] = {LAUNCH_COMMIT_CONFIRM, 0,0,0,0,0,0,0};
    transmissionState = radio.startTransmit(launchCommitMessasge);
    Serial.println("Sent launch commit confirmation");


// The dreaded code that caused far too many hours of debugging - do not touch :)
// /** Wait for "Launch commit" message from ground station */
//     // Set up RFM96W as receiver
//     radio.setDio0Action(setFlagRecieve);
//     // Wait for Launch commit message
//     bool launchCommit = false;
//     while (!launchCommit) {
//         Serial.println("Waiting for launch commit code from ground station...");
//         int state = radio.startReceive();
//         while(!receivedFlag); // wait for packet
//         RFM96WrecieveBytesLORA();
//         Serial.println("Received message");
//         if (byteArr[0] == LAUNCH_COMMIT_CODE) {
//             launchCommit = true;
//             Serial.println("Launch commit message received");
//         }
//         else {
//             Serial.println("That wasnt the launch commit code");
//         }
//     }
//     delay(1000);
//     // Set up RFM96W as transmitter
//     radio.setDio0Action(setFlag);
//     // Send confirmation message to ground station
//     byte launchCommitMessasge[23] = {LAUNCH_COMMIT_CONFIRM, 0,0,0,0,0,0,0}; // TODO: this is not working??? 
//     //transmissionState = radio.startTransmit(launchCommitMessasge);
//     while(!transmittedFlag); // wait for packet
//     transmitData(launchCommitMessasge);
//     Serial.println("Sent launch commit confirmation"); 


// /// We are now on the launch pad, ready to launch.

/** (BNO055) Determine current Orientation using accelerometers and gravity vector*/
    getInitialOrientation();

    Serial.printf("Initial orientation determined: %lf, %lf, %lf \n", ori[0], ori[1], ori[2]);
    launched = false;
    launchInterrupt = false; // for in case the interrupt ran before we got here, due to movement of loading rocket
    // Send GPS request
    sendUbx(NAV, NAV_POSLLH, 0, nullptr);
    delay(200);

    //while(1);

} 

void loop() {
    unsigned long startOfLoopUs = micros();
/** Data acquisition - BNO055, ADXL377, BMP280 */
    // Get timestamp
    uint64_t timestamp = getTimestampMillis();
    
    // Get ADXL377 data
    Vector<double>rawADXLacc = getADXL377Acc();

    // Get and parse BNO055 data
    bno055_burst_t burst = sensor.getAllData();
    Vector<double>rawBNO055Acc = burst.accel;
    Vector<double>rawBNO055Gyro = burst.gyro;

    // Especially for Tom, so he can see the effects that biasing have on dead reckoning results
    SDDataLog.BNO055_acc_x = rawBNO055Acc[0];
    SDDataLog.BNO055_acc_y = rawBNO055Acc[1];
    SDDataLog.BNO055_acc_z = rawBNO055Acc[2];
    SDDataLog.BNO055_gyr_x = rawBNO055Gyro[0];
    SDDataLog.BNO055_gyr_y = rawBNO055Gyro[1];
    SDDataLog.BNO055_gyr_z = rawBNO055Gyro[2];

    // bias correction
    for(int i=0;i<3;i++) { rawBNO055Acc[i] -= acc_biases[i];  rawBNO055Gyro[i] -= gyr_biases[i]; }

    /// Filter updates
    updateBNOFilters(rawBNO055Gyro, rawBNO055Acc);  
    updateADXL377Filters(rawADXLacc);
    tone(buzzerPin, (1000+((int)timestamp%1000)));

    // Launch detection
    if (!launched && ( launchInterrupt || (magnitude(filteredAccBNO055) > launchThreshold) ) ) { 
        // ^ critical efficiency /:)
        launchInterrupt = true;
        launched = true;
        statusCode |= LAUNCH_DETECT_MASK;
    }
    sensor.clearInterrupt();
    //Serial.printf("%d, %d, %lf, %lf\n", launched, launchInterrupt, launchThreshold, magnitude(filteredAccBNO055)); //For testing interrupts and launch detection
    
    // Get BMP280 data
    getBMP280Data(SEA_LEVEL_HPA);

    // Get GPS data every 200ms - 5Hz
    if ((cycleCounter % 20) == 0) {
        getGPSData();
        sendUbx(NAV, NAV_POSLLH, 0, nullptr); // send request for next GPS data, will be available in 200ms (20 cycles)
        SDDataLog.GPS_lat = GPSdata.lat;
        SDDataLog.GPS_lon = GPSdata.lon;
        SDDataLog.GPS_alt = GPSdata.alt;
        SDDataLog.GPS_tow = GPSdata.towMs;
        SDDataLog.GPS_hacc = GPSdata.hAcc;
        SDDataLog.GPS_vacc = GPSdata.vAcc; 
    }

    
/** Calculations */
    if (launched) {
        if (magnitude(filteredAccBNO055) > 14*9.81) { // over 14g, we use the ADXL377 data
            deadReckoning(filteredADXL, filteredGyro, cycleTimeus);
        }
        else {
            deadReckoning(filteredAccBNO055, filteredGyro, cycleTimeus);
        }
    }
    

/** Data logging */
    SDDataLog.logCode = statusCode;
    SDDataLog.timeStamp = timestamp;
    // Raw, (NOT bias corrected) filtered sensor data especially for Tom.
    //   ^ done above, before bias correction

    // Filtered, bias corrected sensor data that is used in dead reckoning
    SDDataLog.BNO055_acc_x_filt = filteredAccBNO055[0];
    SDDataLog.BNO055_acc_y_filt = filteredAccBNO055[1];
    SDDataLog.BNO055_acc_z_filt = filteredAccBNO055[2];
    SDDataLog.BNO055_gyr_x_filt = filteredGyro[0];
    SDDataLog.BNO055_gyr_y_filt = filteredGyro[1];
    SDDataLog.BNO055_gyr_z_filt = filteredGyro[2];

    // ADXL377 
    SDDataLog.ADXL_acc_x = filteredADXL[0];
    SDDataLog.ADXL_acc_y = filteredADXL[1];
    SDDataLog.ADXL_acc_z = filteredADXL[2];

    // BMP280
    SDDataLog.BMP280_temp = bmpData.temperature;
    SDDataLog.BMP280_pres = bmpData.pressure;
    SDDataLog.BMP280_alt = bmpData.altitude;

    // NEO6M 
    //  ^ above 

    // Dead reckoning
    SDDataLog.DR_pos_x = pos[0];
    SDDataLog.DR_pos_y = pos[1];
    SDDataLog.DR_pos_z = pos[2];
    SDDataLog.DR_vel_x = vel[0];
    SDDataLog.DR_vel_y = vel[1];
    SDDataLog.DR_vel_z = vel[2];
    SDDataLog.DR_ori_x = ori[0];
    SDDataLog.DR_ori_y = ori[1];
    SDDataLog.DR_ori_z = ori[2];

    // Write to SD card
    logData();
    

/** Data transmission every third cycle */
    if ((cycleCounter % 3) == 0) {
        // Encode data as bytes: // most of this section was automatically generated by github copilot - incredible!
        // Status Code
        if (cycleCounter == 3) {
            Serial.printf("BMPAlt: %lf\n", bmpData.altitude);
            Serial.printf("DRZ: %lf\n", pos[3]);
            Serial.printf("absAcc: %lf\n", magnitude(filteredAccBNO055));
            Serial.printf("absVel: %lf\n", magnitude(vel));
            Serial.printf("GPS Lat, lon: %lf, %lf\n", GPSdata.lat, GPSdata.lon);
            Serial.printf("GPS Alt: %lf\n", GPSdata.alt);

        }
        byteArr[0] = DATA_CODE; 

        
        union byte_float_union {
            byte bytesFromFloat[4];
            float floatVal;
        };
        

        // BMP280  altitude
        float bmpAltTransmit = static_cast<float>(bmpData.altitude);
        // assign the bytes to the float
        byte_float_union bmpAltUnion;
        bmpAltUnion.floatVal = bmpAltTransmit;
        // copy the bytes to the byte array
        byteArr[1] = bmpAltUnion.bytesFromFloat[0];
        byteArr[2] = bmpAltUnion.bytesFromFloat[1];
        byteArr[3] = bmpAltUnion.bytesFromFloat[2];
        byteArr[4] = bmpAltUnion.bytesFromFloat[3];

        // Dead reckoning position Z
        short DRZ = static_cast<short>(pos[2]*10);
        byteArr[5] = DRZ >> 8;
        byteArr[6] = DRZ & 0xFF; // Big endian
        
        // Absolute Acceleration (BNO055 or ADXL)
        if (magnitude(filteredAccBNO055) > 14*9.81) { // over 14g, we use the ADXL377 data
            short absAccTransmit = static_cast<short>(magnitude(filteredADXL)*100);
            byteArr[7] = absAccTransmit >> 8;
            byteArr[8] = absAccTransmit & 0xFF; // Big endian :( 
        }   
        else {
            short absAccTransmit = static_cast<short>(magnitude(filteredAccBNO055)*100);
            byteArr[7] = absAccTransmit >> 8;
            byteArr[8] = absAccTransmit & 0xFF; // Big endian :( 
        }
        
        // Absolute Velocity (Dead reckoning)
        short absVelTransmit = static_cast<short>(magnitude(vel)*10);
        byteArr[9] = absVelTransmit >> 8;
        byteArr[10] = absVelTransmit & 0xFF; 

        // GPS latitiude
        float latTransmit = static_cast<float>(GPSdata.lat);
        byte_float_union latUnion;
        latUnion.floatVal = latTransmit;
        byteArr[11] = latUnion.bytesFromFloat[0];
        byteArr[12] = latUnion.bytesFromFloat[1];
        byteArr[13] = latUnion.bytesFromFloat[2];
        byteArr[14] = latUnion.bytesFromFloat[3];

        // byteArr[11] = (latTransmit >> 24) & 0xFF;
        // byteArr[12] = (latTransmit >> 16) & 0xFF;
        // byteArr[13] = (latTransmit >> 8) & 0xFF;
        // byteArr[14] = latTransmit & 0xFF;

        // GPS longitude
        float lonTransmit = static_cast<float>(GPSdata.lon);
        byte_float_union lonUnion;
        lonUnion.floatVal = lonTransmit;
        byteArr[15] = lonUnion.bytesFromFloat[0];
        byteArr[16] = lonUnion.bytesFromFloat[1];
        byteArr[17] = lonUnion.bytesFromFloat[2];
        byteArr[18] = lonUnion.bytesFromFloat[3];

        // byteArr[15] = (lonTransmit >> 24) & 0xFF;
        // byteArr[16] = (lonTransmit >> 16) & 0xFF;
        // byteArr[17] = (lonTransmit >> 8) & 0xFF;
        // byteArr[18] = lonTransmit & 0xFF;
        
        // GPS altitude
        float GPSaltTransmit = static_cast<float>(GPSdata.alt);
        byte_float_union GPSaltUnion;
        GPSaltUnion.floatVal = GPSaltTransmit;
        byteArr[19] = GPSaltUnion.bytesFromFloat[0];
        byteArr[20] = GPSaltUnion.bytesFromFloat[1];
        byteArr[21] = GPSaltUnion.bytesFromFloat[2];
        byteArr[22] = GPSaltUnion.bytesFromFloat[3];

        // byteArr[19] = (GPSaltTransmit >> 24) & 0xFF;
        // byteArr[20] = (GPSaltTransmit >> 16) & 0xFF;
        // byteArr[21] = (GPSaltTransmit >> 8) & 0xFF;
        // byteArr[22] = GPSaltTransmit & 0xFF;


        // Transmit byteArr
        unsigned long transmitTimoutMicros = micros();
        bool transmitTimeout = false;
        while (!transmittedFlag) {
            if (micros() - transmitTimoutMicros > 1000) { // 1ms timeout
                transmitTimeout = true;
                break;
            }
        }
        if (!transmitTimeout) {
            transmitData(byteArr);
        }


    }


/** Increment cycle counter */
    cycleCounter++; 

/** Ensuring cycles are 10ms long */
    unsigned long endOfLoopus = micros();
    if ((endOfLoopus-startOfLoopUs) < 10000) {
        delayMicroseconds(10000 - (endOfLoopus-startOfLoopUs));
    } // if the loop exceeded 10ms, there's nothing we can do. :(
}