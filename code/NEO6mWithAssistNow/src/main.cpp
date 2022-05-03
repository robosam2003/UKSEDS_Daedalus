// Created by Samuel scott (robosam2003) on 30/04/2022
// This program sets up the neo6m gps and performs assistNow loading.
/// Datasheet url: https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#page=118&zoom=100,0,0
/// Also see: https://content.u-blox.com/sites/default/files/products/documents/MultiGNSS-Assistance_UserGuide_%28UBX-13004360%29.pdf
/// little endian format /:)

// This program reads the mgaonline.ubx file from the sd card and then writes it to the NEO6M GPS


#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h> // : https://github.com/PaulStoffregen/Time
#include "assistNow.h"
#include <ctime>



#define transistorPin 32
#define ubxHeader1 0xB5
#define ubxHeader2 0x62
#define gpsBaudRate 9600


/// Change this what Serial port the GPS is connected to
#define gpsSerial Serial2 // Hardware serial port for GPS


time_t getTeensyTime() {
    return Teensy3Clock.get();
}

void rtcSetup()  {
    setSyncProvider(getTeensyTime);
    Serial.begin(9600);
    delay(100);
    if (timeStatus()!= timeSet) {
        Serial.println("Unable to sync with the RTC");
    } else {
        Serial.println("RTC has set the system time");
    }
    // Usually about 2 seconds difference from compile time to runtime. Set the RTC to + 2 seconds to compensate.
    setTime(hour()-1, minute(), second() + 2, day(), month(), year()); // we are on bst time rn
}

time_t TimeFromYMD(int year, int month, int day) {
    struct tm tm = {0};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    return mktime(&tm);
}

unsigned short GPSweek() {
    double diff = difftime(TimeFromYMD(year(), month(), day()), TimeFromYMD(1980, 1, 1));
    return (unsigned short) (diff / SECS_PER_WEEK);
}

unsigned int actualTimeOfWeekms() {
    return (unsigned int) (1000*((weekday()-1)*SECS_PER_DAY + hour()*SECS_PER_HOUR + minute()*SECS_PER_MIN + second())) +18000;
}

void serialClear() {
    while (gpsSerial.available()) {
        gpsSerial.read();
    }
}


void sendUbx(byte ubxClassID, byte messageID, short payloadLength, const byte payload[]) { // Send a UBX message

    serialClear();


    byte buffer[payloadLength + 8];
    buffer[0] = ubxHeader1;
    buffer[1] = ubxHeader2;
    buffer[2] = ubxClassID;
    buffer[3] = messageID;
    buffer[4] = (payloadLength & 0x00FF); // little endian: lsB first
    buffer[5] = (payloadLength & 0xFF00) >> 8;
    for (int i = 0; i < payloadLength; i++) {
        buffer[i + 6] = payload[i];
    }
    /// Calculate the checksum
    byte ckBuffer[payloadLength + 4]; // includes ubxClassID, messageID, payloadLength, and payload
    ckBuffer[0] = ubxClassID;
    ckBuffer[1] = messageID;
    ckBuffer[2] = (payloadLength & 0x00FF); // little endian: lsB first
    ckBuffer[3] = (payloadLength & 0xFF00) >> 8;
    for (int i = 0; i < payloadLength; i++) {
        ckBuffer[i + 4] = payload[i];
    }
    int n = payloadLength + 4;
    uint8_t CK_A= 0;
    uint8_t CK_B = 0;
    for (int i=0;i<n;i++)
    {
        CK_A = CK_A + ckBuffer[i];
        CK_B = CK_B + CK_A;
    }
    buffer[payloadLength + 6] = CK_A;
    buffer[payloadLength + 7] = CK_B;
    Serial.println("\nThis is the buffer:");
    for (int i=0;i<payloadLength+8;i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    /// Send the message
    for (int i = 0; i < payloadLength + 8; i++) {
        gpsSerial.write(buffer[i]);
    }

    gpsSerial.flush();

    char ackOrNack[64] = {0};
    if (ubxClassID == CFG) {
        delay(500); // needed for the reciever to have time to send ACK/NACK message.
        if (!gpsSerial.available()) { Serial.printf("The gps serial is not available."); };
        while (!gpsSerial.available());
        int a = gpsSerial.available();
        Serial.println();
        Serial.println(gpsSerial.available());
        gpsSerial.readBytes(ackOrNack, a);

        for (int i = 0; i < a; i++) {
            Serial.printf("%02X ", ackOrNack[i]);
        }
        if (ackOrNack[2] == 0x05 && ackOrNack[3] == 0x01) {
            Serial.println("ACK-ACK");
        } else {
            Serial.println("NACK-NACK");
        }
        Serial.println();

    }

}

void getUbx(byte ubxClassID, byte messageID, short payloadLength, byte payload[]) { // Get a UBX message
    // As a quick aside, Most of this function was generated almost entirely by Github copilot, and I am truly shocked.

    /** The UBX protocol was designed such, that when sending a message with no payload
     (or just a single parameter which identifies the poll request) the
     message is polled.  - Datasheet: 27.2 */

    /// Message structure:
    /// ubxHeader1, ubxHeader2, ubxClassID, messageID, payloadLength, payload, checksumA, checksumB
    // Send the poll request with empty payload to poll.

    Serial.println(gpsSerial.available());
    while (gpsSerial.available()) { gpsSerial.read(); }// should clear the buffer

    Serial.println(gpsSerial.available());
    sendUbx(ubxClassID, messageID, 0, payload);
    Serial.clear();
    Serial.println(gpsSerial.available());

    // Wait for the response
    while (gpsSerial.available() < (payloadLength + 8));
    Serial.println("bytes available:");
    Serial.printf("%d\n", gpsSerial.available());



    byte buffer[payloadLength + 8];
    while (gpsSerial.available()) { gpsSerial.readBytes(buffer, payloadLength + 8); }

    Serial.println("BUFFER");
    for (int i = 0; i < payloadLength+8; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    /// Check the header
    if (buffer[0] != ubxHeader1 || buffer[1] != ubxHeader2) {
        Serial.println("Bad header");
        return;
    }
    /// Check the class ID
    if (buffer[2] != ubxClassID) {
        Serial.println("Bad class ID");
        return;
    }
    /// Check the message ID
    if (buffer[3] != messageID) {
        Serial.println("Bad message ID");
        return;
    }
    /// Check the payload length
    if (buffer[4] != (payloadLength & 0xFF00) >> 8 || buffer[5] != (payloadLength & 0x00FF)) {
        Serial.println("Bad payload length");
        return;
    }
    /// Check the checksum
    byte ckBuffer[payloadLength + 4]; // includes ubxClassID, messageID, payloadLength, and payload
    int n = payloadLength + 4;
    uint8_t CK_A= 0, CK_B = 0;
    for (int i=0;i<n;i++)
    {
        CK_A = CK_A + ckBuffer[i];
        CK_B = CK_B + CK_A;
    }
    if (buffer[payloadLength + 6] != CK_A || buffer[payloadLength + 7] != CK_B) {
        printf("Checksum failed\n");
        return;
    }
    /// Copy the payload
    for (int i = 0; i < payloadLength; i++) {
        payload[i] = buffer[i + 6];
    }
}

int getUbxFromFile(File fptr, byte ubxClassID, byte messageID, short payloadLength, byte payload[]) {
    fptr.seek(0);
    int fileSize = fptr.size();
    for (int i=0;i<fileSize;i++) {
        byte b = fptr.read();
        // get the current file position
        int pos = fptr.position();

        if (b == ubxHeader1) {
            Serial.println("Found header 1");
            if (fptr.read() == ubxHeader2) {
                Serial.println("Found header 2");
                if (fptr.read() == ubxClassID) {
                    Serial.println("Found class ID");
                    if (fptr.read() == messageID) {
                        Serial.println("Found message ID");
                        if (fptr.read() == (payloadLength & 0x00FF)) { // little endian
                            Serial.println("Found payload length LSB");
                            if (fptr.read() == (payloadLength & 0xFF00) >> 8) {
                                Serial.println("Found payload length MSB");
                                fptr.readBytes(reinterpret_cast<char *>(payload), payloadLength);

                                Serial.println("Found payload");
                                return 1; // 1 for success
                            }
                        }
                    }
                }
            }
        }
        // seek back to the current position (if we didn't return)
        fptr.seek(pos);
    }
    return 0; // 0 for failure (could not find)


}

void performOnlineAssist() {
    /** ---------AIDING SEQUENCE--------- Datasheet section 13.5 (pg. 34)
     • Power-up the GPS receiver
     • Send UBX-AID-INI (time, clock and position) message.
     • Send UBX-AID-EPH (ephemeris) message.
     • Apply optional hardware time synchronization pulse within 0.5 s after (or before, depending on the
       configuration in UBX-AID-INI) sending the UBX-AID-INI message if hardware time synchronization is
       required. When sending the message before applying the pulse, make sure to allow the GPS receiver to
       parse and process the aiding message. The time for parsing depends on the baud rate. The processing time
       is 100 ms maximum.
     • Send optional UBX-AID-HUI (health, UTC and ionosphere parameters) message.
     • Send optional UBX-AID-ALM (almanac) message. **/

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Could not mount SD card");
        while (true);
    }
    Serial.println("Card initialised");

    // this file ("mgaonline.ubx") should be loaded into the onboard SD card
    // the generator token can be obtained from thingstream from ublocks
    File dataFile = SD.open("mgaonline.ubx");
    const int numbytes = dataFile.available();
    Serial.printf("File size: %d\n", numbytes);
    if (!dataFile) {
        Serial.println("Failed to open file");
        while (true);
    }
    byte * fileBuffer = new byte[numbytes];
    dataFile.readBytes(reinterpret_cast<char *>(fileBuffer), numbytes);

    Serial.printf("%d:%d:%d,  %d/%d/%d", hour(), minute(), second(), day(), month(), year());
    Serial.printf("GPS WEEK: %d\n", GPSweek()-1);
    Serial.printf("GPS time of week: %d\n", actualTimeOfWeekms());

    // alter the necessary fields in the file buffer AID_INI

    // configure week number, little endian
    int headerLength = 6;
    unsigned short gpsweek = GPSweek()-1;
    fileBuffer[18+headerLength] = (gpsweek & 0x00FF);
    fileBuffer[19+headerLength] = (gpsweek & 0xFF00) >> 8;

    // configure time of week, little endian
    unsigned long timeOfWeekms = actualTimeOfWeekms();
    fileBuffer[20+headerLength] = (timeOfWeekms & 0x000000FF);
    fileBuffer[21+headerLength] = (timeOfWeekms & 0x0000FF00) >> 8;
    fileBuffer[22+headerLength] = (timeOfWeekms & 0x00FF0000) >> 16;
    fileBuffer[23+headerLength] = (timeOfWeekms & 0xFF000000) >> 24; // This doesn't work because we change the checksum ey.

    Serial.println("\n\n\n");
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;

    for (int i=2;i<54;i++) {
        CK_A += fileBuffer[i];
        CK_B += CK_A;
    }
    fileBuffer[54] = CK_A;
    fileBuffer[55] = CK_B;
    Serial.printf("CK_A: %02X      CK_B: %02X", CK_A, CK_B);

    for (int i=0;i<56;i++) {
        Serial.printf("%02X ",fileBuffer[i]);
    }
    Serial.printf("\nAvailable for write: %d\n", gpsSerial.availableForWrite());
    for (int  i=0;i<numbytes; i++) {
        gpsSerial.write(fileBuffer[i]);
        gpsSerial.flush();

    }

    dataFile.close();
    delete[] fileBuffer;

    Serial.println("\nFinished performOnlineAssist()\n");

}

void NEO6mConfig() {
    /// Configure the NEO6m using the CFG registers

    // Set power mode to Max performance mode
    byte CFG_RXM_payload[2] = {0x08, 0x00}; // reserved, lpMode (0 for max performance)
    sendUbx(CFG, CFG_RXM, 2, CFG_RXM_payload);


    // Set the NEO6m to use the onboard battery backed ram
    byte CFG_NAV5_payload[36] = {0x00, 0b00000101, 8, 3}; // mask, mask, airborne (<4g), auto fix,
    sendUbx(CFG, CFG_NAV5, 36, CFG_NAV5_payload); // RECIEVED ACK-ACK - Success!

    /// Reconfiguring a port from one protocol to another is a two-step process: - Datasheet: 4.5
    /** First of all, the preferred protocol(s) needs to be enabled on a port using CFG-PRT. One port can handle
    several protocols at the same time (e.g. NMEA and UBX). By default, all ports are configured for UBX and
    NMEA protocol so in most cases, it’s not necessary to change the port settings at all. Port settings can be
    viewed and changed using the CFG-PRT messages.*/

    byte CFG_PRT_payload[20] = {0x01, // portID PORT1 (uart)
                                0x00, // reserved
                                0x00, 0x00, // txReady
                                0b11000000, 0b00001000, 0x00, 0x00,   // mode mask
                                (gpsBaudRate & 0x000000FF), (gpsBaudRate & 0x0000FF00) >> 8, (gpsBaudRate & 0x00FF0000) >> 16, (gpsBaudRate & 0xFF000000) >> 24, // baud rate
                                0b00000001, 0x00,  // inProtoMask, ubx selected
                                0b00000001, 0x00,  // outProtoMaks, ubx selected
                                0x00, 0x00, // reserved
                                0x00, 0x00}; // reserved
    sendUbx(CFG, CFG_PRT, 20, CFG_PRT_payload); // RECIEVED ACK-NACK

    // set update rate to 5Hz
    short measrate = 100; // ms
    byte CFG_RATE_payload[6] = {(measrate & 0xFF), ((measrate>>8) & 0xFF), // update rate
                                0x01, 0x00, // nav rate - always set to 1
                                0x01, 0x00}; // time ref - 1 for gps time, 0 for utc
    sendUbx(CFG, CFG_RATE, 6, CFG_RATE_payload);


    ///  As a second step, activate certain messages on each port using CFG_MSG
    byte CFG_MSG_NAV_POLLSH_payload[8] = {NAV, NAV_POSLLH, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
    sendUbx(CFG, CFG_MSG, 8, CFG_MSG_NAV_POLLSH_payload);
//    // add nmea sentences:
//    byte CFG_MSG_GPGSA[8] = {0xF0, 0x02 ,0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
//    sendUbx(CFG, CFG_MSG, 8, CFG_MSG_GPGSA);
//
//    byte CFG_MSG_GPGSV[8] = {0xF0, 0x03 ,0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
//    sendUbx(CFG, CFG_MSG, 8, CFG_MSG_GPGSV);

//    byte CFG_MSG_GPGAA[8] = {0xF0, 0x00 ,0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
//    sendUbx(CFG, CFG_MSG, 8, CFG_MSG_GPGAA);



    byte CFG_CFG_payload[12] = {0,0,0,0, 0x00, 0x00, 0b00000110, 0b00011111, 0,0,0,0};
    sendUbx(CFG, CFG_CFG, 12, CFG_CFG_payload);


}

void setup() {
    rtcSetup();

    // turn on the GPS
    pinMode(transistorPin, OUTPUT);
    digitalWrite(transistorPin, HIGH);
    delay(3000);
    Serial.begin(9600);
    gpsSerial.begin(gpsBaudRate);
    serialClear();
    // PUBX commands to turn off the NMEA messages
//    byte a[] = "PUBX,40,GGA,0,0,0,0";
//    int ck = 0;
//    for (auto x : a) {
//        ck = ck ^ x;
//    }
//    Serial.printf("THE CHECKSUM FOR GAA IS %02X", ck);


    gpsSerial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    gpsSerial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
    gpsSerial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    gpsSerial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
    gpsSerial.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    gpsSerial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
    gpsSerial.print("$PUBX,40,GGA,0,0,0,0*5A\r\n"); // DISABLE THEM ALLLL
    delay(100);


    NEO6mConfig();



    performOnlineAssist();


    //gpsSerial.print("$PUBX,40,GGA,0,0,0,0*5A\r\n"); // DISABLE THEM ALLLL

    //gpsSerial.clear();
}





void loop() {
    // Read NAV-POSLLH
    byte POSLLH[36] = {}; // TODO: switch this to polling so that we can use multiple commands?
    while (gpsSerial.available() < 36);
    while (gpsSerial.peek() != 0xB5) { gpsSerial.read(); }
    gpsSerial.readBytes(POSLLH, 36);
    for (unsigned char i : POSLLH) {
        Serial.printf("%02X ", i);
    }
    unsigned int towMs = POSLLH[6] | POSLLH[7] << 8 | POSLLH[8] << 16 | POSLLH[9] << 24;
    double longitude = ((signed int) (POSLLH[10] | POSLLH[11] << 8 | POSLLH[12] << 16 | POSLLH[13] << 24)) / 10000000.0;
    double latitude = ((signed int) (POSLLH[14] | POSLLH[15] << 8 | POSLLH[16] << 16 | POSLLH[17] << 24)) / 10000000.0;
    double height = ((signed int) (POSLLH[18] | POSLLH[19] << 8 | POSLLH[20] << 16 | POSLLH[21] << 24)) / 1000.0;
    double hMSL = POSLLH[22] | POSLLH[23] << 8 | POSLLH[24] << 16 | POSLLH[25] << 24;
    signed int hAcc = POSLLH[26] | POSLLH[27] << 8 | POSLLH[28] << 16 | POSLLH[29] << 24;
    signed int vAcc = POSLLH[30] | POSLLH[31] << 8 | POSLLH[32] << 16 | POSLLH[33] << 24;
    Serial.printf("\nTOW:        %d\n", towMs);
    Serial.printf("Actual TOW: %d\n", actualTimeOfWeekms()); // actual tow and tow from gps are synced
    Serial.printf("Time %d:%d:%d\n", hour(), minute(), second());
    Serial.printf("LONG: %lf\n", longitude);
    Serial.printf("LAT: %lf\n", latitude);
    Serial.printf("HEIGHT(m): %lf\n", height);
    Serial.printf("HMSL(m): %lf\n", hMSL);
    Serial.printf("HACC(m): %d\n", hAcc/1000);
    Serial.printf("VACC(m): %d\n", vAcc/1000);




    Serial.println();



}











/*
#include <Arduino.h>
#include <SoftwareSerial.h>
SoftwareSerial GPS(7, 8);
byte gps_set_sucess = 0 ;


// prototypes
void sendUBX(uint8_t *MSG, uint8_t len);
boolean getUBX_ACK(uint8_t *MSG);
void setup()
{
    GPS.begin(9600);
    // START OUR SERIAL DEBUG PORT
    Serial.begin(9600);
    Serial.println("GPS Level Convertor Board Test Script");
    Serial.println("03/06/2012 2E0UPU");
    Serial.println("Initialising....");
    //
    // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
    // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
    //
    GPS.print("$PUBX,41,1,0007,0003,4800,0*13\r\n");
    GPS.begin(9600);
    GPS.flush();

    //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT
    Serial.println("Setting uBlox nav mode: ");
    uint8_t setNav[] = {
            0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
            0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
    while(!gps_set_sucess)
    {
        sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
        gps_set_sucess=getUBX_ACK(setNav);
    }
    gps_set_sucess=0;

    // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
    // UNCOMMENT AS NEEDED
    */
/*
    Serial.println("Switching off NMEA GLL: ");
     uint8_t setGLL[] = {
     0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                   };
     while(!gps_set_sucess)
     {
     sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
     gps_set_sucess=getUBX_ACK(setGLL);
     }
     gps_set_sucess=0;
     Serial.println("Switching off NMEA GSA: ");
     uint8_t setGSA[] = {
     0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                   };
     while(!gps_set_sucess)
     {
     sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
     gps_set_sucess=getUBX_ACK(setGSA);
     }
     gps_set_sucess=0;
     Serial.println("Switching off NMEA GSV: ");
     uint8_t setGSV[] = {
     0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                   };
     while(!gps_set_sucess)
     {
     sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
     gps_set_sucess=getUBX_ACK(setGSV);
     }
     gps_set_sucess=0;
     Serial.print("Switching off NMEA RMC: ");
     uint8_t setRMC[] = {
     0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                   };
     while(!gps_set_sucess)
     {
     sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
     gps_set_sucess=getUBX_ACK(setRMC);
     }
     */
/*


}

void loop()
{
    while(1)
    {
        if(GPS.available())
        {
            // THIS IS THE MAIN LOOP JUST READS IN FROM THE GPS SERIAL AND ECHOS OUT TO THE ARDUINO SERIAL.
            Serial.write(GPS.read());
        }

    }
}


// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
    for(int i=0; i<len; i++) {
        GPS.write(MSG[i]);
        Serial.print(MSG[i], HEX);
    }
    GPS.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
    uint8_t b;
    uint8_t ackByteID = 0;
    uint8_t ackPacket[10];
    unsigned long startTime = millis();
    Serial.print(" * Reading ACK response: ");

    // Construct the expected ACK packet
    ackPacket[0] = 0xB5;	// header
    ackPacket[1] = 0x62;	// header
    ackPacket[2] = 0x05;	// class
    ackPacket[3] = 0x01;	// id
    ackPacket[4] = 0x02;	// length
    ackPacket[5] = 0x00;
    ackPacket[6] = MSG[2];	// ACK class
    ackPacket[7] = MSG[3];	// ACK id
    ackPacket[8] = 0;		// CK_A
    ackPacket[9] = 0;		// CK_B

    // Calculate the checksums
    for (uint8_t i=2; i<8; i++) {
        ackPacket[8] = ackPacket[8] + ackPacket[i];
        ackPacket[9] = ackPacket[9] + ackPacket[8];
    }

    while (1) {

        // Test for success
        if (ackByteID > 9) {
            // All packets in order!
            Serial.println(" (SUCCESS!)");
            return true;
        }

        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000) {
            Serial.println(" (FAILED!)");
            return false;
        }

        // Make sure data is available to read
        if (GPS.available()) {
            b = GPS.read();

            // Check that bytes arrive in sequence as per expected ACK packet
            if (b == ackPacket[ackByteID]) {
                ackByteID++;
                Serial.print(b, HEX);
            }
            else {
                ackByteID = 0;	// Reset and look again, invalid order
            }

        }
    }
}
*/
