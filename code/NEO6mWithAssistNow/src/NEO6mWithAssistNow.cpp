// Created by Samuel scott (robosam2003) on 30/04/2022
// This program sets up the neo6m gps and performs assistNow loading.
/// Datasheet url: https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
/// Also see: https://content.u-blox.com/sites/default/files/products/documents/MultiGNSS-Assistance_UserGuide_%28UBX-13004360%29.pdf
/// little endian format /:)


#include "NEO6mWithAssistNow.h"

GPSDataStruct GPSdata = {0};

void NEO6mSetup() {
    getTimestampMillis();

    // turn on the GPS
    pinMode(transistorPin, OUTPUT);
    digitalWrite(transistorPin, HIGH);
    // must delay on first power up, because GPS sends text messages.
    delay(3000);
    Serial.begin(9600);
    gpsSerial.begin(9600); // defaults to 9600
    serialClear();

    delay(100);

    NEO6mConfig();

    performOnlineAssist();
}

void getGPSData() {
    // get the GPS data and store it in the GPSdata struct

    // Read NAV-POSLLH
    byte POSLLH[36] = {0};
    getUbx(NAV, NAV_POSLLH, NAV_POSLLH_PAYLOAD_LENGTH, POSLLH);
    // parsing the received data
    GPSdata.towMs = POSLLH[6] | POSLLH[7] << 8 | POSLLH[8] << 16 | POSLLH[9] << 24;
    GPSdata.lon = ((signed int) (POSLLH[10] | POSLLH[11] << 8 | POSLLH[12] << 16 | POSLLH[13] << 24)) / 10000000.0;
    GPSdata.lat = ((signed int) (POSLLH[14] | POSLLH[15] << 8 | POSLLH[16] << 16 | POSLLH[17] << 24)) / 10000000.0;

    /// height (below) is the "height above ellipsoid" - it's not anywhere near the mean sea level value so we arent using it.
    //double height = ((signed int) (POSLLH[18] | POSLLH[19] << 8 | POSLLH[20] << 16 | POSLLH[21] << 24)) / 1000.0;

    GPSdata.alt = ((signed int) (POSLLH[22] | POSLLH[23] << 8 | POSLLH[24] << 16 | POSLLH[25] << 24 )) / 1000.0; // HSML ;)
    GPSdata.hAcc = POSLLH[26] | POSLLH[27] << 8 | POSLLH[28] << 16 | POSLLH[29] << 24;
    GPSdata.vAcc = POSLLH[30] | POSLLH[31] << 8 | POSLLH[32] << 16 | POSLLH[33] << 24;



}

time_t getTeensy3Time()
{
    return Teensy3Clock.get();
}

uint64_t getTimestampMillis()
{
    // Created by Ashley Shaw on 19/04/2022 using stuff from https://forum.pjrc.com/threads/68062-Teensy-4-1-RTC-get-milliseconds-correctly
    // 2022 TeamSunride.
    //

    uint64_t periods;
    uint32_t hi1 = SNVS_HPRTCMR, lo1 = SNVS_HPRTCLR;
    while (true)
    {
        uint32_t hi2 = SNVS_HPRTCMR, lo2 = SNVS_HPRTCLR;
        if (lo1 == lo2 && hi1 == hi2)
        {
            periods = (uint64_t)hi2 << 32 | lo2;
            break;
        }
        hi1 = hi2;
        lo1 = lo2;
    }
    uint32_t ms = (1000 * (periods % 32768)) / 32768;
    time_t sec = periods / 32768;
    tm t = *gmtime(&sec);

    setTime(t.tm_hour-1, t.tm_min, t.tm_sec, t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);

    uint64_t unixTime = (uint64_t(now()) * 1000) + ms;
    return unixTime;
}

// rtcSetup has been made redundant by getTimeStampMillis which is much more accurate.
/*void rtcSetup()  {
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
}*/

time_t TimeFromYMD(int year, int month, int day) {
    struct tm tm = {0};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    return mktime(&tm);
}

unsigned short GPSweek() {
    // 315964800 is the unix timestamp (s) of midnight 6th Jan 1980 - the start of GPS time |
    // There has been 18 leap seconds since this date (unix time does not account for leap seconds)
    // not sure when the next leap second is due
    u_int64_t diff = (getTimestampMillis()/1000) - 315964800 + 18;
    return (unsigned short) (diff / SECS_PER_WEEK);
}

unsigned int actualTimeOfWeekms() {
    // 315964800000 is the unix timestamp (ms) of 6th Jan 1980 - the start of GPS time |
    // There has been 18 leap seconds since this date (unix time does not account for leap seconds)
    // not sure when the next leap second is due
    u_int64_t diff = (getTimestampMillis()) - 315964800000 + 18000;
    return (unsigned int) ((diff) % (SECS_PER_WEEK*1000));
    //return (unsigned int) (1000*((weekday()-1)*SECS_PER_DAY + hour()*SECS_PER_HOUR + minute()*SECS_PER_MIN + second())) +18000;
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
    if (Serial) Serial.println("\nThis is the buffer:");
    for (int i=0;i<payloadLength+8;i++) {
        if (Serial) Serial.printf("%02X ", buffer[i]);
    }
    /// Send the message
    for (int i = 0; i < payloadLength + 8; i++) {
        if (gpsSerial) gpsSerial.write(buffer[i]);
    }

    if (gpsSerial) gpsSerial.flush();

    if (messageID == CFG_PRT) {
        gpsSerial.begin(gpsBaudRate);
    }
    char ackOrNack[64] = {0};
    if (ubxClassID == CFG) { // only for CFG messages
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


    // Wait for the response with timeout
    int TIMEOUT = 1000; // us
    unsigned long startTime = micros();
    while (gpsSerial.available() < (payloadLength + 8)) {
        if (micros() - startTime > TIMEOUT) {
            Serial.println("Timeout");
            return;
        }
    }

    while (gpsSerial.peek() != ubxHeader1) { 
        gpsSerial.read(); 
    } // ensures reading starts from start of ubx packet
    byte buffer[payloadLength + 8];
    gpsSerial.readBytes(buffer, payloadLength + 8);


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
    if (buffer[5] != (payloadLength & 0xFF00) >> 8 || buffer[4] != (payloadLength & 0x00FF)) { // LITTLE ENDIAN !
        Serial.println("Bad payload length");
        return;
    }
    /// Check the checksum
    byte * ckBuffer = new byte[payloadLength + 4]; // includes ubxClassID, messageID, payloadLength, and payload
    for (int i=0;i<payloadLength+4; i++) {
        ckBuffer[i] = buffer[i+2];
    }

    int n = payloadLength + 4;
    uint8_t CK_A= 0, CK_B = 0;
    for (int i=0;i<n;i++)
    {
        CK_A = CK_A + ckBuffer[i];
        CK_B = CK_B + CK_A;
    }

    if (buffer[payloadLength + 6] != CK_A || buffer[payloadLength + 7] != CK_B) {
        Serial.printf("Checksum failed\n");
        return;
    }

    /// Copy the payload
    for (int i = 0; i < payloadLength+8; i++) {
        payload[i] = buffer[i];
    }
    delete[] ckBuffer; // Very important

}

int getUbxFromFile(File fptr, byte ubxClassID, byte messageID, short payloadLength, byte payload[]) {
    // For finding a particular UBX sequence in a file
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
    /** --------- AIDING SEQUENCE--------- Datasheet section 13.5 (pg. 34)
     *   --- Note that we are altering the AID_INI part of the message and leaving the rest as is. ---
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

    /// this file ("mgaonline.ubx") should be loaded into the onboard SD card
    /// the file should be obtained from the ublocks server
    /// the generator token for that can be obtained from thingstream from ublocks
    /// See https://developer.thingstream.io/guides/location-services/assistnow-getting-started-guide for more details
    File dataFile = SD.open("mgaonline.ubx");
    const int numbytes = dataFile.available();
    Serial.printf("File size: %d\n", numbytes);
    if (!dataFile) {
        Serial.println("Failed to open file");
        while (true);
    }
    byte * fileBuffer = new byte[numbytes]; // use new for array of variable size - remember to delete[] !
    dataFile.readBytes(reinterpret_cast<char *>(fileBuffer), numbytes);

    Serial.printf("%d:%d:%d,  %d/%d/%d", hour(), minute(), second(), day(), month(), year());
    Serial.printf("GPS WEEK: %d\n", GPSweek());
    Serial.printf("GPS time of week: %d\n", actualTimeOfWeekms());

    // alter the necessary fields in the file buffer AID_INI

    // configure week number, little endian /:)
    int headerLength = 6;
    unsigned short gpsweek = GPSweek();
    fileBuffer[18+headerLength] = (gpsweek & 0x00FF);
    fileBuffer[19+headerLength] = (gpsweek & 0xFF00) >> 8;

    // configure time of week, little endian /:)
    unsigned long timeOfWeekms = actualTimeOfWeekms()+150;
    fileBuffer[20+headerLength] = (timeOfWeekms & 0x000000FF);
    fileBuffer[21+headerLength] = (timeOfWeekms & 0x0000FF00) >> 8;
    fileBuffer[22+headerLength] = (timeOfWeekms & 0x00FF0000) >> 16;
    fileBuffer[23+headerLength] = (timeOfWeekms & 0xFF000000) >> 24; 

    Serial.println("\n\n\n");

    // setting the new checksums
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
        gpsSerial.flush(); // flush waits for the above write to finish

    }

    dataFile.close();
    delete[] fileBuffer; // delete[] - very important - we don't like them segfaults

    Serial.println("\nFinished performOnlineAssist()\n");

}

void NEO6mConfig() {
    // turn off the NMEA sentences first.
    gpsSerial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    gpsSerial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
    gpsSerial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    gpsSerial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
    gpsSerial.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    gpsSerial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
    gpsSerial.print("$PUBX,40,GGA,0,0,0,0*5A\r\n"); // DISABLE THEM ALLLL

    /// Configure the NEO6m using the CFG registers

    // Set power mode to Max performance mode
    byte CFG_RXM_payload[2] = {0x08, 0x00}; // reserved, Mode (0 for max performance)
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
    short measrate = 200; // ms
    byte CFG_RATE_payload[6] = {(measrate & 0xFF), ((measrate>>8) & 0xFF), // update rate
                                0x01, 0x00, // nav rate - always set to 1
                                0x01, 0x00}; // time ref - 1 for gps time, 0 for utc
    sendUbx(CFG, CFG_RATE, 6, CFG_RATE_payload);


    ///  As a second step, activate certain messages on each port using CFG_MSG
    ///  NOTE: configuring messages in this way sets up "periodic polling" - The GPS will spit out this data very (measrate) milliseconds
//    byte CFG_MSG_NAV_POLLSH_payload[8] = {NAV, NAV_POSLLH, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
//    sendUbx(CFG, CFG_MSG, 8, CFG_MSG_NAV_POLLSH_payload);


    // set the neo6m config to ubx
    byte CFG_CFG_payload[13] = {0,0,0,0,
                                0, 0, 0b00000110, 0b00011111, 
                                0, 0, 0b00000110, 0b00011111,
                                0b00000001};
    sendUbx(CFG, CFG_CFG, 13, CFG_CFG_payload);


}
