//
// Created by robos on 04/05/2022.
//

#include "NEO6mWithAssistNow.h"

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
