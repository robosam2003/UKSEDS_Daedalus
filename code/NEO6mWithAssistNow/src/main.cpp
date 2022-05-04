// Created by Samuel scott (robosam2003) on 30/04/2022
// This program sets up the neo6m gps and performs assistNow loading.
/// Datasheet url: https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
/// Also see: https://content.u-blox.com/sites/default/files/products/documents/MultiGNSS-Assistance_UserGuide_%28UBX-13004360%29.pdf
/// little endian format /:)

#include "NEO6mWithAssistNow.h"

void setup() {
    getTimestampMillis();

    // turn on the GPS
    pinMode(transistorPin, OUTPUT);
    digitalWrite(transistorPin, HIGH);
    delay(3000);
    Serial.begin(9600);
    gpsSerial.begin(gpsBaudRate);
    serialClear();

    delay(100);

    NEO6mConfig();

    performOnlineAssist();


    //gpsSerial.clear();
}


void loop() {


    for (int i=0;i<100;i++) {
        // Read NAV-POSLLH
        byte POSLLH[36] = {0};
        getUbx(NAV, NAV_POSLLH, NAV_POSLLH_PAYLOAD_LENGTH, POSLLH);

        Serial.printf("\nThis is the buffer from main: \n");
        for (unsigned char i: POSLLH) {
            Serial.printf("%02X ", i);
        }

        // parsing the received data
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
        Serial.printf("HACC(m): %d\n", hAcc / 1000);
        Serial.printf("VACC(m): %d\n", vAcc / 1000);

        Serial.println();
    }
    Serial.printf("TURNING GPS OFF\n");
    digitalWrite(transistorPin, LOW);
    delay(5000);
    Serial.printf("TURNING GPS ON\n");
    digitalWrite(transistorPin, HIGH);
//    uint64_t unixTime = getTimestampMillis();
//    Serial.println(unixTime);
//    Serial.printf("%d:%d:%d \n", hour(), minute(), second());
//    Serial.printf("%d, %d\n", GPSweek(), actualTimeOfWeekms());
//    delay(1000);



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
