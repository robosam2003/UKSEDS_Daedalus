
// This program reads the mgaonline.ubx file from the sd card and then writes it to the NEO6M GPS
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>

#define transistorPin 32




void setup() {
    // turn on the GPS
    pinMode(transistorPin, OUTPUT);
    digitalWrite(transistorPin, HIGH);
    Serial.begin(9600);
    Serial2.begin(9600);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Could not mount SD card");
        digitalWrite(LED_BUILTIN, HIGH);
        while (true);
    }
    Serial.println("Card initialised");



    File dataFile = SD.open("mgaonline.ubx");
    const int numbytes = dataFile.available();


    while(dataFile.available()){
        byte b = dataFile.read();
        Serial.print(b, HEX);
        Serial2.write(&b,1);
        Serial2.flush();
    }

    dataFile.close();
    Serial.printf("Successfully read %d bytes and wrote them to the GPS.\n", numbytes);
//        if ((i>3) && (data[i] == '\n') && (data[i-1] == '\r') && (data[i-2] == '\n') && (data[i-3] == '\r')) {
//            headerEndsIndex = i;
//            Serial.printf("Header ends at %d", headerEndsIndex);
//            break;
//        }
//    }
    // get the time from the RTC:



    Serial2.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    Serial2.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
    Serial2.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    Serial2.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
    Serial2.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    Serial2.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
    Serial2.print("$PUBX,40,GGA,205401,19,04,3.0,50.9,N,042.6,W,1,08,1.0,0.0,M,0.0,M,,");

    Serial2.flush();
    Serial2.clear();
}





void loop() {

    while (Serial2.available() > 0) {
        Serial.write(Serial2.read());
    }


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
     *//*


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
