/*
   RadioLib SX127x Receive with Interrupts Example

   This example listens for LoRa transmissions and tries to
   receive them. Once a packet is received, an interrupt is
   triggered. To successfully receive data, the following
   settings have to be the same on both transmitter
   and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

#include <Arduino.h>
#include <RFM96WrecieveLORA.h>
#include "LineProtocolBuilder.h"

// Transmittsion/reception codes
#define RTC_SYNC_BYTE 0x99
#define TEST_CODE 0xAA
#define LAUNCH_COMMIT_CODE 0xBB
#define LAUNCH_COMMIT_CONFIRM 0xCC
#define DATA_CODE 0xDD


#define LAUNCH_COMMIT_BUTTON_PIN 3
#define RED_LED_PIN 4
#define GREEN_LED_PIN 5
byte rtcSynced = 0;
uint64_t unixTimeMs = 0; // the unix timestamp in milliseconds, synced by the rocket when it starts up.
uint32_t RTCsyncMillis = 0; // the millis() time we aquired the unixTime

char line[500] = "";

char * uintToStr(  uint64_t num, char *str )
{
    uint8_t i = 0;
    uint64_t n = num;

    do
        i++;
    while ( n /= 10 );

    str[i] = '\0';
    n = num;

    do
        str[--i] = ( n % 10 ) + '0';
    while ( n /= 10 );

    return str;
}

void sendLaunchCommit() {
    Serial.println("Sending launch commit");


    // set up the ground station as a transmitter
    radio.setDio0Action(setFlag);
    byte arr[lenReceiveBytes] = {LAUNCH_COMMIT_CODE};
    int state = radio.startTransmit(arr, lenReceiveBytes);
    delay(200);
    // Set up ground station as a receiver
    radio.setDio0Action(setFlagRecieve);

    detachInterrupt(digitalPinToInterrupt(LAUNCH_COMMIT_BUTTON_PIN)); // So that we dont "launch commit twice"
    Serial.println("Succesfully sent launch commit and detatched interrupt");
    delay(100);
}

void setup() {
    Serial.begin(115200);
    RFM96WrecieveLORASetup();
    pinMode(LAUNCH_COMMIT_BUTTON_PIN, INPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(500);
    attachInterrupt(digitalPinToInterrupt(LAUNCH_COMMIT_BUTTON_PIN), sendLaunchCommit, RISING);

    

}


void loop() {
    //Serial.println("\nReady to recieve");
    while (!receivedFlag); // wait for packet
    //Serial.println("Received packet");
    //Serial.println(millis());
    RFM96WrecieveBytesLORA();
    //Serial.println(byteArr[0], HEX);

    switch (byteArr[0]) {
        case RTC_SYNC_BYTE:{ 
            rtcSynced = 1;
            RTCsyncMillis = millis();
            for (int i=0; i<8; i++) {
                unixTimeMs |= ((uint64_t)byteArr[i+1]) << (7 * 8 - (i * 8));
            }
            Serial.println("\nRTC synced!");
            char unixTimeStr[20] = {0};
            uintToStr(unixTimeMs, unixTimeStr);
            Serial.print(unixTimeStr);   
            
            break; }
        case TEST_CODE: {
            // If there's a test code, we can test influx and Grafana as well
            uint64_t testTime = unixTimeMs + (millis() - RTCsyncMillis);
            char test1[20] = "";
            char test2[20] = "";
            char test3[20] = "";
            char test4[20] = "";
            char test5[20] = "";
            char test6[20] = "";
            char test7[20] = "";

            dtostrf(0, 7, 7, test1); // github copilot for the win
            dtostrf(1, 7, 7, test2);
            dtostrf(2, 7, 7, test3);
            dtostrf(3, 7, 7, test4);
            dtostrf(4, 7, 7, test5);
            dtostrf(5, 7, 7, test6);
            dtostrf(6, 7, 7, test7);
    

            if (rtcSynced) {
                sprintf(line, "Daedalus BMPAlt=%s,DRZ=%s,absAcc=%s,absVel=%s,GPSLat=%s,GPSLon=%s,GPSAlt=%s %llu",
                        test1, test2, test3, test4, test5, test6, test7, testTime);
            }
            else {
                sprintf(line, "Daedalus BMPAlt=%s,DRZ=%s,absAcc=%s,absVel=%s,GPSLat=%s,GPSLon=%s,GPSAlt=%s",
                        test1, test2, test3, test4, test5, test6, test7);
            }

            Serial.println(line);
            break; }
        case LAUNCH_COMMIT_CONFIRM: {
            Serial.println("\nLAUNCH COMMIT CONFIRMED\n");
            for (int i=0; i<lenReceiveBytes; i++) {
                Serial.print(byteArr[i], HEX);
                Serial.print(", ");
            }
            digitalWrite(RED_LED_PIN, LOW);
            digitalWrite(GREEN_LED_PIN, HIGH);
            break; }
        case DATA_CODE: {
            uint64_t timestamp = unixTimeMs + (millis() - RTCsyncMillis);
            double BMP280Alt = ((byteArr[1] << 24) | (byteArr[2] << 16) | (byteArr[3] << 8) | (byteArr[4]) ) / 1000.0;
            double DRZ = ((byteArr[5] << 8) | (byteArr[6]) ) / 10.0;
            double absAcc = ((byteArr[7] << 8) | (byteArr[8]) ) / 100.0;
            double absVel = ((byteArr[9] << 8) | (byteArr[10]) ) / 100.0;
            double GPSLat = ((byteArr[11] << 24) | (byteArr[12] << 16) | (byteArr[13] << 8) | (byteArr[14]) ) / 1000000.0;
            double GPSLon = ((byteArr[15] << 24) | (byteArr[16] << 16) | (byteArr[17] << 8) | (byteArr[18]) ) / 1000000.0;
            double GPSAlt = ((byteArr[19] << 24) | (byteArr[20] << 16) | (byteArr[21] << 8) | (byteArr[22]) ) / 1000.0;

            char BMP280AltStr[20] = "";
            char DRZStr[20] = "";
            char absAccStr[20] = "";
            char absVelStr[20] = "";
            char GPSLatStr[20] = "";
            char GPSLonStr[20] = "";
            char GPSAltStr[20] = "";

            dtostrf(BMP280Alt, 7, 7, BMP280AltStr); // github copilot for the win
            dtostrf(DRZ, 7, 7, DRZStr);
            dtostrf(absAcc, 7, 7, absAccStr);
            dtostrf(absVel, 7, 7, absVelStr);
            dtostrf(GPSLat, 7, 7, GPSLatStr);
            dtostrf(GPSLon, 7, 7, GPSLonStr);
            dtostrf(GPSAlt, 7, 7, GPSAltStr);

            


            if (rtcSynced) {
                sprintf(line, "Daedalus BMPAlt=%s,DRZ=%s,absAcc=%s,absVel=%s,GPSLat=%s,GPSLon=%s,GPSAlt=%s %llu",
                        BMP280AltStr, DRZStr, absAccStr, absVelStr, GPSLatStr, GPSLonStr, GPSAltStr, timestamp);
            }
            else {
                sprintf(line, "Daedalus BMPAlt=%s,DRZ=%s,absAcc=%s,absVel=%s,GPSLat=%s,GPSLon=%s,GPSAlt=%s",
                        BMP280AltStr, DRZStr, absAccStr, absVelStr, GPSLatStr, GPSLonStr, GPSAltStr);
            }

            Serial.println(line);

            


            break; }
        default: {
            Serial.println("\nUnknown code\n");
            for (int i=0; i<lenReceiveBytes; i++) {
                Serial.print(byteArr[i], HEX);
                Serial.print(", ");
            }
            break; }
    }
    
    


}