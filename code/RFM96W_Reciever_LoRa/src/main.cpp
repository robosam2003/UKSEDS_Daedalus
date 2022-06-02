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

#define RTC_SYNC_BYTE 0x99
byte rtcSynced = 0;
uint64_t unixTimeMs = 0; // the unix timestamp in milliseconds, synced by the rocket when it starts up.
uint32_t RTCsyncMillis = 0; // the millis() time we aquired the unixTime

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

void setup() {
    Serial.begin(115200);
    RFM96WrecieveLORASetup();
    delay(500);

    // sync RTC
    while (!rtcSynced){
        Serial.println("Waiting for RTC sync...");
        while (!receivedFlag); // wait for packet
        RFM96WrecieveBytesLORA();

        if (byteArr[0] == RTC_SYNC_BYTE) {
            rtcSynced = 1;
            RTCsyncMillis = millis();
            for (int i=0; i<8; i++) {
                unixTimeMs |= ((uint64_t)byteArr[i+1]) << (7 * 8 - (i * 8));
            }
        }
        char unixTimeStr[20] = {0};
        uintToStr(unixTimeMs, unixTimeStr);
        Serial.print(unixTimeStr);
    }

    Serial.println("\nRTC synced!");

}


void loop() {
    while(1){
        uint64_t currentTimeStamp = unixTimeMs + (millis() - RTCsyncMillis);
        uint32_t startTime = millis();
        char buf[30] = {0};
        uintToStr(currentTimeStamp, buf);
        Serial.println(buf);
        // TODO: sort out the uint64_t to uint32_t conversion
        uint32_t endTime = millis();
        delay(1000-(endTime-startTime));
    }


}