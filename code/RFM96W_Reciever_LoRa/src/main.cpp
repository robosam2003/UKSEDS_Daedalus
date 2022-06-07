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
    // // sync RTC
    // while (!rtcSynced){
    //     Serial.println("Waiting for RTC sync...");
    //     while (!receivedFlag); // wait for packet
    //     RFM96WrecieveBytesLORA();

    //     if (byteArr[0] == RTC_SYNC_BYTE) {
    //         rtcSynced = 1;
    //         RTCsyncMillis = millis();
    //         for (int i=0; i<8; i++) {
    //             unixTimeMs |= ((uint64_t)byteArr[i+1]) << (7 * 8 - (i * 8));
    //         }
    //     }
    //     char unixTimeStr[20] = {0};
    //     uintToStr(unixTimeMs, unixTimeStr);
    //     Serial.print(unixTimeStr);
    // }
}


void loop() {
    Serial.println("\nReady to recieve");
    while (!receivedFlag); // wait for packet
    Serial.println("Received packet");
    RFM96WrecieveBytesLORA();
    Serial.println(byteArr[0]);

    switch (byteArr[0]) {
    case RTC_SYNC_BYTE:
        rtcSynced = 1;
        if (byteArr[0] == RTC_SYNC_BYTE) {
            rtcSynced = 1;
            RTCsyncMillis = millis();
            for (int i=0; i<8; i++) {
                unixTimeMs |= ((uint64_t)byteArr[i+1]) << (7 * 8 - (i * 8));
            }
            Serial.println("\nRTC synced!");
            char unixTimeStr[20] = {0};
            uintToStr(unixTimeMs, unixTimeStr);
            Serial.print(unixTimeStr);   
        }
        break;
    case TEST_CODE:
        Serial.println("\nTEST_CODE\n");
        for (int i=0; i<lenReceiveBytes; i++) {
            Serial.print(byteArr[i], HEX);
            Serial.print(", ");
        }
        break;
    case LAUNCH_COMMIT_CONFIRM:
        Serial.println("\nLAUNCH COMMIT CONFIRMED\n");
        for (int i=0; i<lenReceiveBytes; i++) {
            Serial.print(byteArr[i], HEX);
            Serial.print(", ");
        }
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
        break;
    case DATA_CODE:
        // TODO: Add line protocol builder here
        break;
    default:
        Serial.println("\nUnknown code\n");
        for (int i=0; i<lenReceiveBytes; i++) {
            Serial.print(byteArr[i], HEX);
            Serial.print(", ");
        }
        break;
    }
    
    
    

    // while(1){
    //     uint64_t currentTimeStamp = unixTimeMs + (millis() - RTCsyncMillis);
    //     uint32_t startTime = millis();
    //     char buf[30] = {0};
    //     uintToStr(currentTimeStamp, buf);
    //     Serial.println(buf);
    //     // TODO: sort out the uint64_t to uint32_t conversion
    //     uint32_t endTime = millis();
    //     delay(1000-(endTime-startTime));
    // }


}