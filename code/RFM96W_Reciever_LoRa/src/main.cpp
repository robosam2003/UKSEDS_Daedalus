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

// include the library
#include <Arduino.h>
#include <RFM96WrecieveLORA.h>


void setup() {
    Serial.begin(500000);
    RFM96WrecieveLORASetup();

}


void loop() {
    while (!receivedFlag);
    RFM96WrecieveBytesLORA();
    Serial.println(byteArr[50]);
    Serial.println();
}