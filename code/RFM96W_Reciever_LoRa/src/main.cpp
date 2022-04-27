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
#include <RadioLib.h>


#define WRITE 0b10000000
#define READ 0b00000000
#define CS 37

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(10, 2, 9, 3);

// prototypes
void setFlag();
void SPIREGSET(byte address, byte value);
int SPIREADREG(byte address, int bytesToRead);
int state;
// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

void setup() {
    Serial.begin(500000);

    // initialize SX1278 with default settings
    Serial.print(F("[RFM96W] Initializing ... "));
    int state = radio.begin(434.0, 500, 6, 5, RADIOLIB_SX127X_SYNC_WORD_LORAWAN, 17, 8, 0);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // set the function that will be called
    // when new packet is received
    radio.setDio0Action(setFlag);


    // spreading factor 6
    radio.setSpreadingFactor(6);

    radio.implicitHeader(255);
    byte x31Reg = SPIREADREG(0x31, 1);
    // write to the last 3 bits of register 0x31
    x31Reg &= 0b11111101;
    SPIREGSET(0x31, x31Reg);

    SPIREGSET(0x37, 0x0C);

    // start listening for LoRa packets
    Serial.print(F("[RFM96W] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // if needed, 'listen' mode can be disabled by calling
    // any of the following methods:
    //
    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.readData();
    // radio.scanChannel();
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag() {
    // check if the interrupt is enabled
    if(!enableInterrupt) {
        return;
    }

    // we got a packet, set the flag
    receivedFlag = true;
}


byte byteArr[255];
void loop() {
    // check if the flag is set
    if(receivedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        receivedFlag = false;

        // you can read received data as an Arduino String
        //String str;
        //int state = radio.readData(str);

        // you can also read received data as byte array


        state = radio.readData(byteArr, 255);



        if (state == RADIOLIB_ERR_NONE) {
            // packet was successfully received
            //Serial.println(F("[RFM96W] Received packet!"));

            // print data of the packet
            //Serial.print(F("[RFM96W] Data:\t\t\n"));
            //for (auto x : byteArr) { Serial.print(x); Serial.print(", ");}
            Serial.println();
            Serial.println(byteArr[50]);

            // print RSSI (Received Signal Strength Indicator)
            //Serial.print(F("[RFM96W] RSSI:\t\t"));
            //Serial.print(radio.getRSSI());
            //Serial.println(F(" dBm"));

            // print SNR (Signal-to-Noise Ratio)
            //Serial.print(F("[RFM96W] SNR:\t\t"));
            //Serial.print(radio.getSNR());
            //Serial.println(F(" dB"));

            // print frequency error
            //Serial.print(F("[RFM96W] Frequency error:\t"));
            //Serial.print(radio.getFrequencyError());
            //Serial.println(F(" Hz"));

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("[RFM96W] CRC error!"));

        } else {
            // some other error occurred
            Serial.print(F("[RFM96W] Failed, code "));
            Serial.println(state);

        }

        // put module back to listen mode
        state = radio.startReceive();

        // we're ready to receive more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }

}

int SPIREADREG(byte address, int bytesToRead){  // FIFO
    address = READ | address; // puts 0 int the 8th bit.
    byte inByte = 0;
    int result = 0;
    digitalWrite(CS, LOW); // begin transfer
    for (int i=0; i<bytesToRead; i++) {
        result = result << 8;
        inByte = SPI.transfer(0x00);  // transfers 0x00 over MOSI line, recieves a byte over MISO line.
        result = result | inByte;
    }
    digitalWrite(CS, HIGH); // end transfer
    return result;
}

void SPIREGSET(byte address, byte value) {
    address = WRITE | address; //
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer

}