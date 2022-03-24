/*
   RadioLib SX127x Transmit Example

   This example transmits packets using SX1278 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <Arduino.h>
#include <RadioLib.h>

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(10, 2, 9, 3);
byte counter = 20;

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

void setup() {
    Serial.begin(9600);

    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // some modules have an external RF switch
    // controlled via two pins (RX enable, TX enable)
    // to enable automatic control of the switch,
    // call the following method
    // RX enable:   4
    // TX enable:   5
    /*
      radio.setRfSwitchPins(4, 5);
    */
}

void loop() {
    Serial.print(F("[SX1278] Transmitting packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    // NOTE: transmit() is a blocking method!
    //       See example SX127x_Transmit_Interrupt for details
    //       on non-blocking transmission method.
    int state = radio.transmit("Hello World!");

    // you can also transmit byte array up to 256 bytes long
   //byte byteArr[] = {};
    //for (int i=0; i<4;i++) {byteArr[i] = counter<<(8*(3-i));}
    //int state = radio.transmit(byteArr, 4);


    if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));

        // print measured data rate
        Serial.print(F("[SX1278] Datarate:\t"));
        Serial.print(radio.getDataRate());
        Serial.println(F(" bps"));

    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        Serial.println(F("too long!"));

    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        // timeout occurred while transmitting packet
        Serial.println(F("timeout!"));

    } else {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);

    }

    // wait for a second before transmitting again
    delay(1000);
    counter++;
}




/*#include <Arduino.h>
#include <SPI.h>


#define CS 10
#define MOSI 11
#define MISO 12
#define SCK 13 // LED is also on this pin
#define WRITE 0b10000000
#define READ 0b00000000

enum RMF96WRegsLoRa {
    RegFifo = 0x00,
    RegOpMode = 0x01,

    // A few FSK/OOk registers in here -

    // Common registers:
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegOcp = 0x0B,
    RegLna = 0x0C,

    // LoRa mode specific names, has other names (and purposes) in FSK/OOK mode:
    RegFifoAddrPtr = 0x0D,
    RegFifoTxBaseAddr = 0x0E,
    RegFifoRxBaseAddr = 0x0F,
    FifoRxCurrentAddr = 0x10,
    RegIrqFlagsMask = 0x11,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegRxHeaderCntValueMsb = 0x14,
    RegRxHeaderCntValueLsb = 0x15,
    RegRxPacketCntValueMsb = 0x16,
    RegRxPacketCntValueLsb = 0x17




};

void setup() {
    pinMode(CS, OUTPUT); // CS is pulled low when selected
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);

    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

}



void RFM96WInit() {

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


void loop() {

}
*/ // my code
