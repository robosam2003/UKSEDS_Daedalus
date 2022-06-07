//
// Created by robos on 03/05/2022.
//
#include "RFM96WrecieveLORA.h"


// init externs
RFM96 radio = new Module(10, 2, 9, 3);
volatile bool receivedFlag = false;
volatile bool transmittedFlag = true;
volatile bool enableInterrupt = true;
byte byteArr[lenReceiveBytes] = {};
int state = 0;

void RFM96WrecieveLORASetup() {
    Serial.print(F("[RFM96W] Initializing ... "));
    state = radio.begin(434.0, 500, 8, 5, RADIOLIB_SX127X_SYNC_WORD_LORAWAN, 10, 8, 0);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // set the function that will be called
    // when new packet is received
    radio.setDio0Action(setFlagRecieve);
    

//    // spreading factor 6
//    radio.setSpreadingFactor(6);
//
//    radio.implicitHeader(255);
//    byte x31Reg = SPIREADREG(0x31, 1);
//    // write to the last 3 bits of register 0x31
//    x31Reg &= 0b11111101;
//    SPIREGSET(0x31, x31Reg);
//
//    SPIREGSET(0x37, 0x0C);

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
}

void setFlag() {
    // check if the interrupt is enabled
    if(!enableInterrupt) {
        return;
    }
    // we sent a packet, set the flag
    transmittedFlag = true;
}

void setFlagRecieve() {
    // check if the interrupt is enabled
    if(!enableInterrupt) {
        return;
    }
    // we got a packet, set the flag
    receivedFlag = true;
}

void RFM96WrecieveBytesLORA() {
    // check if the flag is set
    if(receivedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        receivedFlag = false;


        state = radio.readData(byteArr, lenReceiveBytes);

        if (state == RADIOLIB_ERR_NONE) {
            // packet was successfully received
            //Serial.println(F("[RFM96W] Received packet!"));

            // print data of the packet
            //Serial.print(F("[RFM96W] Data:\t\t\n"));
            //for (auto x : byteArr) { Serial.print(x); Serial.print(", ");}

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
            //Serial.println(F("[RFM96W] CRC error!"));

        } else {
            // some other error occurred
            //Serial.print(F("[RFM96W] Failed, code "));
            //Serial.println(state);
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