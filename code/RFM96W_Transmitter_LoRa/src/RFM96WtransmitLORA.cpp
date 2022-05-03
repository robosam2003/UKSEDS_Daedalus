//
// Created by robos on 03/05/2022.
//
#include "RFM96WtransmitLORA.h"

void RFM96WtransmitSetup() { // assumes serial is setup.
    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.begin(434.0, 500, 6, 5, RADIOLIB_SX127X_SYNC_WORD_LORAWAN, 17, 8, 0);
    if (state == RADIOLIB_ERR_NONE) {
        if (Serial) { Serial.println(F("success!"));}
    } else {
        if (Serial) { Serial.print(F("failed, code "));}
        if (Serial) { Serial.println(state);}
        while (true);
    }
    radio.setDio0Action(setFlag);

    // spreading factor 6
    radio.setSpreadingFactor(6);

    radio.implicitHeader(255);
    byte x31Reg = SPIREADREG(0x31, 1);
    // write to the last 3 bits of register 0x31
    x31Reg &= 0b11111101;
    SPIREGSET(0x31, x31Reg);

    SPIREGSET(0x37, 0x0C);



    // set the function that will be called
    // when packet transmission is finished


    // start transmitting the first packet
    if (Serial) { Serial.print(F("[RFM96W] Sending first packet ... ")); }

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    transmissionState = radio.startTransmit("Hello World!");
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

void setFlag() {
    // check if the interrupt is enabled
    if(!enableInterrupt) {
        return;
    }
    // we sent a packet, set the flag
    transmittedFlag = true;
}

void transmitData(byte arr[]) {
    // check if the previous transmission finished
    while (!transmittedFlag);
    if(transmittedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        transmittedFlag = false;

        if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            if (Serial) { Serial.println(F("transmission finished!"));}

            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()

        } else {
            if (Serial) { Serial.print(F("failed, code "));}
            if (Serial) { Serial.println(transmissionState);}

        }

        // NOTE: in FSK mode, SX127x will not automatically
        //       turn transmitter off after sending a packet
        //       set mode to standby to ensure we don't jam others
        //radio.standby()

        // wait a second before transmitting again

        // send another one
        if (Serial) { Serial.print(F("[RFM96W] Sending another packet ... "));}

        // you can transmit C-string or Arduino string up to
        // 256 characters long
        //transmissionState = radio.startTransmit("Hello World! THIS IS SAM");

        // you can also transmit byte array up to 256 bytes long

        byte byteArr[255] = {};
        for (int i=0; i<255; i++) { byteArr[i] = arr[i]; }


        int state = radio.startTransmit(byteArr, 255);

        // we're ready to send more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
}