//
// Created by robos on 03/05/2022.
//
#include "RFM96WtransmitLORA.h"



// init externs
elapsedMicros microTimer;
volatile bool transmittedFlag = false;
volatile bool enableInterrupt = true;
int transmissionState = RADIOLIB_ERR_NONE;
RFM96 radio  = new Module(CS, 2, 9, 3);

void RFM96WtransmitSetup() { // assumes serial is setup.
    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.begin(434.0, 500, 8, 5, RADIOLIB_SX127X_SYNC_WORD_LORAWAN, 10, 8, 0);
    if (state == RADIOLIB_ERR_NONE) {
        if (Serial) { Serial.println(F("success!"));}
    } else {
        if (Serial) { Serial.print(F("failed, code "));}
        if (Serial) { Serial.println(state);}
        while (true);
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setDio0Action(setFlag); // interrupt on DIO0

    // spreading factor 6
//    radio.setSpreadingFactor(6);
//
//    radio.implicitHeader(255);
//    byte x31Reg = SPIREADREG(0x31, 1);
//    // write to the last 3 bits of register 0x31
//    x31Reg &= 0b11111101;
//    SPIREGSET(0x31, x31Reg);
//
//    SPIREGSET(0x37, 0x0C);





    // start transmitting the first packet
    if (Serial) { Serial.print(F("[RFM96W] Sending first packet ... ")); }

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    char str[] = {0,0,0,0,0,0,0,0};
    transmissionState = radio.startTransmit(str);
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
    address = WRITE | address;
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
    // This should be called after checking that transmittedFlag is true


    if(transmittedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        transmittedFlag = false;

        if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            //if (Serial) { Serial.println(F("transmission finished!"));}

            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()

        } else {
            if (Serial) { Serial.print(F("failed, code "));}
            if (Serial) { Serial.println(transmissionState);}

        }
        /// transmit lenTransmissionBytes bytes of data from arr[]
        int state = radio.startTransmit(arr, lenTransmissionBytes);

        // we're ready to send more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
    else{

    }
}