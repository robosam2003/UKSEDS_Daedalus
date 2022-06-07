//
// Created by robos on 03/05/2022.
//
#include "RFM96WtransmitLORA.h"



// init externs
elapsedMicros microTimer;
volatile bool receivedFlag = false; // flag to indicate that a packet was received
volatile bool transmittedFlag = false; // flag to indicate that a packet was sent
volatile bool enableInterrupt = true; 
int transmissionState = RADIOLIB_ERR_NONE;
RFM96 radio  = new Module(CS, 2, 9, 3);

byte byteArr[lenTransmissionBytes] = {};
int state = 0;

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

void setFlagRecieve() {
    // check if the interrupt is enabled
    if(!enableInterrupt) {
        return;
    }
    // we got a packet, set the flag
    receivedFlag = true;
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
    if (Serial) { Serial.print(F("[RFM96W] Sending first packet ...\n ")); }
    byteArr[0] = 0xAA; // Test code
    state = radio.startTransmit(byteArr, lenTransmissionBytes);
    delay(1000);
    
}

void RFM96WrecieveBytesLORA() {
    // check if the flag is set
    if(receivedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        receivedFlag = false;


        state = radio.readData(byteArr, lenTransmissionBytes);

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