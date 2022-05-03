//
// Created by robos on 03/05/2022.
//

#include "RFM96WtransmitFSK.h"

void RFM96WtransmitFSKsetup() { // assumes serial is setup
    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.beginFSK(434.0, 300, 100, 250, 17, 8, false);
    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                          0x89, 0xAB, 0xCD, 0xEF};
    state = radio.setSyncWord(syncWord, 8);

    if (state == RADIOLIB_ERR_NONE) {
        if (Serial) { Serial.println("success!");}
    } else {
        if (Serial) { Serial.printf("failed, code ");}
        if (Serial) { Serial.println(state);}
        while (true);
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setDio0Action(setFlag);

    // start transmitting the first packet
    if (Serial) { Serial.printf("[RFM96W] Sending first packet ... ");}

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    transmissionState = radio.startTransmit("Hello World!");

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      state = radio.startTransmit(byteArr, 8);
    */
}


// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!

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
    while (!transmittedFlag) {};
    if(transmittedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;
        // reset flag
        transmittedFlag = false;
        if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            if (Serial) { Serial.println("transmission finished!");}
        } else {
            if (Serial) { Serial.printf("failed, code ");}
            if (Serial) { Serial.println(transmissionState);}
        }

        /// TRANSMIT DATA

        transmissionState = radio.startTransmit(arr, 63);
//        if (state == RADIOLIB_ERR_NONE) {
//            if (Serial) { Serial.println("[RFM96W] Packet transmitted successfully!");}
//        } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
//            if (Serial) { Serial.println(F("[RFM96W] Packet too long!"));}
//        } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
//            if (Serial) { Serial.println(F("[RFM96W] Timed out while transmitting!"));}
//        } else {
//            if (Serial) { Serial.println(F("[RFM96W] Failed to transmit packet, code "));}
//            if (Serial) { Serial.println(state);}
//        }
        // we're ready to send more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
}