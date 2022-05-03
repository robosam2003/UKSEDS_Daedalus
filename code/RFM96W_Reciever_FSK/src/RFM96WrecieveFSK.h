//
// Created by robos on 03/05/2022.
//

#ifndef RFM96WRECIEVEFSK_H
#define RFM96WRECIEVEFSK_H

// include the library
#include <Arduino.h>
#include <RadioLib.h>
/// FSK

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;



// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(10, 2, 9, 3);




void RFM96recieveFSKSetup();

void setFlag();

void RFM96WrecieveBytesFSK();


#endif //RFM96WRECIEVEFSK_H
