//
// Created by robosam2003 on 03/05/2022.
//

#ifndef RFM96WTRANSMITFSK_H
#define RFM96WTRANSMITFSK_H

#include <Arduino.h>
#include <RadioLib.h>

// RFM96W has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(37, 2, 9, 3);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;


void RFM96WtransmitFSKsetup();

void setFlag();

void transmitData(byte arr[]);


#endif //RFM96WTRANSMITFSK_H
