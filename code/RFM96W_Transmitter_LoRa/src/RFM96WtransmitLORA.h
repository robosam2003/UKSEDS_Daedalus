//
// Created by robosam2003 on 03/05/2022.
//

#ifndef RFM96WTRANSMITLORA_H
#define RFM96WTRANSMITLORA_H

// include the library
#include <Arduino.h>
#include <RadioLib.h>

#define WRITE 0b10000000
#define READ 0b00000000
#define CS 37
#define lenTransmissionBytes 23 // one status byte at the start

extern elapsedMicros microTimer;

extern volatile bool receivedFlag;
// flag to indicate that a packet was sent
extern volatile bool transmittedFlag;

// disable interrupt when it's not needed
extern volatile bool enableInterrupt;
extern byte byteArr[lenTransmissionBytes];
extern int state;


// NSS pin:   37
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
extern RFM96 radio;

// prototypes
void setFlag();

void setFlagRecieve();

void SPIREGSET(byte address, byte value);

int SPIREADREG(byte address, int bytesToRead);

void RFM96WtransmitSetup();

void transmitData(byte arr[]);



void RFM96WrecieveBytesLORA();

// save transmission state between loops
extern int transmissionState;




#endif //RFM96WTRANSMITLORA_H
