//
// Created by robos on 03/05/2022.
//

#ifndef RFM96WRECIEVELORA_H
#define RFM96WRECIEVELORA_H
// include the library
#include <Arduino.h>
#include <RadioLib.h>
#define lenReceiveBytes 255


#define WRITE 0b10000000
#define READ 0b00000000
#define CS 37

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
extern RFM96 radio;

// flag to indicate that a packet was received
extern volatile bool receivedFlag;

// disable interrupt when it's not needed
extern volatile bool enableInterrupt;

extern int state;

extern byte byteArr[255];
// prototypes
void setFlag();

void SPIREGSET(byte address, byte value);

int SPIREADREG(byte address, int bytesToRead);


void RFM96WrecieveLORASetup();

void RFM96WrecieveBytesLORA();





#endif //RFM96WRECIEVELORA_H
