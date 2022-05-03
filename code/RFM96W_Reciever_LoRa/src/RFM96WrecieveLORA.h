//
// Created by robos on 03/05/2022.
//

#ifndef RFM96WRECIEVELORA_H
#define RFM96WRECIEVELORA_H
// include the library
#include <Arduino.h>
#include <RadioLib.h>


#define WRITE 0b10000000
#define READ 0b00000000
#define CS 37

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(10, 2, 9, 3);


// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;


// prototypes
void setFlag();
void SPIREGSET(byte address, byte value);
int SPIREADREG(byte address, int bytesToRead);
int state;
void RFM96WrecieveLORASetup();
void RFM96WrecieveBytesLORA();
byte byteArr[255];


#endif //RFM96WRECIEVELORA_H
