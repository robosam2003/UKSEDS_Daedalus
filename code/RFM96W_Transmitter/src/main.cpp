#include <Arduino.h>
#include <SPI.h>


#define CS 10
#define MOSI 11
#define MISO 12
#define SCK 13 // LED is also on this pin
#define WRITE 0b10000000
#define READ 0b00000000

enum RMF96WRegsLoRa {
    RegFifo = 0x00,
    RegOpMode = 0x01,

    // A few FSK/OOk registers in here -

    // Common registers:
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegOcp = 0x0B,
    RegLna = 0x0C,

    // LoRa mode specific names, has other names (and purposes) in FSK/OOK mode:
    RegFifoAddrPtr = 0x0D,
    RegFifoTxBaseAddr = 0x0E,
    RegFifoRxBaseAddr = 0x0F,
    FifoRxCurrentAddr = 0x10,
    RegIrqFlagsMask = 0x11,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegRxHeaderCntValueMsb = 0x14,
    RegRxHeaderCntValueLsb = 0x15,
    RegRxPacketCntValueMsb = 0x16,
    RegRxPacketCntValueLsb = 0x17




};

void setup() {
    pinMode(CS, OUTPUT); // CS is pulled low when selected
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);

    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

}

void loop() {

}

void RFM96WInit() {

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