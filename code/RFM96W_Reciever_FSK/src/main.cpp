#include <Arduino.h>
#include <RadioLib.h>

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(10, 2, 9, 3);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 fsk = RadioShield.ModuleA;

void setup() {
    Serial.begin(9600);

    // initialize RFM96W FSK modem with default settings
    Serial.print(F("[RFM96W] Initializing ... "));
    int state = radio.beginFSK(434.0, 100, 10.0, 250, 10, 8, false);



    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // if needed, you can switch between LoRa and FSK modes
    //
    // radio.begin()       start LoRa mode (and disable FSK)
    // radio.beginFSK()    start FSK mode (and disable LoRa)

    // the following settings can also
    // be modified at run-time
    state = radio.setFrequency(433.5);
    state = radio.setBitRate(100.0);
    state = radio.setFrequencyDeviation(10.0);
    state = radio.setRxBandwidth(250.0);
    state = radio.setOutputPower(10.0);
    state = radio.setCurrentLimit(100);
    state = radio.setDataShaping(RADIOLIB_SHAPING_0_5);

    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                          0x89, 0xAB, 0xCD, 0xEF};
    state = radio.setSyncWord(syncWord, 8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Unable to set configuration, code "));
        Serial.println(state);
        while (true);
    }
    radio.packetMode();

    // FSK modulation can be changed to OOK
    // NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
    //       Also, data shaping changes from Gaussian filter to
    //       simple filter with cutoff frequency. Make sure to call
    //       setDataShapingOOK() to set the correct shaping!
    /*
    state = radio.setOOK(true);
    state = radio.setDataShapingOOK(1);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Unable to change modulation, code "));
        Serial.println(state);
        while (true);
    }

#warning "This sketch is just an API guide! Read the note at line 6."
     */
}

void loop() {


    // receive FSK packet
    String str;
    radio.startReceive(63, RADIOLIB_SX127X_RXCONTINUOUS);
    int state = radio.readData(str, 63);

    /*
      byte byteArr[8];
      int state = radio.receive(byteArr, 8);
    */
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("[SX1278] Received packet!"));
        Serial.print(F("[SX1278] Data:\t"));
        Serial.println(str);
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.println(F("[SX1278] Timed out while waiting for packet!"));
    } else {
        Serial.println(F("[SX1278] Failed to receive packet, code "));
        Serial.println(state);
    }
}
