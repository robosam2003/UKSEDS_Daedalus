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
    radio.packetMode();

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
    // FSK modem can use the same transmit/receive methods
    // as the LoRa modem, even their interrupt-driven versions
    // NOTE: FSK modem maximum packet length is 63 bytes!

    // transmit FSK packet
    int state = radio.startTransmit("Hello World! This is Sam!");
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      int state = radio.transmit(byteArr, 8);
    */
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("[RFM96W] Packet transmitted successfully!"));
    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        Serial.println(F("[RFM96W] Packet too long!"));
    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        Serial.println(F("[RFM96W] Timed out while transmitting!"));
    } else {
        Serial.println(F("[RFM96W] Failed to transmit packet, code "));
        Serial.println(state);
    }

    delay(1000);


}
