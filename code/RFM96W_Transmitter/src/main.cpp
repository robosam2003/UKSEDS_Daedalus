
// include the library
#include <Arduino.h>
#include <RadioLib.h>


elapsedMicros microTimer;


// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
RFM96 radio = new Module(10, 2, 9, 3);

void setFlag();

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

void setup() {
    Serial.begin(9600);

    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.begin(434.0, 500, 9, 7, RADIOLIB_SX127X_SYNC_WORD_LORAWAN, 17, 8, 0);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setDio0Action(setFlag);

    // start transmitting the first packet
    //Serial.print(F("[SX1278] Sending first packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    //transmissionState = radio.startTransmit("Hello World!");

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      state = radio.startTransmit(byteArr, 8);
    */
}

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag() {
    // check if the interrupt is enabled
    if(!enableInterrupt) {
        return;
    }

    // we sent a packet, set the flag
    transmittedFlag = true;
}

void loop() {
    // check if the previous transmission finished
    if(transmittedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        transmittedFlag = false;

        if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));

            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()

        } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);

        }

        // NOTE: in FSK mode, SX127x will not automatically
        //       turn transmitter off after sending a packet
        //       set mode to standby to ensure we don't jam others
        //radio.standby()

        // wait a second before transmitting again
        delay(100);

        // send another one
        Serial.print(F("[RFM96W] Sending another packet ... "));

        // you can transmit C-string or Arduino string up to
        // 256 characters long
        //transmissionState = radio.startTransmit("Hello World! THIS IS SAM");

        // you can also transmit byte array up to 256 bytes long

        byte byteArr[255] = {};
        for (int i=0; i<255; i++) { byteArr[i] = i; }

        unsigned long a = microTimer;
        int state = radio.startTransmit(byteArr, 255); //
        unsigned long b = microTimer;
        Serial.printf("Transmission took (us):  %d\t", b-a);

        // we're ready to send more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
}