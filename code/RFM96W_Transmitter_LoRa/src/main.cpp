
// include the library
#include <RFM96WtransmitLORA.h>

void setup() {
    Serial.begin(9600);
    RFM96WtransmitSetup();

}

int counter = 0;
byte arr[lenTransmissionBytes] = {};
void loop() {
    for (int i=0;i<lenTransmissionBytes;i++) { arr[i] = counter; }
    unsigned long a = micros();
    while(!transmittedFlag);

    transmitData(arr);
    unsigned long b = micros();
    if (Serial) {Serial.printf("Transmission (LoRa) took %d (us)\n", b-a);}

    counter++;
}