
#include "BMP280.h"

void setup() {
    Serial.begin(9600);
    BMP280Setup();
}
void loop() {

    int start = micros();
    double returnVect[3];
    getBMP280Data(returnVect);
    int end = micros();

    Serial.printf("measurement took %d (us)\n", end-start);
    Serial.printf("Altitude: %lf\n", returnVect[2]);

    Serial.println();
    delay(10);

}