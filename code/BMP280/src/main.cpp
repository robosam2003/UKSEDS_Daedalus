
#include "BMP280.h"

void setup() {
    Serial.begin(9600);
    BMP280Setup();
}

void loop() {
    double seaLevelPressure = 1010;
    getBMP280Data(seaLevelPressure);

    //Serial.printf("measurement took %d (us)\n", end-start);
    Serial.printf("Altitude: %lf\n", bmpData.altitude);

    Serial.println();
    delay(10);

}