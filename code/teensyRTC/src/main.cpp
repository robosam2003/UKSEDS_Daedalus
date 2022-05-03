#include <TimeLib.h>
#include <Arduino.h>



time_t getTeensyTime() {
    return Teensy3Clock.get();
}

void rtcSetup()  { // assumes Teensy3Clock is already setup and Serial is running.
    // set the Time library to use Teensy 3.0's RTC to keep time
    setSyncProvider(getTeensyTime);
    Serial.begin(115200);
    while (!Serial);  // Wait for Arduino Serial Monitor to open
    delay(100);
    if (timeStatus()!= timeSet) {
        Serial.println("Unable to sync with the RTC");
    } else {
        Serial.println("RTC has set the system time");
    }
    // Usually about 2 seconds difference from compile time to runtime. Set the RTC to + 2 seconds to compensate.
    setTime(hour(), minute(), second() + 2, day(), month(), year());
}

void setup()  { // assumes Teensy3Clock is already setup and Serial is running.
    rtcSetup();
}

void loop() {
}








/*#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial
int RXPin = 7;
int TXPin = 8;
#define transistorPin 32

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void displayInfo();
void setup()
{
    delay(10000);
    // Start the Arduino hardware serial port at 9600 baud
    Serial.begin(9600);
    pinMode(transistorPin, OUTPUT);
    digitalWrite(transistorPin, HIGH);

    // Start the software serial port at the GPS's default baud
    gpsSerial.begin(GPSBaud);
}

void loop()
{

    digitalWrite(transistorPin, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    // This sketch displays information every time a new sentence is correctly encoded.
    while (gpsSerial.available() > 0)
        if (gps.encode(gpsSerial.read()))
            displayInfo();

    // If 5000 milliseconds pass and there are no characters coming in
    // over the software serial port, show a "No GPS detected" error
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println("No GPS detected");
    }
}

void displayInfo()
{
    if (gps.location.isValid())
    {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Altitude: ");
        Serial.println(gps.altitude.meters());
    }
    else
    {
        Serial.println("Location: Not Available");
    }

    Serial.print("Date: ");
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.println(gps.date.year());
    }
    else
    {
        Serial.println("Not Available");
    }

    Serial.print("Time: ");
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(":");
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(":");
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(".");
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.println(gps.time.centisecond());
    }
    else
    {
        Serial.println("Not Available");
    }
    Serial.println();
    Serial.println();
    delay(1000);
}*/




/*

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
void setup() {
    Serial.begin(9600);
    while ( !Serial ) delay(100);
    Serial.println(F("BMP280 test"));
    unsigned status;
    status = bmp.begin();
    if (!status) {
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }


    */
/* Default settings from datasheet. *//*


    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_500);
}

void loop() {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    float altitude = bmp.readAltitude(1031);
    byte altitude1 = altitude && (0xFF << 24);
    byte altitude2 = altitude && (0xFF << 16);
    byte altitude3 = altitude && (0xFF << 8);
    byte altitude4 = altitude && (0xFF << 0);
    Serial.println("m");
    Serial.println("Ethan is dumb");
    Serial.println();
    delay(10);

}
*/

