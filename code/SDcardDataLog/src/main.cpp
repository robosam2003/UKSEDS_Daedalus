// code to test SD card capabilities of the Teensy 4.1
// BUILTIN_SDCARD (254 (serial)) is the pin for the built in SD card
// Samuel Scott 12/01/2022

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>


const int chipSelect = BUILTIN_SDCARD;
elapsedMicros timer1;

//#define button1 41

//prototypes
void logData();


void setup() {
// write your initialization code here
    delay(3000);
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    //pinMode(button1, INPUT);
    while (!Serial);
    //attachInterrupt(digitalPinToInterrupt(button1), logData, RISING);

    // SD card initialisation
    if (!SD.begin(chipSelect)) {
        Serial.println("Could not mount SD card");
        digitalWrite(LED_BUILTIN, HIGH);
        while (true);
    }
    Serial.println("Card initialised");
}


void logData() {
    int start = timer1;
    //SD.mkdir("a"); // this creates all subdirectories as well
    int madeADir = timer1;
    File dataFile = SD.open("a/testFile3.txt", FILE_WRITE);
    int openedFile = timer1;
    int wrote20Bytes = 0;
    int closeFile = 0;
    if (dataFile) {

        dataFile.println("THIS IS 20 BYTES.... now with flushing hehe");
        wrote20Bytes = timer1;
        dataFile.flush();
        //dataFile.close();
        closeFile = timer1;

        //Serial.println(timer1);

    }
    else{
        Serial.println("Some error has occured");
    }
    Serial.printf("To make a DIR:      %d: \n", madeADir-start);
    Serial.printf("To open the file:   %d: \n", openedFile-madeADir);
    Serial.printf("To write 20 bytes:  %d: \n", wrote20Bytes - openedFile);
    Serial.printf("To flush the file:  %d: \n", closeFile- wrote20Bytes);
    Serial.printf("TOTAL TIME :        %d: \n\n", closeFile-start);
}

void loop() {

    logData();
    delay(3000);




}