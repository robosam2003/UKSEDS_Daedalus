// code to test SD card capabilities of the Teensy 4.1
// BUILTIN_SDCARD (254 (serial)) is the pin for the built in SD card
// Samuel Scott 12/01/2022

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>


const int chipSelect = BUILTIN_SDCARD;
elapsedMicros timer1;

struct dataStruct {
    uint16_t acc_rawx;
    uint16_t acc_rawy;
    uint16_t acc_rawz;
};


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

File SDCardInit(){
    /// use O_CREAT | O_WRITE for faster write times :)
    File dataFile = SD.open("dataLog1.dat", O_CREAT | O_WRITE);
    return dataFile;
}

void logData(File dataFile, struct dataStruct data) {  /// SDFat has a 512 byte buffer, so we only need to flush after 512 bytes have been written to the buffer
    if (dataFile) {
        dataFile.write((const uint8_t *)&data, sizeof(data));
    }
    else{
        Serial.println("Some error has occured");
    }
}

void loop() {
    struct dataStruct mydata = {0,0,0};
    File dataFile = SDCardInit();
    for (int i=0;i<10;i++) {
        mydata.acc_rawx = i;
        mydata.acc_rawy = i+1;
        mydata.acc_rawz = i+2;
        unsigned long a = micros();
        logData(dataFile, mydata);
        unsigned long b = micros();
        Serial.println(b-a);
    }
    unsigned long beforeFlush = micros();
    dataFile.flush();
    unsigned long afterFlush = micros();
    Serial.printf("To flush 512 bytes it takes: %d\n", afterFlush-beforeFlush);
    dataFile.close();
    delay(3000);




}