//
// Created by robosam2003 on 03/05/2022.
//
#include "teensyRTC.h"

time_t getTeensyTime() {
    return Teensy3Clock.get();
}


void rtcSetup()  { // assumes Teensy3Clock is already setup and Serial is running.
    // set the Time library to use Teensy 3.0's RTC to keep time
    setSyncProvider(getTeensyTime);
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
