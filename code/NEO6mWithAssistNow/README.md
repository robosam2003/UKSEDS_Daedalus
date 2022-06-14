# NEO-6M with AssistNow loading

### Links:
- [Datasheet](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf)

- [Ublox GNSS Assistance user guide](https://content.u-blox.com/sites/default/files/products/documents/MultiGNSS-Assistance_UserGuide_%28UBX-13004360%29.pdf)

- To get started with using Ublox servers for assisted GPS, see [assistnow-getting-started-guide](https://developer.thingstream.io/guides/location-services/assistnow-getting-started-guide).

#

These modules use the UBX protocol to communicate with the GPS, in this case, over a UART connection.

There is a lot of configuration that can be done with these modules, but the important ones are demonstrated in `NEO6mConfig()`. The message classes and types are outlined in `assistNow.h`.

The main advantage of setting up the GPS in this way is that it allows for configuration using **assisted GPS** - called **AssistNow** for Ublox GPS modules - which allows for ultrafast GPS satellite lock - a serious problem for many GPS modules, especially cheap ones like the NEO6m.

The UBX protocol structure is as follows:

<img src="https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/code/NEO6mWithAssistNow/resources/uxbPacketStructure.jpg">



The CFG messages set up the GPS for use, and the other message classes are used for things like aiding, acknowledgements, and getting raw data. Note that the CFG messages will send a ACK(acknowledged) or NAK(not acknowledged) message back to the GPS for CFG messages, the `sendUBX()` function deals with this. See the datasheet for more information.


## Usage

If a message is set up with *periodic polling*, it will be sent every `measrate` milliseconds:
```cpp
    delay(measrate);
    getGPSData(); // For if the NAV-POSLLH message is set up to be periodic
```

Otherwise, sending an empty payload will 'poll' the GPS for the message:
```cpp
    sendUbx(NAV, NAV_POSLLH, 0, nullptr); // Sending

    delay(measrate);  // GPS is confifured for 5Hz update rate
    
    getGPSData(); // For NAV-POSLLH message
```

## AssistNow loading
To perform AssistNow loading, first get the `mgaonline.ubx` file from the ublox server (see [`assistnow-getting-started-guide`](https://developer.thingstream.io/guides/location-services/assistnow-getting-started-guide)), load it an SD card and insert the SD card into the teensy.

Ensure you *compile* the program such that the teensy RTC syncs and the `getTimestampMillis()` function works as expected - **An accurate timestamp is vital to get a quick satellite lock**

The `performOnlineAssist()` function will load the file and send it to the GPS, modifying the timestamp field. If the Almanac and Ephemeris are valid, then this will greatly improve GPS lock time.
