# UKSEDS Team Daedalus - "In thrust we trust"

The rocket avionics consisted of a Teensy 4.1 for the main microcontoller, a low-g IMU (BNO055), a high-g accelerometer (ADXL377), a pressure sensor (BMP280), a GPS (NEO-6M) and a 433 MHz radio transciever (RFM96W).

The purpose of the rocket was to collect acceleration and gyroscope data such that we could test our IMU based position estimation algorithm (also called dead reckoning).

# Code
The `code` folder contains code for all the sensors and telemetry onboard and for the ground station. The `masterCode` folder contains the final code loaded onto the rocket.

## Sensors and telemetry:
- **BNO055**: Our low-g IMU, containing accelerometers capable of ±16G, as well as very accurate gyroscopes and magnetometers. Big thanks to SunrIde's [Tom Danvers](https://github.com/TomD53) for writing the library we used to interface with the BNO055: [`Arduino-BNO055`](https://github.com/TeamSunride/Arduino-BNO055). 
- **ADXL377**: Our high-g IMU, capable of up to ±200G. This is an analogue sensor and is *very noisy*, so both hardware filters (capacitors) and software filters ([Kalman filters](https://github.com/denyssene/SimpleKalmanFilter)) were used to get accurate data. 
- **BMP280**: A pressure based altimeter used in conjunction with the brilliant [Adafruit library](https://github.com/adafruit/Adafruit_BMP280_Library).
- **NEO-6M**: A cheap but capable GPS module from ublox. In order to get a fast GPS lock, we implemented assisted GPS loading using the ublox AssistNow service. See [`code/NEO6mWithAssistNow`](https://github.com/robosam2003/UKSEDS_Daedalus/tree/main/code/NEO6mWithAssistNow).
- **RFM96W**: A 433MHz SX1272 based tranciever module that was used both onboard and on our ground station. We used the LoRa protocol for greater range. We implemented an interrupt driven method using [`RadioLib`](https://github.com/jgromes/RadioLib).

## Ground station
The ground station used an old arduino nano with the RFM96W to recieve live telemetrics from the rocket. The launch commit button was used to put the rocket into a launch ready state remotely.

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/groundStation.jpg" width = 800>

It passed this data to Grafana and InfluxDB using the [line protocol](https://docs.influxdata.com/influxdb/cloud/reference/syntax/line-protocol/). Big thanks to SunrIde's [Yomi](https://github.com/abayomi185), [Tom](https://github.com/TomD53) and [Sebastiano](https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/grafanaDashboard.jpg) for building the software to do this [`TeamSunride/GroundStation`](https://github.com/TeamSunride/GroundStation). Our live telemetry Grafana Dashboard is shown below:

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/grafanaDashboard.jpg">



### The rocket launch sequence is as follows:
- Ensure ground station is booted and ready.
- **Compile** masterCode/main.cpp and upload to teensy so that the RTC syncs.
- Setup sensors, datalogging and telemety on the ground.
- Full system test with serial output to double check sensor values look reasonable.
- Load avionics into rocket.
- Wait for "Launch commit" from ground station.
- When rocket is loaded on the launch rail, press launch commit button on ground station (wait for confirmation - ground station light should go green).
- Start data acquisition, datalogging and transmitting data.
- Launch!


Loading avionics:

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/loadingAvionics.jpg" width = 600>


# Circuitry
This rocket used breakout boards to house components. This made testing and verification easier before ordering the final PCB. The PCB was designed in EAGLE using many custom libraries. 

The final schematic:

[`Rocketry_circuit.sch`](https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/circuitry/Rocketry_circuit.sch):

<img src = https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/schProduction.png >


The final pcb board design:

[`Rocketry_circuit.brd`](https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/circuitry/Rocketry_circuit.brd):

<img src = https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/brdProduction.png>


The manufactured PCB (from china):   

<img src = https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/nakedBoard1.jpg width = 400> <img src = https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/nakedBoard2.jpg width = 400>

The assembled pcb with breakouts soldered and mounted:

<img src = https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/finalPcbFront.jpg width = 900>

<img src = https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/finalPcbBack.jpg width = 900>

The avionics recovered after flight:

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/afterRecovery.jpg">

# Flight data

The launch was a ~success!~... it *went up* anyway. We suspect excess friction on the launch rail caused it to fly off course at about a 45° angle. (See https://www.youtube.com/watch?v=rMJzLpDIJ3U)

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/_onTheLaunchRail.jpg" height = 600>

The rocket still achieved an apogee of 641m according to the BMP280 altimeter (note that the spike at apogee caused by the pressure from the ejection charge). The GPS lost satellite lock on launch as predicted, but regrained lock after apogee:

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/altitudeGraph.jpg">

The acceleration data followed our expectations from the OpenRocket simulation, with a slightly lower max acceleration of 14.4g. Unfortunately, the BNO055 maxed out at 4G, rather than the desired 16g, despite having configured it to read up to 16g. The ADXL377 data is decent though: (note the spike at ejection charge)
<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/accelerationGraph.jpg">

The onboard dead reckoning algorithm also didn't work as expected, due to the BNO055 maxing out earlier than anticipated and due to the initial launch being so oscillatory (which threw off orientation estimation) the data is not an accurate measurement of position. Much has been learned from this launch and will be refined in future rockets. :)

<img src = "https://github.com/robosam2003/UKSEDS_Daedalus/blob/main/resources/DRZplot.jpg">

The **telemetry** range was not very good, likely due to bad antennas, and because of UK regulations which limited our transmit power down to 10dbm at a max duty cycle of 10%. :(

