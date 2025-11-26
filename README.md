# Indoor air quality monitoring system

## Team & Contribution
* X Ritschel (xxx)
* Ivan Pavlov (Particulate sensor)
* Matúš Repáš (Pressure sensor)
* Anton Panteleev (Digital Temperature and Humidity Sensor, GitHub management)

## Project Overview


## Objectives


## Components Used

### 1. Arduino UNO

* Central controller for processing data from sensors and managing outputs.

  <img src="images/arduino_uno.png" alt="Arduino Uno" width="450" height="500">
### 2. Sensors:

* Sharp GP2Y1010AU0F (Optical Dust Sensor)

  <img src="images/sharp.png" alt="Sharp" width="450" height="500">

* MQ-5 (Gas Sensor)

<img src="images/mq-5.png" alt="MQ-5" width="450" height="500">

* DHT12 (Digital Temperature and Humidity Sensor)

<img src="images/dht12.png" alt="DHT12" width="450" height="500">

### 3. Output Devices:

* OLED Display

<img src="images/oled.png" alt="OLED" width="450" height="500">

## System Block Diagram

<img src="images/block_diagram.png" alt="BLOCK">

## How It Works



## Software Description

The main.c file is written for a microcontroller and handles communication with several connected sensors and modules. It utilizes libraries for I2C (twi.h), OLED display control (oled.h), and timer management (timer.h).

Key functionalities include:

* Reading sensor's data

* Dislapying information on an OLED display

## Possible Improvements


## References and tools
