# Indoor air quality monitoring system

## Team & Contribution
* X Ritschel (Digital barometric pressure, altitude and temperature sensor)
* Ivan Pavlov (Particulate sensor)
* Matúš Repáš (Altitude sensor)
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

* MQ-135 (Gas Sensor)

<img src="images/mq-5.png" alt="MQ-5" width="450" height="500">

* DHT12 (Digital Temperature and Humidity Sensor)

<img src="images/dht12.png" alt="DHT12" width="450" height="500">

* MPL3115A2 (Digital barometric pressure, altitude and temperature sensor)

<img src="images/altimeter.png" alt="ALTIMETER" width="450" height="500">

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

Function description:

`oled_setup()`

* Calls OLED initialization routines

* Prints static text

* Updates the dispaly buffer

`timer1_init()`

* Sets the timer prescaler and overflow period (1 sec)

* Enable Timer1 overflow interrupt

`main()`

* Initializes TWI, OLED, TIMER1

* Enable globar interrupts

* Waits for flag_update_oled

* Formats and prints temperature and humidity values to OLED

* Updates the dispaly buffer

* Infinite loop

`ISR(TIMER1_OVF_vect)`

* Counts timer overflows

* Every 5 seconds reads 5 bytes from DHT12 via I2C

* Stores them in dht12_values

* Sets flag_update_oled to trigger update in main loop

## Possible Improvements


## References and tools
