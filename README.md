# 10S BLE smart BMS
A diy charge only bms for a 10S battery pack.
Cell monitoring and balancing using a BQ76930.
Main MCU is an ESP32-C3 that sends the data from the BQ76930 via BLE to an app.
The PCB was designed in Altium Designer.\
This PCB is meant for a diy E-skate, that's why it has some extra functions.
There is a header for connecting servo signals and a mosfet to switch 12V leds. The leds are supposed to light up when braking.
The programming circuitry for the ESP32-C3 is on an external board to save space.\
There is also a power latch circuit with a button and the mcu can turn power to itself off.\
The firmware was made in ESP-IDF 5.1.3

## Problem: The charge fet doesn't seem to turn off when it should!

## Cloning
Clone with this command to also get the bq76930 driver component
```
git clone https://github.com/TimBilliet/10s-bq76930-smart-bms.git --recurse-submodules
```
![image](https://github.com/TimBilliet/10s-bq76930-smart-bms/assets/47719114/006d26ed-25e1-4275-9ca6-337fc7d32524)

## App
https://github.com/TimBilliet/smart-bms-app

## DONE:
Testing led mosfet (works, so does pwm dimming)\
Writing BQ76930 esp-idf driver\
Write firmware\
Implement OTA

## TODO:
Write app(need to implement ota)\
Fix charge FET not turning off

