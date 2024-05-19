# 10S BLE smart BMS
A diy charge only bms for a 10S battery pack.
Cell monitoring and balancing using a BQ76930.
Main MCU is an ESP32-C3 that sends the data from the BQ76930 via BLE to an app.
The PCB was designed in Altium Designer.
This PCB is meant for a diy E-skate, that's why it has some extra functions.
There is a header for connecting servo signals and a mosfet to switch 12V leds. The leds are supposed to light up when braking.
The programming circuitry for the ESP32-C3 is on an external board to save space.  

![image](https://github.com/TimBilliet/10s-bq76930-smart-bms/assets/47719114/a311099c-05f8-4fef-ab2e-2fc20e6ad72a)

## DONE:
Testing the cell voltage reading function(tested with existing Arduino library)\
Testing charge fet cutoff when overvoltage occurs(tested with existing Arduino library)\
Testing led mosfet (works, so does pwm dimming)

## TODO:
Write app\
Write BQ76930 esp-idf driver\
Write firmware
