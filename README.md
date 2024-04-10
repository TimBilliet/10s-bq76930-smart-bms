# diy-10s-smart-bms
A diy charge only bms for a 10S battery pack.
Cell monitoring and balancing using a BQ76930.
Main MCU is an ESP32-C3 that sends the data from the BQ76930 via BLE to an app.
This PCB is meant for a diy E-skate, that's why it has some extra functions.
There is a header for connecting servo signals and a mosfet to switch 12V leds. The leds are supposed to light up when braking.

## DONE:
Testing the cell voltage reading function(tested with existing Arduino library)\
Testing charge fet cutoff when overvoltage occurs(tested with existing Arduino library)\
Testing led mosfet (works, so does pwm dimming)

## TODO:
Write app\
Write BQ76930 esp-idf driver\
Write firmware
