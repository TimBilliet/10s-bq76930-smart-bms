# 10S BLE smart BMS
A "smart" charge only BMS for a 10S battery pack.\
Cell monitoring and balancing using a BQ76930 BMS chip from Texas Instruments.\
This chip is controlled by an ESP32-C3 microcontroller via I²C.\
The battery pack voltage and bms parameters are presented via Bluetooth Low Energy on the ESP32-C3\
This data can then be read and written by my Smart BMS App.

The PCB was designed in Altium Designer.\
The programming circuitry for the ESP32-C3 is on an external board to save space.

The firmware was made in ESP-IDF 5.1.3

<img width="1062" height="530" alt="image" src="https://github.com/user-attachments/assets/508994c3-5ff3-4a01-b2df-e24be5f06039" />

## Cloning
Clone with this command to also get the bq76930 driver component
```
git clone https://github.com/TimBilliet/10s-bq76930-smart-bms.git --recurse-submodules
```
## Updating BQ76930 component
Pull the latest changes of the BQ76930 driver with this command
```
git submodule foreach git pull origin main
```
## App
https://github.com/TimBilliet/smart-bms-app

## TODO:
- [ ] Implement over charge current protection in firmware 

