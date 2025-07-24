# 10S BLE smart BMS
A "smart" charge only BMS for a 10S battery pack.\
Cell monitoring and balancing using a BQ76930 BMS chip from Texas Instruments.\
This chip is controlled by an ESP32-C3 microcontroller via I²C.\
The battery pack voltage and bms parameters are presented via Bluetooth Low Energy on the ESP32-C3\
This data can then be read and written by my Smart BMS App.

The PCB was designed in Altium Designer.\
The programming circuitry for the ESP32-C3 is on an external board to save space.

The firmware was made in ESP-IDF 5.1.3

![image](https://github.com/user-attachments/assets/d45c9571-51e3-44de-8a31-cbb1526678b2)

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

## DONE:
- [x] Writing BQ76930 esp-idf driver\
- [x] Write firmware\
- [x] Write app
- [x] Implement OTA (in firmware and in app)\

## TODO:
- [ ] Implement over charge current protection in firmware 

