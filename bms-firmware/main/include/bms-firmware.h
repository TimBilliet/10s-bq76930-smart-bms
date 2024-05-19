#include <stdio.h>
#include <map>

std::map<uint16_t, uint16_t> attribute_handles_;

uint8_t max_connected_devices_ = 1;
uint8_t connected_devices_ = 0;

uint16_t battery_voltage_ = 0;
uint16_t battery_cell_voltage_[10] = {0};

bool enable_balancing_ = false;
bool enable_charging_ = false;

const uint16_t battery_service_uuid_ = 0x3000;
const uint16_t battery_voltage_char_uuid_ = 0x3001;
const uint16_t battery_cell_voltage_char_uuid_ = 0x3002;
const uint16_t battery_current_char_uuid_ = 0x3003;
const uint16_t enable_balancing_char_uuid_ = 0x3004;
const uint16_t enable_charging_char_uuid_ = 0x3005;

