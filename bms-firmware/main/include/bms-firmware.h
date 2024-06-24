#pragma once
#include <stdio.h>
#include <map>

const char *ble_device_name = "SmartBMS";

uint8_t max_connected_devices_ = 1;
uint8_t connected_devices_ = 0;
bool connected_ = false;

uint16_t dummy_[2] = {0, 0};

uint16_t previous_battery_voltage_ = 0;
uint16_t battery_voltage_ = 0;

uint16_t previous_battery_cell_voltage_[10] = {0};
uint16_t battery_cell_voltage_[10] = {0};

bool previous_cell_balancing_state_[10] = {false};
bool cell_balancing_state_[10] = {false};

uint16_t previous_charge_current_ = 0;
uint16_t charge_current_ = 0;

bool previous_enable_balancing_ = false;
bool enable_balancing_ = false;

bool previous_enable_charging_ = true;
bool enable_charging_ = true;

uint8_t previous_fault_value_ = 0;
uint8_t fault_ = 0;

bool power_on_ = true;
bool only_balance_when_charging_ = true;

bool fault_notifications_enabled_ = false;
bool balancing_notifications_enabled_ = false;
bool charging_notifications_enabled_ = false;
bool voltage_current_notifications_enabled_ = false;
bool cell_voltage_notifications_enabled_ = false;
bool cell_balancing_state_notifications_enabled_ = false;

uint8_t shunt_resistor_ = 5; // mOhm
uint16_t overcurrent_charge_ = 10000; // mA
uint16_t undervolt_ = 3000; // mV
uint16_t overvolt_ = 4200; // mV
uint16_t balancing_thresholds_[2] = {3900, 15}; // mV, ms
uint16_t idle_current_ = 100; // mA

const uint16_t battery_service_uuid_ = 0x3000;
const uint16_t battery_voltage_current_char_uuid_ = 0x3001;
const uint16_t battery_cell_voltage_char_uuid_ = 0x3002;
const uint16_t cell_balancing_state_char_uuid_ = 0x3003;
const uint16_t enable_balancing_char_uuid_ = 0x3005;
const uint16_t enable_charging_char_uuid_ = 0x3006;
const uint16_t fault_char_uuid_ = 0x3007;
const uint8_t fault_ccc_[2] = {0x00, 0x00};
const uint8_t balancing_ccc_[2] = {0x00, 0x00};
const uint8_t charging_ccc_[2] = {0x00, 0x00};
const uint8_t voltage_current_ccc_[2] = {0x00, 0x00};
const uint8_t cell_voltage_ccc_[2] = {0x00, 0x00};
const uint8_t cell_balancing_state_ccc_[2] = {0x00, 0x00};


const uint16_t parameters_service_uuid_ = 0x4000;
const uint16_t shunt_resistor_char_uuid_ = 0x4001;
const uint16_t overcurrent_charge_char_uuid_ = 0x4002;
const uint16_t undervolt_char_uuid_ = 0x4003;
const uint16_t overvolt_char_uuid_ = 0x4004;
const uint16_t balancing_thresholds_char_uuid_ = 0x4005;
const uint16_t idle_current_char_uuid_ = 0x4006;
const uint16_t power_on_char_uuid_ = 0x4007;
const uint16_t only_balance_when_charging_char_uuid_ = 0x4008;

enum {
    IDX_INFO_SERVICE,
    IDX_CHAR_BAT_VOLTAGE_CURRENT,
    IDX_CHAR_VAL_BAT_VOLTAGE_CURRENT,
    IDX_CHAR_CFG_BAT_VOLTAGE_CURRENT,

    IDX_CHAR_BATTERY_CELL_VOLTAGE,
    IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE,
    IDX_CHAR_CFG_BATTERY_CELL_VOLTAGE,

    IDX_CHAR_CELL_BALANCING_STATE,
    IDX_CHAR_VAL_CELL_BALANCING_STATE,
    IDX_CHAR_CFG_CELL_BALANCING_STATE,

    IDX_CHAR_BALANCING,
    IDX_CHAR_VAL_BALANCING,
    IDX_CHAR_CFG_BALANCING,

    IDX_CHAR_CHARGING,
    IDX_CHAR_VAL_CHARGING,
    IDX_CHAR_CFG_CHARGING,

    IDX_CHAR_FAULT,
    IDX_CHAR_VAL_FAULT,
    IDX_CHAR_CFG_FAULT,

    INFO_TABLE_ITEM_COUNT,
};

enum {
    IDX_PARAM_SERVICE,
    IDX_CHAR_SHUNT_RESISTOR,
    IDX_CHAR_VAL_SHUNT_RESISTOR,

    IDX_CHAR_OVERCURRENT_CHARGE,
    IDX_CHAR_VAL_OVERCURRENT_CHARGE,

    IDX_CHAR_UNDERVOLT,
    IDX_CHAR_VAL_UNDERVOLT,

    IDX_CHAR_OVERVOLT,
    IDX_CHAR_VAL_OVERVOLT,

    IDX_CHAR_BALANCING_THRESHOLDS,
    IDX_CHAR_VAL_BALANCING_THRESHOLDS,

    IDX_CHAR_IDLE_CURRENT,
    IDX_CHAR_VAL_IDLE_CURRENT,

    IDX_CHAR_POWER_ON,
    IDX_CHAR_VAL_POWER_ON,

    IDX_CHAR_ONLY_BALANCE_WHEN_CHARGING,
    IDX_CHAR_VAL_ONLY_BALANCE_WHEN_CHARGING,

    PARAM_TABLE_ITEM_COUNT,
};


