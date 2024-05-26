#include <stdio.h>
#include <map>

uint8_t max_connected_devices_ = 1;
uint8_t connected_devices_ = 0;

uint16_t battery_voltage_ = 0;
//uint16_t battery_cell_voltage_[10] = {0};
uint16_t battery_cell_voltage_[10] = {3501, 3628, 3967, 3321, 4134, 3800, 4102, 3267, 3790, 3938};
uint16_t charge_current_ = 0;
bool enable_balancing_ = true;
bool enable_charging_ = false;
uint8_t fault_ = 0;

bool fault_notifications_enabled_ = false;

uint8_t shunt_resistor_ = 5;
uint16_t overcurrent_charge_ = 0;
uint16_t undervolt_ = 0;
uint16_t overvolt_ = 0;
uint16_t balancing_thresholds_[2] = {3700, 15};
uint16_t idle_current_ = 0;

const uint16_t battery_service_uuid_ = 0x3000;
const uint16_t battery_voltage_char_uuid_ = 0x3001;
const uint16_t battery_cell_voltage_char_uuid_ = 0x3002;
const uint16_t battery_current_char_uuid_ = 0x3003;
const uint16_t enable_balancing_char_uuid_ = 0x3004;
const uint16_t enable_charging_char_uuid_ = 0x3005;
const uint16_t fault_char_uuid_ = 0x3006;
const uint8_t fault_ccc_[2]      = {0x00, 0x00};

const uint16_t parameters_service_uuid_ = 0x4000;
const uint16_t shunt_resistor_char_uuid_ = 0x4001;
const uint16_t overcurrent_charge_char_uuid_ = 0x4002;
const uint16_t undervolt_char_uuid_ = 0x4003;
const uint16_t overvolt_char_uuid_ = 0x4004;
const uint16_t balancing_thresholds_char_uuid_ = 0x4005;
const uint16_t idle_current_char_uuid_ = 0x4006;

enum {
    IDX_INFO_SERVICE,
    IDX_CHAR_BATTERY_VOLTAGE,
    IDX_CHAR_VAL_BATTERY_VOLTAGE,

    IDX_CHAR_BATTERY_CELL_VOLTAGE,
    IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE,

    IDX_CHAR_BATTERY_CURRENT,
    IDX_CHAR_VAL_BATTERY_CURRENT,

    IDX_CHAR_BALANCING,
    IDX_CHAR_VAL_BALANCING,

    IDX_CHAR_CHARGING,
    IDX_CHAR_VAL_CHARGING,

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

    PARAM_TABLE_ITEM_COUNT,
};


