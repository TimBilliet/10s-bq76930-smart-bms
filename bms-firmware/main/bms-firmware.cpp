#include <stdio.h>
#include <stdlib.h>
#include "esp_bt.h"
#include "bq76930.h"
#include "esp_log.h"
#include <string.h>

#include <inttypes.h>
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "bms-firmware.h"

extern "C" {void app_main(void);}

#define TAG "SmartBMS-main"
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

int sda_pin = 5;
int scl_pin = 4;

int boot_pin = 6;
int alert_pin = 7;

int keep_on_pin = 1;

uint8_t address = 0x08;

bq76930 bms(address, sda_pin, scl_pin);

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

uint16_t info_handle_table_[INFO_TABLE_ITEM_COUNT];
uint16_t param_handle_table_[PARAM_TABLE_ITEM_COUNT];


static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0200, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0200, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x40,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};
static void gattServerEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static struct gatts_profile_inst bms_profile_tab = {
    .gatts_cb = gattServerEventHandler,
    .gatts_if = ESP_GATT_IF_NONE,
};

static const esp_gatts_attr_db_t gatt_info_db[INFO_TABLE_ITEM_COUNT] = {
    // Service Declaration
    [IDX_INFO_SERVICE]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(battery_service_uuid_), (uint8_t *)&battery_service_uuid_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BAT_VOLTAGE_CURRENT]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BAT_VOLTAGE_CURRENT] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&battery_voltage_current_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    4, 4, (uint8_t *)&dummy_}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_BAT_VOLTAGE_CURRENT]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(voltage_current_ccc_), (uint8_t *)voltage_current_ccc_}},  

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY_CELL_VOLTAGE]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&battery_cell_voltage_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(battery_cell_voltage_), sizeof(battery_cell_voltage_), (uint8_t *)battery_cell_voltage_}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_BATTERY_CELL_VOLTAGE]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(cell_voltage_ccc_), (uint8_t *)cell_voltage_ccc_}},

    /* Characteristic Declaration */
    [IDX_CHAR_CELL_BALANCING_STATE]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_CELL_BALANCING_STATE]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&cell_balancing_state_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(cell_balancing_state_), sizeof(cell_balancing_state_), (uint8_t *)cell_balancing_state_}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_CELL_BALANCING_STATE]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(cell_balancing_state_ccc_), (uint8_t *)cell_balancing_state_ccc_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BALANCING]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BALANCING]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&enable_balancing_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(enable_balancing_), sizeof(enable_balancing_), (uint8_t *)&enable_balancing_}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_BALANCING]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(balancing_ccc_), (uint8_t *)balancing_ccc_}},

    /* Characteristic Declaration */
    [IDX_CHAR_CHARGING]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_CHARGING]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&enable_charging_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(enable_charging_), sizeof(enable_charging_), (uint8_t *)&enable_charging_}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_CHARGING]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(charging_ccc_), (uint8_t *)charging_ccc_}},  

    /* Characteristic Declaration */
    [IDX_CHAR_FAULT]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_FAULT]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&fault_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(fault_), sizeof(fault_), &fault_}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_FAULT]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(fault_ccc_), (uint8_t *)fault_ccc_}},    
};

static const esp_gatts_attr_db_t gatt_parameters_db[PARAM_TABLE_ITEM_COUNT] = {
    // Service Declaration
    [IDX_PARAM_SERVICE]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(parameters_service_uuid_), (uint8_t *)&parameters_service_uuid_}},

    /* Characteristic Declaration */
    [IDX_CHAR_SHUNT_RESISTOR]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_SHUNT_RESISTOR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&shunt_resistor_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(shunt_resistor_), sizeof(shunt_resistor_), &shunt_resistor_}},

    /* Characteristic Declaration */
    [IDX_CHAR_OVERCURRENT_CHARGE]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_OVERCURRENT_CHARGE]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&overcurrent_charge_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(overcurrent_charge_), sizeof(overcurrent_charge_), (uint8_t *)&overcurrent_charge_}},

    /* Characteristic Declaration */
    [IDX_CHAR_UNDERVOLT]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_UNDERVOLT]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&undervolt_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(undervolt_), sizeof(undervolt_), (uint8_t *)&undervolt_}},

    /* Characteristic Declaration */
    [IDX_CHAR_OVERVOLT]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_OVERVOLT]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&overvolt_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(overvolt_), sizeof(overvolt_), (uint8_t *)&overvolt_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BALANCING_THRESHOLDS]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BALANCING_THRESHOLDS]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&balancing_thresholds_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(balancing_thresholds_), sizeof(balancing_thresholds_), (uint8_t *)balancing_thresholds_}},

    /* Characteristic Declaration */
    [IDX_CHAR_IDLE_CURRENT]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_IDLE_CURRENT]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&idle_current_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(idle_current_), sizeof(idle_current_), (uint8_t *)&idle_current_}},

       /* Characteristic Declaration */
    [IDX_CHAR_POWER_ON]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_POWER_ON]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&power_on_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(power_on_), sizeof(power_on_), (uint8_t *)&power_on_}},

       /* Characteristic Declaration */
    [IDX_CHAR_ONLY_BALANCE_WHEN_CHARGING]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_ONLY_BALANCE_WHEN_CHARGING]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&only_balance_when_charging_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(only_balance_when_charging_), sizeof(only_balance_when_charging_), (uint8_t *)&only_balance_when_charging_}},

};


void createAttributeTables(esp_gatt_if_t gatts_if) {
    esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_info_db, gatts_if, INFO_TABLE_ITEM_COUNT, 0);
    
    if (create_attr_ret){
        ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
    }

    create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_parameters_db, gatts_if, PARAM_TABLE_ITEM_COUNT, 0);
    
    if (create_attr_ret){
        ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
    }
}

void printBin(unsigned int num, int size) {
    size = 16;
    for (int i = size - 1; i >= 0; i--) {
        if ((num >> i) & 1)
            printf("1");
        else
            printf("0");

        if (i % 4 == 0)
            printf(" ");
    }
    printf("\n");
}

void onReadEvent(esp_ble_gatts_cb_param_t::gatts_read_evt_param read, esp_gatt_if_t gatts_if) {
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = read.handle;
    bool auto_response = false;
    if(read.handle == info_handle_table_[IDX_CHAR_VAL_BAT_VOLTAGE_CURRENT]) { // reading of the battery voltage and current characteristic
        battery_voltage_ = bms.getBatteryVoltage();
        charge_current_ = bms.getBatteryCurrent();
        rsp.attr_value.len = 4;
        rsp.attr_value.value[1] = battery_voltage_ >> 8;
        rsp.attr_value.value[0] = battery_voltage_ & 0xFF;
        rsp.attr_value.value[3] = charge_current_ >> 8;
        rsp.attr_value.value[2] = charge_current_ & 0xFF;
    } else if (read.handle == info_handle_table_[IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE]) { // reading of the battery cell voltage characteristic
        rsp.attr_value.len = 20;
        uint8_t response_index = 0;
        for (int i = 0; i < 10; i++) {
            battery_cell_voltage_[i] = bms.getCellVoltage(i + 1);
            uint8_t MSByte = battery_cell_voltage_[i] >> 8;
            uint8_t LSByte = battery_cell_voltage_[i] & 0xFF;
            rsp.attr_value.value[response_index] = LSByte;
            rsp.attr_value.value[response_index + 1] = MSByte;
            response_index += 2;
        }
    } else if (read.handle == info_handle_table_[IDX_CHAR_VAL_CELL_BALANCING_STATE]) { // reading of the cell balancing state characteristic
        rsp.attr_value.len = 10;
        for (int i = 0; i < 10; i++) {
            cell_balancing_state_[i] = bms.getBalancingState(i + 1);
            rsp.attr_value.value[i] = cell_balancing_state_[i] ? 1 : 0;
        }
    } else if(read.handle == info_handle_table_[IDX_CHAR_VAL_BALANCING]) { // reading of the balancing characteristic
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = enable_balancing_;
    } else if(read.handle == info_handle_table_[IDX_CHAR_VAL_CHARGING]) { // reading of the charging characteristic
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = enable_charging_;
    } else if(read.handle == info_handle_table_[IDX_CHAR_VAL_FAULT]) { // reading of the fault characteristic
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = fault_;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_SHUNT_RESISTOR]) { //reading of the shunt resistor characteristic
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = shunt_resistor_;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_OVERCURRENT_CHARGE]) { //reading of the overcurrent charge characteristic
        rsp.attr_value.len = 2;
        uint8_t MSByte = overcurrent_charge_ >> 8;
        uint8_t LSByte = overcurrent_charge_ & 0xFF;
        rsp.attr_value.value[0] = MSByte;
        rsp.attr_value.value[1] = LSByte;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_UNDERVOLT]) { //reading of the undervoltage characteristic
        rsp.attr_value.len = 2;
        uint8_t MSByte = undervolt_ >> 8;
        uint8_t LSByte = undervolt_ & 0xFF;
        rsp.attr_value.value[0] = MSByte;
        rsp.attr_value.value[1] = LSByte;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_OVERVOLT]) { //reading of the overvoltage characteristic
        rsp.attr_value.len = 2;
        uint8_t MSByte = overvolt_ >> 8;
        uint8_t LSByte = overvolt_ & 0xFF;
        rsp.attr_value.value[0] = MSByte;
        rsp.attr_value.value[1] = LSByte;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_BALANCING_THRESHOLDS]) { //reading of the balancing thresholds characteristic
        rsp.attr_value.len = 4;
        uint8_t response_index = 0;
        for (int i = 0; i < 2; i++) {
            uint8_t MSByte = balancing_thresholds_[i] >> 8;
            uint8_t LSByte = balancing_thresholds_[i] & 0xFF;
            rsp.attr_value.value[response_index] = MSByte;
            rsp.attr_value.value[response_index + 1] = LSByte;
            response_index += 2;
        }
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_IDLE_CURRENT]) { //reading of the idle current characteristic
        rsp.attr_value.len = 2;
        uint8_t MSByte = idle_current_ >> 8;
        uint8_t LSByte = idle_current_ & 0xFF;
        rsp.attr_value.value[0] = MSByte;
        rsp.attr_value.value[1] = LSByte;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_POWER_ON]) {  //reading of the power on characteristic
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = power_on_;
    } else if(read.handle == param_handle_table_[IDX_CHAR_VAL_ONLY_BALANCE_WHEN_CHARGING]) {
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = only_balance_when_charging_;
    }
    else {
        auto_response = true;
    }
    if(!auto_response) {
            esp_ble_gatts_send_response(gatts_if, read.conn_id, read.trans_id,ESP_GATT_OK, &rsp);
    }
}

void onWriteEvent(esp_ble_gatts_cb_param_t::gatts_write_evt_param write, esp_gatt_if_t gatts_if) {
    if(!write.is_prep) {
        ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value :", write.len);
        ESP_LOG_BUFFER_HEX(TAG, write.value, write.len);
        if(info_handle_table_[IDX_CHAR_VAL_BALANCING] == write.handle && write.len == 1) {
            enable_balancing_ = static_cast<bool>(write.value[0]);
            bms.toggleBalancing(enable_balancing_);
        } else if(info_handle_table_[IDX_CHAR_VAL_CHARGING] == write.handle && write.len == 1) {
            enable_charging_ = static_cast<bool>(write.value[0]);
            bms.toggleCharging(enable_charging_);
        } else if(info_handle_table_[IDX_CHAR_CFG_FAULT] == write.handle ) {
            uint16_t descr_value = write.value[1]<<8 | write.value[0];
            if (descr_value == 0x0001){
                ESP_LOGI(TAG, "fault notify enable");
                fault_notifications_enabled_ = true;
            } else if (descr_value == 0x0000){
                ESP_LOGI(TAG, "fault notify disable ");
                fault_notifications_enabled_ = false;
            }
        }else if(info_handle_table_[IDX_CHAR_CFG_BALANCING] == write.handle ) {
            uint16_t descr_value = write.value[1]<<8 | write.value[0];
            if (descr_value == 0x0001){
                ESP_LOGI(TAG, "balancing notify enable");
                balancing_notifications_enabled_ = true;
            } else if (descr_value == 0x0000){
                ESP_LOGI(TAG, "balancing notify disable ");
                balancing_notifications_enabled_ = false;
            }
        } else if(info_handle_table_[IDX_CHAR_CFG_CHARGING] == write.handle ) {
            uint16_t descr_value = write.value[1]<<8 | write.value[0];
            if (descr_value == 0x0001){
                ESP_LOGI(TAG, "charging notify enable");
                charging_notifications_enabled_ = true;
            } else if (descr_value == 0x0000){
                ESP_LOGI(TAG, "charging notify disable ");
                charging_notifications_enabled_ = false;
            }
        } else if(info_handle_table_[IDX_CHAR_CFG_BAT_VOLTAGE_CURRENT] == write.handle ) {
            uint16_t descr_value = write.value[1]<<8 | write.value[0];
            if (descr_value == 0x0001){
                ESP_LOGI(TAG, "voltage & current notify enable");
                voltage_current_notifications_enabled_ = true;
            } else if (descr_value == 0x0000){
                ESP_LOGI(TAG, "voltage & current notify disable ");
                voltage_current_notifications_enabled_ = false;
            }
        } else if(info_handle_table_[IDX_CHAR_CFG_BATTERY_CELL_VOLTAGE] == write.handle ) {
            uint16_t descr_value = write.value[1]<<8 | write.value[0];
            if (descr_value == 0x0001){
                ESP_LOGI(TAG, "cell voltage notify enable");
                cell_voltage_notifications_enabled_ = true;
            } else if (descr_value == 0x0000){
                ESP_LOGI(TAG, "cell voltage notify disable ");
                cell_voltage_notifications_enabled_ = false;
            }
        } else if(info_handle_table_[IDX_CHAR_CFG_CELL_BALANCING_STATE] == write.handle ) {
            uint16_t descr_value = write.value[1]<<8 | write.value[0];
            if (descr_value == 0x0001){
                ESP_LOGI(TAG, "cell balancing state notify enable");
                cell_balancing_state_notifications_enabled_ = true;
            } else if (descr_value == 0x0000){
                ESP_LOGI(TAG, "cell balancing state notify disable ");
                cell_balancing_state_notifications_enabled_ = false;
            }
        } else if(param_handle_table_[IDX_CHAR_VAL_SHUNT_RESISTOR] == write.handle && write.len == 1) {
            shunt_resistor_ = write.value[0];
            bms.setShuntResistorValue(shunt_resistor_);
        } else if(param_handle_table_[IDX_CHAR_VAL_OVERCURRENT_CHARGE] == write.handle){
            overcurrent_charge_ = write.value[1]<<8 | write.value[0];
            bms.setOvercurrentChargeProtection(overcurrent_charge_);
        } else if(param_handle_table_[IDX_CHAR_VAL_UNDERVOLT] == write.handle){
            undervolt_ = write.value[1]<<8 | write.value[0];
            bms.setCellUndervoltageProtection(undervolt_, 2);
        } else if(param_handle_table_[IDX_CHAR_VAL_OVERVOLT] == write.handle){
            overvolt_ = write.value[1]<<8 | write.value[0];
            bms.setCellOvervoltageProtection(overvolt_, 2);
        } else if(param_handle_table_[IDX_CHAR_VAL_BALANCING_THRESHOLDS] == write.handle){
            balancing_thresholds_[0] = write.value[1]<<8 | write.value[0];
            balancing_thresholds_[1] = write.value[3]<<8 | write.value[2];
            bms.setBalancingThresholds(0, balancing_thresholds_[0], balancing_thresholds_[1]);
        } else if(param_handle_table_[IDX_CHAR_VAL_IDLE_CURRENT] == write.handle){
            idle_current_ = write.value[1]<<8 | write.value[0];
            bms.setIdleCurrentThreshold(idle_current_);
        } else if(param_handle_table_[IDX_CHAR_VAL_POWER_ON] == write.handle){
            power_on_ = write.value[0];
            if(power_on_ == 0) {
                bms.shutdown();
                gpio_set_level((gpio_num_t)1,power_on_);
            }
        } else if(param_handle_table_[IDX_CHAR_VAL_ONLY_BALANCE_WHEN_CHARGING] == write.handle){
            only_balance_when_charging_ = write.value[0];
        }
        if (write.need_rsp){
            esp_ble_gatts_send_response(gatts_if, write.conn_id, write.trans_id, ESP_GATT_OK, NULL);
        }
    } else {
        ESP_LOGI(TAG, "WRITE PREP");
    }
}

static void gattServerEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
            createAttributeTables(gatts_if);
        break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_READ_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);  
            onReadEvent(param->read, gatts_if);
        break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            onWriteEvent(param->write, gatts_if);
        break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else {
                if(param->add_attr_tab.svc_uuid.uuid.uuid16 == battery_service_uuid_) {
                    ESP_LOGI(TAG, "create battery info attribute table successfully, number handle = %d\n",param->add_attr_tab.num_handle);
                    memcpy(info_handle_table_, param->add_attr_tab.handles, sizeof(info_handle_table_));
                    esp_ble_gatts_start_service(info_handle_table_[IDX_INFO_SERVICE]);
                } else if(param->add_attr_tab.svc_uuid.uuid.uuid16 == parameters_service_uuid_) {
                    ESP_LOGI(TAG, "create parameter attribute table successfully, number handle = %d\n",param->add_attr_tab.num_handle);
                    memcpy(param_handle_table_, param->add_attr_tab.handles, sizeof(param_handle_table_));
                    esp_ble_gatts_start_service(param_handle_table_[IDX_PARAM_SERVICE]);
                }
            }
        
        break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x", param->connect.conn_id,
                    param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                    param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            connected_devices_++;
            connection_id_ = param->connect.conn_id;
            connected_ = true;
            if(connected_devices_ < max_connected_devices_) {
                esp_ble_gap_start_advertising(&adv_params);
            }     
        break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            connected_devices_--;
            connected_ = false;
            esp_ble_gap_start_advertising(&adv_params);
        break;
        case ESP_GATTS_RESPONSE_EVT:
        break;
        default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            bms_profile_tab.gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == bms_profile_tab.gatts_if) {
        if (bms_profile_tab.gatts_cb) {
            bms_profile_tab.gatts_cb(event, gatts_if, param);
        }
    }
}

bool bluetoothInit() {
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(600 / portTICK_PERIOD_MS);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gatts_app_register(0);
    if (ret){
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gap_set_device_name(ble_device_name);
    if (ret){
        ESP_LOGE(TAG, "set name failed, error code = %x", ret);
        return false;
    } 
    return true;
}

void retrieveValues() {
    for (int i = 0; i < 10; i++) {
        battery_cell_voltage_[i] = bms.getCellVoltage(i + 1);
        cell_balancing_state_[i] = bms.getBalancingState(i + 1);
    }
    battery_voltage_ = bms.getBatteryVoltage();
    charge_current_ = bms.getBatteryCurrent();

}
void bmsUpdateTask(void *pvParameters) {
    while (1) {
        bms.update();
        fault_ = bms.getErrorState();
        if (fault_ != previous_fault_value_ && fault_notifications_enabled_ && fault_ != previous_fault_value_) {
            esp_ble_gatts_send_indicate(bms_profile_tab.gatts_if, connection_id_, info_handle_table_[IDX_CHAR_VAL_FAULT], 1, &fault_, false);
            previous_fault_value_ = fault_;
        }
        
        if(only_balance_when_charging_ && bms.getBatteryCurrent() < 0.05) {
            bms.toggleBalancing(false);
            enable_balancing_ = false;
        }
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

void sendNotificationsTask(void *pvParameters) {
    while(1) {
        if(connected_) {
            retrieveValues();
            if(previous_enable_balancing_ != enable_balancing_ && balancing_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab.gatts_if, connection_id_, info_handle_table_[IDX_CHAR_VAL_BALANCING], 1, (uint8_t*)&enable_balancing_, false);
                previous_enable_balancing_ = enable_balancing_;
            }
            if(previous_enable_charging_ != enable_charging_ && charging_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab.gatts_if, connection_id_, info_handle_table_[IDX_CHAR_VAL_CHARGING], 1, (uint8_t*)&enable_charging_, false);
                previous_enable_charging_ = enable_charging_;
            }
            if(!std::equal(std::begin(previous_battery_cell_voltage_), std::end(previous_battery_cell_voltage_), std::begin(battery_cell_voltage_)) && cell_voltage_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab.gatts_if, connection_id_, info_handle_table_[IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE], 20, (uint8_t*)&battery_cell_voltage_, false);
                memcpy(previous_battery_cell_voltage_, battery_cell_voltage_, sizeof(battery_cell_voltage_));
            }
            if(!std::equal(std::begin(previous_cell_balancing_state_), std::end(previous_cell_balancing_state_), std::begin(cell_balancing_state_)) && cell_balancing_state_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab.gatts_if, connection_id_, info_handle_table_[IDX_CHAR_VAL_CELL_BALANCING_STATE], 10, (uint8_t*)&cell_balancing_state_, false);
                memcpy(previous_cell_balancing_state_, cell_balancing_state_, sizeof(cell_balancing_state_));
            }
            if((previous_battery_voltage_ != battery_voltage_ || previous_charge_current_ != charge_current_) && voltage_current_notifications_enabled_) {
                dummy_[0] = battery_voltage_;
                dummy_[1] = charge_current_;
                esp_ble_gatts_send_indicate(bms_profile_tab.gatts_if, connection_id_, info_handle_table_[IDX_CHAR_VAL_BAT_VOLTAGE_CURRENT], 4, (uint8_t*)&dummy_, false);
                previous_battery_voltage_ = battery_voltage_;
                previous_charge_current_ = charge_current_;
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    gpio_set_direction((gpio_num_t)1, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)1,power_on_);
    bms.initialize(alert_pin, boot_pin);
    bms.setShuntResistorValue(5);
    bms.setOvercurrentChargeProtection(5000);
    bms.setCellUndervoltageProtection(3200, 2);
    bms.setCellOvervoltageProtection(4240, 2);
    bms.setBalancingThresholds(0, 3700, 10);
    bms.setIdleCurrentThreshold(100);
    bms.toggleBalancing(enable_balancing_);
    bms.toggleCharging(enable_charging_);
    bluetoothInit();
    xTaskCreate(bmsUpdateTask, "bmsUpdateTask", 2048, NULL, 5, NULL);
    xTaskCreate(sendNotificationsTask, "sendNotificationsTask", 2048, NULL, 5, NULL);
    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gap_start_advertising(&adv_params);
}
