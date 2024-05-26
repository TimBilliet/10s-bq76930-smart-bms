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

#define PROFILE_APP_ID 0

int sda_pin = 5;
int scl_pin = 4;

int boot_pin = 6;
int alert_pin = 7;

//bq76930 bms(0x08, sda_pin, scl_pin);

const char *ble_device_name = "SmartBMS";

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
    //.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST,
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
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static struct gatts_profile_inst bms_profile_tab[1] = {
    [0] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static const esp_gatts_attr_db_t gatt_info_db[INFO_TABLE_ITEM_COUNT] = {
    // Service Declaration
    [IDX_INFO_SERVICE]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(battery_service_uuid_), (uint8_t *)&battery_service_uuid_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY_VOLTAGE]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY_VOLTAGE] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&battery_voltage_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(battery_voltage_), sizeof(battery_voltage_), (uint8_t *)&battery_voltage_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY_CELL_VOLTAGE]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&battery_cell_voltage_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(battery_cell_voltage_), sizeof(battery_cell_voltage_), (uint8_t *)battery_cell_voltage_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY_CURRENT]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY_CURRENT]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&battery_current_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(battery_current_), sizeof(battery_current_), (uint8_t *)&battery_current_}},

    /* Characteristic Declaration */
    [IDX_CHAR_BALANCING]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BALANCING]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&enable_balancing_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(enable_balancing_), sizeof(enable_balancing_), (uint8_t *)&enable_balancing_}},

    /* Characteristic Declaration */
    [IDX_CHAR_CHARGING]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_CHARGING]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&enable_charging_char_uuid_, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(enable_charging_), sizeof(enable_charging_), (uint8_t *)&enable_charging_}},

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
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
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

    if(read.handle == info_handle_table_[IDX_CHAR_VAL_BATTERY_VOLTAGE]) { // reading of the battery voltage characteristic
    printf("battery voltage read");
        // battery_voltage_ = bms.getBatteryVoltage();
        battery_voltage_ = 42;
        rsp.attr_value.len = 2;
        uint8_t MSByte = battery_voltage_ >> 8;
        uint8_t LSByte = battery_voltage_ & 0xFF;
        rsp.attr_value.value[0] = MSByte;
        rsp.attr_value.value[1] = LSByte;
        esp_ble_gatts_send_response(gatts_if, read.conn_id, read.trans_id,ESP_GATT_OK, &rsp);
    } else if (read.handle == info_handle_table_[IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE]) { // reading of the battery cell voltage characteristic
        rsp.attr_value.len = 20;
        uint8_t response_index = 0;
        for (int i = 0; i < 10; i++) {
            // battery_cell_voltage_[i] = bms.getCellVoltage(i + 1);
            battery_cell_voltage_[i] = i+1;
            uint8_t MSByte = battery_cell_voltage_[i] >> 8;
            uint8_t LSByte = battery_cell_voltage_[i] & 0xFF;
            rsp.attr_value.value[response_index] = MSByte;
            rsp.attr_value.value[response_index + 1] = LSByte;
            response_index += 2;
        }
        esp_ble_gatts_send_response(gatts_if, read.conn_id, read.trans_id,ESP_GATT_OK, &rsp);
    } else if (read.handle == info_handle_table_[IDX_CHAR_VAL_BATTERY_CURRENT]) { // reading of the charge current characteristic
        //TODO handle read event of the charge current characteristic
    } else if(read.handle == info_handle_table_[IDX_CHAR_VAL_BALANCING]) { // reading of the balancing characteristic
        //TODO handle read event of the balancing characteristic
    } else if(read.handle == info_handle_table_[IDX_CHAR_VAL_CHARGING]) { // reading of the charging characteristic
        //TODO handle read event of the charging characteristic
    } else if(read.handle == info_handle_table_[IDX_CHAR_VAL_FAULT]) { // reading of the fault characteristic
        //TODO handle read event of the fault characteristic
    }
}

void onWriteEvent(esp_ble_gatts_cb_param_t::gatts_write_evt_param write) {
    //TODO handle write event of the CCCD and the BMS setting characteristics
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
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
            onWriteEvent(param->write);
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
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        break;
        case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x", param->connect.conn_id,
                param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        connected_devices_++;
        if(connected_devices_ < max_connected_devices_) {
            esp_ble_gap_start_advertising(&adv_params);
        }     
        break;
        case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        connected_devices_--;
        esp_ble_gap_start_advertising(&adv_params);
        break;
        case ESP_GATTS_RESPONSE_EVT:
        break;
        default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    printf("event = %d\n", event);
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        printf("ESP_GATTS_REG_EVT\n");
        if (param->reg.status == ESP_GATT_OK) {
            bms_profile_tab[0].gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    int idx = 0;
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == bms_profile_tab[idx].gatts_if) {
        if (bms_profile_tab[idx].gatts_cb) {
            bms_profile_tab[idx].gatts_cb(event, gatts_if, param);
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
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
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

// void bmsUpdateTask(void *pvParameters) {
//     while (1) {
//         // bms.update();
//         //battery_voltage = bms.getBatteryVoltage();
//     }
// }

void app_main(void) {
    // bms.initialize(alert_pin, boot_pin);
    // bms.setShuntResistorValue(5);
    // bms.setOvercurrentChargeProtection(5000);
    // bms.setCellUndervoltageProtection(3200, 2);
    // bms.setCellOvervoltageProtection(4240, 2);
    // bms.setBalancingThresholds(0, 3700, 15);
    // bms.setIdleCurrentThreshold(100);
    // xTaskCreate(bmsUpdateTask, "bmsUpdateTask", 2048, NULL, 5, NULL);
    bluetoothInit();
    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gap_start_advertising(&adv_params);

    // while (1) {
    //     bms.update();
    //     printf("cell1: %d\n", bms.getCellVoltage(1));
    //     printf("cell2: %d\n", bms.getCellVoltage(2));
    //     printf("cell3: %d\n", bms.getCellVoltage(3));
    //     printf("cell4: %d\n", bms.getCellVoltage(4));
    //     printf("cell5: %d\n", bms.getCellVoltage(5));
    //     printf("cell6: %d\n", bms.getCellVoltage(6));
    //     printf("cell7: %d\n", bms.getCellVoltage(7));
    //     printf("cell8: %d\n", bms.getCellVoltage(8));
    //     printf("cell9: %d\n", bms.getCellVoltage(9));
    //     printf("cell10: %d\n", bms.getCellVoltage(10));
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }
}
