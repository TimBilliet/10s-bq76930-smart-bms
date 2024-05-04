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

int sda_pin = 5;
int scl_pin = 4;

int boot_pin = 6;
int alert_pin = 7;

bq76930 bms(0x08, sda_pin, scl_pin);

extern "C" {void app_main(void);}

#define TAG "SmartBMS-main"
// const std::string ble_device_name = "SmartBMS";
bool is_advertising_initialized = false;
#define PROFILE_A_APP_ID 0

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)



static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
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
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        }
        break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        }
        break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        break;
        default:
        break;
    }
}

void addService(esp_gatt_if_t gatts_if) {
    // Add test service.
    esp_gatt_srvc_id_t test_service = { .id = { .uuid = { .len = ESP_UUID_LEN_16, .uuid = { battery_service_uuid_ } }, .inst_id = 0x00 }, .is_primary = true };
    uint16_t test_service_handles_count = 8;
    esp_ble_gatts_create_service(gatts_if, &test_service, test_service_handles_count);
}

void onServiceCreated(esp_ble_gatts_cb_param_t::gatts_create_evt_param create) {
    uint16_t service_handle = create.service_handle;
    uint16_t service_id = create.service_id.id.uuid.uuid.uuid16;
    //characteristic_handles_[service_id] = service_handle;
    esp_ble_gatts_start_service(service_handle);
    if(service_id == battery_service_uuid_) {
        // Add battery voltage charactersitic.
        esp_bt_uuid_t bat_voltage_char_uuid = { };
        bat_voltage_char_uuid.len = ESP_UUID_LEN_16;
        bat_voltage_char_uuid.uuid.uuid16 = battery_voltage_char_uuid_;   
        esp_attr_value_t bat_value = { .attr_max_len = sizeof(battery_voltage_), .attr_len = sizeof(battery_voltage_), .attr_value = (uint8_t*)&battery_voltage_};
        esp_err_t add_char_ret = esp_ble_gatts_add_char(service_handle,
            &bat_voltage_char_uuid,
            (esp_gatt_perm_t)ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &bat_value,
            NULL);
            
        if (add_char_ret) { 
            ESP_LOGE(TAG, "Adding characteristic %04X failed, error code =%x", bat_voltage_char_uuid.uuid.uuid16, add_char_ret);
            return;
        }

        // Add battery cell voltage description charactersitic.
        esp_bt_uuid_t CCCD_uuid = { };
        CCCD_uuid.len = ESP_UUID_LEN_16;
        CCCD_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(service_handle, &CCCD_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret) {
            ESP_LOGE(TAG, "Adding characteristic descriptor failed, error code =%x", add_descr_ret);
            return;
        }

        // Add battery cell voltage charactersitic.
        esp_bt_uuid_t bat_cell_voltage_char_uuid = { };
        bat_cell_voltage_char_uuid.len = ESP_UUID_LEN_16;
        bat_cell_voltage_char_uuid.uuid.uuid16 = battery_cell_voltage_char_uuid_;   
        esp_attr_value_t bat_cell_value = { .attr_max_len = sizeof(battery_cell_voltage_), .attr_len = sizeof(battery_cell_voltage_), .attr_value = (uint8_t*)&battery_cell_voltage_};
        add_char_ret = esp_ble_gatts_add_char(service_handle,
            &bat_cell_voltage_char_uuid,
            (esp_gatt_perm_t)ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &bat_cell_value,
            NULL);
            
        if (add_char_ret) { 
            ESP_LOGE(TAG, "Adding characteristic %04X failed, error code =%x", bat_cell_voltage_char_uuid.uuid.uuid16, add_char_ret);
            return;
        }

        // Add battery voltage description charactersitic.
        esp_bt_uuid_t descr_uuid2 = { };
        descr_uuid2.len = ESP_UUID_LEN_16;
        descr_uuid2.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        add_descr_ret = esp_ble_gatts_add_char_descr(service_handle, &descr_uuid2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret) {
            ESP_LOGE(TAG, "Adding characteristic descriptor failed, error code =%x", add_descr_ret);
            return;
        }
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
    if(read.handle == characteristic_handles_[0x3001]) { // reading of the battery voltage characteristic
        battery_voltage_ = bms.getBatteryVoltage();
        rsp.attr_value.len = 2;
        uint8_t MSByte = battery_voltage_ >> 8;
        uint8_t LSByte = battery_voltage_ & 0xFF;
        rsp.attr_value.value[0] = MSByte;
        rsp.attr_value.value[1] = LSByte;
        esp_ble_gatts_send_response(gatts_if, read.conn_id, read.trans_id,ESP_GATT_OK, &rsp);
    } else if (read.handle == characteristic_handles_[0x3002]) { // reading of the battery cell voltage characteristic
        rsp.attr_value.len = 20;
        uint8_t response_index = 0;
        for (int i = 0; i < 10; i++) {
            battery_cell_voltage_[i] = bms.getCellVoltage(i + 1);
            uint8_t MSByte = battery_cell_voltage_[i] >> 8;
            uint8_t LSByte = battery_cell_voltage_[i] & 0xFF;
            rsp.attr_value.value[response_index] = MSByte;
            rsp.attr_value.value[response_index + 1] = LSByte;
            response_index += 2;
        }
        esp_ble_gatts_send_response(gatts_if, read.conn_id, read.trans_id,ESP_GATT_OK, &rsp);
    } else if (read.handle == characteristic_handles_[0x3003]) { // reading of the charge current characteristic
        //TODO handle read event of the charge current characteristic
    }
}

void onWriteEvent(esp_ble_gatts_cb_param_t::gatts_write_evt_param write) {
    //TODO handle write event of the CCCD and the BMS setting characteristics
}
void gattServerEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
            //esp_ble_gatts_create_service(gatts_if, &param->reg.app_id, 8);
            addService(gatts_if);
            
        break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_READ_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);  
            onReadEvent(param->read, gatts_if);
        break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);

        break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI("SensorServer", "ESP_GATTS_EXEC_WRITE_EVT");

        break;
        case ESP_GATTS_MTU_EVT:
        break;
        case ESP_GATTS_CONF_EVT:
        break;
        case ESP_GATTS_UNREG_EVT:
        
        break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI("SensorServer", "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
            onServiceCreated(param->create);
        break;
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI("SensorServer", "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            characteristic_handles_[param->add_char.char_uuid.uuid.uuid16] = param->add_char.attr_handle;
        break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        break;
        case ESP_GATTS_DELETE_EVT:
        break;
        case ESP_GATTS_START_EVT:
        break;
        case ESP_GATTS_STOP_EVT:
        break;
        case ESP_GATTS_CONNECT_EVT:
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
        case ESP_GATTS_OPEN_EVT:
        break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
        case ESP_GATTS_CLOSE_EVT:
        break;
        case ESP_GATTS_LISTEN_EVT:
        break;
        case ESP_GATTS_CONGEST_EVT:
        break;
        case ESP_GATTS_RESPONSE_EVT:
        break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        break;
        case ESP_GATTS_SET_ATTR_VAL_EVT:
        break;

        default:
        break;
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
    ret = esp_ble_gap_register_callback(gapEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gatts_register_callback(gattServerEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gap_set_device_name("test_server");
    if (ret){
        ESP_LOGE(TAG, "set name failed, error code = %x", ret);
        return false;
    } 
    return true;
}

void configAdvData() {
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret){
        ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
    }
    adv_config_done |= adv_config_flag;
}
void bmsUpdateTask(void *pvParameters) {
    while (1) {
        bms.update();
        //battery_voltage = bms.getBatteryVoltage();
    }
}

void app_main(void) {
    bms.initialize(alert_pin, boot_pin);
    bms.setTemperatureLimits(-20, 45, 0, 45);
    bms.setShuntResistorValue(5);
    bms.setOvercurrentChargeProtection(5000);
    bms.setCellUndervoltageProtection(3200, 2);
    bms.setCellOvervoltageProtection(4240, 2);
    bms.setBalancingThresholds(0, 3700, 15);
    bms.setIdleCurrentThreshold(100);
    xTaskCreate(bmsUpdateTask, "bmsUpdateTask", 2048, NULL, 5, NULL);
    bluetoothInit();
    configAdvData();
    // esp_ble_gatts_create_service(ESP_GATT_IF_NONE)

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
