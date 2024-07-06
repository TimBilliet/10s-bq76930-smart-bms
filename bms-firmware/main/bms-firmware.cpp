#include <stdio.h>
#include <stdlib.h>
#include "esp_bt.h"
#include "bq76930.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/ringbuf.h"
#include <inttypes.h>
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "bms-firmware.h"
#include "ble_ota.h"
#include "esp_ota_ops.h"
#include "driver/rmt_rx.h"

extern "C" {void app_main(void);}

#define TAG "SmartBMS-main"
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define OTA_TASK_SIZE 8192
#define OTA_RINGBUF_SIZE 8192

#define OTA_PROFILE_NUM           2

#define BMS_PROFILE_APP_IDX       0
#define OTA_PROFILE_APP_IDX       1
#define BUF_LENGTH                4098

#define BLE_OTA_MAX_CHAR_VAL_LEN  600

#define BLE_OTA_START_CMD         0x0001
#define BLE_OTA_STOP_CMD          0x0002
#define BLE_OTA_ACK_CMD           0x0003

esp_ble_ota_callback_funs_t ota_cb_fun_t = {
    .recv_fw_cb = NULL
};

esp_ble_ota_notification_check_t ota_notification = {
    .recv_fw_ntf_enable = false,
    .process_bar_ntf_enable = false,
    .command_ntf_enable = false,
    .customer_ntf_enable = false,
};

RingbufHandle_t otaRingbufHandle = NULL;
rmt_channel_handle_t rx_channel = NULL;
rmt_symbol_word_t raw_symbols[16];
rmt_rx_done_event_data_t rx_data;
rmt_receive_config_t receive_config;
QueueHandle_t receive_queue = NULL;

int sda_pin = 5;
int scl_pin = 4;

int boot_pin = 6;
int alert_pin = 7;

int keep_on_pin = 1;
esp_err_t ret;

uint8_t address = 0x08;

bq76930 bms(address, sda_pin, scl_pin);

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_indicate = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_write_indicate = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static bool start_ota = false;
static unsigned int ota_total_len = 0;
static unsigned int cur_sector = 0;
static unsigned int cur_packet = 0;
static uint8_t *fw_buf = NULL;
static unsigned int fw_buf_offset = 0;
static uint8_t *temp_prep_write_buf = NULL;
static unsigned int temp_buf_len = 0;

static const uint16_t BLE_OTA_SERVICE_UUID              = 0x8018;

static const uint16_t RECV_FW_UUID                      = 0x8020;
static const uint16_t OTA_BAR_UUID                      = 0x8021;
static const uint16_t COMMAND_UUID                      = 0x8022;
static const uint16_t CUSTOMER_UUID                     = 0x8023;

static uint8_t receive_fw_val[BLE_OTA_MAX_CHAR_VAL_LEN] = {0};
static uint8_t receive_fw_val_ccc[2] = {0x00, 0x00};

static uint8_t ota_status_val[20] = {0};
static uint8_t ota_status_val_ccc[2] = {0x00, 0x00};

static uint8_t command_val[20] = {0};
static uint8_t command_val_ccc[2] = {0x00, 0x00};

static uint8_t custom_val[20] = {0};
static uint8_t custom_val_ccc[2] = {0x00, 0x00};

uint16_t info_handle_table_[INFO_TABLE_ITEM_COUNT];
uint16_t param_handle_table_[PARAM_TABLE_ITEM_COUNT];
static uint16_t ota_handle_table[OTA_IDX_NB];


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

typedef enum {
    BLE_OTA_CMD_ACK = 0,        /*!< Command Ack */
    BLE_OTA_FW_ACK  = 1,        /*!< Firmware Ack */
} ble_ota_ack_type_t;

typedef enum {
    BLE_OTA_CMD_SUCCESS = 0x0000,       /*!< Success Ack */
    BLE_OTA_REJECT      = 0x0001,       /*!< Reject Cmd Ack */
} ble_ota_cmd_ack_status_t;

typedef enum {
    BLE_OTA_FW_SUCCESS = 0x0000,        /*!< Success */
    BLE_OTA_FW_CRC_ERR = 0x0001,        /*!< CRC error */
    BLE_OTA_FW_IND_ERR = 0x0002,        /*!< Sector Index error*/
    BLE_OTA_FW_LEN_ERR = 0x0003,        /*!< Payload length error*/
} ble_ota_fw_ack_statuc_t;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t conn_id;
    uint16_t mtu_size;
};
static void gatts_bms_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_ota_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

esp_err_t esp_ble_ota_notification_data(esp_ble_ota_char_t ota_char, uint8_t *value, uint8_t length);
esp_err_t esp_ble_ota_set_value(esp_ble_ota_char_t ota_char, uint8_t *value, uint8_t length);

static struct gatts_profile_inst bms_profile_tab[OTA_PROFILE_NUM] = {
    [BMS_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_bms_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [OTA_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_ota_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .mtu_size = 23,
    },
};

static const esp_gatts_attr_db_t ota_gatt_db[OTA_IDX_NB] = {
    // Service Declaration
    [OTA_SVC_IDX]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(BLE_OTA_SERVICE_UUID), sizeof(BLE_OTA_SERVICE_UUID), (uint8_t *) &BLE_OTA_SERVICE_UUID}},

    /* Characteristic Declaration */
    [RECV_FW_CHAR_IDX]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_write_indicate}},

    /* Characteristic Value */
    [RECV_FW_CHAR_VAL_IDX]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &RECV_FW_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(receive_fw_val), sizeof(receive_fw_val), (uint8_t *)receive_fw_val}},

    //data notify characteristic Declaration
    [RECV_FW_CHAR_NTF_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(receive_fw_val_ccc), sizeof(receive_fw_val_ccc), (uint8_t *)receive_fw_val_ccc}},

    //data receive characteristic Declaration
    [OTA_STATUS_CHAR_IDX]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_read_indicate}},

    //data receive characteristic Value
    [OTA_STATUS_CHAR_VAL_IDX]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &OTA_BAR_UUID, ESP_GATT_PERM_READ,
    sizeof(ota_status_val), sizeof(ota_status_val), (uint8_t *)ota_status_val}},

    //data notify characteristic Declaration
    [OTA_STATUS_NTF_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(ota_status_val_ccc), sizeof(ota_status_val_ccc), (uint8_t *)ota_status_val_ccc}},

    //data receive characteristic Declaration
    [CMD_CHAR_IDX]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_write_indicate}},

    //data receive characteristic Value
    [CMD_CHAR_VAL_IDX]              =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &COMMAND_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(command_val), sizeof(command_val), (uint8_t *)command_val}},

    //data notify characteristic Declaration
    [CMD_CHAR_NTF_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(command_val_ccc), sizeof(command_val_ccc), (uint8_t *)command_val_ccc}},

    //data receive characteristic Declaration
    [CUS_CHAR_IDX]            =
    {{ESP_GATT_AUTO_RSP}, {
    ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_write_indicate}},

    //data receive characteristic Value
    [CUS_CHAR_VAL_IDX]              =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &CUSTOMER_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(custom_val), sizeof(custom_val), (uint8_t *)custom_val}},

    //data notify characteristic Declaration
    [CUS_CHAR_NTF_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(custom_val_ccc), sizeof(custom_val_ccc), (uint8_t *)custom_val_ccc}},
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

static uint16_t crc16_ccitt(const unsigned char *buf, int len) {
    uint16_t crc16 = 0;
    int32_t i;

    while (len--) {
        crc16 ^= *buf++ << 8;

        for (i = 0; i < 8; i++) {
            if (crc16 & 0x8000) {
                crc16 = (crc16 << 1) ^ 0x1021;
            } else {
                crc16 = crc16 << 1;
            }
        }
    }
    return crc16;
}

void esp_ble_ota_set_fw_length(unsigned int length) {
    ota_total_len = length;
}

unsigned int esp_ble_ota_get_fw_length(void) {
    return ota_total_len;
}

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
        } else if(info_handle_table_[IDX_CHAR_CFG_BALANCING] == write.handle ) {
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

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    esp_bd_addr_t bd_addr;
    switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            ESP_LOGI(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x", \
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

static void gatts_bms_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
            createAttributeTables(gatts_if);
        break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);  
            onReadEvent(param->read, gatts_if);
        break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
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
            bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
            connected_devices_++;
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

static esp_ble_ota_char_t find_ota_char_and_desr_by_handle(uint16_t handle) {
    esp_ble_ota_char_t ret = INVALID_CHAR;
    for (int i = 0; i < OTA_IDX_NB ; i++) {
        if (handle == ota_handle_table[i]) {
            switch (i) {
            case RECV_FW_CHAR_VAL_IDX:
                ret = RECV_FW_CHAR;
                break;
            case RECV_FW_CHAR_NTF_CFG:
                ret = RECV_FW_CHAR_CCC;
                break;
            case OTA_STATUS_CHAR_VAL_IDX:
                ret = OTA_STATUS_CHAR;
                break;
            case OTA_STATUS_NTF_CFG:
                ret = OTA_STATUS_CHAR_CCC;
                break;
            case CMD_CHAR_VAL_IDX:
                ret = CMD_CHAR;
                break;
            case CMD_CHAR_NTF_CFG:
                ret = CMD_CHAR_CCC;
                break;
            case CUS_CHAR_VAL_IDX:
                ret = CUS_CHAR;
                break;
            case CUS_CHAR_NTF_CFG:
                ret = CUS_CHAR_CCC;
                break;
            default:
                ret = INVALID_CHAR;
                break;
            }
        }
    }
    return ret;
}
void esp_ble_ota_send_ack_data(ble_ota_ack_type_t ack_type, uint16_t ack_status, uint16_t ack_param) {
    uint8_t cmd_ack[20] = {0};
    uint16_t crc16 = 0;

    switch (ack_type) {
    case BLE_OTA_CMD_ACK:
        cmd_ack[0] = (BLE_OTA_ACK_CMD & 0xff);
        cmd_ack[1] = (BLE_OTA_ACK_CMD & 0xff00) >> 8;

        cmd_ack[2] = (ack_param & 0xff);
        cmd_ack[3] = (ack_param & 0xff00) >> 8;

        cmd_ack[4] = (ack_status & 0xff);
        cmd_ack[5] = (ack_status & 0xff00) >> 8;

        crc16 = crc16_ccitt(cmd_ack, 18);
        cmd_ack[18] = crc16 & 0xff;
        cmd_ack[19] = (crc16 & 0xff00) >> 8;

        esp_ble_ota_notification_data(CMD_CHAR, cmd_ack, 20);
        break;
    case BLE_OTA_FW_ACK:
        cmd_ack[0] = (ack_param & 0xff);
        cmd_ack[1] = (ack_param & 0xff00) >> 8;

        cmd_ack[2] = (ack_status & 0xff);
        cmd_ack[3] = (ack_status & 0xff00) >> 8;

        cmd_ack[4] = (cur_sector & 0xff);
        cmd_ack[5] = (cur_sector & 0xff00) >> 8;

        crc16 = crc16_ccitt(cmd_ack, 18);
        cmd_ack[18] = crc16 & 0xff;
        cmd_ack[19] = (crc16 & 0xff00) >> 8;

        esp_ble_ota_notification_data(RECV_FW_CHAR, cmd_ack, 20);
        break;
    default:
        break;
    }
}

esp_err_t esp_ble_ota_recv_fw_handler(uint8_t *buf, unsigned int length) {
    if (ota_cb_fun_t.recv_fw_cb) {
        ota_cb_fun_t.recv_fw_cb(buf, length);
    }

    return ESP_OK;
}

void esp_ble_ota_process_recv_data(esp_ble_ota_char_t ota_char, uint8_t *val, uint16_t val_len) {
    unsigned int recv_sector = 0;

    switch (ota_char) {
    case CMD_CHAR:
        // Start BLE OTA Process
        if ((val[0] == 0x01) && (val[1] == 0x00)) {
            if (start_ota) {
                esp_ble_ota_send_ack_data(BLE_OTA_CMD_ACK, BLE_OTA_REJECT, BLE_OTA_START_CMD);
            } else {
                start_ota = true;
                // Calculating Firmware Length
                ota_total_len = (val[2]) +
                                (val[3] * 256) +
                                (val[4] * 256 * 256) +
                                (val[5] * 256 * 256 * 256);
                esp_ble_ota_set_fw_length(ota_total_len);
                ESP_LOGI(TAG, "Recv ota start cmd, fw_length = %d", ota_total_len);
                // Malloc buffer to store receive Firmware
                fw_buf = (uint8_t *)malloc(BUF_LENGTH * sizeof(uint8_t));
                if (fw_buf == NULL) {
                    ESP_LOGE(TAG, "Malloc fail");
                    break;
                } else {
                    memset(fw_buf, 0x0, BUF_LENGTH);
                }

                esp_ble_ota_send_ack_data(BLE_OTA_CMD_ACK, BLE_OTA_CMD_SUCCESS, BLE_OTA_START_CMD);
            }
        }
        // Stop BLE OTA Process
        else if ((val[0] == 0x02) && (val[1] == 0x00)) {
            if (start_ota) {
                start_ota = false;
                esp_ble_ota_set_fw_length(0);
                ESP_LOGI(TAG, "recv ota stop cmd");
                esp_ble_ota_send_ack_data(BLE_OTA_CMD_ACK, BLE_OTA_CMD_SUCCESS, BLE_OTA_STOP_CMD);

                if (fw_buf) {
                    free(fw_buf);
                    fw_buf = NULL;
                }
            } else {
                esp_ble_ota_send_ack_data(BLE_OTA_CMD_ACK, BLE_OTA_REJECT, BLE_OTA_STOP_CMD);
            }

        } else {
            ESP_LOGE(TAG, "Unknow Command [0x%02x%02x]", val[1], val[0]);
        }
        break;
    case RECV_FW_CHAR:
        if (start_ota) {
            // Calculating the received sector index
            recv_sector = (val[0] + (val[1] * 256));

            if (recv_sector != cur_sector) { // sector error
                if (recv_sector == 0xffff) { // last sector
                    ESP_LOGI(TAG, "Laster sector");
                } else {  // sector error
                    ESP_LOGE(TAG, "Sector index error, cur: %d, recv: %d", cur_sector, recv_sector);
                    esp_ble_ota_send_ack_data(BLE_OTA_FW_ACK, BLE_OTA_FW_IND_ERR, recv_sector);
                }
            }

            if (val[2] != cur_packet) { // packet seq error
                if (val[2] == 0xff) { // last packet
                    ESP_LOGI(TAG, "laster packet");
                    goto write_ota_data;
                } else { // packet seq error
                    ESP_LOGE(TAG, "Packet index error, cur: %d, recv: %d", cur_packet, val[2]);
                }
            }
write_ota_data:
            memcpy(fw_buf + fw_buf_offset, val + 3, val_len - 3);
            fw_buf_offset += val_len - 3;
            ESP_LOGI(TAG, "DEBUG: Sector:%d, total length:%d, length:%d", cur_sector, fw_buf_offset, val_len - 3);
            if (val[2] == 0xff) {
                cur_packet = 0;
                cur_sector++;
                ESP_LOGD(TAG, "DEBUG: recv %d sector", cur_sector);
                goto sector_end;
            } else {
                cur_packet++;
            }

            break;
sector_end:
            esp_ble_ota_recv_fw_handler(fw_buf, 4096);
            memset(fw_buf, 0x0, 4096);
            fw_buf_offset = 0;
            esp_ble_ota_send_ack_data(BLE_OTA_FW_ACK, BLE_OTA_FW_SUCCESS, recv_sector);
        } else {
            ESP_LOGE(TAG, "BLE OTA hasn't started yet");
        }
        break;
    case OTA_STATUS_CHAR:
        break;
    case CUS_CHAR:
        break;
    default:
        ESP_LOGW(TAG, "Invalid data was received, char[%d]", ota_char);
        break;
    }
}

static void gatts_ota_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_err_t ret;
    esp_ble_ota_char_t ota_char;

    ESP_LOGI(TAG, "%s - event: %d", __func__, event);

    switch (event) {
    case ESP_GATTS_REG_EVT:
        {
            ret = esp_ble_gatts_create_attr_tab(ota_gatt_db, gatts_if, OTA_IDX_NB, OTA_PROFILE_APP_IDX);
            if (ret) {
                ESP_LOGE(TAG, "Create attr table failed, error code = %x", ret);
            }
        }
        break;
    case ESP_GATTS_READ_EVT:
        ota_char = find_ota_char_and_desr_by_handle(param->read.handle);
        ESP_LOGI(TAG, "Read event - ota_char: %d", ota_char);
        break;
    case ESP_GATTS_WRITE_EVT:
        ota_char = find_ota_char_and_desr_by_handle(param->write.handle);
        ESP_LOGD(TAG, "Write event - ota_char: %d", ota_char);
        // Enable indication
        if ((param->write.len == 2) && (param->write.value[0] == 0x02) && (param->write.value[1] == 0x00)) {
            if (ota_char == OTA_STATUS_CHAR_CCC) {
                ota_notification.process_bar_ntf_enable = true;
            }
            if (ota_char == RECV_FW_CHAR_CCC) {
                ota_notification.recv_fw_ntf_enable = true;
            }
            if (ota_char == CMD_CHAR_CCC) {
                ota_notification.command_ntf_enable = true;
            }
            if (ota_char == CUS_CHAR_CCC) {
                ota_notification.customer_ntf_enable = true;
            }
        }
        // Disable indication
        else if ((param->write.len == 2) && (param->write.value[0] == 0x00) && (param->write.value[1] == 0x00)) {
            if (ota_char == OTA_STATUS_CHAR_CCC) {
                ota_notification.process_bar_ntf_enable = false;
            }
            if (ota_char == RECV_FW_CHAR_CCC) {
                ota_notification.recv_fw_ntf_enable = false;
            }
            if (ota_char == CMD_CHAR_CCC) {
                ota_notification.command_ntf_enable = false;
            }
            if (ota_char == CUS_CHAR_CCC) {
                ota_notification.customer_ntf_enable = false;
            }
        }

        if (param->write.is_prep == false) {
            esp_ble_ota_process_recv_data(ota_char, param->write.value, param->write.len);
        } else {
            if (temp_prep_write_buf == NULL) {
                temp_prep_write_buf = (uint8_t *)malloc(BLE_OTA_MAX_CHAR_VAL_LEN * sizeof(uint8_t));
                if (temp_prep_write_buf == NULL) {
                    ESP_LOGE(TAG, "Malloc buffer for prep write fail");
                    break;
                }

                memset(temp_prep_write_buf, 0x0, BLE_OTA_MAX_CHAR_VAL_LEN);
                temp_buf_len = 0;
            }

            memcpy(temp_prep_write_buf + temp_buf_len, param->write.value, param->write.len);
            temp_buf_len += param->write.len;
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
            if (temp_prep_write_buf) {
                esp_ble_ota_process_recv_data(RECV_FW_CHAR, temp_prep_write_buf, temp_buf_len);
            }
        } else {
            if (temp_prep_write_buf) {
                free(temp_prep_write_buf);
                temp_prep_write_buf = NULL;
            }

            temp_buf_len = 0;
        }
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT - mtu = %d", param->mtu.mtu);
        bms_profile_tab[OTA_PROFILE_APP_IDX].mtu_size = param->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        if (param->start.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "SERVICE START FAIL, status %d", param->start.status);
            break;
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        bms_profile_tab[OTA_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                    param->connect.conn_id,
                    param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                    param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        bms_profile_tab[OTA_PROFILE_APP_IDX].mtu_size = 23;
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != OTA_IDX_NB) {
            ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) doesn't equal to OTA_IDX_NB(%d)", param->add_attr_tab.num_handle, OTA_IDX_NB);
        } else {
            ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(ota_handle_table, param->add_attr_tab.handles, sizeof(ota_handle_table));
            esp_ble_gatts_start_service(ota_handle_table[OTA_SVC_IDX]);
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            bms_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < OTA_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == bms_profile_tab[idx].gatts_if) {
                if (bms_profile_tab[idx].gatts_cb) {
                    bms_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
    
}

static esp_err_t esp_ble_ota_set_characteristic_value(esp_ble_ota_service_index_t index, uint8_t *value, uint8_t length) {
    esp_err_t ret;

    ret = esp_ble_gatts_set_attr_value(ota_handle_table[index], length, value);
    if (ret) {
        ESP_LOGE(TAG, "%s set attribute value fail: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t esp_ble_ota_set_value(esp_ble_ota_char_t ota_char, uint8_t *value, uint8_t length) {
    esp_err_t ret;

    switch (ota_char) {
    case RECV_FW_CHAR:
        ret = esp_ble_ota_set_characteristic_value(RECV_FW_CHAR_VAL_IDX, value, length);
        break;
    case OTA_STATUS_CHAR:
        ret = esp_ble_ota_set_characteristic_value(OTA_STATUS_CHAR_VAL_IDX, value, length);
        break;
    case CMD_CHAR:
        ret = esp_ble_ota_set_characteristic_value(CMD_CHAR_VAL_IDX, value, length);
        break;
    case CUS_CHAR:
        ret = esp_ble_ota_set_characteristic_value(CUS_CHAR_VAL_IDX, value, length);
        break;
    default:
        ret = ESP_FAIL;
        break;
    }

    if (ret) {
        ESP_LOGE(TAG, "%s set characteristic value fail: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t esp_ble_ota_send_indication(esp_ble_ota_service_index_t index, uint8_t *value, uint8_t length, bool need_ack) {
    esp_err_t ret;
    uint16_t offset = 0;

    if (length <= (bms_profile_tab[OTA_PROFILE_APP_IDX].mtu_size - 3)) {
        ret = esp_ble_gatts_send_indicate(bms_profile_tab[OTA_PROFILE_APP_IDX].gatts_if, bms_profile_tab[OTA_PROFILE_APP_IDX].conn_id, ota_handle_table[index], length, value, need_ack);
        if (ret) {
            ESP_LOGE(TAG, "%s send notification fail: %s\n", __func__, esp_err_to_name(ret));
            return ESP_FAIL;
        }
    } else {
        while ((length - offset) > (bms_profile_tab[OTA_PROFILE_APP_IDX].mtu_size - 3)) {
            ret = esp_ble_gatts_send_indicate(bms_profile_tab[OTA_PROFILE_APP_IDX].gatts_if, bms_profile_tab[OTA_PROFILE_APP_IDX].conn_id, ota_handle_table[index], (bms_profile_tab[OTA_PROFILE_APP_IDX].mtu_size - 3), value + offset, need_ack);
            if (ret) {
                ESP_LOGE(TAG, "%s send notification fail: %s\n", __func__, esp_err_to_name(ret));
                return ESP_FAIL;
            }
            offset += (bms_profile_tab[OTA_PROFILE_APP_IDX].mtu_size - 3);
        }

        if ((length - offset) > 0) {
            ret = esp_ble_gatts_send_indicate(bms_profile_tab[OTA_PROFILE_APP_IDX].gatts_if, bms_profile_tab[OTA_PROFILE_APP_IDX].conn_id, ota_handle_table[index], (length - offset), value + offset, need_ack);
            if (ret) {
                ESP_LOGE(TAG, "%s send notification fail: %s\n", __func__, esp_err_to_name(ret));
                return ESP_FAIL;
            }
        }
    }
    return ESP_OK;
}

esp_err_t esp_ble_ota_notification_data(esp_ble_ota_char_t ota_char, uint8_t *value, uint8_t length) {
    esp_err_t ret = ESP_FAIL;
    switch (ota_char) {
    case RECV_FW_CHAR:
        if (ota_notification.recv_fw_ntf_enable) {
            ret = esp_ble_ota_send_indication(RECV_FW_CHAR_VAL_IDX, value, length, false);
        } else {
            ESP_LOGE(TAG, "notify isn't enable");
        }
        break;
    case OTA_STATUS_CHAR:
        if (ota_notification.process_bar_ntf_enable) {
            ret = esp_ble_ota_send_indication(OTA_STATUS_CHAR_VAL_IDX, value, length, false);
        } else {
            ESP_LOGE(TAG, "notify isn't enable");
        }
        break;
    case CMD_CHAR:
        if (ota_notification.command_ntf_enable) {
            ret = esp_ble_ota_send_indication(CMD_CHAR_VAL_IDX, value, length, false);
        } else {
            ESP_LOGE(TAG, "notify isn't enable");
        }
        break;
    case CUS_CHAR:
        if (ota_notification.customer_ntf_enable) {
            ret = esp_ble_ota_send_indication(CUS_CHAR_VAL_IDX, value, length, false);
        } else {
            ESP_LOGE(TAG, "notify isn't enable");
        }
        break;
    default:
        ret = ESP_FAIL;
        break;
    }

    if (ret) {
        ESP_LOGE(TAG, "%s notification fail: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t esp_ble_ota_recv_fw_data_callback(esp_ble_ota_recv_fw_cb_t callback) {
    ota_cb_fun_t.recv_fw_cb = callback;
    return ESP_OK;
}

bool bluetoothInit() {
    ESP_LOGI(TAG, "Initializing Bluetooth");
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
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return false;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return false;
    }

    esp_ble_gatts_app_register(BMS_PROFILE_APP_IDX);
    esp_ble_gatts_app_register(OTA_PROFILE_APP_IDX);
    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    ret = esp_ble_gatt_set_local_mtu(517);
    if (ret) {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
        return false;
    }
    ret = esp_ble_gap_set_device_name(ble_device_name);
    if (ret) {
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
    ESP_LOGI(TAG, "BMS update task started");
    for (;;) {
        bms.update();
        fault_ = bms.getErrorState();
        if (fault_ != previous_fault_value_ && fault_notifications_enabled_ && fault_ != previous_fault_value_) {
            esp_ble_gatts_send_indicate(bms_profile_tab[BMS_PROFILE_APP_IDX].gatts_if, bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id, info_handle_table_[IDX_CHAR_VAL_FAULT], 1, &fault_, false);
            previous_fault_value_ = fault_;
        }
        
        if(only_balance_when_charging_ && bms.getBatteryCurrent() < 0.05) {
            bms.toggleBalancing(false);
            enable_balancing_ = false;
        }
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void sendNotificationsTask(void *pvParameters) {
    ESP_LOGI(TAG, "Send notifications task started");
    for(;;) {
        if(connected_) {
            retrieveValues();
            if(previous_enable_balancing_ != enable_balancing_ && balancing_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab[BMS_PROFILE_APP_IDX].gatts_if, bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id, info_handle_table_[IDX_CHAR_VAL_BALANCING], 1, (uint8_t*)&enable_balancing_, false);
                previous_enable_balancing_ = enable_balancing_;
            }
            if(previous_enable_charging_ != enable_charging_ && charging_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab[BMS_PROFILE_APP_IDX].gatts_if, bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id, info_handle_table_[IDX_CHAR_VAL_CHARGING], 1, (uint8_t*)&enable_charging_, false);
                previous_enable_charging_ = enable_charging_;
            }
            if(!std::equal(std::begin(previous_battery_cell_voltage_), std::end(previous_battery_cell_voltage_), std::begin(battery_cell_voltage_)) && cell_voltage_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab[BMS_PROFILE_APP_IDX].gatts_if, bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id, info_handle_table_[IDX_CHAR_VAL_BATTERY_CELL_VOLTAGE], 20, (uint8_t*)&battery_cell_voltage_, false);
                memcpy(previous_battery_cell_voltage_, battery_cell_voltage_, sizeof(battery_cell_voltage_));
            }
            if(!std::equal(std::begin(previous_cell_balancing_state_), std::end(previous_cell_balancing_state_), std::begin(cell_balancing_state_)) && cell_balancing_state_notifications_enabled_) {
                esp_ble_gatts_send_indicate(bms_profile_tab[BMS_PROFILE_APP_IDX].gatts_if, bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id, info_handle_table_[IDX_CHAR_VAL_CELL_BALANCING_STATE], 10, (uint8_t*)&cell_balancing_state_, false);
                memcpy(previous_cell_balancing_state_, cell_balancing_state_, sizeof(cell_balancing_state_));
            }
            if((previous_battery_voltage_ != battery_voltage_ || previous_charge_current_ != charge_current_) && voltage_current_notifications_enabled_) {
                dummy_[0] = battery_voltage_;
                dummy_[1] = charge_current_;
                esp_ble_gatts_send_indicate(bms_profile_tab[BMS_PROFILE_APP_IDX].gatts_if, bms_profile_tab[BMS_PROFILE_APP_IDX].conn_id, info_handle_table_[IDX_CHAR_VAL_BAT_VOLTAGE_CURRENT], 4, (uint8_t*)&dummy_, false);
                previous_battery_voltage_ = battery_voltage_;
                previous_charge_current_ = charge_current_;
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

bool ble_ota_ringbuf_init(uint32_t ringbuf_size) {
    otaRingbufHandle = xRingbufferCreate(ringbuf_size, RINGBUF_TYPE_BYTEBUF);
    if (otaRingbufHandle == NULL) {
        return false;
    }
    return true;
}

size_t write_to_ringbuf(const uint8_t *data, size_t size) {
    BaseType_t done = xRingbufferSend(otaRingbufHandle, (void *)data, size, (TickType_t)portMAX_DELAY);
    if (done) {
        return size;
    } else {
        return 0;
    }
}

void ota_recv_fw_cb(uint8_t *buf, uint32_t length) {
    write_to_ringbuf(buf, length);
}

void ota_task(void *arg) {
    esp_partition_t *partition_ptr = NULL;
    esp_partition_t partition;
    const esp_partition_t *next_partition = NULL;
    esp_ota_handle_t out_handle = 0;

    uint32_t recv_len = 0;
    uint8_t *data = NULL;
    size_t item_size = 0;
    ESP_LOGI(TAG, "ota_task start");
    partition_ptr = (esp_partition_t *)esp_ota_get_boot_partition();
    if (partition_ptr == NULL) {
        ESP_LOGE(TAG, "boot partition NULL!\r\n");
        goto OTA_ERROR;
    }
    if (partition_ptr->type != ESP_PARTITION_TYPE_APP) {
        ESP_LOGE(TAG, "esp_current_partition->type != ESP_PARTITION_TYPE_APP\r\n");
        goto OTA_ERROR;
    }

    if (partition_ptr->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
    } else {
        next_partition = esp_ota_get_next_update_partition(partition_ptr);
        if (next_partition) {
            partition.subtype = next_partition->subtype;
        } else {
            partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
        }
    }
    partition.type = ESP_PARTITION_TYPE_APP;

    partition_ptr = (esp_partition_t *)esp_partition_find_first(partition.type, partition.subtype, NULL);
    if (partition_ptr == NULL) {
        ESP_LOGE(TAG, "partition NULL!\r\n");
        goto OTA_ERROR;
    }

    memcpy(&partition, partition_ptr, sizeof(esp_partition_t));
    if (esp_ota_begin(&partition, OTA_SIZE_UNKNOWN, &out_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed!\r\n");
        goto OTA_ERROR;
    }

    ESP_LOGI(TAG, "wait for data from ringbuf! fw_len = %u", esp_ble_ota_get_fw_length());
    for (;;) {
        data = (uint8_t *)xRingbufferReceive(otaRingbufHandle, &item_size, (TickType_t)portMAX_DELAY);

        ESP_LOGI(TAG, "recv: %u, recv_total:%"PRIu32"\n", item_size, recv_len + item_size);
        if (item_size != 0) {
            if (esp_ota_write(out_handle, (const void *)data, item_size) != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_write failed!\r\n");
                goto OTA_ERROR;
            }
            recv_len += item_size;
            vRingbufferReturnItem(otaRingbufHandle, (void *)data);

            if (recv_len >= esp_ble_ota_get_fw_length()) {
                break;
            }
        }
    }

    if (esp_ota_end(out_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!\r\n");
        goto OTA_ERROR;
    }

    if (esp_ota_set_boot_partition(&partition) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed!\r\n");
        goto OTA_ERROR;
    }

    esp_restart();

    OTA_ERROR:
        ESP_LOGE(TAG, "OTA failed");
        vTaskDelete(NULL);
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void rmtInit() {
    ESP_LOGI(TAG, "Initialise RMT");
    esp_err_t ret;
    rmt_channel_handle_t led_chan = NULL;
    rmt_rx_channel_config_t rx_chan_config = {
        .gpio_num = (gpio_num_t)rmt_pin_,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
    };
    
    ret = rmt_new_rx_channel(&rx_chan_config, &rx_channel);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT RX channel");
        return;
    }
    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    if(receive_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create receive queue");
        return;
    }
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ret = rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register RMT RX event callback");
        return;
    }
    receive_config = {
        .signal_range_min_ns = 2000,
        .signal_range_max_ns = 3000000,
    };
    ret = rmt_enable(rx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT RX channel");
        return;
    }
    ret = gpio_set_direction((gpio_num_t)lights_pin_, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set lights pin GPIO direction");
        return;
    }
}

void rmtTask(void *pvParameters) {
    ESP_LOGI(TAG, "RMT task started");
    rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);
    for(;;) {
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            //ESP_LOGI(TAG, "Pulse width: %d µs", rx_data.received_symbols->duration0);
            if(rx_data.received_symbols->duration0 < braking_threshold_us_) {
                gpio_set_level((gpio_num_t)lights_pin_, 1);
            } else {
                gpio_set_level((gpio_num_t)lights_pin_, 0);
            }
            rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);
        }
    }
    vTaskDelete(NULL);
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
   
    if (!ble_ota_ringbuf_init(OTA_RINGBUF_SIZE)) {
        ESP_LOGE(TAG, "%s init ringbuf fail", __func__);
        return;
    }
    bluetoothInit();
    esp_ble_ota_recv_fw_data_callback(ota_recv_fw_cb);
    rmtInit();
    xTaskCreate(ota_task, "otaTask", OTA_TASK_SIZE, NULL, 5, NULL);
    xTaskCreate(bmsUpdateTask, "bmsUpdateTask", 2048, NULL, 5, NULL);
    xTaskCreate(sendNotificationsTask, "sendNotificationsTask", 2048, NULL, 5, NULL);
    xTaskCreate(rmtTask, "rmtTask", 3000, NULL, 5, NULL);	
    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gap_start_advertising(&adv_params);
}
