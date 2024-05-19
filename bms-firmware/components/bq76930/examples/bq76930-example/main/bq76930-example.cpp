#include <stdio.h>
#include <bq76930.h>
#include "driver/i2c.h"
#include "esp_log.h"


int sda_pin = 5;
int scl_pin = 4;

int boot_pin = 6;
int alert_pin = 7;

bq76930 bms(0x08, sda_pin, scl_pin);

extern "C" { void app_main(void);}


void update_task(void *pvParameter) {
    while(1) {
        bms.update();
        //ESP_LOGI("BMS", "Current: %d mA", bms.getBatteryCurrent());
        // ESP_LOGI("BMS", "Voltage: %d mV", bms.getBatteryVoltage());
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    bms.initialize(alert_pin, boot_pin);
    bms.setTemperatureLimits(-20, 45, 0, 45);
    bms.setShuntResistorValue(5);
    bms.setOvercurrentChargeProtection(5000);
    bms.setCellUndervoltageProtection(3200, 2);
    bms.setCellOvervoltageProtection(4240, 2);
    bms.setBalancingThresholds(0, 3700, 15);
    bms.setIdleCurrentThreshold(100);
    bms.enableAutoBalancing();
    xTaskCreate(&update_task, "update_task", 2048, NULL, 5, NULL);
    bms.enableCharging();
    while(1) {
        
        //ESP_LOGI("BMS", "Current: %d mA", bms.getBatteryCurrent());
        // printf("cell1: %d\n", bms.getCellVoltage(1));
        // printf("cell2: %d\n", bms.getCellVoltage(2));
        // printf("cell3: %d\n", bms.getCellVoltage(3));
        // printf("cell4: %d\n", bms.getCellVoltage(4));
        // printf("cell5: %d\n", bms.getCellVoltage(5));
        // printf("cell6: %d\n", bms.getCellVoltage(6));
        // printf("cell7: %d\n", bms.getCellVoltage(7));
        // printf("cell8: %d\n", bms.getCellVoltage(8));
        // printf("cell9: %d\n", bms.getCellVoltage(9));
        // printf("cell10: %d\n", bms.getCellVoltage(10));
        // printf("current: %d\n", bms.getBatteryCurrent());
        ESP_LOGI("BMS", "Voltage: %d mV", bms.getBatteryVoltage());
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


