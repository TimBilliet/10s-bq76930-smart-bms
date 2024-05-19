#include <stdio.h>
#include "bq76930.h"
#include "registers.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "BS76930_BMS";

bq76930* bq76930::instance_pointer_ = 0;

bq76930::bq76930(uint8_t address, int sda_pin, int scl_pin) {
    address_ = address;
    cell_count_ = 10;
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
}
void printBinary(unsigned int num) {
    int size = 8;
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

esp_err_t bq76930::initialize(int alert_pin, int boot_pin) {
    esp_err_t err;
    err = i2c_conf(sda_pin_, scl_pin_);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C initialisation error!\n");
    ESP_LOGI(TAG, "I2C initialized\n");
    for (uint8_t i = 0; i < 9; i++) {
        cell_voltages_[i] = 0;
    }
    if (boot_pin >= 0) {
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << boot_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
        };
        err = gpio_config(&conf);
        ESP_RETURN_ON_ERROR(err, TAG, "Boot pin initialisation error!");
        gpio_set_level((gpio_num_t)boot_pin,1);
        vTaskDelay(5 / portTICK_PERIOD_MS); // Atleast 2ms 
        gpio_set_level((gpio_num_t)boot_pin,0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "IC boot sequence done");
    }
    if(determineAddressAndCrc()) {
        ESP_LOGI(TAG, "Address and CRC detected successfully\n");
        ESP_LOGI(TAG, "Address: 0x%x\n", address_);
        ESP_LOGI(TAG, "CRC: %d\n", crc_enabled_);
        writeRegister(SYS_CTRL1, 0b00010000); // Turn ADC on and use internal die temp
        writeRegister(SYS_CTRL2, 0b01000000); // switch CC_EN on
        instance_pointer_ = this;
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << alert_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_POSEDGE,
        };
        err = gpio_config(&conf);
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
        gpio_isr_handler_add((gpio_num_t)alert_pin, alertISR, NULL);
        ESP_LOGI(TAG, "Interrupt configured\n");

        readRegister(ADCOFFSET, &adc_offset_, sizeof(adc_offset_));

        uint8_t tempgain1 = 0;
        uint8_t tempgain2 = 0;
        readRegister(ADCGAIN1, &tempgain1, sizeof(tempgain1));
        readRegister(ADCGAIN2, &tempgain2, sizeof(tempgain2));
        adc_gain_ = 365 + (((tempgain1 & 0b00001100) << 1) | ((tempgain2 & 0b11100000) >> 5));
        ESP_LOGI(TAG, "ADC configured, offset: %d, gain: %d\n", adc_offset_, adc_gain_);

    } else {
        ESP_LOGE(TAG, "BMS communication error\n");
    }
    return err;
}

esp_err_t bq76930::i2c_conf(int sda_pin, int scl_pin) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master={.clk_speed = 100000,},
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    assert(ESP_OK == err);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

esp_err_t bq76930::readRegister(uint8_t reg_addr, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(I2C_NUM_0, address_, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

esp_err_t bq76930::writeRegister(uint8_t reg_addr, uint8_t data) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err = i2c_master_start(cmd);
    assert(ESP_OK == err);
    err = i2c_master_write_byte(cmd, (address_ << 1) | I2C_MASTER_WRITE, true);
    assert(ESP_OK == err);
    err = i2c_master_write_byte(cmd, reg_addr, true);
    assert(ESP_OK == err);
    err = i2c_master_write_byte(cmd, data, true);
    assert(ESP_OK == err);
    err = i2c_master_stop(cmd);
    assert(ESP_OK == err);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

bool bq76930::determineAddressAndCrc() {
    uint8_t data;
    address_ = 0x08;
    crc_enabled_ = true;
    writeRegister(CC_CFG, 0x19);
    readRegister(CC_CFG, &data, sizeof(data));
    if (data == 0x19) {
        return true;
    }
    address_ = 0x18;
    crc_enabled_ = true;
    
    writeRegister(CC_CFG, 0x19);
    readRegister(CC_CFG, &data, sizeof(data));
    if (data == 0x19) {
        return true;
    }

    address_ = 0x08;
    crc_enabled_ = false;

    writeRegister(CC_CFG, 0x19);
    readRegister(CC_CFG, &data, sizeof(data));

    if (data == 0x19) {
        return true;
    }

    address_ = 0x18;
    crc_enabled_ = false;
    writeRegister(CC_CFG, 0x19);
    readRegister(CC_CFG, &data, sizeof(data));

    if (data == 0x19) {
        return true;
    }
    
    return false;
}

void bq76930::setAlertInterruptFlag() {
  interrupt_timestamp_ = (uint32_t)(esp_timer_get_time() / 1000);
  alert_interrupt_flag_ = true;
}

void IRAM_ATTR bq76930::alertISR(void* data) {
  if (instance_pointer_ != 0)
  {
    instance_pointer_->setAlertInterruptFlag();
  }
}

int bq76930::checkStatus() {
  if (alert_interrupt_flag_ == false && error_state_ == 0) {
    return 0;
  }
  else {
    regSYS_STAT_t sys_stat;
    //sys_stat.regByte = readRegister(SYS_STAT);
    readRegister(SYS_STAT, &sys_stat.regByte, sizeof(sys_stat.regByte));
    if (sys_stat.bits.CC_READY == 1) {
      //Serial.println("Interrupt: CC ready");
      updateCurrent(true);  // automatically clears CC ready flag	
    }
    
    // Serious error occured
    if (sys_stat.regByte & 0b00111111) {
      if (alert_interrupt_flag_ == true) {
        sec_since_error_counter_ = 0;
      }
      error_state_ = sys_stat.regByte;
      
      int sec_since_interrupt = (esp_timer_get_time() - interrupt_timestamp_) / 1000;
      
      // check for overrun of millis() or very slow running program
      if (abs(sec_since_interrupt - sec_since_error_counter_) > 2) {
        sec_since_error_counter_ = sec_since_interrupt;
      }
      
      // called only once per second
      if (sec_since_interrupt >= sec_since_error_counter_) {
        if (sys_stat.regByte & 0b00100000) { // XR error
          // datasheet recommendation: try to clear after waiting a few seconds
          if (sec_since_error_counter_ % 3 == 0) {
            ESP_LOGE(TAG, "Attempting to clear XR error");
            writeRegister(SYS_STAT, 0b00100000);
          }
        }
        if (sys_stat.regByte & 0b00010000) { // Alert error
          if (sec_since_error_counter_ % 10 == 0) {
            ESP_LOGE(TAG, "Attempting to clear Alert error");
            writeRegister(SYS_STAT, 0b00010000);
          }
        }
        if (sys_stat.regByte & 0b00001000) { // UV error
          updateVoltages();
          if (cell_voltages_[id_cell_min_voltage_] > min_cell_voltage_) {
            ESP_LOGE(TAG, "Attempting to clear UV error");
            writeRegister(SYS_STAT, 0b00001000);
          }
        }
        if (sys_stat.regByte & 0b00000100) { // OV error
          updateVoltages();
          if (cell_voltages_[id_cell_max_voltage_] < max_cell_voltage_) {
            ESP_LOGE(TAG, "Attempting to clear OV error");
            writeRegister(SYS_STAT, 0b00000100);
          }
        }
        if (sys_stat.regByte & 0b00000010) { // SCD
          if (sec_since_error_counter_ % 60 == 0) {
            ESP_LOGE(TAG, "Attempting to clear SCD error");
            writeRegister(SYS_STAT, 0b00000010);
          }
        }
        if (sys_stat.regByte & 0b00000001) { // OCD
          if (sec_since_error_counter_ % 60 == 0) {
            ESP_LOGE(TAG, "Attempting to clear OCD error");
            writeRegister(SYS_STAT, 0b00000001);
          }
        }
        sec_since_error_counter_++;
      }
    }
    else {
      error_state_ = 0;
    }
    return error_state_;
  }
}

uint8_t _crc8_ccitt_update (uint8_t in_crc, uint8_t in_data) {
    uint8_t i;
    uint8_t data;
    data = in_crc ^ in_data;
    for ( i = 0; i < 8; i++ ) {
        if (( data & 0x80 ) != 0 ) {
            data <<= 1;
            data ^= 0x07;
        } else {
            data <<= 1;
        }
    }
    return data;
}

void bq76930::update() {
  updateCurrent();  // will only read new current value if alert was triggered
  updateVoltages();
  updateTemperatures();
  updateBalancingSwitches();
}

void bq76930::updateCurrent(bool ignore_CC_ready_flag) {
    int adc_val = 0;
    regSYS_STAT_t sys_stat;
    //   sys_stat.regByte = readRegister(SYS_STAT);
    readRegister(SYS_STAT, &sys_stat.regByte, sizeof(sys_stat.regByte));
    
    if (ignore_CC_ready_flag == true || sys_stat.bits.CC_READY == 1) {
        uint8_t cc_high;
        uint8_t cc_low;
        
        readRegister(CC_HI_BYTE, &cc_high, sizeof(cc_high)); // changed CC address to address from header file
        readRegister(CC_LO_BYTE, &cc_low, sizeof(cc_low));
        //adc_val = (readRegister(CC_HI_BYTE) << 8) | readRegister(CC_LO_BYTE);
        adc_val = cc_high << 8 | cc_low;
        //bat_current_ = adc_val * 8.44 / 5.0;  // mA

        bat_current_ = adc_val * 8.44 / shunt_value_mOhm_;  // mA
        if (bat_current_ > 60000) {
            bat_current_ = 0;
        }
        // reset idleTimestamp
        if (abs(bat_current_) > idle_current_threshold_) {
            idle_timestamp_ = esp_timer_get_time();
        }
        // no error occured which caused alert
        if (!(sys_stat.regByte & 0b00111111)) {
            alert_interrupt_flag_ = false;
        }
        writeRegister(SYS_STAT, 0b10000000);  // Clear CC ready flag	
    }
}

void bq76930::updateVoltages() {
    int connected_cells = 0;
    id_cell_max_voltage_ = 0; //resets to zero before writing values to these vars
    id_cell_min_voltage_ = 0;
    uint8_t crc = 0;
    uint8_t buf[4] = {0, 0, 0, 0};
    //buf[0] = VC2_HI_BYTE; // start with the first cell
    uint16_t adc_val = 0;
    // Read battery pack voltage
    for (int i = 0; i < number_of_cells_; i++) { // will run once for each cell up to the total num of cells
        if (crc_enabled_ == true) {
            uint8_t hi_byte = VC_HI_BYTES[i];
            //i2c_master_write_read_device(I2C_NUM_0, address_, &VC_HI_BYTES[i], 1, buf, 4, 1000 / portTICK_PERIOD_MS);
            readRegister(VC_HI_BYTES[i], buf, sizeof(buf));
			// 1st check if CRC matches data bytes
			// CRC of first bytes includes slave address (including R/W bit) and data
            crc = _crc8_ccitt_update(0, (address_ << 1) | 1);
            crc = _crc8_ccitt_update(crc, buf[0]);
            if (crc != buf[1]){
                ESP_LOGE(TAG, "CRC check fail at 1st byte read - crc of 1st bytes and data doesn't match");
				return; //don't save corrupt value and exit
			}
				
			// CRC of subsequent bytes contain only data
            crc = _crc8_ccitt_update(0, buf[2]);
            if (crc != buf[3]) {
                ESP_LOGE(TAG, "CRC check failed upon read - crc of 2nd byte and data doesn't match");
				return; // don't save corrupted value
			}
        } else { // If CRC is disabled only read 2 bytes and call it a day :)
            // Wire.requestFrom(I2CAddress, 2); // requests 4 bytes: 1)hi Byte 2)Hi byte CRC 3) Lo Byte 4) Lo byte crc
            // 	buf[0] = Wire.read(); // hi data - note that only bottom 6 bits are good
            // 	buf[2] = Wire.read(); // lo data byte - all 8 bits are used
            readRegister(VC_HI_BYTES[i], buf, sizeof(buf));
        }

        // Get Cell Voltages now
        adc_val = (buf[0] & 0b00111111) << 8 | buf[2];  // reads 1st byte from register and drop the first two bits, shift left then use lo data
        cell_voltages_[i] = adc_val * adc_gain_ / 1000 + adc_offset_; // calculates real voltage in mV
        // Filter out fake voltage readings from non-connected cells
        if (cell_voltages_[i] > 500) {  
            connected_cells++; //adds one to the temporary cell counter var - only readings above 500mV are counted towards real cell count
        }

        if (cell_voltages_[i] > cell_voltages_[id_cell_max_voltage_]) { // if the current cell voltage is higher than the last cells, bump this up
            id_cell_max_voltage_ = i;
        }
        if (cell_voltages_[i] < cell_voltages_[id_cell_min_voltage_] && cell_voltages_[i] > 500) {
            id_cell_min_voltage_ = i;
        }
    }

    uint8_t bat_high;
    uint8_t bat_low;
    readRegister(BAT_HI_BYTE, &bat_high, sizeof(bat_high));
    readRegister(BAT_LO_BYTE, &bat_low, sizeof(bat_low));
    adc_val = bat_high << 8 | bat_low;
    bat_voltage_ = 4.0 * adc_gain_ * adc_val / 1000.0 + connected_cells * adc_offset_;
    // printf("bat_voltage_: %d\n", bat_voltage_);
}

void bq76930::updateTemperatures() {
    //not implemented
}

void bq76930::updateBalancingSwitches() {
    long idle_seconds = (esp_timer_get_time() - idle_timestamp_) / 1000;
    uint8_t number_of_sections = number_of_cells_/5;
    
    // Check for esp_timer_get_time() overflow
    if (idle_seconds < 0) {
        idle_timestamp_ = 0;
        idle_seconds = esp_timer_get_time() / 1000;
    }
        
    // Check if balancing allowed
    if (checkStatus() == 0 &&
    idle_seconds >= balancing_min_idle_time_s_ && 
    cell_voltages_[id_cell_max_voltage_] > balancing_min_cell_voltage_mV_ &&
    (cell_voltages_[id_cell_max_voltage_] - cell_voltages_[id_cell_min_voltage_]) > balancing_max_voltage_difference_mV_) {

        balancing_active_ = true;
        
        regCELLBAL_t cellbal;
        uint8_t balancing_flags;
        uint8_t balancing_flags_target;
        
        for (int section = 0; section < number_of_sections; section++) {
        balancing_flags = 0;
        for (int i = 0; i < 5; i++) {
            if ((cell_voltages_[section*5 + i] - cell_voltages_[id_cell_min_voltage_]) > balancing_max_voltage_difference_mV_) {
            // try to enable balancing of current cell
            balancing_flags_target = balancing_flags | (1 << i);

            // check if attempting to balance adjacent cells
            bool adjacent_cell_collision = 
                ((balancing_flags_target << 1) & balancing_flags) ||
                ((balancing_flags << 1) & balancing_flags_target);
                
            if (adjacent_cell_collision == false) {
                balancing_flags = balancing_flags_target;
            }          
            }
        }
        ESP_LOGI(TAG, "Setting CELLBAL %d register to %x", (section + 1), balancing_flags);
        // Set balancing register for this section
        writeRegister(CELLBAL1 + section, balancing_flags);
        }
    } else if (balancing_active_ == true) {  
        // clear all CELLBAL registers
        for (int section = 0; section < number_of_sections; section++) {
            ESP_LOGI(TAG, "Clearing Register CELLBAL %d", (section + 1));
            writeRegister(CELLBAL1 + section, 0x0);
        }
        balancing_active_ = false;
    }
}



void bq76930::shutdown() {
    ESP_LOGI(TAG, "Shutting down BQ76930");
    writeRegister(SYS_CTRL1, 0x0);
    writeRegister(SYS_CTRL1, 0x1);
    writeRegister(SYS_CTRL1, 0x2);
    
}

bool bq76930::enableCharging(bool enable_charging) {
    charging_status_ = enable_charging;
    if (enable_charging == true) {
        // uint8_t sys_ctrl2;
        // readRegister(SYS_CTRL2, &sys_ctrl2, sizeof(sys_ctrl2));
        // writeRegister(SYS_CTRL2, sys_ctrl2 & 0b11111110);  // switch CHG off
        // ESP_LOGI(TAG,"Disabling CHG FET");
        // return true;
        if (checkStatus() == 0 &&
        cell_voltages_[id_cell_max_voltage_] < max_cell_voltage_ &&
        temperatures_[0] < max_celltemp_charge_ &&
        temperatures_[0] > min_celltemp_charge_) {
        uint8_t sys_ctrl2;
        readRegister(SYS_CTRL2, &sys_ctrl2, sizeof(sys_ctrl2));
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000001);  // switch CHG on
        ESP_LOGI(TAG,"CHG FET enabled");
        return true;
        } else {
            ESP_LOGI(TAG,"Charging not allowed");
            return false;
        }
    } else {
        uint8_t sys_ctrl2;
        readRegister(SYS_CTRL2, &sys_ctrl2, sizeof(sys_ctrl2));
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b11111110);  // switch CHG off
        ESP_LOGI(TAG,"CHG FET disabled");
        return true;
    }
    
}

bool bq76930::enableDischarging(bool enable_discharge) {
    return false; //not implemented
}

void bq76930::enableAutoBalancing(bool enable_balancing) {
    auto_balancing_enabled_ = enable_balancing;
}

void bq76930::setBalancingThresholds(int idle_time_min, int abs_voltage_mV, int voltage_difference_mV) {
    balancing_min_idle_time_s_ = idle_time_min * 60;
    balancing_min_cell_voltage_mV_ = abs_voltage_mV;
    balancing_max_voltage_difference_mV_ = voltage_difference_mV;
    ESP_LOGI(TAG, "Balancing thresholds set to %d min idle time, %d mV cell voltage, %d mV voltage difference", idle_time_min, abs_voltage_mV, voltage_difference_mV);
}

void bq76930::setShuntResistorValue(int res_mOhm) {
    shunt_value_mOhm_ = res_mOhm;
    ESP_LOGI(TAG, "Shunt resitor value set to %d mOhm", res_mOhm);
}

void bq76930::setThermistorBetaValue(int beta_K) {
    // not implemented
    //thermistor_beta_value_ = beta_K;
}

void bq76930::setTemperatureLimits(int min_discharge_degC, int max_discharge_degC, int min_charge_degC, int max_charge_degC) {
    // Not implemented
    // Temperature limits (°C/10)
    // min_celltemp_discharge_ = min_discharge_degC * 10;
    // max_celltemp_discharge_ = max_discharge_degC * 10;
    // min_celltemp_charge_ = min_charge_degC * 10;
    // max_celltemp_charge_ = max_charge_degC * 10;  
}

void bq76930::setIdleCurrentThreshold(int current_mA) {
    idle_current_threshold_ = current_mA;
    ESP_LOGI(TAG, "Idle current threshold set to %d mA", current_mA);
}

long bq76930::setShortCircuitProtection(long current_mA, int delay_us) {
    // not implemented
    return 0;
}

void bq76930::setOvercurrentChargeProtection(int32_t current_mA, int delay_ms) {
    // ToDo: Software protection for charge overcurrent
    overcurrent_charge_trip_time_ = delay_ms;
    max_charge_current_ = current_mA;
    ESP_LOGI(TAG, "Overcurrent charge protection set to %ld mA(not implemented yet)", current_mA);
}

long bq76930::setOvercurrentDischargeProtection(int32_t current_mA, int delay_ms) {
    // not implemented
    return 0;
}

int bq76930::setCellUndervoltageProtection(int voltage_mV, int delay_s) {
    regPROTECT3_t protect3;
    uint8_t uv_trip = 0;
    
    min_cell_voltage_ = voltage_mV;
    //   protect3.regByte = readRegister(PROTECT3);
    readRegister(PROTECT3, &protect3.regByte, sizeof(protect3.regByte));
    uv_trip = ((((long)voltage_mV - adc_offset_) * 1000 / adc_gain_) >> 4) & 0x00FF;
    uv_trip += 1;   // always round up for lower cell voltage
    writeRegister(UV_TRIP, uv_trip);
    
    protect3.bits.UV_DELAY = 0;
    for (int i = sizeof(UV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= UV_delay_setting[i]) {
        protect3.bits.UV_DELAY = i;
        break;
        }
    }
    
    writeRegister(PROTECT3, protect3.regByte);
    ESP_LOGI(TAG, "Under voltage protection set to %d mV", voltage_mV);
    // returns the actual current threshold value
    return ((long)1 << 12 | uv_trip << 4) * adc_gain_ / 1000 + adc_offset_;
}

int bq76930::setCellOvervoltageProtection(int voltage_mV, int delay_s) {
    regPROTECT3_t protect3;
    uint8_t ov_trip = 0;

    max_cell_voltage_ = voltage_mV;
    
    //   protect3.regByte = readRegister(PROTECT3);
    readRegister(PROTECT3, &protect3.regByte, sizeof(protect3.regByte));
    ov_trip = ((((long)voltage_mV - adc_offset_) * 1000 / adc_gain_) >> 4) & 0x00FF;
    writeRegister(OV_TRIP, ov_trip);
        
    protect3.bits.OV_DELAY = 0;
    for (int i = sizeof(OV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= OV_delay_setting[i]) {
        protect3.bits.OV_DELAY = i;
        break;
        }
    }
    
    writeRegister(PROTECT3, protect3.regByte);
    ESP_LOGI(TAG, "Over voltage protection set to %d mV", voltage_mV);
    // returns the actual current threshold value
    return ((long)1 << 13 | ov_trip << 4) * adc_gain_ / 1000 + adc_offset_;
}

int bq76930::getBatteryCurrent() {
    return bat_current_;
}

int bq76930::getBatteryVoltage() {
    return bat_voltage_;
}

int bq76930::getMaxCellVoltage() {
    return cell_voltages_[id_cell_max_voltage_];
}

int bq76930::getCellVoltage(int id_cell) {
    return cell_voltages_[id_cell-1];
}

float bq76930::getTemperatureDegC(int channel) {
    // if (channel >= 1 && channel <= 3) {
    //     return (float)temperatures_[channel-1] / 10.0;
    // } else {
    //     return -273.15;   // Error: Return absolute minimum temperature
    // }
    return 0;
}
