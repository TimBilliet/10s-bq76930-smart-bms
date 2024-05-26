#pragma once

#include "driver/i2c.h"

#define MAX_NUMBER_OF_CELLS 10
#define MAX_NUMBER_OF_THERMISTORS 2


class bq76930 {

    public:

        bq76930(uint8_t address, int sda_pin, int scl_pin);
        
        esp_err_t initialize(int alert_pin, int = -1);
        
        int checkStatus();  // returns 0 if everything is OK
        void update();
        void shutdown();

        // charging control
        bool enableCharging(bool enable_charging);
        bool enableDischarging(bool enable_discharging);
        bool charging_status_;
        
        // hardware settings
        void setShuntResistorValue(int res_mOhm);
        void setThermistorBetaValue(int beta_K);

        // limit settings (for battery protection)
        void setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC);    // °C
        long setShortCircuitProtection(long current_mA, int delay_us = 70);
        void setOvercurrentChargeProtection(int32_t current_mA, int delay_ms = 8);
        long setOvercurrentDischargeProtection(int32_t current_mA, int delay_ms = 8);
        int setCellUndervoltageProtection(int voltage_mV, int delay_s = 1);
        int setCellOvervoltageProtection(int voltage_mV, int delay_s = 1);

        // balancing settings
        void setBalancingThresholds(int idleTime_min = 30, int absVoltage_mV = 3400, int voltageDifference_mV = 20);
        void setIdleCurrentThreshold(int current_mA);

        // automatic balancing when battery is within balancing thresholds
        void enableAutoBalancing(bool enable_balancing);

        // battery status
        int  getBatteryCurrent();
        int  getBatteryVoltage();
        int  getCellVoltage(int id_cell);    // from 1 to 15
        int  getMinCellVoltage();
        int  getMaxCellVoltage();
        float getTemperatureDegC(int channel = 1);
            
        // interrupt handling (not to be called manually!)
        void setAlertInterruptFlag();
        int getErrorState();
        

    private:
    
        uint8_t address_ = 0x08;
        
        int sda_pin_, scl_pin_;
        int cell_count_;

        esp_err_t i2c_conf(int sda_pin, int scl_pin);

        bool crc_enabled_; //taken from mbed library - needed for determineAddressAndCrc funtion

        int shunt_value_mOhm_;
        int thermistor_beta_value_ = 3435;  // typical value for Semitec 103AT-5 thermistor

        // indicates if a new current reading or an error is available from BMS IC
        bool alert_interrupt_flag_ = true;   // init with true to check and clear errors at start-up   
        
        int number_of_cells_ = 10;
        uint16_t cell_voltages_[MAX_NUMBER_OF_CELLS];          // mV
        int id_cell_max_voltage_;
        int id_cell_min_voltage_;
        uint16_t bat_voltage_;                                // mV
        uint16_t bat_current_;                                // mA
        uint8_t temperatures_[MAX_NUMBER_OF_THERMISTORS];    // °C/10

        // Current limits (mA)
        int32_t max_charge_current_;
        int overcurrent_charge_trip_time_;
        int max_discharge_current_;
        int idle_current_threshold_ = 30; // mA
        
        // Temperature limits (°C/10)
        int min_celltemp_charge_;
        int min_celltemp_discharge_;
        int max_celltemp_charge_;
        int max_celltemp_discharge_;

        // Cell voltage limits (mV)
        int max_cell_voltage_;
        int min_cell_voltage_;
        int balancing_min_cell_voltage_mV_;
        int balancing_max_voltage_difference_mV_;
        
        uint16_t adc_gain_;    // uV/LSB
        uint8_t adc_offset_;  // mV
        
        int error_state_ = 0;
        bool auto_balancing_enabled_ = false;
        bool balancing_active_ = false;
        int balancing_min_idle_time_s_ = 1800;    // default: 30 minutes
        int idle_timestamp_ = 0;
        int sec_since_error_counter_ = 0;
        int interrupt_timestamp_ = 0;

        static bq76930* instance_pointer_;
  
        // Methods
        bool determineAddressAndCrc();
        static void alertISR(void* data);

        void updateVoltages();
        void updateCurrent(bool ignore_CC_ready_flag = false);
        void updateTemperatures();
        
        void updateBalancingSwitches();

        esp_err_t readRegister(uint8_t reg_addr, uint8_t *data, size_t len);

        esp_err_t writeRegister(uint8_t address, uint8_t data);

        esp_err_t i2cRead(uint8_t *data_buf, size_t len);

};


