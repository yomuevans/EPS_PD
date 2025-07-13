#ifndef __I2C_COMM_H
#define __I2C_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

#define I2C_SLAVE_ADDR_BMS    (0x30 << 1)
#define I2C_MAX_FRAME_SIZE    64
#define I2C_TIMEOUT_MS        100
#define EPS_I2C_MAX_FRAME_SIZE 256
#define EPS_BMS_CMD_GET_TELEMETRY  0x01
#define EPS_BMS_I2C_ADDR  0x36  // Replace with actual 7-bit I2C address of STM32L476RCT6
#define BMS_CMD_HEATER1_ON        0xA0
#define BMS_CMD_HEATER1_OFF       0xA1
#define BMS_CMD_HEATER2_ON        0xA2
#define BMS_CMD_HEATER2_OFF       0xA3
#define BMS_CMD_CHARGE_EN         0xA4
#define BMS_CMD_CHARGE_DIS        0xA5
#define BMS_CMD_DISCHARGE_EN      0xA6
#define BMS_CMD_DISCHARGE_DIS     0xA7
//#define CMD_SYNC_COUNTER      0x01
#define NUMBER_OF_CELLS       4 // Number of cells in the battery pack


// Structure to hold telemetry data for BMS system
typedef struct {
    uint16_t vcell_ic1[NUMBER_OF_CELLS]; // Cell voltages for first BMS IC (mV)
    uint16_t vcell_ic2[NUMBER_OF_CELLS]; // Cell voltages for second BMS IC (mV)
    uint16_t vpack_ic1; // Pack voltage for first BMS IC (mV)
    uint16_t vpack_ic2; // Pack voltage for second BMS IC (mV)
    int16_t current_ic1; // Pack current for first BMS IC (mA)
    int16_t current_ic2; // Pack current for second BMS IC (mA)
    float soc; // State of Charge (%)
    float soh; // State of Health (%)
    float pcb_temperature; // PCB temperature (°C)
    float pack_temperature_ic1; // Pack temperature from first BMS IC (°C)
    float pack_temperature_ic2; // Pack temperature from second BMS IC (°C)
    float die_temperature_ic1; // Die temperature of first BMS IC (°C)
    float die_temperature_ic2; // Die temperature of second BMS IC (°C)
    float thermistor_temperature_ic1; // Thermistor temperature for first BMS IC (°C)
    float thermistor_temperature_ic2; // Thermistor temperature for second BMS IC (°C)
    uint8_t heater1_state; // Heater 1 state (1 = ON, 0 = OFF)
    uint8_t heater2_state; // Heater 2 state (1 = ON, 0 = OFF)
    uint8_t balancing_active; // Cell balancing status (1 = active, 0 = inactive)
    uint8_t balancing_mask_ic1; // Balancing mask for first BMS IC
    uint8_t balancing_mask_ic2; // Balancing mask for second BMS IC
    uint8_t charge_immediately; // Flag to trigger immediate charging (1 = yes, 0 = no)
    uint8_t bms_online; // BMS online status (1 = online, 0 = offline)
    uint8_t error_flags[8]; // Array of error flags for fault conditions
    uint8_t ovrd_alert_ic1; // Override alert for first BMS IC
    uint8_t ovrd_alert_ic2; // Override alert for second BMS IC
    uint8_t device_xready_ic1; // Device ready status for first BMS IC
    uint8_t device_xready_ic2; // Device ready status for second BMS IC
    uint8_t load_present_ic1; // Load presence for first BMS IC (1 = present, 0 = absent)
    uint8_t load_present_ic2; // Load presence for second BMS IC (1 = present, 0 = absent)
    uint16_t charge_cycle_count; // Number of charge cycles
    uint32_t total_charge_time; // Total charge time (seconds)
    uint32_t total_discharge_time; // Total discharge time (seconds)
    uint32_t total_operating_time; // Total operating time (seconds)
    uint16_t raw_adc_gain_ic1; // Raw ADC gain for first BMS IC (µV/LSB)
    uint16_t raw_adc_offset_ic1; // Raw ADC offset for first BMS IC (mV)
    uint16_t raw_adc_gain_ic2; // Raw ADC gain for second BMS IC (µV/LSB)
    uint16_t raw_adc_offset_ic2; // Raw ADC offset for second BMS IC (mV)
    uint8_t i2c_comm_error_ic1; // I2C communication error flag for first BMS IC
    uint8_t i2c_comm_error_ic2; // I2C communication error flag for second BMS IC
    uint64_t sync_counter; // Synchronization counter for telemetry updates
    uint8_t sync_valid; // Synchronization validity flag (1 = valid, 0 = invalid)
} BMSTelemetryData;

extern BMSTelemetryData eps_data; // Global telemetry data structure
typedef enum {
    CMD_SYNC_COUNTER        = 0x10, // Sync 8-byte timestamp
    CMD_ENABLE_CHARGING     = 0x20,
    CMD_DISABLE_CHARGING    = 0x21,
    CMD_GET_TELEMETRY      = 0x30,
    CMD_PUT_DATA            = 0x40,
    CMD_READ_DATA           = 0x41,
    CMD_WRITE_DATA          = 0x42
} EPS_I2C_CommandCode_t;

HAL_StatusTypeDef EPS_I2C_TransmitReceiveWithRetry(
   I2C_HandleTypeDef *hi2c,
   uint8_t cmd,
   const uint8_t *tx_payload, uint8_t tx_len,
   uint8_t *rx_payload, uint16_t *rx_len,  // in: buffer size; out: actual length
   uint16_t slave_addr);


// CRC-8/MAXIM (Dallas/1-Wire, polynomial 0x31)
uint8_t EPS_I2C_CRC8(const uint8_t *data, uint8_t len);

HAL_StatusTypeDef EPS_I2C_SendSyncCounter(I2C_HandleTypeDef *hi2c,
										   uint64_t sync_counter,
										   uint16_t i2c_slave_addr);

HAL_StatusTypeDef EPS_I2C_SendCommand(
    I2C_HandleTypeDef *hi2c,
    uint8_t cmd,
    const uint8_t *payload, uint8_t payload_len,
    uint8_t *response_buf, uint16_t response_buf_len,
    uint16_t slave_addr);

HAL_StatusTypeDef EPS_I2C_RequestTelemetry(I2C_HandleTypeDef *hi2c, uint16_t slave_addr, BMSTelemetryData *telemetry);




#ifdef __cplusplus
}
#endif

#endif /* __I2C_COMM_H */
