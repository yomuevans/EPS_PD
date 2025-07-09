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


typedef enum {
    CMD_GET_TELEMETRY        = 0x01,
    CMD_HEATER1_ON           = 0x10,
    CMD_HEATER1_OFF          = 0x11,
    CMD_HEATER2_ON           = 0x12,
    CMD_HEATER2_OFF          = 0x13,
    CMD_ENABLE_CHARGING      = 0x20,
    CMD_DISABLE_CHARGING     = 0x21,
    CMD_ENABLE_DISCHARGING   = 0x22,
    CMD_DISABLE_DISCHARGING  = 0x23,
} EPS_I2C_CommandCode_t;

HAL_StatusTypeDef EPS_I2C_SendCommand(I2C_HandleTypeDef *hi2c, uint8_t cmd,
                                      uint8_t *tx_data, uint8_t tx_len,
                                      uint8_t *rx_data, uint8_t rx_len,
                                      uint64_t sync_counter);

HAL_StatusTypeDef EPS_I2C_RequestBMSTelemetry(I2C_HandleTypeDef *hi2c,
                                              uint8_t bms_addr,
                                              uint64_t sync_counter,
                                              uint8_t *out_buffer,
                                              uint8_t *out_len);

// CRC-8/MAXIM (Dallas/1-Wire, polynomial 0x31)
uint8_t EPS_I2C_CRC8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_COMM_H */
