#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "telemetry.h"
#include <string.h>
#include <stdbool.h>

// ===============================
// EEPROM Device I2C Addresses
// ===============================
// Main memory I2C address (0x50 << 1 = 0xA0 for write, 0xA1 for read)
#define epspd_I2C_ADDR_MEMORY    0x50
// ID Page I2C address (0x58 << 1 = 0xB0 for write, 0xB1 for read)
#define epspd_I2C_ADDR_ID_PAGE   0x58

// ===============================
// EEPROM Memory Map (M24M01-A125 - 128KB / 1Mbit)
// Page size: 256 bytes
// Address range: 0x0000 – 0x1FFF (8 KB used here for EPS purposes)
// ===============================

// Shared & Lock Region
#define EEPROM_ADDR_LOCK                  0x0000  // [0x0000 – 0x00FF] Lock + shared metadata (256 B)

// MCU1 Region (EPS_PD)
#define EEPROM_ADDR_MCU1_TELEMETRY        0x0100  // [0x0100 – 0x08FF] Telemetry log (2 KB)
#define EEPROM_ADDR_MCU1_FAULT            0x1100  // [0x1100 – 0x15FF] Fault log (1.25 KB)

// MCU2 Region (e.g., EPS_BMS)
#define EEPROM_ADDR_MCU2_TELEMETRY        0x0900  // [0x0900 – 0x10FF] Telemetry log (2 KB)
#define EEPROM_ADDR_MCU2_FAULT            0x1600  // [0x1600 – 0x1AFF] Fault log (1.25 KB)

// Shared Reserved Region
#define EEPROM_ADDR_SHARED_RESERVED       0x1B00  // [0x1B00 – 0x1EFF] Reserved / calibration

// Sentinel or CRC Region
#define EEPROM_ADDR_SENTINEL              0x1F00  // [0x1F00 – 0x1FFF] End-of-EEPROM marker

// ===============================
// EEPROM Identification Page (via 0x58)
// ===============================
#define EEPROM_ADDR_ID_PAGE_DEVICE_ID     0x0000  // Device ID/config version in ID page

// ===============================
// Function Prototypes
// ===============================
HAL_StatusTypeDef epspd_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef epspd_WriteTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry);
HAL_StatusTypeDef epspd_ReadTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry);

HAL_StatusTypeDef epspd_WriteParameters(I2C_HandleTypeDef *hi2c, EPSPD_Parameter *parameters, uint8_t count);
HAL_StatusTypeDef epspd_ReadParameters(I2C_HandleTypeDef *hi2c, EPSPD_Parameter *parameters, uint8_t count);

// Locking and Verification Helpers
bool acquire_eeprom_lock(uint8_t mcu_id);
void release_eeprom_lock(void);
void check_and_clear_stale_lock(void);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
