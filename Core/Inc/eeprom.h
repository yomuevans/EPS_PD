#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "telemetry.h" // Include for EEPROM_TelemetryWithTimestamp definition
#include <string.h>

// I2C addresses for M24M01-A125 EEPROM (7-bit format, shifted left by 1 in transactions).
// Derived from device select code: [1010 E2 E1 0] for main memory, [1011 E2 E1 0] for Identification page
// (M24M01-A125 datasheet, page 12, Section 3.5, Table 2). Assumes E2=0, E1=0 (tied to VSS; page 8, Section 2.3).
#define epspd_I2C_ADDR_MEMORY 0x50     // Main memory address (1010 00 0)
#define epspd_I2C_ADDR_ID_PAGE 0x58    // Identification page address (1011 00 0)
// Note: If E2/E1 are configured differently (e.g., E2=0, E1=1), adjust addresses accordingly (e.g., 0x51, 0x59).

// EEPROM internal memory addresses for M24M01-A125 (1 Mbit, 128 KB, 0x0000 to 0x1FFFF).
// Valid within the main memory (512 pages of 256 bytes) or Identification page (256 bytes)
// (M24M01-A125 datasheet, page 6, Section 1; page 14, Section 3.6).
#define epspd_ADDR_DEVICE_ID 0x0000    // Starting address for device ID (Identification page or main memory)
#define epspd_ADDR_TELEMETRY 0x0100    // Starting address for telemetry data (main memory, second page)
#define epspd_ADDR_PARAMETERS 0x0200   // Starting address for parameters (main memory, third page)

// Function prototypes for EEPROM operations using I2C.
// Compatible with M24M01-A125 I2C protocol (up to 1 MHz, page 6, Section 1) and STM32L496xx I2C peripheral
// (DS11585 Rev 19, page 53, Section 3.30). Write operations must respect 256-byte page size and 4 ms write cycle
// (M24M01-A125 datasheet, page 15, Section 4.1; page 29, Table 11; page 30, Table 12).
HAL_StatusTypeDef epspd_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef epspd_WriteTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry);
HAL_StatusTypeDef epspd_ReadTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry);
HAL_StatusTypeDef epspd_WriteParameters(I2C_HandleTypeDef *hi2c, EPSPD_Parameter *parameters, uint8_t count);
HAL_StatusTypeDef epspd_ReadParameters(I2C_HandleTypeDef *hi2c, EPSPD_Parameter *parameters, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
