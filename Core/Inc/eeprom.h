/**
  ******************************************************************************
  * @file    eeprom.h
  * @brief   Header file for EEPROM operations
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported constants --------------------------------------------------------*/
// EEPROM Addresses (for M24M01)
#define EEPROM_I2C_ADDR_MEMORY    0x50      // I2C address for main memory (1010_000)
#define EEPROM_I2C_ADDR_ID_PAGE   0x58      // I2C address for Identification Page (1011_000)
#define EEPROM_ADDR_TELEMETRY     0x0000    // Telemetry data at address 0x0000 (6 bytes)
#define EEPROM_ADDR_PARAMETERS    0x0006    // Parameters at address 0x0006 (63 bytes)
#define EEPROM_ADDR_DEVICE_ID     0x00      // Device ID in Identification Page (4 bytes)

// Telemetry structure (6 bytes)
typedef struct {
    uint16_t Bus12V;
    uint16_t Bus5V;
    uint16_t Bus3V3;
} EEPROM_Telemetry;

// Parameter structure (3 bytes each)
typedef struct {
    uint8_t ParamId;
    uint16_t Value;
} EEPROM_Parameter;

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef EEPROM_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef EEPROM_WriteTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_Telemetry *telemetry);
HAL_StatusTypeDef EEPROM_ReadTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_Telemetry *telemetry);
HAL_StatusTypeDef EEPROM_WriteParameters(I2C_HandleTypeDef *hi2c, EEPROM_Parameter *parameters, uint8_t count);
HAL_StatusTypeDef EEPROM_ReadParameters(I2C_HandleTypeDef *hi2c, EEPROM_Parameter *parameters, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
