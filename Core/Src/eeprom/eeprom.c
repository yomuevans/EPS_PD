/**
  ******************************************************************************
  * @file    eeprom.c
  * @brief   EEPROM operations implementation
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

#include "eeprom.h"
#include <string.h>

/**
  * @brief  Initialize the EEPROM by writing a device identifier to the Identification Page.
  * @param  hi2c Pointer to the I2C handle (e.g., &hi2c2)
  * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise HAL_ERROR
  */
HAL_StatusTypeDef EEPROM_Init(I2C_HandleTypeDef *hi2c)
{
    // Write device identifier to Identification Page
    uint8_t device_id[4] = {'E', 'P', 'S', '1'}; // Example: "EPS1"
    uint8_t buffer[6];
    buffer[0] = EEPROM_ADDR_DEVICE_ID >> 8;   // Address high byte
    buffer[1] = EEPROM_ADDR_DEVICE_ID & 0xFF; // Address low byte
    memcpy(&buffer[2], device_id, 4);

    // Write to Identification Page (address 0x58)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR_ID_PAGE << 1, buffer, 6, 100);
    if (status == HAL_OK) {
        HAL_Delay(4); // Write time is 4 ms max (Table 11, Page 29 of M24M01 datasheet)
    }
    return status;
}

/**
  * @brief  Write telemetry data to EEPROM at the specified address.
  * @param  hi2c Pointer to the I2C handle
  * @param  telemetry Pointer to the telemetry structure to write
  * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise HAL_ERROR
  */
HAL_StatusTypeDef EEPROM_WriteTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_Telemetry *telemetry)
{
    uint8_t buffer[8];
    buffer[0] = EEPROM_ADDR_TELEMETRY >> 8;   // Address high byte
    buffer[1] = EEPROM_ADDR_TELEMETRY & 0xFF; // Address low byte
    buffer[2] = telemetry->Bus12V >> 8;
    buffer[3] = telemetry->Bus12V & 0xFF;
    buffer[4] = telemetry->Bus5V >> 8;
    buffer[5] = telemetry->Bus5V & 0xFF;
    buffer[6] = telemetry->Bus3V3 >> 8;
    buffer[7] = telemetry->Bus3V3 & 0xFF;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, buffer, 8, 100);
    if (status == HAL_OK) {
        HAL_Delay(4); // Write time is 4 ms max
    }
    return status;
}

/**
  * @brief  Read telemetry data from EEPROM at the specified address.
  * @param  hi2c Pointer to the I2C handle
  * @param  telemetry Pointer to the telemetry structure to store the read data
  * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise HAL_ERROR
  */
HAL_StatusTypeDef EEPROM_ReadTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_Telemetry *telemetry)
{
    uint8_t addr[2];
    addr[0] = EEPROM_ADDR_TELEMETRY >> 8;   // Address high byte
    addr[1] = EEPROM_ADDR_TELEMETRY & 0xFF; // Address low byte

    // Set the address to read from
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, addr, 2, 100);
    if (status != HAL_OK) {
        return status;
    }

    // Read 6 bytes of telemetry data
    uint8_t data[6];
    status = HAL_I2C_Master_Receive(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, data, 6, 100);
    if (status == HAL_OK) {
        telemetry->Bus12V = (data[0] << 8) | data[1];
        telemetry->Bus5V = (data[2] << 8) | data[3];
        telemetry->Bus3V3 = (data[4] << 8) | data[5];
    }
    return status;
}

/**
  * @brief  Write parameters to EEPROM at the specified address.
  * @param  hi2c Pointer to the I2C handle
  * @param  parameters Pointer to the array of parameters to write
  * @param  count Number of parameters to write
  * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise HAL_ERROR
  */
HAL_StatusTypeDef EEPROM_WriteParameters(I2C_HandleTypeDef *hi2c, EEPROM_Parameter *parameters, uint8_t count)
{
    if (count == 0) return HAL_OK;

    // Each parameter is 3 bytes (ParamId + 2-byte Value)
    uint8_t buffer[2 + 3 * 21]; // Max 21 parameters (63 bytes total)
    buffer[0] = EEPROM_ADDR_PARAMETERS >> 8;   // Address high byte
    buffer[1] = EEPROM_ADDR_PARAMETERS & 0xFF; // Address low byte

    for (uint8_t i = 0; i < count; i++) {
        buffer[2 + i * 3] = parameters[i].ParamId;
        buffer[3 + i * 3] = parameters[i].Value >> 8;
        buffer[4 + i * 3] = parameters[i].Value & 0xFF;
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, buffer, 2 + 3 * count, 100);
    if (status == HAL_OK) {
        HAL_Delay(4); // Write time is 4 ms max
    }
    return status;
}

/**
  * @brief  Read parameters from EEPROM at the specified address.
  * @param  hi2c Pointer to the I2C handle
  * @param  parameters Pointer to the array to store the read parameters
  * @param  count Number of parameters to read
  * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise HAL_ERROR
  */
HAL_StatusTypeDef EEPROM_ReadParameters(I2C_HandleTypeDef *hi2c, EEPROM_Parameter *parameters, uint8_t count)
{
    if (count == 0) return HAL_OK;

    uint8_t addr[2];
    addr[0] = EEPROM_ADDR_PARAMETERS >> 8;   // Address high byte
    addr[1] = EEPROM_ADDR_PARAMETERS & 0xFF; // Address low byte

    // Set the address to read from
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, addr, 2, 100);
    if (status != HAL_OK) {
        return status;
    }

    // Read 3 bytes per parameter
    uint8_t data[3 * 21]; // Max 21 parameters (63 bytes total)
    status = HAL_I2C_Master_Receive(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, data, 3 * count, 100);
    if (status == HAL_OK) {
        for (uint8_t i = 0; i < count; i++) {
            parameters[i].ParamId = data[i * 3];
            parameters[i].Value = (data[i * 3 + 1] << 8) | data[i * 3 + 2];
        }
    }
    return status;
}
