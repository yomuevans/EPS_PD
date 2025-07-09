// File: eeprom.c
// Include the header file for EEPROM definitions
#include "eeprom.h" // Defines constants (e.g., epspd_I2C_ADDR_ID_PAGE, epspd_ADDR_TELEMETRY) and structures (e.g., EEPROM_TelemetryWithTimestamp)
#include "delay.h" // Defines EEPROM_TelemetryWithTimestamp structure for telemetry data




// Function: epspd_Init
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface (e.g., I2C2) used to communicate with the EEPROM
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if initialization succeeds, HAL_ERROR if it fails
// Significance:
//   - Initializes the EEPROM by writing a device ID ('EPS1') to a specific address, ensuring the EEPROM is ready for telemetry and parameter storage. Critical for setting up persistent storage in the EPS.
// Function:
HAL_StatusTypeDef epspd_Init(I2C_HandleTypeDef *hi2c)
{
    // Define a 4-byte device ID ('E', 'P', 'S', '1') to identify the EPS in EEPROM
    uint8_t device_id[4] = {'E', 'P', 'S', '1'};
    // Declare a 6-byte buffer for I2C transmission (2 bytes for address + 4 bytes for ID)
    uint8_t buffer[6];
    // Set high byte of the device ID address (shift right by 8 bits)
    buffer[0] = epspd_ADDR_DEVICE_ID >> 8;
    // Set low byte of the device ID address (mask with 0xFF to get lower 8 bits)
    buffer[1] = epspd_ADDR_DEVICE_ID & 0xFF;
    // Copy the 4-byte device ID into the buffer starting at index 2
    memcpy(&buffer[2], device_id, 4);

    // Send the buffer to the EEPROM’s ID page address (shifted left by 1 for STM32 HAL)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_ID_PAGE << 1, buffer, 6, 100);
    // If transmission succeeds, wait 4ms for EEPROM write cycle
    if (status == HAL_OK) {
    	SoftwareDelay(1);
    }
    // Return the transmission status (HAL_OK or HAL_ERROR)
    return status;
}

// Function: epspd_WriteTelemetry
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface for EEPROM communication
//   - telemetry: A pointer to an EEPROM_TelemetryWithTimestamp, containing telemetry data (bus voltages, subtick, counter)
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if write succeeds, HAL_ERROR if it fails
// Significance:
//   - Writes telemetry data (12V, 5V, 3.3V bus voltages, subtick, and sync counter) to EEPROM, ensuring persistent storage across power cycles. Called by telemetry.c to save data periodically.
// Function:
HAL_StatusTypeDef epspd_WriteTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry)
{
    // Declare a 20-byte buffer for I2C transmission (2 bytes address + 18 bytes data)
    uint8_t buffer[20]; // Increased to 20 bytes (2 address + 18 data)
    // Set high byte of telemetry address
    buffer[0] = epspd_ADDR_TELEMETRY >> 8;
    // Set low byte of telemetry address
    buffer[1] = epspd_ADDR_TELEMETRY & 0xFF;
    // Store high byte of 12V bus voltage (uint16_t, shift right by 8)
    buffer[2] = (uint8_t)(telemetry->telemetry.Bus12V >> 8);
    // Store low byte of 12V bus voltage
    buffer[3] = (uint8_t)(telemetry->telemetry.Bus12V);
    // Store high byte of 5V bus voltage
    buffer[4] = (uint8_t)(telemetry->telemetry.Bus5V >> 8);
    // Store low byte of 5V bus voltage
    buffer[5] = (uint8_t)(telemetry->telemetry.Bus5V);
    // Store high byte of 3.3V bus voltage
    buffer[6] = (uint8_t)(telemetry->telemetry.Bus3V3 >> 8);
    // Store low byte of 3.3V bus voltage
    buffer[7] = (uint8_t)(telemetry->telemetry.Bus3V3);
    // Store byte 3 of subtick_us (uint32_t, shift right by 24)
    buffer[8] = (uint8_t)(telemetry->telemetry.subtick_us >> 24);
    // Store byte 2 of subtick_us
    buffer[9] = (uint8_t)(telemetry->telemetry.subtick_us >> 16);
    // Store byte 1 of subtick_us
    buffer[10] = (uint8_t)(telemetry->telemetry.subtick_us >> 8);
    // Store byte 0 of subtick_us
    buffer[11] = (uint8_t)(telemetry->telemetry.subtick_us);
    // Store byte 7 of sync counter (uint64_t, shift right by 56)
    buffer[12] = (uint8_t)(telemetry->counter >> 56);
    // Store byte 6 of sync counter
    buffer[13] = (uint8_t)(telemetry->counter >> 48);
    // Store byte 5 of sync counter
    buffer[14] = (uint8_t)(telemetry->counter >> 40);
    // Store byte 4 of sync counter
    buffer[15] = (uint8_t)(telemetry->counter >> 32);
    // Store byte 3 of sync counter
    buffer[16] = (uint8_t)(telemetry->counter >> 24);
    // Store byte 2 of sync counter
    buffer[17] = (uint8_t)(telemetry->counter >> 16);
    // Store byte 1 of sync counter
    buffer[18] = (uint8_t)(telemetry->counter >> 8);
    // Store byte 0 of sync counter
    buffer[19] = (uint8_t)(telemetry->counter);

    // Write the buffer to the EEPROM’s memory address (shifted left by 1 for HAL)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_MEMORY << 1, buffer, 20, 100);
    // If write succeeds, wait 4ms for EEPROM write cycle
    if (status == HAL_OK) {
    	SoftwareDelay(1);
    }
    // Return the write status
    return status;
}

// Function: epspd_ReadTelemetry
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface for EEPROM communication
//   - telemetry: A pointer to an EEPROM_TelemetryWithTimestamp, where read data will be stored
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if read succeeds, HAL_ERROR if it fails
// Significance:
//   - Reads telemetry data (bus voltages, subtick, sync counter) from EEPROM, used by ssp.c
//     to retrieve stored data for subsystem communication (e.g., responding to SSP_CMD_GD).
// Function:
HAL_StatusTypeDef epspd_ReadTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry)
{
    // Declare a 2-byte buffer for the telemetry address
    uint8_t addr[2];
    // Set high byte of telemetry address
    addr[0] = epspd_ADDR_TELEMETRY >> 8;
    // Set low byte of telemetry address
    addr[1] = epspd_ADDR_TELEMETRY & 0xFF;

    // Send the telemetry address to the EEPROM to specify what to read
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_MEMORY << 1, addr, 2, 100);
    // If address transmission fails, return the error status
    if (status != HAL_OK) {
        return status;
    }

    // Declare an 18-byte buffer to receive telemetry data
    uint8_t data[18];
    // Read 18 bytes of telemetry data from EEPROM
    status = HAL_I2C_Master_Receive(hi2c, epspd_I2C_ADDR_MEMORY << 1, data, 18, 100);
    // If read succeeds, parse the data into the telemetry structure
    if (status == HAL_OK) {
        // Combine bytes 0 and 1 into 12V bus voltage (uint16_t)
        telemetry->telemetry.Bus12V = (data[0] << 8) | data[1];
        // Combine bytes 2 and 3 into 5V bus voltage
        telemetry->telemetry.Bus5V = (data[2] << 8) | data[3];
        // Combine bytes 4 and 5 into 3.3V bus voltage
        telemetry->telemetry.Bus3V3 = (data[4] << 8) | data[5];
        // Combine bytes 6-9 into subtick_us (uint32_t)
        telemetry->telemetry.subtick_us = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];
        // Combine bytes 10-17 into sync counter (uint64_t)
        telemetry->counter = ((uint64_t)data[10] << 56) | ((uint64_t)data[11] << 48) | ((uint64_t)data[12] << 40) |
                            ((uint64_t)data[13] << 32) | ((uint64_t)data[14] << 24) | ((uint64_t)data[15] << 16) |
                            ((uint64_t)data[16] << 8) | (uint64_t)data[17];
    }
    // Return the read status
    return status;
}

// Function: epspd_WriteParameters
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface for EEPROM communication
//   - parameters: A pointer to an EPSPD_Parameter array, containing parameters to write
//   - count: A uint8_t, the number of parameters to write
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if write succeeds, HAL_ERROR if it fails
// Significance:
//   - Writes an array of parameters (e.g., voltages, currents) to EEPROM, used to store
//     configuration or telemetry data persistently. Called when parameters are updated.
// Function:
HAL_StatusTypeDef epspd_WriteParameters(I2C_HandleTypeDef *hi2c, EPSPD_Parameter *parameters, uint8_t count)
{
    // If no parameters to write, return success
    if (count == 0) return HAL_OK;

    // Declare a buffer for I2C transmission (2 bytes address + 3 bytes per parameter, up to 25 parameters)
    uint8_t buffer[2 + 3 * 25];
    // Set high byte of parameters address
    buffer[0] = epspd_ADDR_PARAMETERS >> 8;
    // Set low byte of parameters address
    buffer[1] = epspd_ADDR_PARAMETERS & 0xFF;

    // Loop through each parameter to populate the buffer
    for (uint8_t i = 0; i < count; i++) {
        // Store parameter ID
        buffer[2 + i * 3] = parameters[i].ParamId;
        // Store high byte of parameter value
        buffer[3 + i * 3] = parameters[i].Value >> 8;
        // Store low byte of parameter value
        buffer[4 + i * 3] = parameters[i].Value & 0xFF;
    }

    // Write the buffer to the EEPROM’s memory address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_MEMORY << 1, buffer, 2 + 3 * count, 100);
    // If write succeeds, wait 4ms for EEPROM write cycle
    if (status == HAL_OK) {
        SoftwareDelay(4);
    }
    // Return the write status
    return status;
}

// Function: epspd_ReadParameters
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface for EEPROM communication
//   - parameters: A pointer to an EPSPD_Parameter array, where read parameters will be stored
//   - count: A uint8_t, the number of parameters to read
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if read succeeds, HAL_ERROR if it fails
// Significance:
//   - Reads an array of parameters from EEPROM, used to restore configuration or telemetry
//     data, critical for system initialization or SSP command responses.
// Function:
HAL_StatusTypeDef epspd_ReadParameters(I2C_HandleTypeDef *hi2c, EPSPD_Parameter *parameters, uint8_t count)
{
    // If no parameters to read, return success
    if (count == 0) return HAL_OK;

    // Declare a 2-byte buffer for the parameters address
    uint8_t addr[2];
    // Set high byte of parameters address
    addr[0] = epspd_ADDR_PARAMETERS >> 8;
    // Set low byte of parameters address
    addr[1] = epspd_ADDR_PARAMETERS & 0xFF;

    // Send the parameters address to the EEPROM to specify what to read
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_MEMORY << 1, addr, 2, 100);
    // If address transmission fails, return the error status
    if (status != HAL_OK) {
        return status;
    }

    // Declare a buffer to receive parameter data (3 bytes per parameter, up to 25 parameters)
    uint8_t data[3 * 25];
    // Read the parameter data from EEPROM
    status = HAL_I2C_Master_Receive(hi2c, epspd_I2C_ADDR_MEMORY << 1, data, 3 * count, 100);
    // If read succeeds, parse the data into the parameters array
    if (status == HAL_OK) {
        // Loop through each parameter
        for (uint8_t i = 0; i < count; i++) {
            // Store parameter ID
            parameters[i].ParamId = data[i * 3];
            // Combine high and low bytes into parameter value
            parameters[i].Value = (data[i * 3 + 1] << 8) | data[i * 3 + 2];
        }
    }
    // Return the read status
    return status;
}
