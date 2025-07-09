/*
 * i2c_comm.c
 *
 *  Created on: Jun 30, 2025
 *      Author: yomue
 */

// Include the header file for I2C communication definitions
#include "i2c_comm.h" // Defines constants (e.g., EPS_I2C_MAX_FRAME_SIZE, I2C_SLAVE_ADDR_BMS) and function prototypes
#include <string.h>   // Standard C library for memory operations (e.g., memcpy)

// Function: EPS_I2C_CRC8
// Inputs:
//   - data: A pointer to a uint8_t array, the data to calculate the CRC for
//   - len: A uint8_t, the length of the data array
// Output:
//   - Returns a uint8_t, the calculated CRC-8 checksum
// Significance:
//   - Calculates a CRC-8 checksum using the polynomial x⁸ + x⁵ + x⁴ + 1 (0x31, reflected)
//     to ensure data integrity for I2C commands sent to the BMS. Critical for reliable
//     communication in the satellite’s EPS-BMS interface.
// Function:
uint8_t EPS_I2C_CRC8(const uint8_t *data, uint8_t len) {
    // Initialize CRC to 0x00 (starting value for CRC-8)
    uint8_t crc = 0x00;
    // Loop through each byte in the data array
    for (uint8_t i = 0; i < len; i++) {
        // XOR the current byte with the CRC
        crc ^= data[i];
        // Process each bit in the byte (8 bits)
        for (uint8_t j = 0; j < 8; j++) {
            // If the most significant bit is 1, shift left and XOR with polynomial 0x31
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    // Return the final CRC-8 value
    return crc;
}

// Function: EPS_I2C_SendCommand
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface (e.g., I2C2 or I2C3) for BMS communication
//   - cmd: A uint8_t, the command code (e.g., BMS_CMD_HEATER1_ON)
//   - tx_data: A pointer to a uint8_t array, optional data to send with the command
//   - tx_len: A uint8_t, the length of tx_data
//   - rx_data: A pointer to a uint8_t array, optional buffer for response data (not used here)
//   - rx_len: A uint8_t, the expected length of response data (not used here)
//   - sync_counter: A uint64_t, the sync counter for timestamping the command
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if transmission succeeds, HAL_ERROR if it fails
// Significance:
//   - Sends a command to the BMS via I2C, including a sync counter and CRC-8 for data integrity,
//     used by satellite_modes.c to control BMS-managed power lines (e.g., heaters, charge/discharge).
// Function:
HAL_StatusTypeDef EPS_I2C_SendCommand(I2C_HandleTypeDef *hi2c, uint8_t cmd,
                                      uint8_t *tx_data, uint8_t tx_len,
                                      uint8_t *rx_data, uint8_t rx_len,
                                      uint64_t sync_counter) {
    // Declare a buffer for the I2C frame (max size defined in i2c_comm.h)
    uint8_t frame[EPS_I2C_MAX_FRAME_SIZE];
    // Initialize position index for building the frame
    uint8_t pos = 0;

    // Header: Add command code to the frame
    frame[pos++] = cmd;

    // Calculate total payload length: 8 bytes for sync_counter + tx_data length
    uint8_t payload_len = 8 + tx_len;
    // Check if frame size (payload + 3 for CMD, LEN, CRC) exceeds max size
    if (payload_len + 3 > EPS_I2C_MAX_FRAME_SIZE) // +3 for CMD, LEN, CRC
        return HAL_ERROR; // Return error if frame is too large

    // Add payload length to the frame
    frame[pos++] = payload_len;

    // Add sync counter (8 bytes, most significant byte first)
    frame[pos++] = (sync_counter >> 56) & 0xFF; // Byte 7
    frame[pos++] = (sync_counter >> 48) & 0xFF; // Byte 6
    frame[pos++] = (sync_counter >> 40) & 0xFF; // Byte 5
    frame[pos++] = (sync_counter >> 32) & 0xFF; // Byte 4
    frame[pos++] = (sync_counter >> 24) & 0xFF; // Byte 3
    frame[pos++] = (sync_counter >> 16) & 0xFF; // Byte 2
    frame[pos++] = (sync_counter >> 8) & 0xFF;  // Byte 1
    frame[pos++] = sync_counter & 0xFF;         // Byte 0

    // Append optional command data if provided
    if (tx_data && tx_len > 0) {
        // Copy tx_data into the frame at the current position
        memcpy(&frame[pos], tx_data, tx_len);
        // Update position by adding data length
        pos += tx_len;
    }

    // Calculate CRC-8 over all bytes in the frame so far
    uint8_t crc = EPS_I2C_CRC8(frame, pos);
    // Append CRC to the frame
    frame[pos++] = crc;

    // Send the frame to the BMS via I2C (address shifted left by 1 for STM32 HAL)
    return HAL_I2C_Master_Transmit(hi2c, I2C_SLAVE_ADDR_BMS, frame, pos, I2C_TIMEOUT_MS);
}
