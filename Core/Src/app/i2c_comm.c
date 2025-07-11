/*
 * i2c_comm.c
 *
 *  Created on: Jun 30, 2025
 *      Author: yomue
 */

#include "i2c_comm.h"
#include <string.h>
#include "main.h"

//#define I2C_SLAVE_ADDR_BMS        (0x08 << 1)
#define I2C_TIMEOUT_MS            100
#define MAX_FRAME_SIZE            256
#define CRC_SIZE                  1

uint8_t EPS_I2C_CRC8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

HAL_StatusTypeDef EPS_I2C_SendCommand(I2C_HandleTypeDef *hi2c,
                                      uint8_t cmd,
                                      uint8_t *tx_data, uint8_t tx_len,
                                      uint8_t *rx_data, uint8_t rx_len,
                                      uint16_t i2c_slave_addr)
{
    uint8_t frame[MAX_FRAME_SIZE];
    uint8_t pos = 0;

    frame[pos++] = cmd;
    frame[pos++] = tx_len;

    if (tx_data && tx_len > 0) {
        memcpy(&frame[pos], tx_data, tx_len);
        pos += tx_len;
    }

    uint8_t *crc_start = &frame[0]; // Include CMD and LEN in CRC
    uint8_t crc = EPS_I2C_CRC8(crc_start, pos);
    frame[pos++] = crc;

    return HAL_I2C_Master_Transmit(hi2c, i2c_slave_addr, frame, pos, I2C_TIMEOUT_MS);
}


HAL_StatusTypeDef EPS_I2C_SendSyncCounter(I2C_HandleTypeDef *hi2c,
                                          uint64_t sync_counter,
                                          uint16_t i2c_slave_addr)
{
    uint8_t payload[8];
    payload[0] = (sync_counter >> 56) & 0xFF;
    payload[1] = (sync_counter >> 48) & 0xFF;
    payload[2] = (sync_counter >> 40) & 0xFF;
    payload[3] = (sync_counter >> 32) & 0xFF;
    payload[4] = (sync_counter >> 24) & 0xFF;
    payload[5] = (sync_counter >> 16) & 0xFF;
    payload[6] = (sync_counter >> 8)  & 0xFF;
    payload[7] =  sync_counter        & 0xFF;

    return EPS_I2C_SendCommand(hi2c, CMD_SYNC_COUNTER,
                               payload, sizeof(payload),
                               NULL, 0, i2c_slave_addr);
}
