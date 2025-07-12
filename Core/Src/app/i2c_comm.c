/*
 * i2c_comm.c
 *
 *  Created on: Jun 30, 2025
 *      Author: yomue
 */

#include "i2c_comm.h"
#include <string.h>
#include "main.h"

#define I2C_TIMEOUT_MS      100
#define MAX_FRAME_SIZE      256
#define CRC_SIZE            1
#define MAX_I2C_RETRIES     5

// CRC-8/MAXIM: x⁸ + x⁵ + x⁴ + 1 → Polynomial = 0x31
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


/**
 * Transmit a command frame and receive a response from BMS over I2C.
 * Frame Format (TX or RX):
 *   [0] CMD
 *   [1] LEN
 *   [2:N] DATA
 *   [N+1] CRC
 */
HAL_StatusTypeDef EPS_I2C_TransmitReceiveWithRetry(
    I2C_HandleTypeDef *hi2c,
    uint8_t cmd,
    const uint8_t *tx_payload, uint8_t tx_len,
    uint8_t *rx_payload, uint8_t *rx_len,  // in: max size; out: actual
    uint16_t slave_addr)
{
    if (tx_len > BMS_MAX_PAYLOAD_LEN || *rx_len > BMS_MAX_PAYLOAD_LEN)
        return HAL_ERROR;

    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
        uint8_t tx_buf[MAX_FRAME_SIZE];
        uint8_t tx_pos = 0;

        // [0] CMD
        tx_buf[tx_pos++] = cmd;

        // [1] LEN
        tx_buf[tx_pos++] = tx_len;

        // [2:N] PAYLOAD
        if (tx_len && tx_payload) {
            memcpy(&tx_buf[tx_pos], tx_payload, tx_len);
            tx_pos += tx_len;
        }

        // [N+1] CRC
        tx_buf[tx_pos++] = EPS_I2C_CRC8(tx_buf, 2 + tx_len);

        // Transmit full frame
        status = HAL_I2C_Master_Transmit(hi2c, slave_addr, tx_buf, tx_pos, I2C_TIMEOUT_MS);
        if (status != HAL_OK) continue;

        // Prepare RX buffer
        uint8_t rx_buf[MAX_FRAME_SIZE];
        uint8_t expected_rx_len = *rx_len + 3; // CMD + LEN + PAYLOAD + CRC

        status = HAL_I2C_Master_Receive(hi2c, slave_addr, rx_buf, expected_rx_len, I2C_TIMEOUT_MS);
        if (status != HAL_OK) continue;

        // Validate response
        uint8_t response_cmd  = rx_buf[0];
        uint8_t response_len  = rx_buf[1];
        uint8_t response_crc  = rx_buf[2 + response_len];

        if (response_cmd != cmd) continue;
        if (response_len > *rx_len) continue;
        if (EPS_I2C_CRC8(&rx_buf[0], 2 + response_len) != response_crc) continue;

        // Copy payload to user buffer
        memcpy(rx_payload, &rx_buf[2], response_len);
        *rx_len = response_len;
        return HAL_OK;
    }

    return HAL_ERROR;
}


/**
 * Send CMD_SYNC_COUNTER with 8-byte timestamp and expect ACK response
 */
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

    // Expect 0-byte ACK response
    uint8_t dummy_rx[1] = {0};
    uint8_t rx_len = 0;

    return EPS_I2C_TransmitReceiveWithRetry(
        hi2c,
        CMD_SYNC_COUNTER,
        payload, sizeof(payload),
        dummy_rx, &rx_len,
        i2c_slave_addr
    );
}
