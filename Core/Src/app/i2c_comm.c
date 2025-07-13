#include "i2c_comm.h"
#include "main.h"
#include "Log.h"
#include <stdio.h>
#include <string.h>

#define I2C_TIMEOUT_MS         100
#define MAX_FRAME_SIZE         256
#define MAX_I2C_RETRIES        5
#define MAX_TELEMETRY_FRAMES   32

// ─────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────
// Generic transmit + multi-frame receive with retry
HAL_StatusTypeDef EPS_I2C_TransmitReceiveWithRetry(
    I2C_HandleTypeDef *hi2c,
    uint8_t cmd,
    const uint8_t *tx_payload, uint8_t tx_len,
    uint8_t *rx_payload, uint16_t *rx_leng,
    uint16_t slave_addr)
{
    if (tx_len > MAX_FRAME_SIZE - 3) return HAL_ERROR;

    HAL_StatusTypeDef status = HAL_ERROR;
    *rx_leng = 0;

    for (uint8_t attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
        uint8_t tx_buf[MAX_FRAME_SIZE];
        uint8_t tx_pos = 0;

        tx_buf[tx_pos++] = cmd;
        tx_buf[tx_pos++] = tx_len;

        if (tx_len && tx_payload) {
            memcpy(&tx_buf[tx_pos], tx_payload, tx_len);
            tx_pos += tx_len;
        }

        // Compute CRC for the current buffer, append at current tx_pos
        uint8_t crc = EPS_I2C_CRC8(tx_buf, tx_pos);
        tx_buf[tx_pos++] = crc;

        status = HAL_I2C_Master_Transmit(hi2c, slave_addr, tx_buf, tx_pos, I2C_TIMEOUT_MS);
        if (status != HAL_OK) {
            EPS_Log_Message(EPS_LOG_WARNING, "I2C TX retry %u failed", attempt);
            continue;
        }

        for (uint8_t frame_idx = 0; frame_idx < MAX_TELEMETRY_FRAMES; ++frame_idx) {
            uint8_t rx_buf[MAX_FRAME_SIZE];
            status = HAL_I2C_Master_Receive(hi2c, slave_addr, rx_buf, MAX_FRAME_SIZE, I2C_TIMEOUT_MS);
            if (status != HAL_OK) {
                EPS_Log_Message(EPS_LOG_ERROR, "I2C RX failed at frame %u", frame_idx);
                break;
            }

            uint8_t rx_cmd = rx_buf[0];
            uint8_t rx_len = rx_buf[1];
            uint8_t rx_crc = rx_buf[2 + rx_len];
            uint8_t calc_crc = EPS_I2C_CRC8(&rx_buf[1], 1 + rx_len); // Only LEN + payload

            EPS_Log_Message(EPS_LOG_INFO,
                "[Frame %u] CMD=0x%02X LEN=%u CRC=0x%02X %s",
                frame_idx, rx_cmd, rx_len, rx_crc,
                (rx_crc == calc_crc && rx_cmd == cmd) ? "(OK)" : "(BAD)");

            if (rx_crc != calc_crc || rx_cmd != cmd) {
                EPS_Log_Message(EPS_LOG_ERROR, "CRC or CMD mismatch in frame %u", frame_idx);
                break;
            }

            // Hex dump payload
            char hex[512] = {0}, *p = hex;
            for (uint8_t i = 0; i < rx_len && i < 64; ++i)
                p += snprintf(p, sizeof(hex) - (p - hex), "%02X ", rx_buf[2 + i]);
            EPS_Log_Message(EPS_LOG_DEBUG, "Payload: %s", hex);

            // Copy payload to destination buffer
            memcpy(&rx_payload[*rx_leng], &rx_buf[2], rx_len);
            *rx_leng += rx_len;

            // Stop if last frame (shorter than full frame size)
            if (rx_len < MAX_FRAME_SIZE - 3) break;
        }

        if (*rx_leng > 0)
            return HAL_OK;
    }

    return HAL_ERROR;
}



HAL_StatusTypeDef EPS_I2C_SendCommand(
    I2C_HandleTypeDef *hi2c,
    uint8_t cmd,
    const uint8_t *payload, uint8_t payload_len,
    uint8_t *response_buf, uint16_t response_buf_len,
    uint16_t slave_addr)
{
    uint16_t received_len = 0;

    HAL_StatusTypeDef status = EPS_I2C_TransmitReceiveWithRetry(
        hi2c,
        cmd,
        payload,
        payload_len,
        response_buf,
        &received_len,
        slave_addr
    );

    // If the caller provided a response buffer length, enforce bounds check
    if (response_buf && response_buf_len > 0) {
        if (received_len > response_buf_len) {
            EPS_Log_Message(EPS_LOG_ERROR, "Received data exceeds caller buffer (%u > %u)", received_len, response_buf_len);
            return HAL_ERROR;
        }
    }

    return status;
}


// ─────────────────────────────────────────────────────────────
// Wrapper: Request Telemetry and cast result into TelemetryData
HAL_StatusTypeDef EPS_I2C_RequestTelemetry(
    I2C_HandleTypeDef *hi2c,
    uint16_t slave_addr,
    BMSTelemetryData *out_telemetry)
{
    if (!out_telemetry) return HAL_ERROR;

    uint8_t raw_payload[sizeof(BMSTelemetryData)] = {0};
    uint16_t received_len = 0;

    HAL_StatusTypeDef status = EPS_I2C_TransmitReceiveWithRetry(
        hi2c,
        CMD_GET_TELEMETRY,
        NULL, 0,
        raw_payload, &received_len,
        slave_addr
    );

    if (status != HAL_OK || received_len < sizeof(BMSTelemetryData)) {
        EPS_Log_Message(EPS_LOG_ERROR, "Telemetry receive failed or size mismatch: %u bytes", received_len);
        return HAL_ERROR;
    }

    memcpy(out_telemetry, raw_payload, sizeof(BMSTelemetryData));
    return HAL_OK;
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

    // Expect 0-byte ACK response
    uint8_t dummy_rx[1] = {0};
    uint16_t rx_len = 0;

    return EPS_I2C_TransmitReceiveWithRetry(
        hi2c,
        CMD_SYNC_COUNTER,
        payload, sizeof(payload),
        dummy_rx, &rx_len,
        i2c_slave_addr
    );
}
