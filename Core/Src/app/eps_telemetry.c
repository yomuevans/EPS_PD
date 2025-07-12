/*
 * eps_telemetry.c
 *
 *  Created on: Jul 12, 2025
 *      Author: yomue
 */


// File: eps_telemetry.c

#include "eps_telemetry.h"
#include <string.h>

void EPS_SendCombinedTelemetry(UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef *hdma_tx,
                               I2C_HandleTypeDef *hi2c,
                               const SSP_Frame_t *frame)
{
    CombinedTelemetry_t combined = {0};
    SSP_Frame_t reply = {0};

    // --- EPS Telemetry ---
    HAL_StatusTypeDef eps_status = epspd_ReadTelemetry(hi2c, &combined.eps_telemetry);
    if (eps_status != HAL_OK) {
        reply.cmd = SSP_CMD_NACK;
        reply.len = 1;
        reply.data[0] = frame->cmd;
        SSP_SendCommand(huart, hdma_tx, &reply);
        return;
    }

    // --- BMS Telemetry ---
    uint8_t bms_rx_len = MAX_BMS_DATA_LEN;
    if (EPS_I2C_TransmitReceiveWithRetry(hi2c,
                                         CMD_GET_TELEMETRY,
                                         NULL, 0,
                                         combined.bms_data, &bms_rx_len,
                                         EPS_BMS_I2C_ADDR) != HAL_OK)
    {
        reply.cmd = SSP_CMD_NACK;
        reply.len = 1;
        reply.data[0] = frame->cmd;
        SSP_SendCommand(huart, hdma_tx, &reply);
        return;
    }

    // --- Fault Log ---
    if (EPS_GetFaultLogCount() > 0) {
        if (EPS_ReadFaultLog(hi2c, 0, &combined.fault_log) != HAL_OK) {
            reply.cmd = SSP_CMD_NACK;
            reply.len = 1;
            reply.data[0] = frame->cmd;
            SSP_SendCommand(huart, hdma_tx, &reply);
            return;
        }
    }

    // --- Send Combined Telemetry ---
    reply.cmd = SSP_CMD_GD;
    reply.len = sizeof(combined);
    memcpy(reply.data, &combined, reply.len);
    SSP_SendCommand(huart, hdma_tx, &reply);
}
