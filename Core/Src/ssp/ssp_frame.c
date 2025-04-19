/**
  ******************************************************************************
  * @file    ssp_frame.c
  * @brief   SSP frame handling implementation
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

#include "ssp_frame.h"
#include <string.h>

void SSP_PackFrame(SSP_Frame *frame, uint8_t src_addr, uint8_t dest_addr, uint8_t cmd_id,
                   const uint8_t *data, uint8_t data_len)
{
    frame->StartByte = SSP_START_END_BYTE;
    frame->SrcAddr = src_addr;
    frame->DestAddr = dest_addr;
    frame->CmdId = cmd_id;
    frame->DataLen = data_len;
    if (data_len > 0 && data != NULL) {
        memcpy(frame->Data, data, data_len);
    }
    uint8_t crc_buffer[SSP_MAX_DATA_LEN + 4];
    crc_buffer[0] = src_addr;
    crc_buffer[1] = dest_addr;
    crc_buffer[2] = cmd_id;
    crc_buffer[3] = data_len;
    if (data_len > 0) {
        memcpy(&crc_buffer[4], frame->Data, data_len);
    }
    frame->Crc = SSP_CalculateCRC(crc_buffer, 4 + data_len);
    frame->EndByte = SSP_START_END_BYTE;
}

HAL_StatusTypeDef SSP_UnpackFrame(const uint8_t *buffer, uint16_t len, SSP_Frame *frame)
{
    if (len < SSP_FRAME_OVERHEAD + 1) return HAL_ERROR;
    if (buffer[0] != SSP_START_END_BYTE || buffer[len - 1] != SSP_START_END_BYTE) return HAL_ERROR;

    frame->StartByte = buffer[0];
    frame->SrcAddr = buffer[1];
    frame->DestAddr = buffer[2];
    frame->CmdId = buffer[3];
    frame->DataLen = buffer[4];

    if (frame->DataLen > SSP_MAX_DATA_LEN) return HAL_ERROR;
    if (len != (SSP_FRAME_OVERHEAD + frame->DataLen + 1)) return HAL_ERROR;

    if (frame->DataLen > 0) {
        memcpy(frame->Data, &buffer[5], frame->DataLen);
    }

    frame->Crc = (buffer[5 + frame->DataLen] << 8) | buffer[5 + frame->DataLen + 1];
    frame->EndByte = buffer[len - 1];

    uint8_t crc_buffer[SSP_MAX_DATA_LEN + 4];
    crc_buffer[0] = frame->SrcAddr;
    crc_buffer[1] = frame->DestAddr;
    crc_buffer[2] = frame->CmdId;
    crc_buffer[3] = frame->DataLen;
    if (frame->DataLen > 0) {
        memcpy(&crc_buffer[4], frame->Data, frame->DataLen);
    }
    uint16_t calculated_crc = SSP_CalculateCRC(crc_buffer, 4 + frame->DataLen);
    if (calculated_crc != frame->Crc) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

uint16_t SSP_CalculateCRC(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
