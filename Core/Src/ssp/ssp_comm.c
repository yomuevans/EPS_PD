/**
  ******************************************************************************
  * @file    ssp_comm.c
  * @brief   SSP communication implementation
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

#include "ssp_comm.h"
#include "ssp_frame.h"
#include <string.h>

extern UART_HandleTypeDef *HuartRs485_1;
extern UART_HandleTypeDef *HuartRs485_2;
extern GPIO_TypeDef *De1Port;
extern uint16_t De1Pin;
extern GPIO_TypeDef *De2Port;
extern uint16_t De2Pin;
extern uint8_t RxBuffer1[SSP_MAX_FRAME_LEN];
extern uint8_t RxBuffer2[SSP_MAX_FRAME_LEN];
extern volatile uint8_t Rx1Complete;
extern volatile uint8_t Rx2Complete;

HAL_StatusTypeDef SSP_SendFrame(UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin, SSP_Frame *frame)
{
    uint8_t buffer[SSP_MAX_FRAME_LEN];
    uint16_t len = 0;

    buffer[len++] = frame->StartByte;
    buffer[len++] = frame->SrcAddr;
    buffer[len++] = frame->DestAddr;
    buffer[len++] = frame->CmdId;
    buffer[len++] = frame->DataLen;
    if (frame->DataLen > 0) {
        memcpy(&buffer[len], frame->Data, frame->DataLen);
        len += frame->DataLen;
    }
    buffer[len++] = (frame->Crc >> 8) & 0xFF;
    buffer[len++] = frame->Crc & 0xFF;
    buffer[len++] = frame->EndByte;

    if (de_port != NULL) {
        HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_SET);
        // Delay not needed; THVD1450DR enable time is ~18-65ns (Page 9)
        // Ensure GPIO write is complete before transmission
        __NOP(); // Small delay for GPIO stability
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(huart, buffer, len);
    if (status != HAL_OK) {
        // Immediately disable driver if transmission fails
        if (de_port != NULL) {
            HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
        }
    }
    return status;
}

HAL_StatusTypeDef SSP_ReceiveFrame(UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin,
                                   SSP_Frame *frame, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    volatile uint8_t *rx_complete = (huart == HuartRs485_1) ? &Rx1Complete : &Rx2Complete;
    uint8_t *rx_buffer = (huart == HuartRs485_1) ? RxBuffer1 : RxBuffer2;

    if (de_port != NULL) {
        HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
        // THVD1450DR receiver enable time is ~50-130ns (Page 9)
        __NOP(); // Small delay for stability
    }

    while (!(*rx_complete)) {
        if (HAL_GetTick() - start_time >= timeout) {
            // Check for failsafe condition (Page 17)
            // THVD1450DR outputs logic high if bus is open, shorted, or idle for >25-35ns
            return HAL_TIMEOUT;
        }
    }

    *rx_complete = 0;

    uint16_t start_idx = 0;
    while (start_idx < SSP_MAX_FRAME_LEN && rx_buffer[start_idx] != SSP_START_END_BYTE) {
        start_idx++;
    }

    if (start_idx >= SSP_MAX_FRAME_LEN) return HAL_ERROR;

    uint8_t buffer[SSP_MAX_FRAME_LEN];
    uint16_t len = 0;
    for (uint16_t i = start_idx; i < SSP_MAX_FRAME_LEN; i++) {
        buffer[len++] = rx_buffer[i];
        if (rx_buffer[i] == SSP_START_END_BYTE && len > 1) break;
    }

    return SSP_UnpackFrame(buffer, len, frame);
}

void SSP_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == HuartRs485_1) {
        Rx1Complete = 1;
        HAL_UART_Receive_IT(HuartRs485_1, RxBuffer1, SSP_MAX_FRAME_LEN);
    } else if (huart == HuartRs485_2) {
        Rx2Complete = 1;
        HAL_UART_Receive_IT(HuartRs485_2, RxBuffer2, SSP_MAX_FRAME_LEN);
    }
}

void SSP_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == HuartRs485_1) {
        HAL_GPIO_WritePin(De1Port, De1Pin, GPIO_PIN_RESET);
    } else if (huart == HuartRs485_2) {
        HAL_GPIO_WritePin(De2Port, De2Pin, GPIO_PIN_RESET);
    }
}
