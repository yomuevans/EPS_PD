/**
  ******************************************************************************
  * @file    ssp_init.h
  * @brief   Initialization for SSP protocol
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

#ifndef __SSP_INIT_H
#define __SSP_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ssp_common.h"

void SSP_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, UART_HandleTypeDef *huart3,
              GPIO_TypeDef *de1_port, uint16_t de1_pin, GPIO_TypeDef *de2_port, uint16_t de2_pin);

// External variable declarations
extern UART_HandleTypeDef *HuartRs485_1;
extern UART_HandleTypeDef *HuartRs485_2;
extern UART_HandleTypeDef *HuartRs485_Alt;
extern GPIO_TypeDef *De1Port;
extern uint16_t De1Pin;
extern GPIO_TypeDef *De2Port;
extern uint16_t De2Pin;
extern uint8_t RxBuffer1[SSP_MAX_FRAME_LEN];
extern uint8_t RxBuffer2[SSP_MAX_FRAME_LEN];
extern uint8_t RxBufferAlt[SSP_MAX_FRAME_LEN];
extern volatile uint8_t Rx1Complete;
extern volatile uint8_t Rx2Complete;
extern volatile uint8_t RxAltComplete;

#ifdef __cplusplus
}
#endif

#endif /* __SSP_INIT_H */
