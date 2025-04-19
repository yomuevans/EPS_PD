/**
  ******************************************************************************
  * @file    ssp_init.c
  * @brief   Initialization implementation for SSP protocol
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

#include "ssp_init.h"
#include "ssp_common.h"
#include <string.h>

// Private variables
// Global variables (no static)
UART_HandleTypeDef *HuartRs485_1 = NULL;    // RS485_1 (PD5/PD6, USART2) - Communicates with subsystems (OBC, CCU, etc.)
UART_HandleTypeDef *HuartRs485_2 = NULL;    // RS485_2 (PB10/PB11, USART3) - Communicates with BMS (STM32L476RCT6)
UART_HandleTypeDef *HuartRs485_Alt = NULL;  // Not used (previously for toggling RS485_1)
GPIO_TypeDef *De1Port = NULL;               // DE pin for RS485_1 (PD4)
uint16_t De1Pin = 0;
GPIO_TypeDef *De2Port = NULL;               // DE pin for RS485_2 (PD12)
uint16_t De2Pin = 0;

// Receive Buffers for Interrupt-Driven UART
uint8_t RxBuffer1[SSP_MAX_FRAME_LEN];
uint8_t RxBuffer2[SSP_MAX_FRAME_LEN];
uint8_t RxBufferAlt[SSP_MAX_FRAME_LEN];     // Not used
volatile uint8_t Rx1Complete = 0;
volatile uint8_t Rx2Complete = 0;
volatile uint8_t RxAltComplete = 0;         // Not used

void SSP_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, UART_HandleTypeDef *huart3,
              GPIO_TypeDef *de1_port, uint16_t de1_pin, GPIO_TypeDef *de2_port, uint16_t de2_pin)
{
    HuartRs485_1 = huart2; // Use USART2 for RS485_1 (subsystem communication)
    HuartRs485_2 = huart3; // Use USART3 for RS485_2 (BMS communication)
    HuartRs485_Alt = NULL; // Not used
    De1Port = de1_port;
    De1Pin = de1_pin;
    De2Port = de2_port;
    De2Pin = de2_pin;

    // Start UART interrupt reception
    HAL_UART_Receive_IT(HuartRs485_1, RxBuffer1, SSP_MAX_FRAME_LEN);
    HAL_UART_Receive_IT(HuartRs485_2, RxBuffer2, SSP_MAX_FRAME_LEN);
}
