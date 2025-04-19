/**
  ******************************************************************************
  * @file    ssp_command.h
  * @brief   SSP command processing
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

#ifndef __SSP_COMMAND_H
#define __SSP_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ssp_common.h"  // Include SSP_Frame definition
#include "ssp_power.h"
#include "ssp_telemetry.h"

// Command IDs
#define SSP_CMD_PING 0x01
#define SSP_CMD_GD   0x02
#define SSP_CMD_PD   0x03
#define SSP_CMD_RD   0x04
#define SSP_CMD_WD   0x05
#define SSP_CMD_SON  0x06
#define SSP_CMD_SOF  0x07
#define SSP_CMD_SM   0x08
#define SSP_CMD_GM   0x09
#define SSP_CMD_GSC  0x0A
#define SSP_CMD_SSC  0x0B
#define SSP_CMD_GFP  0x0C
#define SSP_CMD_SFP  0x0D
#define SSP_CMD_FON  0x0E
#define SSP_CMD_FOF  0x0F
#define SSP_CMD_GOSTM 0x10
#define SSP_CMD_KEN   0x11
#define SSP_CMD_KDIS  0x12
#define SSP_CMD_RESET_SUBSYSTEM 0x13

// Mode Symbols
#define SSP_PARAM_INITIALIZE   0x01
#define SSP_PARAM_DETUMBLE     0x02
#define SSP_PARAM_NORMAL       0x03
#define SSP_PARAM_COMMUNICATION 0x04
#define SSP_PARAM_PAYLOAD      0x05
#define SSP_PARAM_IMAGE        0x06
#define SSP_PARAM_EMERGENCY    0x07

// Function IDs
#define SSP_FUNC_ID_BATTERY_MONITOR  0x01
#define SSP_FUNC_ID_POWER_REGULATION 0x02

typedef struct {
    uint8_t FuncId;
    uint16_t Value;
} SSP_FunctionParam;

void SSP_HandleCommand(SSP_Frame *rx_frame, SSP_Frame *tx_frame, UART_HandleTypeDef *huart,
                       UART_HandleTypeDef *huart_log, GPIO_TypeDef *de_port, uint16_t de_pin);

#ifdef __cplusplus
}
#endif

#endif /* __SSP_COMMAND_H */
