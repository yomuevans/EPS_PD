/**
  ******************************************************************************
  * @file    ssp_power.h
  * @brief   SSP power management
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

#ifndef __SSP_POWER_H
#define __SSP_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "main.h"

// Power Line Symbols
#define SSP_PARAM_GPS_EN        0x01
#define SSP_PARAM_CCU5V_EN      0x02
#define SSP_PARAM_ADCS5V_EN     0x03
#define SSP_PARAM_PL5V_EN       0x04
#define SSP_PARAM_RS5V_EN       0x05
#define SSP_PARAM_ADCS12_EN     0x06
#define SSP_PARAM_XB12V_EN      0x07
#define SSP_PARAM_RS12V_EN      0x08
#define SSP_PARAM_RS3V3_EN      0x09
#define SSP_PARAM_PL_EN         0x0A
#define SSP_PARAM_ADCS_EN       0x0B
#define SSP_PARAM_UHF_EN        0x0C
#define SSP_PARAM_ADCS12V_EN    0x0D

// Function Declarations
void SSP_SetPowerLine(uint8_t power_line_symbol);
void SSP_DisablePowerLine(uint8_t power_line_symbol);

#ifdef __cplusplus
}
#endif

#endif /* __SSP_POWER_H */
