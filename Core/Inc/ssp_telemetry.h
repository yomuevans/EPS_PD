/**
  ******************************************************************************
  * @file    ssp_telemetry.h
  * @brief   SSP telemetry and parameter management
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

#ifndef __SSP_TELEMETRY_H
#define __SSP_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "main.h"
#include <stdint.h>

// Parameter IDs
#define SSP_PARAM_ID_VOLTAGE_12V    0x01  // 12V bus voltage (mV)
#define SSP_PARAM_ID_VOLTAGE_5V     0x02  // 5V bus voltage (mV)
#define SSP_PARAM_ID_VOLTAGE_3V3    0x03  // 3.3V bus voltage (mV)
#define SSP_PARAM_ID_CURRENT_12V    0x04  // 12V bus current (mA)
#define SSP_PARAM_ID_CURRENT_5V     0x05  // 5V bus current (mA)
#define SSP_PARAM_ID_CURRENT_3V3    0x06  // 3.3V bus current (mA)
#define SSP_PARAM_ID_CURRENT_SA1    0x07  // Solar Array 1 current (mA)
#define SSP_PARAM_ID_CURRENT_SA2    0x08  // Solar Array 2 current (mA)
#define SSP_PARAM_ID_CURRENT_SA3    0x09  // Solar Array 3 current (mA)
#define SSP_PARAM_ID_CURRENT_XB     0x0A  // X-Band transmitter current (mA)
#define SSP_PARAM_ID_CURRENT_CCU    0x0B  // CCU general current (mA)
#define SSP_PARAM_ID_CURRENT_ADCS   0x0C  // ADCS general current (mA)
#define SSP_PARAM_ID_CURRENT_GPS    0x0D  // GPS current (mA)
#define SSP_PARAM_ID_CURRENT_PL     0x0E  // PL general current (mA)
#define SSP_PARAM_ID_CURRENT_UHF    0x0F  // UHF current (mA)
#define SSP_PARAM_ID_CURRENT_OBC    0x10  // OBC current (mA)
#define SSP_PARAM_ID_CURRENT_CCU5V  0x11  // CCU 5V current (mA)
#define SSP_PARAM_ID_CURRENT_ADCS5V 0x12  // ADCS 5V current (mA)
#define SSP_PARAM_ID_CURRENT_PL5V   0x13  // PL 5V current (mA)
#define SSP_PARAM_ID_CURRENT_RS5V   0x14  // Reserved 5V current (mA)
#define SSP_PARAM_ID_CURRENT_ADCS12V 0x15 // ADCS VBAT (12V) current (mA)
#define SSP_PARAM_ID_CURRENT_XB12V  0x16  // X-Band 12V current (mA)
#define SSP_PARAM_ID_VOLTAGE_SA1    0x17  // Solar Array 1 voltage (mV, multiplexed)
#define SSP_PARAM_ID_VOLTAGE_SA2    0x18  // Solar Array 2 voltage (mV, multiplexed)
#define SSP_PARAM_ID_VOLTAGE_SA3    0x19  // Solar Array 3 voltage (mV, multiplexed)
// Note: SSP_PARAM_ID_CURRENT_RS12V is not defined as RS12V_I is not assigned

/* Telemetry Structure */
typedef struct {
    uint16_t Bus12V; // in mV
    uint16_t Bus5V;  // in mV
    uint16_t Bus3V3; // in mV
} SSP_Telemetry;

/* Parameter Structure */
typedef struct {
    uint8_t ParamId;
    uint16_t Value;
} SSP_Parameter;

/* Function Declarations */
void SSP_UpdateTelemetryAndParameters(uint16_t *adc_values);
SSP_Telemetry* SSP_GetTelemetry(void);
SSP_Parameter* SSP_GetParameters(uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* __SSP_TELEMETRY_H */
