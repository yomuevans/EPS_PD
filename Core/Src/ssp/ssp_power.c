/**
  ******************************************************************************
  * @file    ssp_power.c
  * @brief   SSP power management implementation
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

#include "ssp_power.h"

// Enable specific power lines
void SSP_EnableGPSPower(void)
{
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET); // PB6 HIGH
}

void SSP_EnableReserved12VPower(void)
{
    HAL_GPIO_WritePin(RS12V_EN_GPIO_Port, RS12V_EN_Pin, GPIO_PIN_RESET); // PB2 LOW (inverted logic)
}

void SSP_EnableADCSSubsystemPower(void)
{
    HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET); // PA12 HIGH
}

void SSP_EnableADCSPower(void)
{
    HAL_GPIO_WritePin(ADCS5V_EN_GPIO_Port, ADCS5V_EN_Pin, GPIO_PIN_SET); // PE4 HIGH
}

void SSP_EnableADCSVBATPower(void)
{
    HAL_GPIO_WritePin(ADCS12V_EN_GPIO_Port, ADCS12V_EN_Pin, GPIO_PIN_SET); // PB8 HIGH
}

void SSP_EnablePLSubsystemPower(void)
{
    HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, GPIO_PIN_SET); // PD13 HIGH
}

void SSP_EnablePLPower(void)
{
    HAL_GPIO_WritePin(PL5V_EN_GPIO_Port, PL5V_EN_Pin, GPIO_PIN_SET); // PE3 HIGH
}

void SSP_EnableXBPower(void)
{
    HAL_GPIO_WritePin(XB12V_EN_GPIO_Port, XB12V_EN_Pin, GPIO_PIN_SET); // PE8 HIGH
}

void SSP_EnableReservedPower(void)
{
    HAL_GPIO_WritePin(RS5V_EN_GPIO_Port, RS5V_EN_Pin, GPIO_PIN_SET); // PE2 HIGH
}

void SSP_EnableRS3V3Power(void)
{
    HAL_GPIO_WritePin(RS3V3_EN_GPIO_Port, RS3V3_EN_Pin, GPIO_PIN_SET); // PB15 HIGH
}

void SSP_EnableCCUPower(void)
{
    HAL_GPIO_WritePin(CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, GPIO_PIN_SET); // PE10 HIGH
}

void SSP_EnableUHFPower(void)
{
    HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, GPIO_PIN_SET); // PD7 HIGH
}

void SSP_EnableADCS12VPower(void)
{
    HAL_GPIO_WritePin(ADCS12V_EN_GPIO_Port, ADCS12V_EN_Pin, GPIO_PIN_SET); // PB8 HIGH (same as ADCS VBAT)
}

// Disable specific power lines
void SSP_DisableGPSPower(void)
{
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET); // PB6 LOW
}

void SSP_DisableReserved12VPower(void)
{
    HAL_GPIO_WritePin(RS12V_EN_GPIO_Port, RS12V_EN_Pin, GPIO_PIN_SET); // PB2 HIGH (inverted logic)
}

void SSP_DisableADCSSubsystemPower(void)
{
    HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_RESET); // PA12 LOW
}

void SSP_DisableADCSPower(void)
{
    HAL_GPIO_WritePin(ADCS5V_EN_GPIO_Port, ADCS5V_EN_Pin, GPIO_PIN_RESET); // PE4 LOW
}

void SSP_DisableADCSVBATPower(void)
{
    HAL_GPIO_WritePin(ADCS12V_EN_GPIO_Port, ADCS12V_EN_Pin, GPIO_PIN_RESET); // PB8 LOW
}

void SSP_DisablePLSubsystemPower(void)
{
    HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, GPIO_PIN_RESET); // PD13 LOW
}

void SSP_DisablePLPower(void)
{
    HAL_GPIO_WritePin(PL5V_EN_GPIO_Port, PL5V_EN_Pin, GPIO_PIN_RESET); // PE3 LOW
}

void SSP_DisableXBPower(void)
{
    HAL_GPIO_WritePin(XB12V_EN_GPIO_Port, XB12V_EN_Pin, GPIO_PIN_RESET); // PE8 LOW
}

void SSP_DisableReservedPower(void)
{
    HAL_GPIO_WritePin(RS5V_EN_GPIO_Port, RS5V_EN_Pin, GPIO_PIN_RESET); // PE2 LOW
}

void SSP_DisableRS3V3Power(void)
{
    HAL_GPIO_WritePin(RS3V3_EN_GPIO_Port, RS3V3_EN_Pin, GPIO_PIN_RESET); // PB15 LOW
}

void SSP_DisableCCUPower(void)
{
    HAL_GPIO_WritePin(CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, GPIO_PIN_RESET); // PE10 LOW
}

void SSP_DisableUHFPower(void)
{
    HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, GPIO_PIN_RESET); // PD7 LOW
}

void SSP_DisableADCS12VPower(void)
{
    HAL_GPIO_WritePin(ADCS12V_EN_GPIO_Port, ADCS12V_EN_Pin, GPIO_PIN_RESET); // PB8 LOW (same as ADCS VBAT)
}

// Main function to set power lines based on symbol
void SSP_SetPowerLine(uint8_t power_line_symbol)
{
    switch (power_line_symbol) {
        case SSP_PARAM_GPS_EN:
            SSP_EnableGPSPower();
            break;
        case SSP_PARAM_CCU5V_EN:
            SSP_EnableCCUPower();
            break;
        case SSP_PARAM_ADCS5V_EN:
            SSP_EnableADCSPower();
            break;
        case SSP_PARAM_PL5V_EN:
            SSP_EnablePLPower();
            break;
        case SSP_PARAM_RS5V_EN:
            SSP_EnableReservedPower();
            break;
        case SSP_PARAM_ADCS12_EN:
            SSP_EnableADCSVBATPower();
            break;
        case SSP_PARAM_XB12V_EN:
            SSP_EnableXBPower();
            break;
        case SSP_PARAM_RS12V_EN:
            SSP_EnableReserved12VPower();
            break;
        case SSP_PARAM_RS3V3_EN:
            SSP_EnableRS3V3Power();
            break;
        case SSP_PARAM_PL_EN:
            SSP_EnablePLSubsystemPower();
            break;
        case SSP_PARAM_ADCS_EN:
            SSP_EnableADCSSubsystemPower();
            break;
        case SSP_PARAM_UHF_EN:
            SSP_EnableUHFPower();
            break;
        case SSP_PARAM_ADCS12V_EN:
            SSP_EnableADCS12VPower();
            break;
        default:
            break;
    }
}

// Main function to disable power lines based on symbol
void SSP_DisablePowerLine(uint8_t power_line_symbol)
{
    switch (power_line_symbol) {
        case SSP_PARAM_GPS_EN:
            SSP_DisableGPSPower();
            break;
        case SSP_PARAM_CCU5V_EN:
            SSP_DisableCCUPower();
            break;
        case SSP_PARAM_ADCS5V_EN:
            SSP_DisableADCSPower();
            break;
        case SSP_PARAM_PL5V_EN:
            SSP_DisablePLPower();
            break;
        case SSP_PARAM_RS5V_EN:
            SSP_DisableReservedPower();
            break;
        case SSP_PARAM_ADCS12_EN:
            SSP_DisableADCSVBATPower();
            break;
        case SSP_PARAM_XB12V_EN:
            SSP_DisableXBPower();
            break;
        case SSP_PARAM_RS12V_EN:
            SSP_DisableReserved12VPower();
            break;
        case SSP_PARAM_RS3V3_EN:
            SSP_DisableRS3V3Power();
            break;
        case SSP_PARAM_PL_EN:
            SSP_DisablePLSubsystemPower();
            break;
        case SSP_PARAM_ADCS_EN:
            SSP_DisableADCSSubsystemPower();
            break;
        case SSP_PARAM_UHF_EN:
            SSP_DisableUHFPower();
            break;
        case SSP_PARAM_ADCS12V_EN:
            SSP_DisableADCS12VPower();
            break;
        default:
            break;
    }
}
