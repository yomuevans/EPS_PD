/**
  ******************************************************************************
  * @file    ssp_telemetry.c
  * @brief   SSP telemetry and parameter implementation
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

#include "main.h" // For GPIO pin definitions (S0, S1, S2) and SSP includes

// ADC Scaling Constants
#define ADC_VREF_MV 3300
#define ADC_MAX_COUNT 4096
#define VOLTAGE_PER_COUNT (ADC_VREF_MV / (float)ADC_MAX_COUNT)

// Voltage Divider Scaling for SAx_V (SA1_V, SA2_V, SA3_V)
// Voltage Divider: R1 = 100kΩ (top), R2 = 12.4kΩ (bottom, RC0805FR-0712K4L)
// V_out = V_in * (R2 / (R1 + R2)) = V_in * (12.4k / (100k + 12.4k)) ≈ V_in * 0.1103
// Inverse scaling to get V_in: V_in = V_out * (112.4k / 12.4k) ≈ V_out * 9.0645
// Combined with ADC scaling (VOLTAGE_PER_COUNT ≈ 0.805 mV/count):
// V_in (mV) = (adc_values * VOLTAGE_PER_COUNT) * 9.0645 ≈ adc_values * 7.297 mV/count
#define VOLTAGE_SCALING_FACTOR 7.297

// Current Scaling for SAx_I (SA1_I, SA2_I, SA3_I) via ZXCT1086E5TA
// ZXCT1086E5TA gain: V_out = 50 * V_SENSE = 50 * (I * R_SHUNT), R_SHUNT = 0.1Ω
// V_out = 5I (I in A, V_out in V) → I (mA) = (adc_values * VOLTAGE_PER_COUNT) / 5
// I (mA) = (adc_values * 0.805) / 5 ≈ adc_values * 0.161 mA/count
#define CURRENT_SCALING_FACTOR 0.161  // For SAx_I: (3300 / 4096) / 5 ≈ 0.161 mA/count

// IMON Scaling for TPS259621DDAT and TPS259631DDAT
#define IMON_K_ILM 656
#define R_IMON 464
#define IMON_CURRENT_PER_VOLT (1000.0 / (R_IMON * IMON_K_ILM * 1e-6))

// Shunt Resistor for Other Current Measurements
#define SHUNT_RESISTOR_OHMS 0.1
#define CURRENT_PER_COUNT (VOLTAGE_PER_COUNT / SHUNT_RESISTOR_OHMS)

// Telemetry Data
static SSP_Telemetry TelemetryData = {
    .Bus12V = 0,
    .Bus5V = 0,
    .Bus3V3 = 0
};

// Parameter Data
static SSP_Parameter Parameters[] = {
    {SSP_PARAM_ID_VOLTAGE_12V, 0},    // Bus12V
    {SSP_PARAM_ID_VOLTAGE_5V, 0},     // Bus5V
    {SSP_PARAM_ID_VOLTAGE_3V3, 0},    // Bus3V3
    {SSP_PARAM_ID_CURRENT_12V, 0},    // Current for 12V bus
    {SSP_PARAM_ID_CURRENT_5V, 0},     // Current for 5V bus
    {SSP_PARAM_ID_CURRENT_3V3, 0},    // Current for 3.3V bus
    {SSP_PARAM_ID_CURRENT_SA1, 0},    // Solar Array 1 current (SA1_I, multiplexed)
    {SSP_PARAM_ID_CURRENT_SA2, 0},    // Solar Array 2 current (SA2_I, direct)
    {SSP_PARAM_ID_CURRENT_SA3, 0},    // Solar Array 3 current (SA3_I, direct)
    {SSP_PARAM_ID_CURRENT_XB, 0},     // X-Band transmitter current (XB12V_I)
    {SSP_PARAM_ID_CURRENT_CCU, 0},    // CCU general current (CCU_I)
    {SSP_PARAM_ID_CURRENT_ADCS, 0},   // ADCS general current (ADCS_I)
    {SSP_PARAM_ID_CURRENT_GPS, 0},    // GPS current (GPS_I)
    {SSP_PARAM_ID_CURRENT_PL, 0},     // PL general current (PL_I)
    {SSP_PARAM_ID_CURRENT_UHF, 0},    // UHF current (UHF_I)
    {SSP_PARAM_ID_CURRENT_OBC, 0},    // OBC current (OBC_I)
    {SSP_PARAM_ID_CURRENT_CCU5V, 0},  // CCU 5V current (CCU5V_I)
    {SSP_PARAM_ID_CURRENT_ADCS5V, 0}, // ADCS 5V current (ADCS5V_I)
    {SSP_PARAM_ID_CURRENT_PL5V, 0},   // PL 5V current (PL5V_I)
    {SSP_PARAM_ID_CURRENT_RS5V, 0},   // Reserved 5V current (RS5V_I)
    {SSP_PARAM_ID_CURRENT_ADCS12V, 0},// ADCS VBAT current (ADCS12V_I)
    {SSP_PARAM_ID_CURRENT_XB12V, 0},  // XB 12V current (XB12V_I)
    {SSP_PARAM_ID_VOLTAGE_SA1, 0},    // Solar Array 1 voltage (SA1_V, multiplexed)
    {SSP_PARAM_ID_VOLTAGE_SA2, 0},    // Solar Array 2 voltage (SA2_V, multiplexed)
    {SSP_PARAM_ID_VOLTAGE_SA3, 0}     // Solar Array 3 voltage (SA3_V, multiplexed)
};
static uint8_t ParameterCount = sizeof(Parameters) / sizeof(Parameters[0]);

// Function to select a channel on the CD74HC4051 multiplexer
static void SelectMultiplexerChannel(uint8_t channel)
{
    // Channel selection using S0 (PC7), S1 (PC8), S2 (PC9)
    // S2 S1 S0 -> Channel -> Signal
    // 0  0  0  -> A0      -> SA1_I
    // 0  0  1  -> A1      -> SA3_V
    // 0  1  0  -> A2      -> SA2_V
    // 0  1  1  -> A3      -> SA1_V
    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // S0
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // S1
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // S2
}

void SSP_UpdateTelemetryAndParameters(uint16_t *adc_values)
{
    // Multiplexed measurements via PA0 (ADC1_IN5, index 4 in AdcValues)
    // Channel 0 (A0): SA1_I
    SelectMultiplexerChannel(0); // S2=0, S1=0, S0=0
    HAL_Delay(1); // Small delay to allow multiplexer to settle
    Parameters[6].Value = (uint16_t)(adc_values[4] * CURRENT_SCALING_FACTOR); // SA1_I in mA

    // Channel 1 (A1): SA3_V
    SelectMultiplexerChannel(1); // S2=0, S1=0, S0=1
    HAL_Delay(1);
    Parameters[24].Value = (uint16_t)(adc_values[4] * VOLTAGE_SCALING_FACTOR); // SA3_V in mV

    // Channel 2 (A2): SA2_V
    SelectMultiplexerChannel(2); // S2=0, S1=1, S0=0
    HAL_Delay(1);
    Parameters[23].Value = (uint16_t)(adc_values[4] * VOLTAGE_SCALING_FACTOR); // SA2_V in mV

    // Channel 3 (A3): SA1_V
    SelectMultiplexerChannel(3); // S2=0, S1=1, S0=1
    HAL_Delay(1);
    Parameters[22].Value = (uint16_t)(adc_values[4] * VOLTAGE_SCALING_FACTOR); // SA1_V in mV

    // Direct ADC measurements (using DMA values)
    // Index 0: ADC1_IN1 (PC0, RS5V_I)
    float v_imon = adc_values[0] * VOLTAGE_PER_COUNT;
    Parameters[19].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // RS5V_I

    // Index 1: ADC1_IN2 (PC1, PL5V_I)
    v_imon = adc_values[1] * VOLTAGE_PER_COUNT;
    Parameters[18].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // PL5V_I

    // Index 2: ADC1_IN3 (PC2, ADCS5V_I)
    v_imon = adc_values[2] * VOLTAGE_PER_COUNT;
    Parameters[17].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // ADCS5V_I

    // Index 3: ADC1_IN4 (PC3, GPS_I)
    v_imon = adc_values[3] * VOLTAGE_PER_COUNT;
    Parameters[12].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // GPS_I

    // Index 4: ADC1_IN5 (PA0, M/OUT, handled above via multiplexer)

    // Index 5: ADC1_IN6 (PA1, SA2_I)
    Parameters[7].Value = (uint16_t)(adc_values[5] * CURRENT_SCALING_FACTOR); // SA2_I in mA

    // Index 6: ADC1_IN7 (PA2, SA3_I)
    Parameters[8].Value = (uint16_t)(adc_values[6] * CURRENT_SCALING_FACTOR); // SA3_I in mA

    // Index 7: ADC1_IN8 (PA3, XB12V_I)
    Parameters[9].Value = (uint16_t)(adc_values[7] * CURRENT_PER_COUNT); // XB12V_I
    Parameters[21].Value = (uint16_t)((adc_values[7] * VOLTAGE_PER_COUNT / 1000.0) * IMON_CURRENT_PER_VOLT); // XB12V_I

    // Index 8: ADC1_IN9 (PA4, CCU5V_I)
    v_imon = adc_values[8] * VOLTAGE_PER_COUNT;
    Parameters[16].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // CCU5V_I

    // Index 9: ADC1_IN10 (PA5, CCU_I)
    Parameters[10].Value = (uint16_t)(adc_values[9] * CURRENT_PER_COUNT); // CCU_I

    // Index 10: ADC1_IN11 (PA6, RS3V3_I)
    v_imon = adc_values[10] * VOLTAGE_PER_COUNT;
    Parameters[5].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // RS3V3_I

    // Index 11: ADC1_IN12 (PA7, ADCS_I)
    Parameters[11].Value = (uint16_t)(adc_values[11] * CURRENT_PER_COUNT); // ADCS_I

    // Index 12: ADC1_IN13 (PC4, PL_I)
    Parameters[13].Value = (uint16_t)(adc_values[12] * CURRENT_PER_COUNT); // PL_I

    // Index 13: ADC1_IN14 (PC5, ADCS12V_I)
    v_imon = adc_values[13] * VOLTAGE_PER_COUNT;
    Parameters[20].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // ADCS12V_I

    // Index 14: ADC1_IN15 (PB0, UHF_I)
    v_imon = adc_values[14] * VOLTAGE_PER_COUNT;
    Parameters[14].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // UHF_I

    // Index 15: ADC1_IN16 (PB1, OBC_I)
    v_imon = adc_values[15] * VOLTAGE_PER_COUNT;
    Parameters[15].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT); // OBC_I

    // Telemetry: Use PL5V_I as the representative 5V bus voltage, ADCS12V_I for 12V
    TelemetryData.Bus5V  = (uint16_t)(adc_values[1] * VOLTAGE_PER_COUNT);  // PL5V_I (PC1)
    TelemetryData.Bus3V3 = (uint16_t)(adc_values[10] * VOLTAGE_PER_COUNT); // RS3V3_I (PA6)
    TelemetryData.Bus12V = (uint16_t)(adc_values[13] * VOLTAGE_PER_COUNT); // ADCS12V_I (PC5)

    Parameters[0].Value = TelemetryData.Bus12V;
    Parameters[1].Value = TelemetryData.Bus5V;
    Parameters[2].Value = TelemetryData.Bus3V3;
}

SSP_Telemetry* SSP_GetTelemetry(void)
{
    return &TelemetryData;
}

SSP_Parameter* SSP_GetParameters(uint8_t *count)
{
    *count = ParameterCount;
    return Parameters;
}
