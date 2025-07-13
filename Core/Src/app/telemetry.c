// File: telemetry.c
// Include header files needed for the code
#include "main.h"             // Main header with GPIO pin definitions and hardware handles
#include "telemetry.h"        // Defines EPSPD_Telemetry, EPSPD_Parameter structures
#include "sync_counter.h"     // Functions for getting sync counter and timestamp
#include "eeprom.h"
#include "delay.h"// Functions for saving telemetry to EEPROM via I2C

// Define constants for ADC conversion
#define ADC_VREF_MV 3300      // ADC reference voltage (3.3V, 3300mV)
#define ADC_MAX_COUNT 4096    // Maximum ADC count for 12-bit resolution (2^12 = 4096)
#define VOLTAGE_PER_COUNT (ADC_VREF_MV / (float)ADC_MAX_COUNT) // Voltage per ADC count (3300mV / 4096 ≈ 0.8057mV)
#define VOLTAGE_SCALING_FACTOR 9.26446 // Scaling factor for voltage measurements (e.g., for solar arrays)
#define CURRENT_SCALING_FACTOR 0.16117  // Scaling factor for current measurements
#define IMON_K_ILM 656               // Current monitor gain constant (in µA/V)
#define R_IMON 464                   // Current monitor resistor (464 ohms)
#define IMON_CURRENT_PER_VOLT (1000.0 / (R_IMON * IMON_K_ILM * 1e-6)) // Current per volt (mA/V)
#define CURRENT_PER_COUNT (VOLTAGE_PER_COUNT / 0.1) // Current per ADC count (using 0.1Ω shunt resistor)
#define SHUNT_RESISTOR_OHMS 0.1    // Shunt resistor value for current measurements (0.1 ohms)
#define SHUNT_RESISTOR_OHMS 0.1
#define ZXCT_GAIN 50.0f




// Define a static EPSPD_Telemetry structure to store telemetry data
static EPSPD_Telemetry EPSPDTelemetryData = {
    .Bus12V = 0,     // 12V bus voltage (mV), initialized to 0
    .Bus5V = 0,      // 5V bus voltage (mV), initialized to 0
    .Bus3V3 = 0,     // 3.3V bus voltage (mV), initialized to 0
};

// Define an array of EPSPD_Parameter structures for telemetry parameters
static EPSPD_Parameter Parameters[] = {
    {EPSPD_PARAM_ID_VOLTAGE_12V, 0},   // 12V bus voltage (mV)
    {EPSPD_PARAM_ID_VOLTAGE_5V, 0},    // 5V bus voltage (mV)
    {EPSPD_PARAM_ID_VOLTAGE_3V3, 0},   // 3.3V bus voltage (mV)
    {EPSPD_PARAM_ID_CURRENT_12V, 0},   // 12V bus current (mA)
    {EPSPD_PARAM_ID_CURRENT_5V, 0},    // 5V bus current (mA)
    {EPSPD_PARAM_ID_CURRENT_3V3, 0},   // 3.3V bus current (mA)
    {EPSPD_PARAM_ID_CURRENT_SA1, 0},   // Solar array 1 current (mA)
    {EPSPD_PARAM_ID_CURRENT_SA2, 0},   // Solar array 2 current (mA)
    {EPSPD_PARAM_ID_CURRENT_SA3, 0},   // Solar array 3 current (mA)
    {EPSPD_PARAM_ID_CURRENT_XB, 0},    // XB subsystem current (mA)
    {EPSPD_PARAM_ID_CURRENT_CCU, 0},   // CCU subsystem current (mA)
    {EPSPD_PARAM_ID_CURRENT_ADCS, 0},  // ADCS subsystem current (mA)
    {EPSPD_PARAM_ID_CURRENT_GPS, 0},   // GPS subsystem current (mA)
    {EPSPD_PARAM_ID_CURRENT_PL, 0},    // Payload current (mA)
    {EPSPD_PARAM_ID_CURRENT_UHF, 0},   // UHF subsystem current (mA)
    {EPSPD_PARAM_ID_CURRENT_OBC, 0},   // OBC subsystem current (mA)
    {EPSPD_PARAM_ID_CURRENT_CCU5V, 0}, // CCU 5V current (mA)
    {EPSPD_PARAM_ID_CURRENT_ADCS5V, 0},// ADCS 5V current (mA)
    {EPSPD_PARAM_ID_CURRENT_PL5V, 0},  // Payload 5V current (mA)
    {EPSPD_PARAM_ID_CURRENT_RS5V, 0},  // RS 5V current (mA)
    {EPSPD_PARAM_ID_CURRENT_ADCS12V, 0}, // ADCS 12V current (mA)
    {EPSPD_PARAM_ID_CURRENT_XB12V, 0}, // XB 12V current (mA)
    {EPSPD_PARAM_ID_VOLTAGE_SA1, 0},   // Solar array 1 voltage (mV)
    {EPSPD_PARAM_ID_VOLTAGE_SA2, 0},   // Solar array 2 voltage (mV)
    {EPSPD_PARAM_ID_VOLTAGE_SA3, 0}    // Solar array 3 voltage (mV)
};

// Calculate the number of parameters in the array
static uint8_t ParameterCount = sizeof(Parameters) / sizeof(Parameters[0]);

// Function: SelectMultiplexerChannel
// Inputs:
//   - channel: A uint8_t, the multiplexer channel to select (0 to 7)
// Output:
//   - None (void), sets GPIO pins to select the multiplexer channel
// Significance:
//   - Configures the multiplexer (controlled by S0, S1, S2 pins) to select a specific
//     input channel for ADC measurements (e.g., solar array voltages or currents).
static void SelectMultiplexerChannel(uint8_t channel)
{
    // Set S0 pin (bit 0 of channel) to high (1) or low (0)
    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // Set S1 pin (bit 1 of channel) to high (1) or low (0)
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // Set S2 pin (bit 2 of channel) to high (1) or low (0)
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Function: EPSPD_UpdateTelemetryAndParameters
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface (e.g., hi2c2)
//   - adc_values: A pointer to an array of uint16_t, containing 16 ADC values
// Output:
//   - None (void), updates TelemetryData and Parameters, saves to EEPROM
// Significance:
//   - Processes raw ADC values to update telemetry (voltages, currents) and saves
//     them to EEPROM via I2C, critical for monitoring power distribution.
void EPSPD_UpdateTelemetryAndParameters(I2C_HandleTypeDef *hi2c, uint16_t *adc_values)
{
    // Declare variables for sync counter and subtick
    uint64_t counter;
    uint32_t subtick;
    // Get the sync counter and subtick timestamp from the BMS (via sync_counter.h)
    GetSyncTimestamp(&counter, &subtick);
    // Store subtick in telemetry (microseconds)

    // Update 12V bus voltage (mV) from ADC channel 13
    EPSPDTelemetryData.Bus12V = (uint16_t)(adc_values[13] * VOLTAGE_PER_COUNT);
    // Update 5V bus voltage (mV) from ADC channel 1
    EPSPDTelemetryData.Bus5V = (uint16_t)(adc_values[1] * VOLTAGE_PER_COUNT);
    // Update 3.3V bus voltage (mV) from ADC channel 10
    EPSPDTelemetryData.Bus3V3 = (uint16_t)(adc_values[10] * VOLTAGE_PER_COUNT);

    // Select multiplexer channel 0 for solar array 1 current
    SelectMultiplexerChannel(0);
    // Wait 1ms for multiplexer to stabilize
    SoftwareDelay(1);

    // Update solar array 1 current (mA) from ADC channel 4
    Parameters[6].Value = (uint16_t)(adc_values[4] * CURRENT_SCALING_FACTOR);

    // Select multiplexer channel 1 for solar array 3 voltage
    SelectMultiplexerChannel(1);
    // Wait 1ms for multiplexer to stabilize
    SoftwareDelay(1);
    // Update solar array 3 voltage (mV) from ADC channel 4
    Parameters[24].Value = (uint16_t)(adc_values[4] * VOLTAGE_SCALING_FACTOR);

    // Select multiplexer channel 2 for solar array 2 voltage
    SelectMultiplexerChannel(2);
    // Wait 1ms for multiplexer to stabilize
    SoftwareDelay(1);
    // Update solar array 2 voltage (mV) from ADC channel 4
    Parameters[23].Value = (uint16_t)(adc_values[4] * VOLTAGE_SCALING_FACTOR);

    // Select multiplexer channel 3 for solar array 1 voltage
    SelectMultiplexerChannel(3);
    // Wait 1ms for multiplexer to stabilize
    SoftwareDelay(1);
    // Update solar array 1 voltage (mV) from ADC channel 4
    Parameters[22].Value = (uint16_t)(adc_values[4] * VOLTAGE_SCALING_FACTOR);

    // Calculate current for RS 5V rail (mA) from ADC channel 0
    float v_imon = adc_values[0] * VOLTAGE_PER_COUNT;
    Parameters[19].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Calculate current for PL 5V rail (mA) from ADC channel 1
    v_imon = adc_values[1] * VOLTAGE_PER_COUNT;
    Parameters[18].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Calculate current for CCU 5V rail (mA) from ADC channel 2
    v_imon = adc_values[2] * VOLTAGE_PER_COUNT;
    Parameters[17].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Calculate current for GPS (mA) from ADC channel 3
    v_imon = adc_values[3] * VOLTAGE_PER_COUNT;
    Parameters[12].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Update solar array 2 current (mA) from ADC channel 5
    Parameters[7].Value = (uint16_t)(adc_values[5] * CURRENT_SCALING_FACTOR);

    // Update solar array 3 current (mA) from ADC channel 6
    Parameters[8].Value = (uint16_t)(adc_values[6] * CURRENT_SCALING_FACTOR);

    // Update XB subsystem current (mA) from ADC channel 7
    Parameters[9].Value = (uint16_t)(adc_values[7] * CURRENT_PER_COUNT);
    // Update XB 12V current (mA) from ADC channel 7
    Parameters[21].Value = (uint16_t)((adc_values[7] * VOLTAGE_PER_COUNT / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Calculate current for ADCS 5V rail (mA) from ADC channel 8
    v_imon = adc_values[8] * VOLTAGE_PER_COUNT;
    Parameters[16].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Update CCU subsystem current (mA) from ADC channel 9
    Parameters[10].Value = (uint16_t)(adc_values[9] * CURRENT_PER_COUNT);

    // Calculate current for 3.3V bus (mA) from ADC channel 10
    v_imon = adc_values[10] * VOLTAGE_PER_COUNT;
    Parameters[5].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Update ADCS subsystem current (mA) from ADC channel 11
    Parameters[11].Value = (uint16_t)(adc_values[11] * CURRENT_PER_COUNT);

    // Update payload current (mA) from ADC channel 12
    Parameters[13].Value = (uint16_t)(adc_values[12] * CURRENT_PER_COUNT);

    // Calculate current for ADCS 12V rail (mA) from ADC channel 13
    v_imon = adc_values[13] * VOLTAGE_PER_COUNT;
    Parameters[20].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Calculate current for UHF subsystem (mA) from ADC channel 14
    v_imon = adc_values[14] * VOLTAGE_PER_COUNT;
    Parameters[14].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Calculate current for OBC subsystem (mA) from ADC channel 15
    v_imon = adc_values[15] * VOLTAGE_PER_COUNT;
    Parameters[15].Value = (uint16_t)((v_imon / 1000.0) * IMON_CURRENT_PER_VOLT);

    // Copy bus voltages to parameter array for consistency
    Parameters[0].Value = EPSPDTelemetryData.Bus12V;
    Parameters[1].Value = EPSPDTelemetryData.Bus5V;
    Parameters[2].Value = EPSPDTelemetryData.Bus3V3;

    // Prepare EEPROM data structure with telemetry and timestamp
    EEPROM_TelemetryWithTimestamp eeprom_data;
    eeprom_data.telemetry = EPSPDTelemetryData;
    eeprom_data.counter = counter;
    eeprom_data.subtick_us = subtick;
    // Save telemetry to EEPROM via I2C
    epspd_WriteTelemetry(hi2c, &eeprom_data);
}

// Function: EPSPD_GetTelemetry
// Inputs:
//   - None (void)
// Output:
//   - Returns a pointer to EPSPD_Telemetry, the current telemetry data
// Significance:
//   - Provides access to the current telemetry data (bus voltages, subtick) for other
//     parts of the system to read or process.
EPSPD_Telemetry* EPSPD_GetTelemetry(void)
{
    // Return a pointer to the global TelemetryData structure
    return &EPSPDTelemetryData;
}

// Function: EPSPD_GetParameters
// Inputs:
//   - count: A pointer to a uint8_t, to store the number of parameters
// Output:
//   - Returns a pointer to the Parameters array
// Significance:
//   - Provides access to the array of telemetry parameters (voltages, currents) and
//     the number of parameters, used for monitoring or reporting.
EPSPD_Parameter* EPSPD_GetParameters(uint8_t *count)
{
    // Store the number of parameters in the provided pointer
    *count = ParameterCount;
    // Return a pointer to the Parameters array
    return Parameters;
}

