// Include header files needed for the code to work
#include "satellite_modes.h" // Defines SatelliteMode_t, PowerLineID_t, and function prototypes
#include "main.h"           // Includes GPIO pin definitions (e.g., GPS_EN_Pin) and hardware handles
#include <string.h>         // Standard C library for memory operations (e.g., memcpy, not used here but included)
// Include additional headers for BMS power line control
#include "i2c_comm.h"     // Defines EPS_I2C_SendCommand for BMS communication
#include "sync_counter.h" // Defines GetSyncCounter for timestamping


// Declare external I2C handle for communicating with the BMS
extern I2C_HandleTypeDef hi2c2; // Assuming EPS_BMS is connected via I2C2

// Define a static structure to store the satellite’s mode state
static SatelliteModeState_t satellite_mode_state = {
    .current_mode = SAT_MODE_INITIALIZATION, // Start in Initialization mode
    .mode_start_time = 0,                   // Time when mode started (ms)
    .mode_duration = SAT_MODE_INIT_DURATION * 1000 // Duration of Initialization mode (ms)
};

// Function: SatelliteModes_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes the satellite mode state
// Significance:
//   - Sets up the satellite’s initial mode (Initialization) and configures power lines accordingly,
//     critical for starting the EPS in a known state during satellite boot-up.
// Function:
void SatelliteModes_Init(void) {
    // Set the current mode to Initialization
    satellite_mode_state.current_mode = SAT_MODE_INITIALIZATION;
    // Record the current system tick (milliseconds) as the mode start time
    satellite_mode_state.mode_start_time = HAL_GetTick();
    // Set the mode duration (convert seconds to milliseconds)
    satellite_mode_state.mode_duration = SAT_MODE_INIT_DURATION * 1000;
    // Configure power lines for Initialization mode
    SatelliteModes_ConfigurePowerLines(SAT_MODE_INITIALIZATION);
}

// Function: SatelliteModes_SetMode
// Inputs:
//   - mode: A SatelliteMode_t, the desired satellite mode (e.g., SAT_MODE_NORMAL)
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if mode is set successfully, HAL_ERROR if invalid
// Significance:
//   - Changes the satellite’s operating mode and configures power lines, ensuring the correct
//     subsystems are powered based on the mode (e.g., Normal, Emergency). Called by ssp.c for SSP_CMD_SM.
// Function:
HAL_StatusTypeDef SatelliteModes_SetMode(SatelliteMode_t mode) {
    // Declare a variable to store the mode duration in milliseconds
    uint32_t duration_ms = 0;
    // Determine the duration based on the mode
    switch (mode) {
        // Initialization mode
        case SAT_MODE_INITIALIZATION:
            // Set duration (convert seconds to milliseconds)
            duration_ms = SAT_MODE_INIT_DURATION * 1000;
            break;
        // Detumbling mode (stabilizing satellite orientation)
        case SAT_MODE_DETUMBLING:
            duration_ms = SAT_MODE_DETUMBLING_DURATION * 1000;
            break;
        // Normal operation mode
        case SAT_MODE_NORMAL:
            duration_ms = SAT_MODE_NORMAL_DURATION * 1000;
            break;
        // Communication mode (e.g., for UHF)
        case SAT_MODE_COMMUNICATION:
            duration_ms = SAT_MODE_COMM_DURATION * 1000;
            break;
        // Payload mode (e.g., for imaging)
        case SAT_MODE_PAYLOAD:
            duration_ms = SAT_MODE_PAYLOAD_PRE_DURATION * 1000; // Default to Pre-Image Download
            break;
        // Download mode (e.g., for image download)
        case SAT_MODE_DOWNLOAD:
            duration_ms = SAT_MODE_DOWNLOAD_PRE_DURATION * 1000; // Default to Pre-Image Download
            break;
        // Emergency mode (minimal power usage)
        case SAT_MODE_EMERGENCY:
            duration_ms = SAT_MODE_EMERGENCY_PRE_DURATION * 1000; // Default to Pre-Image Download
            break;
        // Safe mode (TBD in ICD)
        case SAT_MODE_SAFE:
            duration_ms = SAT_MODE_SAFE_DURATION * 1000; // TBD
            break;
        // Backup mode (similar to Emergency)
        case SAT_MODE_BACKUP:
            duration_ms = SAT_MODE_BACKUP_DURATION * 1000; // Assumed same as Emergency
            break;
        // Invalid mode
        default:
            return HAL_ERROR; // Return error for unknown mode
    }

    // Update the current mode
    satellite_mode_state.current_mode = mode;
    // Record the current system tick as the mode start time
    satellite_mode_state.mode_start_time = HAL_GetTick();
    // Set the mode duration
    satellite_mode_state.mode_duration = duration_ms;
    // Configure power lines for the new mode
    return SatelliteModes_ConfigurePowerLines(mode);
}

// Function: SatelliteModes_GetMode
// Inputs:
//   - None (void)
// Output:
//   - Returns SatelliteMode_t, the current satellite mode
// Significance:
//   - Retrieves the current operating mode, used by ssp.c for SSP_CMD_GM to report the mode to other subsystems.
// Function:
SatelliteMode_t SatelliteModes_GetMode(void) {
    // Return the current mode from the state structure
    return satellite_mode_state.current_mode;
}

// Function: SatelliteModes_GetModeString
// Inputs:
//   - mode: A SatelliteMode_t, the mode to get the string for
// Output:
//   - Returns a const char*, the string name of the mode
// Significance:
//   - Converts a mode enum to a human-readable string, useful for debugging or logging the satellite’s state.
// Function:
const char* SatelliteModes_GetModeString(SatelliteMode_t mode) {
    // Return the string name based on the mode
    switch (mode) {
        case SAT_MODE_INITIALIZATION: return "Initialization"; // Initialization mode
        case SAT_MODE_DETUMBLING:    return "Detumbling";     // Detumbling mode
        case SAT_MODE_NORMAL:        return "Normal";         // Normal operation mode
        case SAT_MODE_COMMUNICATION: return "Communication";  // Communication mode
        case SAT_MODE_PAYLOAD:       return "Payload";        // Payload mode
        case SAT_MODE_DOWNLOAD:      return "Download";       // Download mode
        case SAT_MODE_SAFE:          return "Safe";           // Safe mode
        case SAT_MODE_EMERGENCY:     return "Emergency";      // Emergency mode
        case SAT_MODE_BACKUP:        return "Backup";         // Backup mode
        default:                     return "Unknown";        // Unknown mode
    }
}

// Function: SatelliteModes_ConfigurePowerLines
// Inputs:
//   - mode: A SatelliteMode_t, the mode to configure power lines for
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if configuration succeeds, HAL_ERROR if invalid mode
// Significance:
//   - Configures GPIO pins to enable/disable power lines (e.g., 5V GPS, 12V ADCS) based on the satellite mode,
//     ensuring only necessary subsystems are powered (ref. ICD pages 26-51).
// Function:
HAL_StatusTypeDef SatelliteModes_ConfigurePowerLines(SatelliteMode_t mode) {
    // Reset all switchable power lines to off (GPIO_PIN_RESET = low)
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);       // PWRL1 (5V GPS)
    HAL_GPIO_WritePin(RS12V_EN_GPIO_Port, RS12V_EN_Pin, GPIO_PIN_RESET);   // PWRL2 (12V Reserved)
    HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_RESET);     // PWRL3 (3.3V ADCS)
    HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_RESET);     // PWRL4 (5V ADCS, assumed separate)
    HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_RESET); // PWRL5 (ADCS VBAT 12V)
    HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, GPIO_PIN_RESET);         // PWRL6 (3.3V PL)
    HAL_GPIO_WritePin(PL5V_EN_GPIO_Port, PL5V_EN_Pin, GPIO_PIN_RESET);     // PWRL7 (5V PL)
    HAL_GPIO_WritePin(XB12V_EN_GPIO_Port, XB12V_EN_Pin, GPIO_PIN_RESET);   // PWRL8 (12V XB)
    HAL_GPIO_WritePin(RS5V_EN_GPIO_Port, RS5V_EN_Pin, GPIO_PIN_RESET);     // PWRL9 (5V Reserved)
    HAL_GPIO_WritePin(RS3V3_EN_GPIO_Port, RS3V3_EN_Pin, GPIO_PIN_RESET);   // PWRL10 (3.3V Reserved)
    HAL_GPIO_WritePin(CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, GPIO_PIN_RESET);   // PWRL13 (5V CCU)
    HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, GPIO_PIN_RESET);       // PWRL15 (3.3V UHF)

    // Note: PWRL0 (3.3V OBC) and PWRL14 (3.3V CCU) are non-switchable (E_RST0, E_RST1)

    // Configure power lines based on the mode (per ICD pages 26-51)
    switch (mode) {
        case SAT_MODE_INITIALIZATION:
            // Active: OBC (PWRL0, non-switchable), CCU (PWRL13, 5V, 1A, 5W), EPS (3.3V/5V, 0.1A, 0.83W)
            // Enable 5V CCU power line
            HAL_GPIO_WritePin(CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, GPIO_PIN_SET);  // PWRL13
            break;

        case SAT_MODE_DETUMBLING:
            // Active: OBC_beacon (PWRL0, non-switchable), CCU_beacon (PWRL14, non-switchable),
            // EPS (3.3V/5V, 0.1A, 0.83W), GPS (PWRL1, 5V, 0.6A, 3W),
            // ADCS_Detumbling (PWRL3/PWRL4/PWRL5, 3.3V/5V/12V, 0.21A, 1.8W)
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate, same pin as PWRL3)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            break;

        case SAT_MODE_NORMAL:
            // Active: OBC_beacon (PWRL0, non-switchable), CCU_beacon (PWRL14, non-switchable),
            // EPS (3.3V/5V, 0.1A, 0.83W), GPS (PWRL1, 5V, 0.6A, 3W),
            // ADCS_SunPointing (PWRL3/PWRL4/PWRL5, 3.3V/5V/12V, 0.29A, 2.9W)
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            // Enable 3.3V UHF power line
            HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, GPIO_PIN_SET);       // PWRL15 (3.3V UHF)
            break;

        case SAT_MODE_COMMUNICATION:
            // Active: OBC_UHF (PWRL0, non-switchable), CCU (PWRL14, non-switchable),
            // EPS (3.3V/5V, 0.1A, 0.83W), GPS (PWRL1, 5V, 0.6A, 3W),
            // ADCS_SunPointing (PWRL3/PWRL4/PWRL5, 3.3V/5V/12V, 0.29A, 2.9W)
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            // Enable 3.3V UHF power line
            HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, GPIO_PIN_SET);       // PWRL15 (3.3V UHF)
            break;

        case SAT_MODE_PAYLOAD:
            // Handle sub-modes: Pre-Image Download (15*60s), Imaging with Nadir (25s), Discharge (15*60s)
            // Pre-Image Download: OBC, CCU, EPS, GPS, ADCS_Nadir
            // Imaging with Nadir: Adds PL Control Board, PL Camera Handler, 3 Cameras
            // Discharge: Same as Pre-Image Download
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            // Enable 3.3V Payload power line
            HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, GPIO_PIN_SET);         // PWRL6 (3.3V PL)
            // Enable 5V Payload power line
            HAL_GPIO_WritePin(PL5V_EN_GPIO_Port, PL5V_EN_Pin, GPIO_PIN_SET);     // PWRL7 (5V PL)
            break;

        case SAT_MODE_DOWNLOAD:
            // Handle sub-modes: Pre-Image Download (15*60s), Image Download (4*60s), Discharge (15*60s)
            // Pre-Image Download: OBC, CCU, EPS, GPS, ADCS_Ground_Tracking
            // Image Download: Adds PL Control Board, X-Band
            // Discharge: Same as Pre-Image Download
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            // Enable 12V X-Band power line
            HAL_GPIO_WritePin(XB12V_EN_GPIO_Port, XB12V_EN_Pin, GPIO_PIN_SET);   // PWRL8 (12V XB)
            // Enable 3.3V Payload power line
            HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, GPIO_PIN_SET);         // PWRL6 (3.3V PL)
            break;

        case SAT_MODE_EMERGENCY:
            // Handle sub-modes: Pre-Image Download (15*60s), Imaging during Emergency (15*60s), Discharge (15*60s)
            // Pre-Image Download: CCU, EPS, GPS, ADCS_Nadir
            // Imaging: Adds higher CCU power
            // Discharge: Same as Pre-Image Download
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            break;

        case SAT_MODE_SAFE:
            // TBD in ICD, assume minimal configuration (non-switchable PWRL0, PWRL14)
            // No power lines enabled (minimal mode)
            break;

        case SAT_MODE_BACKUP:
            // Assume same as Emergency for now (non-switchable PWRL0, PWRL14)
            // Enable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);       // PWRL1 (5V GPS)
            // Enable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL3 (3.3V ADCS)
            // Enable 5V ADCS power line (assumed separate)
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_SET);     // PWRL4 (5V ADCS, assumed separate)
            // Enable 12V ADCS power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, GPIO_PIN_SET); // PWRL5 (12V ADCS VBAT)
            break;

        default:
            // Return error for invalid mode
            return HAL_ERROR;
    }
    // Return success if mode configuration is valid
    return HAL_OK;
}



// Function: SatelliteModes_SwitchPowerLine
// Inputs:
//   - pwrl_id: A PowerLineID_t, the ID of the power line to switch (e.g., PWRL_ID_1 for GPS)
//   - state: A GPIO_PinState, GPIO_PIN_SET (on) or GPIO_PIN_RESET (off)
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if switch succeeds, HAL_ERROR if invalid or fails
// Significance:
//   - Switches specific power lines on or off via GPIO or I2C commands to the BMS (for heaters,
//     charge/discharge control), critical for controlling subsystem power. Called by ssp.c for SSP_CMD_SON/SOF.
// Function:
HAL_StatusTypeDef SatelliteModes_SwitchPowerLine(PowerLineID_t pwrl_id, GPIO_PinState state) {

    // Switch based on the power line ID
    switch (pwrl_id) {
        // EPS_PD-controlled GPIO power lines
        case PWRL_ID_1:
            // Enable/disable 5V GPS power line
            HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, state); break;
        case PWRL_ID_2:
            // Enable/disable 12V Reserved power line
            HAL_GPIO_WritePin(RS12V_EN_GPIO_Port, RS12V_EN_Pin, state); break;
        case PWRL_ID_3:
            // Enable/disable 3.3V ADCS power line
            HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, state); break;
        case PWRL_ID_4:
            // Enable/disable 5V ADCS power line
            HAL_GPIO_WritePin(ADCS5V_EN_GPIO_Port, ADCS5V_EN_Pin, state); break;
        case PWRL_ID_5:
            // Enable/disable 12V ADCS VBAT power line
            HAL_GPIO_WritePin(ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, state); break;
        case PWRL_ID_6:
            // Enable/disable 3.3V Payload power line
            HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, state); break;
        case PWRL_ID_7:
            // Enable/disable 5V Payload power line
            HAL_GPIO_WritePin(PL5V_EN_GPIO_Port, PL5V_EN_Pin, state); break;
        case PWRL_ID_8:
            // Enable/disable 12V X-Band power line
            HAL_GPIO_WritePin(XB12V_EN_GPIO_Port, XB12V_EN_Pin, state); break;
        case PWRL_ID_9:
            // Enable/disable 5V Reserved power line
            HAL_GPIO_WritePin(RS5V_EN_GPIO_Port, RS5V_EN_Pin, state); break;
        case PWRL_ID_10:
            // Enable/disable 3.3V Reserved power line
            HAL_GPIO_WritePin(RS3V3_EN_GPIO_Port, RS3V3_EN_Pin, state); break;
        case PWRL_ID_13:
            // Enable/disable 5V CCU power line
            HAL_GPIO_WritePin(CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, state); break;
        case PWRL_ID_15:
            // Enable/disable 3.3V UHF power line
            HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, state); break;

        // Reserved or invalid power line IDs
        case PWRL_ID_0:
        case PWRL_ID_11:
        case PWRL_ID_12:
        case PWRL_ID_14:
        default:
            // Return error for invalid or reserved IDs
            return HAL_ERROR;
    }

    // Return success for valid GPIO-controlled power lines
    return HAL_OK;
}
