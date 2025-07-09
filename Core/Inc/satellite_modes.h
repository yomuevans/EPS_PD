#ifndef __SATELLITE_MODES_H
#define __SATELLITE_MODES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "i2c_comm.h"
#include "sync_counter.h"

#define PWRL_ID_HEATER_1      0x20
#define PWRL_ID_HEATER_2      0x21
#define PWRL_ID_CHARGE_CTRL   0x22
#define PWRL_ID_DISCHG_CTRL   0x23

// Satellite Modes of Operation Identifiers (AFDEVSAT ICD Table 22)
typedef enum {
    SAT_MODE_INITIALIZATION = 0x01,
    SAT_MODE_DETUMBLING     = 0x02,
    SAT_MODE_NORMAL         = 0x03,
    SAT_MODE_COMMUNICATION  = 0x04,
    SAT_MODE_PAYLOAD        = 0x05,
    SAT_MODE_DOWNLOAD       = 0x06,
    SAT_MODE_SAFE           = 0x07,
    SAT_MODE_EMERGENCY      = 0x08,
    SAT_MODE_BACKUP         = 0x09
} SatelliteMode_t;

// Power Line Identifiers (AFDEVSAT ICD Table 23 with updated mappings)
typedef enum {
    PWRL_ID_0  = 0x00, // 3.3V OBC, E_RST0 (non-switchable, EPS not responsible)
    PWRL_ID_1  = 0x01, // 5V GPS, GPS_EN
    PWRL_ID_2  = 0x02, // 12V Reserved, RS12V_EN
    PWRL_ID_3  = 0x03, // 3.3V ADCS, ADCS_EN
    PWRL_ID_4  = 0x04, // 5V ADCS, ADCS_EN (assumed separate control)
    PWRL_ID_5  = 0x05, // ADCS VBAT (12V), ADCS12_EN
    PWRL_ID_6  = 0x06, // 3.3V PL, PL_EN
    PWRL_ID_7  = 0x07, // 5V PL, PL5V_EN
    PWRL_ID_8  = 0x08, // 12V XB (X-Band), XB12V_EN
    PWRL_ID_9  = 0x09, // 5V Reserved, RS5V_EN
    PWRL_ID_10 = 0x0A, // 3.3V Reserved, RS3V3_EN
    PWRL_ID_11 = 0x0B, // Reserved (no pin specified)
    PWRL_ID_12 = 0x0C, // Not Connected
    PWRL_ID_13 = 0x0D, // 5V CCU, CCU5V_EN
    PWRL_ID_14 = 0x0E, // 3.3V CCU, E_RST1 (non-switchable, EPS not responsible)
    PWRL_ID_15 = 0x0F,  // 3.3V UHF, UHF_EN
	PWRL_ID_20 = 0x20, // Heater 1
	PWRL_ID_21 = 0x21, // Heater 2
	PWRL_ID_22 = 0x22, // Battery Charging Control
	PWRL_ID_23 = 0x23  // Battery Discharging Control
} PowerLineID_t;

// Mode Durations (seconds, from ICD pages 26-51)
#define SAT_MODE_INIT_DURATION      (6 * 60)
#define SAT_MODE_DETUMBLING_DURATION (270 * 60)
#define SAT_MODE_NORMAL_DURATION    (91.5 * 60)
#define SAT_MODE_COMM_DURATION      (7 * 60)
#define SAT_MODE_PAYLOAD_PRE_DURATION (15 * 60) // Pre-Image Download
#define SAT_MODE_PAYLOAD_IMAGING_DURATION (25) // Imaging with Nadir
#define SAT_MODE_PAYLOAD_DISCHARGE_DURATION (15 * 60) // Discharge
#define SAT_MODE_PAYLOAD_COMM_PRE_DURATION (15 * 60) // Pre-Image during Comm
#define SAT_MODE_PAYLOAD_COMM_IMAGING_DURATION (10) // Imaging with Nadir during Comm
#define SAT_MODE_PAYLOAD_COMM_DISCHARGE_DURATION (15 * 60) // Discharge during Comm
#define SAT_MODE_DOWNLOAD_PRE_DURATION (15 * 60) // Pre-Image Download
#define SAT_MODE_DOWNLOAD_IMAGING_DURATION (4 * 60) // Image Download
#define SAT_MODE_DOWNLOAD_DISCHARGE_DURATION (15 * 60) // Discharge
#define SAT_MODE_EMERGENCY_PRE_DURATION (15 * 60) // Pre-Image Download
#define SAT_MODE_EMERGENCY_IMAGING_DURATION (15 * 60) // Imaging during Emergency
#define SAT_MODE_EMERGENCY_DISCHARGE_DURATION (15 * 60) // Discharge
#define SAT_MODE_SAFE_DURATION      0 // TBD in ICD
#define SAT_MODE_BACKUP_DURATION    0 // Assumed same as Emergency

// Satellite Mode State Structure
typedef struct {
    SatelliteMode_t current_mode; // Current operational mode
    uint32_t mode_start_time;     // Timestamp when mode was set (ms)
    uint32_t mode_duration;       // Expected duration of mode (ms)
    // uint32_t current_power;     // Current power consumption (W), requires ADC integration
} SatelliteModeState_t;

// Function Prototypes
void SatelliteModes_Init(void);
HAL_StatusTypeDef SatelliteModes_SetMode(SatelliteMode_t mode);
SatelliteMode_t SatelliteModes_GetMode(void);
const char* SatelliteModes_GetModeString(SatelliteMode_t mode);
HAL_StatusTypeDef SatelliteModes_ConfigurePowerLines(SatelliteMode_t mode);
HAL_StatusTypeDef SatelliteModes_SwitchPowerLine(PowerLineID_t pwrl_id, GPIO_PinState state);

#ifdef __cplusplus
}
#endif

#endif /* __SATELLITE_MODES_H */
