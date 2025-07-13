#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define EPSPD_PARAM_ID_VOLTAGE_12V    0x01
#define EPSPD_PARAM_ID_VOLTAGE_5V     0x02
#define EPSPD_PARAM_ID_VOLTAGE_3V3    0x03
#define EPSPD_PARAM_ID_CURRENT_12V    0x04
#define EPSPD_PARAM_ID_CURRENT_5V     0x05
#define EPSPD_PARAM_ID_CURRENT_3V3    0x06
#define EPSPD_PARAM_ID_CURRENT_SA1    0x07
#define EPSPD_PARAM_ID_CURRENT_SA2    0x08
#define EPSPD_PARAM_ID_CURRENT_SA3    0x09
#define EPSPD_PARAM_ID_CURRENT_XB     0x0A
#define EPSPD_PARAM_ID_CURRENT_CCU    0x0B
#define EPSPD_PARAM_ID_CURRENT_ADCS   0x0C
#define EPSPD_PARAM_ID_CURRENT_GPS    0x0D
#define EPSPD_PARAM_ID_CURRENT_PL     0x0E
#define EPSPD_PARAM_ID_CURRENT_UHF    0x0F
#define EPSPD_PARAM_ID_CURRENT_OBC    0x10
#define EPSPD_PARAM_ID_CURRENT_CCU5V  0x11
#define EPSPD_PARAM_ID_CURRENT_ADCS5V 0x12
#define EPSPD_PARAM_ID_CURRENT_PL5V   0x13
#define EPSPD_PARAM_ID_CURRENT_RS5V   0x14
#define EPSPD_PARAM_ID_CURRENT_ADCS12V 0x15
#define EPSPD_PARAM_ID_CURRENT_XB12V  0x16
#define EPSPD_PARAM_ID_VOLTAGE_SA1    0x17
#define EPSPD_PARAM_ID_VOLTAGE_SA2    0x18
#define EPSPD_PARAM_ID_VOLTAGE_SA3    0x19

typedef struct {
    uint16_t Bus12V;
    uint16_t Bus5V;
    uint16_t Bus3V3;
    //uint32_t subtick_us; // Added microseconds since last sync pulse
} EPSPD_Telemetry;

typedef struct {
    EPSPD_Telemetry telemetry;
    uint64_t counter;      // Sync counter
    uint32_t subtick_us;   // Microseconds since last sync pulse (duplicate for EEPROM storage)
} EEPROM_TelemetryWithTimestamp;

typedef struct {
    uint8_t ParamId;
    uint16_t Value;
} EPSPD_Parameter;

void EPSPD_UpdateTelemetryAndParameters(I2C_HandleTypeDef *hi2c, uint16_t *adc_values);
EPSPD_Telemetry* EPSPD_GetTelemetry(void);
EPSPD_Parameter* EPSPD_GetParameters(uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H */
