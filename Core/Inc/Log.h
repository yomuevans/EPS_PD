/*
 * log.h - Logging interface for EPS Power Distribution (EPS_PD) project
 *
 * Provides categorized logging with timestamp support, USART1 output,
 * telemetry/parameter dump functions, and configurable verbosity levels.
 */

#ifndef EPSPD_LOG_H
#define EPSPD_LOG_H

#include "main.h"
#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include "telemetry.h"
#include "sync_counter.h"

#ifdef __cplusplus
extern "C" {
#endif

// Logging levels
typedef enum {
    EPS_LOG_OFF = 0,
    EPS_LOG_ERROR,
    EPS_LOG_WARNING,
    EPS_LOG_INFO,
    EPS_LOG_VERBOSE,
    EPS_LOG_DEBUG
} EPS_LogLevel;

// Set current logging level (e.g., EPS_LOG_INFO)
void EPS_Log_SetLevel(EPS_LogLevel level);

// Core logging function
void EPS_Log_Message(EPS_LogLevel level, const char *format, ...);

// Log telemetry structure values
void EPS_Log_Telemetry(EPS_LogLevel level, const EPSPD_Telemetry *telemetry);

// Log all EPSPD_Parameters
void EPS_Log_ParameterDump(EPS_LogLevel level, const EPSPD_Parameter *params, uint8_t count);

// Optional future expansion: save logs to EEPROM or circular RAM buffer
// void EPS_Log_SaveToEEPROM();
// void EPS_Log_Flush();

#ifdef __cplusplus
}
#endif

#endif // EPSPD_LOG_H
