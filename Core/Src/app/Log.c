// File: log.c (EPS_PD Logging System)

#include "log.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "sync_counter.h"

extern UART_HandleTypeDef huart1;

#define LOG_BUFFER_SIZE 256

static const char *log_prefixes[] = {
    "",        // EPS_LOG_OFF
    "[E] ",    // EPS_LOG_ERROR
    "[W] ",    // EPS_LOG_WARNING
    "[I] ",    // EPS_LOG_INFO
    "[V] ",    // EPS_LOG_VERBOSE
    "[D] "     // EPS_LOG_DEBUG
};

static EPS_LogLevel current_log_level = EPS_LOG_INFO;

void EPS_Log_SetLevel(EPS_LogLevel level) {
    current_log_level = level;
}

void EPS_Log_Message(EPS_LogLevel level, const char *format, ...) {
    if (level > current_log_level || level == EPS_LOG_OFF) return;

    char buffer[LOG_BUFFER_SIZE];
    int offset = 0;

    uint64_t counter;
    uint32_t subtick;
    GetSyncTimestamp(&counter, &subtick);

    offset += snprintf(buffer, LOG_BUFFER_SIZE, "%s[%llu.%06lu] ",
                       log_prefixes[level], counter, (unsigned long)subtick);

    if (offset < 0 || offset >= LOG_BUFFER_SIZE) return;

    va_list args;
    va_start(args, format);
    vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, format, args);
    va_end(args);

    size_t msg_len = strnlen(buffer, LOG_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, msg_len, HAL_MAX_DELAY);
}

void EPS_Log_Telemetry(EPS_LogLevel level, const EPSPD_Telemetry *telemetry) {
    if (level > current_log_level || level == EPS_LOG_OFF) return;

    EPS_Log_Message(level, "Telemetry - 12V: %umV, 5V: %umV, 3.3V: %umV, Subtick: %luus\n",
                    telemetry->Bus12V, telemetry->Bus5V, telemetry->Bus3V3);
}

void EPS_Log_ParameterDump(EPS_LogLevel level, const EPSPD_Parameter *params, uint8_t count) {
    if (level > current_log_level || level == EPS_LOG_OFF) return;

    for (uint8_t i = 0; i < count; ++i) {
    	EPS_Log_Message(level, "Param ID: 0x%02X, Value: %u\n", params[i].ParamId, params[i].Value);

    }
}
