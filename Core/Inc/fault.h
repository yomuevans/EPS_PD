#ifndef __FAULT_H
#define __FAULT_H

#include "stm32l4xx_hal.h"
#include "main.h"
#include "sync_counter.h"
#include "eeprom.h"
#include "delay.h"

#define MAX_FAULT_LOGS        20                    // Max number of fault logs stored in EEPROM
#define FAULT_DESC_LEN        32                    // Max length of fault description
#define FAULT_LOG_START_ADDR  0x0300                // EEPROM start address for fault logs


// Structure stored in EEPROM for each fault log entry
typedef struct {
    uint64_t counter;                  // Sync counter timestamp
    uint32_t subtick_us;              // Microsecond-level timestamp
    uint8_t subsystem_id;             // Subsystem identifier (e.g. 0x01 for GPS)
    uint8_t retry_count;              // Retry attempts before giving up
    char description[FAULT_DESC_LEN]; // Human-readable subsystem name
} EEPROM_FaultLog;

// Optional in-memory summary log format for light-weight display/logging
typedef struct {
    uint8_t subsystem_id;
    uint8_t retry_count;
    char description[FAULT_DESC_LEN];
} EPS_FaultLog;

// Live polling snapshot for each monitored fault line
typedef struct {
    char description[FAULT_DESC_LEN]; // Subsystem name (e.g., "ADCS12V")
    uint8_t subsystem_id;             // Subsystem ID (same as EEPROM)
    bool is_active;                   // true if fault is currently asserted
    uint8_t retry_count;              // Retry counter for fault recovery
    uint32_t last_fault_time;        // Last tick (HAL_GetTick) fault was seen
} EPS_FaultPollSnapshot;

// Function declarations
void Fault_PollAndHandle(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_log);

// Log a fault to EEPROM and return the record
EEPROM_FaultLog EPS_LogFault(I2C_HandleTypeDef *hi2c, const char *desc, uint8_t subsystem_id, uint8_t retry_count);

// Read a previously stored EEPROM fault log entry by index
HAL_StatusTypeDef EPS_ReadFaultLog(I2C_HandleTypeDef *hi2c, uint8_t index, EEPROM_FaultLog *entry);

// Return the maximum number of fault logs supported
uint8_t EPS_GetFaultLogCount(void);

// Return a pointer to a static array of live fault polling results
const EPS_FaultPollSnapshot* EPS_GetAllPolledFaults(uint8_t *count);

#endif // __FAULT_H
