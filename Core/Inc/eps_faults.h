#ifndef EPS_FAULTS_H
#define EPS_FAULTS_H

#include "main.h"
#include "eeprom.h"

#define MAX_FAULT_LOGS 10
#define FAULT_LOG_SIZE (sizeof(FaultLogEntry))

typedef struct {
    uint32_t timestamp;      // System tick
    char description[32];    // Fault description
    uint8_t retry_count;     // Number of auto-retry attempts (for eFuse)
    uint8_t subsystem_id;    // Subsystem identifier (e.g., PWRL15 for UHF)
} FaultLogEntry;

void LogFault(I2C_HandleTypeDef *hi2c, const char* description, uint8_t subsystem_id);
void HandleFaults(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_log, GPIO_TypeDef *de_port, uint16_t de_pin);
void PollFaults(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_log);

#endif /* EPS_FAULTS_H */
