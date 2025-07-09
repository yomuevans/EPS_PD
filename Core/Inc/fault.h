#ifndef __FAULT_H
#define __FAULT_H

#include "stm32l4xx_hal.h"
#include "main.h"
#include "fault.h"
#include "sync_counter.h"
#include "eeprom.h"
#include "delay.h"

#define MAX_FAULT_LOGS        20
#define FAULT_DESC_LEN        32
#define FAULT_LOG_START_ADDR  0x0300

typedef struct {
    uint64_t counter;
    uint32_t subtick_us;
    uint8_t subsystem_id;
    uint8_t retry_count;
    char description[FAULT_DESC_LEN];
} EEPROM_FaultLog;

void Fault_PollAndHandle(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_log);
HAL_StatusTypeDef EPS_ReadFaultLog(I2C_HandleTypeDef *hi2c, uint8_t index, EEPROM_FaultLog *entry);
uint8_t EPS_GetFaultLogCount(void);

#endif
