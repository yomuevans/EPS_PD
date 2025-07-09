#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "fault.h"

#define EEPROM_ADDR (epspd_I2C_ADDR_MEMORY << 1)
#define FAULT_LOG_SIZE sizeof(EEPROM_FaultLog)

typedef struct {
    GPIO_TypeDef* fault_port;
    uint16_t fault_pin;
    GPIO_TypeDef* en_port;
    uint16_t en_pin;
    const char* description;
    uint8_t subsystem_id;
} FaultConfig;

static FaultConfig fault_config[] = {
    {ADCS12V_FLT_GPIO_Port, ADCS12V_FLT_Pin, ADCS12_EN_GPIO_Port, ADCS12_EN_Pin, "ADCS12V", 0x0D},
    {ADCS5V_FLT_GPIO_Port,  ADCS5V_FLT_Pin,  ADCS5V_EN_GPIO_Port, ADCS5V_EN_Pin, "ADCS5V",  0x03},
    {CCU_FAULT_GPIO_Port,   CCU_FAULT_Pin,   CCU5V_EN_GPIO_Port,  CCU5V_EN_Pin,  "CCU",     0x02},
    {CCU5V_FLT_GPIO_Port,   CCU5V_FLT_Pin,   CCU5V_EN_GPIO_Port,  CCU5V_EN_Pin,  "CCU5V",   0x12},
    {GPS_FLT_GPIO_Port,     GPS_FLT_Pin,     GPS_EN_GPIO_Port,    GPS_EN_Pin,    "GPS",     0x01},
    {OBC_FAULT_GPIO_Port,   OBC_FAULT_Pin,   NULL,                0,             "OBC",     0x06},
    {PL_FLT_GPIO_Port,      PL_FLT_Pin,      PL_EN_GPIO_Port,     PL_EN_Pin,     "PL",      0x0A},
    {PL5V_FLT_GPIO_Port,    PL5V_FLT_Pin,    PL5V_EN_GPIO_Port,   PL5V_EN_Pin,   "PL5V",    0x04},
    {RS12V_FLT_GPIO_Port,   RS12V_FLT_Pin,   RS12V_EN_GPIO_Port,  RS12V_EN_Pin,  "RS12V",   0x08},
    {RS3V3_FLT_GPIO_Port,   RS3V3_FLT_Pin,   RS3V3_EN_GPIO_Port,  RS3V3_EN_Pin,  "RS3V3",   0x09},
    {RS5V_FLT_GPIO_Port,    RS5V_FLT_Pin,    RS5V_EN_GPIO_Port,   RS5V_EN_Pin,   "RS5V",    0x05},
    {UHF_FLT_GPIO_Port,     UHF_FLT_Pin,     UHF_EN_GPIO_Port,    UHF_EN_Pin,    "UHF",     0x0F},
    {XB12V_FLT_GPIO_Port,   XB12V_FLT_Pin,   XB12V_EN_GPIO_Port,  XB12V_EN_Pin,  "XB12V",   0x07},
};

typedef struct {
    bool is_active;
    uint8_t retry_count;
    uint32_t last_fault_time;
} FaultState;

static FaultState fault_states[sizeof(fault_config) / sizeof(FaultConfig)];
static uint8_t fault_log_index = 0;

void EPS_LogFault(I2C_HandleTypeDef *hi2c, const char *desc, uint8_t subsystem_id, uint8_t retry_count) {
    EEPROM_FaultLog log;
    memset(&log, 0, sizeof(log));
    log.subsystem_id = subsystem_id;
    log.retry_count = retry_count;
    strncpy(log.description, desc, FAULT_DESC_LEN - 1);
    GetSyncTimestamp(&log.counter, &log.subtick_us);

    uint16_t eeprom_addr = FAULT_LOG_START_ADDR + (fault_log_index * FAULT_LOG_SIZE);
    uint8_t buffer[2 + FAULT_LOG_SIZE];
    buffer[0] = eeprom_addr >> 8;
    buffer[1] = eeprom_addr & 0xFF;
    memcpy(&buffer[2], &log, FAULT_LOG_SIZE);

    if (HAL_I2C_Master_Transmit(hi2c, EEPROM_ADDR, buffer, sizeof(buffer), 100) == HAL_OK) {
        SoftwareDelay(4);
        fault_log_index = (fault_log_index + 1) % MAX_FAULT_LOGS;
    }
}

void Fault_PollAndHandle(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_log) {
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < sizeof(fault_config)/sizeof(FaultConfig); ++i) {
        GPIO_PinState state = HAL_GPIO_ReadPin(fault_config[i].fault_port, fault_config[i].fault_pin);

        if (state == GPIO_PIN_RESET && !fault_states[i].is_active) {
            // Fault detected
            fault_states[i].is_active = true;
            fault_states[i].retry_count = 0;
            fault_states[i].last_fault_time = now;
            EPS_LogFault(hi2c, fault_config[i].description, fault_config[i].subsystem_id, 0);

            char msg[64];
            snprintf(msg, sizeof(msg), "%s Fault Detected\n", fault_config[i].description);
            HAL_UART_Transmit(huart_log, (uint8_t*)msg, strlen(msg), 100);

        } else if (state == GPIO_PIN_SET && fault_states[i].is_active) {
            if (now - fault_states[i].last_fault_time > 5) {
                fault_states[i].is_active = false;

                char msg[64];
                snprintf(msg, sizeof(msg), "%s Fault Cleared\n", fault_config[i].description);
                HAL_UART_Transmit(huart_log, (uint8_t*)msg, strlen(msg), 100);
            }

        } else if (fault_states[i].is_active && fault_states[i].retry_count < 5 && fault_config[i].en_port) {
            HAL_GPIO_WritePin(fault_config[i].en_port, fault_config[i].en_pin, GPIO_PIN_RESET);
            SoftwareDelay(100);
            HAL_GPIO_WritePin(fault_config[i].en_port, fault_config[i].en_pin, GPIO_PIN_SET);
            fault_states[i].retry_count++;
            fault_states[i].last_fault_time = now;

        } else if (fault_states[i].is_active && fault_states[i].retry_count >= 5 && fault_config[i].en_port) {
            HAL_GPIO_WritePin(fault_config[i].en_port, fault_config[i].en_pin, GPIO_PIN_RESET);
            fault_states[i].is_active = false;

            char msg[64];
            snprintf(msg, sizeof(msg), "%s Fault Persistent: Disabled\n", fault_config[i].description);
            HAL_UART_Transmit(huart_log, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}

HAL_StatusTypeDef EPS_ReadFaultLog(I2C_HandleTypeDef *hi2c, uint8_t index, EEPROM_FaultLog *entry) {
    if (index >= MAX_FAULT_LOGS) return HAL_ERROR;

    uint16_t eeprom_addr = FAULT_LOG_START_ADDR + (index * FAULT_LOG_SIZE);
    uint8_t addr[2] = { eeprom_addr >> 8, eeprom_addr & 0xFF };

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_ADDR, addr, 2, 100);
    if (status != HAL_OK) return status;

    return HAL_I2C_Master_Receive(hi2c, EEPROM_ADDR, (uint8_t*)entry, FAULT_LOG_SIZE, 100);
}

uint8_t EPS_GetFaultLogCount(void) {
    return MAX_FAULT_LOGS;
}
