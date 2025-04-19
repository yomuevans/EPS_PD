#include "eps_faults.h"
#include "ssp_common.h"
#include "ssp_comm.h"
#include <string.h>

FaultLogEntry fault_log[MAX_FAULT_LOGS];
uint8_t fault_log_index = 0;
volatile bool fault_detected = false;

// Fault states for polling and retry tracking
typedef struct {
    bool is_active;
    uint8_t retry_count;
    uint32_t last_fault_time;
} FaultState;

FaultState fault_states[] = {
    {false, 0, 0}, // ADCS_FAULT
    {false, 0, 0}, // ADCS12V_FLT
    {false, 0, 0}, // ADCS5V_FLT
    {false, 0, 0}, // CCU_FAULT (polling)
    {false, 0, 0}, // CCU5V_FLT
    {false, 0, 0}, // GPS_FLT
    {false, 0, 0}, // OBC_FAULT
    {false, 0, 0}, // PL_FLT
    {false, 0, 0}, // PL5V_FLT (polling)
    {false, 0, 0}, // RS12V_FLT (polling)
    {false, 0, 0}, // RS3V3_FLT
    {false, 0, 0}, // RS5V_FLT
    {false, 0, 0}, // UHF_FLT
    {false, 0, 0}  // XB12V_FLT (polling)
};

void LogFault(I2C_HandleTypeDef *hi2c, const char* description, uint8_t subsystem_id) {
    if (fault_log_index < MAX_FAULT_LOGS) {
        FaultLogEntry entry = {
            .timestamp = HAL_GetTick(),
            .retry_count = 0, // Will be updated in HandleFaults
            .subsystem_id = subsystem_id
        };
        strncpy(entry.description, description, sizeof(entry.description) - 1);
        entry.description[sizeof(entry.description) - 1] = '\0';
        fault_log[fault_log_index] = entry;

        // Write to EEPROM at address = fault_log_index * FAULT_LOG_SIZE + 0x0100 (after telemetry/parameters)
        uint16_t eeprom_addr = 0x0100 + (fault_log_index * FAULT_LOG_SIZE);
        uint8_t buffer[2 + FAULT_LOG_SIZE];
        buffer[0] = eeprom_addr >> 8;
        buffer[1] = eeprom_addr & 0xFF;
        memcpy(&buffer[2], &entry, FAULT_LOG_SIZE);
        HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR_MEMORY << 1, buffer, 2 + FAULT_LOG_SIZE, 100);
        HAL_Delay(4); // Write time

        fault_log_index++;
        if (fault_log_index >= MAX_FAULT_LOGS) fault_log_index = 0; // Circular buffer
    }
}

void HandleFaults(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_log, GPIO_TypeDef *de_port, uint16_t de_pin) {
    if (!fault_detected) return;
    fault_detected = false;

    SSP_Frame tx_frame;

    // Helper macro to handle fault
    #define HANDLE_FAULT(index, pin_port, pin, en_port, en_pin, desc, subsystem_id) \
    if (fault_states[index].is_active) { \
        if (HAL_GPIO_ReadPin(pin_port, pin) == GPIO_PIN_SET) { \
            if (HAL_GetTick() - fault_states[index].last_fault_time > 5) { \
                fault_states[index].is_active = false; \
                fault_log[fault_log_index - 1].retry_count = fault_states[index].retry_count; \
                char log_msg[64]; \
                snprintf(log_msg, sizeof(log_msg), "%s_Fault_Cleared\n", desc); \
                HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
            } \
        } else if (fault_states[index].retry_count < 5) { \
            HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET); \
            HAL_Delay(100); \
            HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET); \
            fault_states[index].retry_count++; \
            fault_states[index].last_fault_time = HAL_GetTick(); \
        } else { \
            HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET); \
            char log_msg[64]; \
            snprintf(log_msg, sizeof(log_msg), "%s_Fault_Persistent:_Subsystem_Disabled\n", desc); \
            HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
            fault_states[index].is_active = false; \
        } \
    }

    // Helper macro for reset requests
    #define REQUEST_RESET(index, pin_port, pin, target_addr, desc, reset_id) \
    if (fault_states[index].is_active) { \
        if (HAL_GPIO_ReadPin(pin_port, pin) == GPIO_PIN_SET) { \
            if (HAL_GetTick() - fault_states[index].last_fault_time > 5) { \
                fault_states[index].is_active = false; \
                fault_log[fault_log_index - 1].retry_count = fault_states[index].retry_count; \
                char log_msg[64]; \
                snprintf(log_msg, sizeof(log_msg), "%s_Fault_Cleared\n", desc); \
                HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
            } \
        } else if (fault_states[index].retry_count < 5) { \
            uint8_t reset_data = reset_id; \
            SSP_PackFrame(&tx_frame, SSP_ADDR_EPS, target_addr, SSP_CMD_RESET_SUBSYSTEM, &reset_data, 1); \
            SSP_SendFrame(huart, de_port, de_pin, &tx_frame); \
            fault_states[index].retry_count++; \
            fault_states[index].last_fault_time = HAL_GetTick(); \
        } else { \
            char log_msg[64]; \
            snprintf(log_msg, sizeof(log_msg), "%s_Fault_Persistent:_Reset_Failed\n", desc); \
            HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
            fault_states[index].is_active = false; \
        } \
    }

    // Handle all faults
    HANDLE_FAULT(0, ADCS_FAULT_GPIO_Port, ADCS_FAULT_Pin, ADCS_EN_GPIO_Port, ADCS_EN_Pin, "ADCS", 0x0B);
    HANDLE_FAULT(1, ADCS12V_FLT_GPIO_Port, ADCS12V_FLT_Pin, ADCS12V_EN_GPIO_Port, ADCS12V_EN_Pin, "ADCS12V", 0x0D);
    HANDLE_FAULT(2, ADCS5V_FLT_GPIO_Port, ADCS5V_FLT_Pin, ADCS5V_EN_GPIO_Port, ADCS5V_EN_Pin, "ADCS5V", 0x03);
    HANDLE_FAULT(4, CCU5V_FLT_GPIO_Port, CCU5V_FLT_Pin, CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, "CCU5V", 0x02);
    HANDLE_FAULT(5, GPS_FLT_GPIO_Port, GPS_FLT_Pin, GPS_EN_GPIO_Port, GPS_EN_Pin, "GPS", 0x01);
    HANDLE_FAULT(7, PL_FLT_GPIO_Port, PL_FLT_Pin, PL_EN_GPIO_Port, PL_EN_Pin, "PL", 0x0A);
    HANDLE_FAULT(10, RS3V3_FLT_GPIO_Port, RS3V3_FLT_Pin, RS3V3_EN_GPIO_Port, RS3V3_EN_Pin, "RS3V3", 0x09);
    HANDLE_FAULT(11, RS5V_FLT_GPIO_Port, RS5V_FLT_Pin, RS5V_EN_GPIO_Port, RS5V_EN_Pin, "RS5V", 0x05);
    HANDLE_FAULT(12, UHF_FLT_GPIO_Port, UHF_FLT_Pin, UHF_EN_GPIO_Port, UHF_EN_Pin, "UHF", 0x0F);

    REQUEST_RESET(3, CCU_FAULT_GPIO_Port, CCU_FAULT_Pin, SSP_ADDR_OBC, "CCU", 0x02);
    REQUEST_RESET(6, OBC_FAULT_GPIO_Port, OBC_FAULT_Pin, SSP_ADDR_CCU, "OBC", 0x01);
}

void PollFaults(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_log) {
    #define POLL_FAULT(index, pin_port, pin, en_port, en_pin, desc, subsystem_id) \
    if (HAL_GPIO_ReadPin(pin_port, pin) == GPIO_PIN_RESET && !fault_states[index].is_active) { \
        fault_states[index].is_active = true; \
        fault_states[index].last_fault_time = HAL_GetTick(); \
        LogFault(hi2c, desc "_Fault_Detected", subsystem_id); \
        char log_msg[64]; \
        snprintf(log_msg, sizeof(log_msg), "%s_Fault_Detected\n", desc); \
        HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
    } else if (HAL_GPIO_ReadPin(pin_port, pin) == GPIO_PIN_SET && fault_states[index].is_active) { \
        if (HAL_GetTick() - fault_states[index].last_fault_time > 5) { \
            fault_states[index].is_active = false; \
            fault_log[fault_log_index - 1].retry_count = fault_states[index].retry_count; \
            char log_msg[64]; \
            snprintf(log_msg, sizeof(log_msg), "%s_Fault_Cleared\n", desc); \
            HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
        } \
    } else if (fault_states[index].is_active && fault_states[index].retry_count < 5) { \
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET); \
        HAL_Delay(100); \
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET); \
        fault_states[index].retry_count++; \
        fault_states[index].last_fault_time = HAL_GetTick(); \
    } else if (fault_states[index].is_active && fault_states[index].retry_count >= 5) { \
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET); \
        fault_states[index].is_active = false; \
        char log_msg[64]; \
        snprintf(log_msg, sizeof(log_msg), "%s_Fault_Persistent:_Subsystem_Disabled\n", desc); \
        HAL_UART_Transmit(huart_log, (uint8_t*)log_msg, strlen(log_msg), 100); \
    }

    POLL_FAULT(8, PL5V_FLT_GPIO_Port, PL5V_FLT_Pin, PL5V_EN_GPIO_Port, PL5V_EN_Pin, "PL5V", 0x04);
    POLL_FAULT(9, RS12V_FLT_GPIO_Port, RS12V_FLT_Pin, RS12V_EN_GPIO_Port, RS12V_EN_Pin, "RS12V", 0x08);
    POLL_FAULT(3, CCU_FAULT_GPIO_Port, CCU_FAULT_Pin, CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, "CCU", 0x02);
    POLL_FAULT(13, XB12V_FLT_GPIO_Port, XB12V_FLT_Pin, XB12V_EN_GPIO_Port, XB12V_EN_Pin, "XB12V", 0x07);
}
