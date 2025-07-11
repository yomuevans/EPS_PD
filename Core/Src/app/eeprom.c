// File: eeprom.c
#include "eeprom.h"
#include "delay.h"
#include <stdbool.h>
#include <string.h>

#define LOCK_ADDRESS                 EEPROM_ADDR_LOCK
#define LOCK_MCU1                    0x55AA
#define LOCK_MCU2                    0xAA55
#define LOCK_TIMEOUT_MS              5000
#define WRITE_DELAY_MS               5
#define MCU_ID                       1

extern I2C_HandleTypeDef hi2c2;

typedef struct {
    uint16_t lock_value;
    uint32_t timestamp;
} EEPROM_Lock;

bool acquire_eeprom_lock(uint8_t mcu_id)
{
    uint32_t start_time = HAL_GetTick();
    EEPROM_Lock current_lock;
    EEPROM_Lock new_lock = {
        .lock_value = (mcu_id == 1) ? LOCK_MCU1 : LOCK_MCU2,
        .timestamp = HAL_GetTick()
    };

    uint8_t lock_addr[2] = { LOCK_ADDRESS >> 8, LOCK_ADDRESS & 0xFF };

    while ((HAL_GetTick() - start_time) < LOCK_TIMEOUT_MS)
    {
        HAL_I2C_Master_Transmit(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, lock_addr, 2, 100);
        HAL_I2C_Master_Receive(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, (uint8_t *)&current_lock, sizeof(current_lock), 100);

        if (current_lock.lock_value == 0 ||
            (HAL_GetTick() - current_lock.timestamp) > LOCK_TIMEOUT_MS)
        {
            uint8_t txbuf[2 + sizeof(EEPROM_Lock)];
            txbuf[0] = lock_addr[0];
            txbuf[1] = lock_addr[1];
            memcpy(&txbuf[2], &new_lock, sizeof(EEPROM_Lock));

            HAL_I2C_Master_Transmit(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, txbuf, sizeof(txbuf), 100);
            HAL_Delay(WRITE_DELAY_MS);

            HAL_I2C_Master_Transmit(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, lock_addr, 2, 100);
            HAL_I2C_Master_Receive(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, (uint8_t *)&current_lock, sizeof(current_lock), 100);

            if (current_lock.lock_value == new_lock.lock_value)
                return true;
        }
        HAL_Delay(10);
    }
    return false;
}

void release_eeprom_lock(void)
{
    EEPROM_Lock unlock = { .lock_value = 0, .timestamp = 0 };
    uint8_t txbuf[2 + sizeof(EEPROM_Lock)];
    txbuf[0] = LOCK_ADDRESS >> 8;
    txbuf[1] = LOCK_ADDRESS & 0xFF;
    memcpy(&txbuf[2], &unlock, sizeof(EEPROM_Lock));

    HAL_I2C_Master_Transmit(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, txbuf, sizeof(txbuf), 100);
    HAL_Delay(WRITE_DELAY_MS);
}

void check_and_clear_stale_lock(void)
{
    EEPROM_Lock current_lock;
    uint8_t lock_addr[2] = { LOCK_ADDRESS >> 8, LOCK_ADDRESS & 0xFF };

    HAL_I2C_Master_Transmit(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, lock_addr, 2, 100);
    HAL_I2C_Master_Receive(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, (uint8_t *)&current_lock, sizeof(current_lock), 100);

    if (current_lock.lock_value != 0 &&
        (HAL_GetTick() - current_lock.timestamp) > LOCK_TIMEOUT_MS)
    {
        release_eeprom_lock();
    }
}

static bool verify_write_success(uint16_t address, uint8_t* data, uint16_t size) {
    uint8_t tx[2] = { address >> 8, address & 0xFF };
    uint8_t read_back[64];

    if (size > sizeof(read_back)) return false;

    if (HAL_I2C_Master_Transmit(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, tx, 2, 100) != HAL_OK)
        return false;

    if (HAL_I2C_Master_Receive(&hi2c2, epspd_I2C_ADDR_MEMORY << 1, read_back, size, 100) != HAL_OK)
        return false;

    return (memcmp(data, read_back, size) == 0);
}

HAL_StatusTypeDef epspd_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t device_id[4] = {'E', 'P', 'S', '1'};
    uint8_t buffer[6];
    buffer[0] = EEPROM_ADDR_ID_PAGE_DEVICE_ID >> 8;
    buffer[1] = EEPROM_ADDR_ID_PAGE_DEVICE_ID & 0xFF;
    memcpy(&buffer[2], device_id, 4);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_ID_PAGE << 1, buffer, 6, 100);
    if (status == HAL_OK) {
        SoftwareDelay(1);
    }
    return status;
}

HAL_StatusTypeDef epspd_WriteTelemetry(I2C_HandleTypeDef *hi2c, EEPROM_TelemetryWithTimestamp *telemetry) {
    if (!acquire_eeprom_lock(MCU_ID)) return HAL_ERROR;

    uint8_t buffer[20];
    buffer[0] = EEPROM_ADDR_MCU1_TELEMETRY >> 8;
    buffer[1] = EEPROM_ADDR_MCU1_TELEMETRY & 0xFF;
    buffer[2] = telemetry->telemetry.Bus12V >> 8;
    buffer[3] = telemetry->telemetry.Bus12V;
    buffer[4] = telemetry->telemetry.Bus5V >> 8;
    buffer[5] = telemetry->telemetry.Bus5V;
    buffer[6] = telemetry->telemetry.Bus3V3 >> 8;
    buffer[7] = telemetry->telemetry.Bus3V3;
    buffer[8] = telemetry->telemetry.subtick_us >> 24;
    buffer[9] = telemetry->telemetry.subtick_us >> 16;
    buffer[10] = telemetry->telemetry.subtick_us >> 8;
    buffer[11] = telemetry->telemetry.subtick_us;
    buffer[12] = telemetry->counter >> 56;
    buffer[13] = telemetry->counter >> 48;
    buffer[14] = telemetry->counter >> 40;
    buffer[15] = telemetry->counter >> 32;
    buffer[16] = telemetry->counter >> 24;
    buffer[17] = telemetry->counter >> 16;
    buffer[18] = telemetry->counter >> 8;
    buffer[19] = telemetry->counter;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, epspd_I2C_ADDR_MEMORY << 1, buffer, 20, 100);
    if (status == HAL_OK) {
        SoftwareDelay(1);

        if (!verify_write_success(EEPROM_ADDR_MCU1_TELEMETRY, &buffer[2], 18)) {
            EPS_Log_Message(EPS_LOG_DEBUG, "EEPROM write verification failed");
            release_eeprom_lock();
            Error_Handler();
        }
    }

    release_eeprom_lock();
    return status;
}

// Optional: Wrap WriteParameters and ReadParameters in lock logic if concurrent access is expected
