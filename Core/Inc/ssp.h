#ifndef __SSP_H
#define __SSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "main.h"
#include "satellite_modes.h"
#include "sync_counter.h"
#include "i2c_comm.h"

#define SSP_CMD_PING    0x00
#define SSP_CMD_INIT    0x01
#define SSP_CMD_ACK     0x02
#define SSP_CMD_NACK    0x03
#define SSP_CMD_READ    0x04  // Read parameter value
#define SSP_CMD_WRITE   0x05  // Write parameter value
#define SSP_CMD_GD      0x06  // General data (optional, for future use)
#define SSP_CMD_SON     0x0B
#define SSP_CMD_SOF     0x0C
#define SSP_CMD_SM      0x15
#define SSP_CMD_GM      0x16
#define SSP_CMD_GSC     0x17
#define SSP_CMD_SSC     0x18
#define SSP_CMD_KEN     0x31
#define SSP_CMD_KDIS    0x32

#define SSP_FRAME_START 0xC0
#define SSP_MAX_DATA_LEN 248 // Max data size = 256 - 8 bytes overhead
#define SSP_FRAME_OVERHEAD 8 // DEST, SRC, CMD, D_Len, CRC (2), End Flag
#define SSP_GD_BLOCK_SIZE 32 // Example block size (adjust as needed)

typedef struct {
    uint8_t module_addr;      // Subsystem module address
    uint16_t seq_num;         // Sequence number of data block
    uint8_t data[SSP_GD_BLOCK_SIZE]; // Data block
} SSP_GD_DataBlock_t;

typedef struct {
    uint8_t start;        // 0xC0
    uint8_t src;          // Source address
    uint8_t dest;         // Destination address
    uint8_t cmd;          // Command ID
    uint8_t len;          // Data length
    uint8_t data[SSP_MAX_DATA_LEN]; // Data payload (includes 2-byte ID + parameter value)
    uint16_t crc;         // CRC16
    uint8_t end;          // 0xC0
} SSP_Frame_t;

#define ADDR_OBC        0x01
#define ADDR_CCU        0x02
#define ADDR_EPS        0x02
#define ADDR_PL         0x05

// Target Function IDs (Subsystem-specific, second byte of ID)
// EPS Function IDs (0x0201 to 0x02FF)
#define EPS_F1   0x0201
#define EPS_F2   0x0202
#define EPS_F3   0x0203
#define EPS_F4   0x0204
#define EPS_F5   0x0205
#define EPS_F255 0x02FF

// Function Prototypes
void SSP_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, DMA_HandleTypeDef *hdma_tx1, DMA_HandleTypeDef *hdma_rx1, DMA_HandleTypeDef *hdma_tx2, DMA_HandleTypeDef *hdma_rx2, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SSP_SendCommand(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_tx, SSP_Frame_t *frame);
HAL_StatusTypeDef SSP_ReceiveCommand(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx, SSP_Frame_t *frame);
void SSP_ProcessCommand(SSP_Frame_t *frame);
uint16_t SSP_CalculateCRC(uint8_t *data, uint16_t len);
uint32_t SSP_GetCRCErrors(void);
uint32_t SSP_GetFramingErrors(void);
void SSP_ResetErrorCounters(void);

typedef struct {
    uint16_t id;           // Two-byte ID (subsystem | function)
    uint32_t value;        // Parameter value
} ConfigurableParameter_t;

#ifdef __cplusplus
}
#endif

#endif /* __SSP_H */
