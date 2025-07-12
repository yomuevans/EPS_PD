//File: ssp.c
// Include header files needed for the code to work
#include "ssp.h"              // Defines SSP constants (e.g., SSP_FRAME_START), structures (e.g., SSP_Frame_t), and function prototypes
#include "satellite_modes.h"   // Functions for managing satellite operating modes (e.g., normal, low power)
#include "main.h"             // Main project header with GPIO pin definitions and hardware handles (e.g., UART, I2C)
#include "sync_counter.h"     // Functions for getting and setting sync counter and timestamp
#include "telemetry.h"        // Telemetry structures (e.g., EPSPD_Telemetry) and functions for EPS data
#include "eeprom.h"           // Functions for reading/writing telemetry and fault logs to EEPROM via I2C
#include "fault.h"            // Functions for handling and logging faults in the EPS
#include <string.h>           // Standard C library for memory operations like memcpy (copying data)

// Declare global pointers to store UART and DMA handles for SSP communication
static UART_HandleTypeDef *ssp_uart1;  // Pointer to UART handle (likely UART2) for SSP communication
static UART_HandleTypeDef *ssp_uart2;  // Pointer to UART handle (likely UART3) for additional SSP communication
static DMA_HandleTypeDef *ssp_dma_tx1; // Pointer to DMA handle for UART2 transmit
static DMA_HandleTypeDef *ssp_dma_rx1; // Pointer to DMA handle for UART2 receive
static DMA_HandleTypeDef *ssp_dma_tx2; // Pointer to DMA handle for UART3 transmit
static DMA_HandleTypeDef *ssp_dma_rx2; // Pointer to DMA handle for UART3 receive
static I2C_HandleTypeDef *ssp_i2c;     // Pointer to I2C handle (likely I2C2 or I2C3) for EEPROM and BMS communication


// Declare global buffers for receiving SSP frames via UART DMA
static uint8_t rx_buffer1[SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD]; // Buffer for UART2 receive (size includes data + frame overhead)
static uint8_t rx_buffer2[SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD]; // Buffer for UART3 receive (size includes data + frame overhead)

// Declare counters to track communication errors
static uint32_t crc_errors = 0;       // Counts CRC errors in received SSP frames (invalid checksums)
static uint32_t framing_errors = 0;   // Counts UART framing errors (e.g., incorrect frame structure)

// Function: SSP_GetRxBuffer
// Inputs:
//   - huart: A pointer to a UART_HandleTypeDef, the UART interface (ssp_uart1 or ssp_uart2)
// Output:
//   - Returns a pointer to a uint8_t array, the appropriate receive buffer (rx_buffer1 or rx_buffer2)
// Significance:
//   - A helper function that selects the correct receive buffer based on the UART interface,
//     used to manage DMA reception of SSP frames. It’s marked "inline" for faster execution.
static inline uint8_t* SSP_GetRxBuffer(UART_HandleTypeDef *huart) {
    // Check if the UART handle matches ssp_uart1; if so, return rx_buffer1, else return rx_buffer2
    return (huart == ssp_uart1) ? rx_buffer1 : rx_buffer2;
}

// Function: SSP_Init
// Inputs:
//   - huart1: A pointer to a UART_HandleTypeDef, the first UART interface (likely UART2)
//   - huart2: A pointer to a UART_HandleTypeDef, the second UART interface (likely UART3)
//   - hdma_tx1: A pointer to a DMA_HandleTypeDef, DMA for UART2 transmit
//   - hdma_rx1: A pointer to a DMA_HandleTypeDef, DMA for UART2 receive
//   - hdma_tx2: A pointer to a DMA_HandleTypeDef, DMA for UART3 transmit
//   - hdma_rx2: A pointer to a DMA_HandleTypeDef, DMA for UART3 receive
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface (likely I2C2 or I2C3)
// Output:
//   - None (void), initializes SSP and related systems
// Significance:
//   - Sets up the SSP by storing UART, DMA, and I2C handles, starting DMA reception for
//     UARTs, and initializing satellite modes, sync counter, and EEPROM. Critical for
//     enabling communication with subsystems like OBC and BMS.
void SSP_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, DMA_HandleTypeDef *hdma_tx1, DMA_HandleTypeDef *hdma_rx1, DMA_HandleTypeDef *hdma_tx2, DMA_HandleTypeDef *hdma_rx2, I2C_HandleTypeDef *hi2c) {
    // Store the first UART handle (likely UART2 for OBC communication)
    ssp_uart1 = huart1;
    // Store the second UART handle (likely UART3 for other subsystems)
    ssp_uart2 = huart2;
    // Store the DMA transmit handle for UART2
    ssp_dma_tx1 = hdma_tx1;
    // Store the DMA receive handle for UART2
    ssp_dma_rx1 = hdma_rx1;
    // Store the DMA transmit handle for UART3
    ssp_dma_tx2 = hdma_tx2;
    // Store the DMA receive handle for UART3
    ssp_dma_rx2 = hdma_rx2;
    // Store the I2C handle for EEPROM and BMS communication
    ssp_i2c = hi2c; // Store the I2C handle

    // Start UART2 reception in DMA mode, listening for SSP frames in rx_buffer1
    HAL_UART_Receive_DMA(ssp_uart1, rx_buffer1, sizeof(rx_buffer1));
    // Start UART3 reception in DMA mode, listening for SSP frames in rx_buffer2
    HAL_UART_Receive_DMA(ssp_uart2, rx_buffer2, sizeof(rx_buffer2));

    // Initialize satellite modes (e.g., normal, low power) for power management
    SatelliteModes_Init();
    // Initialize the sync counter for timestamping and BMS synchronization
    SyncCounter_Init();
    // Initialize the EEPROM for telemetry and fault log storage
    epspd_Init(ssp_i2c);
}

// Function: SSP_CalculateCRC
// Inputs:
//   - data: A pointer to a uint8_t array, the data to calculate the CRC for
//   - len: A uint16_t, the length of the data array
// Output:
//   - Returns a uint16_t, the calculated CRC-16-CCITT checksum
// Significance:
//   - Calculates a CRC-16 checksum to verify data integrity in SSP frames, ensuring
//     reliable communication between subsystems (Reference Manual, Section 36.8, page 1235).
uint16_t SSP_CalculateCRC(uint8_t *data, uint16_t len) {
    // Initialize CRC to 0xFFFF (standard starting value for CRC-16-CCITT)
    uint16_t crc = 0xFFFF;
    // Loop through each byte in the data array
    for (uint16_t i = 0; i < len; i++) {
        // XOR the current byte with the CRC
        crc ^= data[i];
        // Process each bit in the byte (8 bits)
        for (uint8_t j = 0; j < 8; j++) {
            // If the least significant bit of crc is 1
            if (crc & 0x0001) {
                // Shift crc right by 1
                crc >>= 1;
                // XOR with CRC-16 polynomial 0xA001
                crc ^= 0xA001;
            } else {
                // Otherwise, just shift crc right by 1
                crc >>= 1;
            }
        }
    }
    // Return the final CRC value
    return crc;
}

// Function: SSP_SendCommand
// Inputs:
//   - huart: A pointer to a UART_HandleTypeDef, the UART to send the command on
//   - hdma_tx: A pointer to a DMA_HandleTypeDef, the DMA channel for transmission
//   - frame: A pointer to an SSP_Frame_t, the frame containing the command to send
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if transmission starts successfully, HAL_ERROR if failed
// Significance:
//   - Constructs and sends an SSP frame over UART using DMA, including CRC for data integrity,
//     critical for sending commands or replies to subsystems like OBC or BMS.
HAL_StatusTypeDef SSP_SendCommand(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_tx, SSP_Frame_t *frame) {
    // Declare a static buffer to hold the frame (max data length + overhead)
    static uint8_t buffer[SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD];
    // Initialize position index for building the frame
    uint16_t pos = 0;

    // Construct frame by adding bytes to the buffer
    // Add the start byte (defined in ssp.h, e.g., a unique marker like 0x7E)
    buffer[pos++] = SSP_FRAME_START;
    // Add the source address (who is sending this frame, e.g., ADDR_EPS)
    buffer[pos++] = frame->src;
    // Add the destination address (who the frame is for, e.g., ADDR_OBC)
    buffer[pos++] = frame->dest;
    // Add the command code (e.g., SSP_CMD_PING)
    buffer[pos++] = frame->cmd;
    // Add the data length
    buffer[pos++] = frame->len;
    // Copy the frame’s data into the buffer
    memcpy(&buffer[pos], frame->data, frame->len);
    // Update position by adding data length
    pos += frame->len;

    // Prepare data for CRC calculation (source, destination, command, length, data)
    uint8_t crc_data[SSP_MAX_DATA_LEN + 4];
    // Copy source address to CRC data
    crc_data[0] = frame->src;
    // Copy destination address to CRC data
    crc_data[1] = frame->dest;
    // Copy command code to CRC data
    crc_data[2] = frame->cmd;
    // Copy data length to CRC data
    crc_data[3] = frame->len;
    // Copy frame data to CRC data
    memcpy(&crc_data[4], frame->data, frame->len);
    // Calculate CRC for the frame (source, dest, cmd, len, data)
    frame->crc = SSP_CalculateCRC(crc_data, frame->len + 4);

    // Add low byte of CRC to buffer
    buffer[pos++] = frame->crc & 0xFF; // CRC_0 (LSB)
    // Add high byte of CRC to buffer
    buffer[pos++] = (frame->crc >> 8) & 0xFF; // CRC_1 (MSB)
    // Add end byte (same as start byte, e.g., 0x7E)
    buffer[pos++] = SSP_FRAME_START; // End flag

    // Select the RS485 driver enable pin based on UART (USART2 or USART3)
    GPIO_PinState de_pin = (huart->Instance == USART2) ? RS4852_DE_Pin : RS4851_DE_Pin;
    // Both pins are on GPIOD (same port for both UARTs)
    GPIO_TypeDef *de_port = (huart->Instance == USART2) ? GPIOD : GPIOD;
    // Set the driver enable pin high to enable RS485 transmission
    HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_SET);

    // Send the frame using DMA over the specified UART
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, buffer, pos);

    // The driver enable pin will be reset in the DMA transfer complete callback
    // Return the status of the transmission (HAL_OK or HAL_ERROR)
    return status;
}

// Function: SSP_ReceiveCommand
// Inputs:
//   - huart: A pointer to a UART_HandleTypeDef, the UART receiving the command
//   - hdma_rx: A pointer to a DMA_HandleTypeDef, the DMA channel for reception
//   - frame: A pointer to an SSP_Frame_t, where the received frame will be stored
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if a valid frame is received, HAL_ERROR if invalid
// Significance:
//   - Receives an SSP frame via UART DMA, validates its structure and CRC, and extracts
//     its contents, critical for processing commands from subsystems.
HAL_StatusTypeDef SSP_ReceiveCommand(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx, SSP_Frame_t *frame) {
    // Calculate received data length by subtracting remaining DMA bytes from buffer size
    uint16_t rx_len = (SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD) - __HAL_DMA_GET_COUNTER(hdma_rx);

    // Check if the received length is valid (at least overhead size, not too large)
    if (rx_len < SSP_FRAME_OVERHEAD || rx_len > SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD) {
        // If length is invalid, restart DMA reception to listen for new data
        HAL_UART_Receive_DMA(huart, SSP_GetRxBuffer(huart), SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD);
        // Return error status
        return HAL_ERROR;
    }

    // Stop DMA to safely access the received data
    HAL_UART_DMAStop(huart);
    // Get the correct receive buffer for this UART
    uint8_t *rx_buffer = SSP_GetRxBuffer(huart);
    // Copy received data from DMA memory to the buffer
    memcpy(rx_buffer, (uint8_t *)hdma_rx->Instance->CMAR, rx_len);

    // Validate frame structure: must start and end with SSP_FRAME_START
    if (rx_buffer[0] == SSP_FRAME_START && rx_buffer[rx_len - 1] == SSP_FRAME_START) {
        // Extract start byte
        frame->start = rx_buffer[0];
        // Extract source address
        frame->src = rx_buffer[1];
        // Extract destination address
        frame->dest = rx_buffer[2];
        // Extract command code
        frame->cmd = rx_buffer[3];
        // Extract data length
        frame->len = rx_buffer[4];
        // Check if data length is valid (not exceeding max data length)
        if (frame->len > SSP_MAX_DATA_LEN) {
            // Restart DMA reception for new data
            HAL_UART_Receive_DMA(huart, SSP_GetRxBuffer(huart), SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD);
            // Return error status
            return HAL_ERROR;
        }
        // Copy data into frame
        memcpy(frame->data, &rx_buffer[5], frame->len);
        // Extract CRC (combine low and high bytes)
        frame->crc = (rx_buffer[5 + frame->len] << 8) | rx_buffer[5 + frame->len + 1];
        // Extract end byte
        frame->end = rx_buffer[5 + frame->len + 2];

        // Prepare data for CRC validation (source, dest, cmd, len, data)
        uint8_t crc_data[SSP_MAX_DATA_LEN + 4];
        crc_data[0] = frame->src;
        crc_data[1] = frame->dest;
        crc_data[2] = frame->cmd;
        crc_data[3] = frame->len;
        memcpy(&crc_data[4], frame->data, frame->len);
        // Calculate CRC for the received data
        uint16_t calculated_crc = SSP_CalculateCRC(crc_data, frame->len + 4);
        // If CRC doesn’t match, increment error counter
        if (calculated_crc != frame->crc) {
            crc_errors++; // Increment CRC error counter
        }

        // If CRC is valid, restart DMA reception and return success
        if (calculated_crc == frame->crc) {
            HAL_UART_Receive_DMA(huart, SSP_GetRxBuffer(huart), SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD);
            return HAL_OK;
        }
    }

    // If frame is invalid, restart DMA reception to listen for new data
    HAL_UART_Receive_DMA(huart, SSP_GetRxBuffer(huart), SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD);
    // Return error status
    return HAL_ERROR;
}

// Function: SSP_ProcessCommand
// Inputs:
//   - frame: A pointer to an SSP_Frame_t, the received command frame to process
// Output:
//   - None (void), processes the command and sends a reply via UART
// Significance:
//   - Handles various SSP commands (e.g., ping, sync counter, power control, telemetry)
//     and sends appropriate replies, enabling coordination with subsystems like OBC and BMS.
void SSP_ProcessCommand(SSP_Frame_t *frame) {
    // Initialize a reply frame with default values
    SSP_Frame_t reply = {
        .start = SSP_FRAME_START, // Set start byte (e.g., 0x7E)
        .src = ADDR_EPS,          // Set source as EPS
        .dest = frame->src,       // Reply to the sender
        .cmd = SSP_CMD_ACK,       // Default to ACK (acknowledge)
        .len = 1,                 // Default data length of 1 byte
        .end = SSP_FRAME_START    // Set end byte
    };

    // Check if the frame is addressed to the EPS
    if (frame->dest != ADDR_EPS) {
        // If not for EPS, ignore the frame and return
        return;
    }

    // Select the appropriate UART and DMA based on the source address
    UART_HandleTypeDef *huart = (frame->src == ADDR_OBC) ? ssp_uart1 : ssp_uart2;
    DMA_HandleTypeDef *hdma_tx = (frame->src == ADDR_OBC) ? ssp_dma_tx1 : ssp_dma_tx2;

    // Extract 2-byte target ID from data (subsystem | function)
    uint8_t subsystem_addr = frame->data[0]; // First byte is subsystem address
    uint8_t func_addr = frame->data[1];     // Second byte is function address
    // Combine into a 16-bit ID (shift subsystem left by 8 and OR with function)
    uint16_t target_id = (subsystem_addr << 8) | func_addr;

    // Process the command based on its code
    switch (frame->cmd) {
        case SSP_CMD_PING: // Handle ping command
            // Reply with the same command code to acknowledge
            reply.data[0] = SSP_CMD_PING;
            break;

        case SSP_CMD_GSC: // Get sync counter command
            // Set reply command to ACK
            reply.cmd = SSP_CMD_ACK;
            // Set data length to 8 bytes (for 64-bit counter)
            reply.len = 8;
            // Get the current sync counter value
            uint64_t counter = GetSyncCounter();
            // Split 64-bit counter into 8 bytes (big-endian)
            reply.data[0] = (counter >> 56) & 0xFF; // Most significant byte
            reply.data[1] = (counter >> 48) & 0xFF;
            reply.data[2] = (counter >> 40) & 0xFF;
            reply.data[3] = (counter >> 32) & 0xFF;
            reply.data[4] = (counter >> 24) & 0xFF;
            reply.data[5] = (counter >> 16) & 0xFF;
            reply.data[6] = (counter >> 8) & 0xFF;
            reply.data[7] = counter & 0xFF; // Least significant byte
            break;

        case SSP_CMD_SSC: // Set sync counter command
            // Check if data length is 8 bytes (for 64-bit counter)
            if (frame->len == 8) {
                // Combine 8 data bytes into a 64-bit counter
                uint64_t new_counter = ((uint64_t)frame->data[0] << 56) |
                                      ((uint64_t)frame->data[1] << 48) |
                                      ((uint64_t)frame->data[2] << 40) |
                                      ((uint64_t)frame->data[3] << 32) |
                                      ((uint64_t)frame->data[4] << 24) |
                                      ((uint64_t)frame->data[5] << 16) |
                                      ((uint64_t)frame->data[6] << 8)  |
                                      (uint64_t)frame->data[7];
                // Disable interrupts to safely update the counter
                __disable_irq();
                // Set the new sync counter value
                SetSyncCounter(new_counter);
                // Re-enable interrupts
                __enable_irq();
                // Reply with the command code
                reply.data[0] = SSP_CMD_SSC;
            } else {
                // If length is invalid, send NACK (negative acknowledge)
                reply.cmd = SSP_CMD_NACK;
                reply.data[0] = SSP_CMD_SSC;
            }
            break;

        case SSP_CMD_SON: // Switch power line on command
            // Check if data length is 1 byte (power line ID)
            if (frame->len == 1) {
                // Get the power line ID from data
                PowerLineID_t pwrl_id = (PowerLineID_t)frame->data[0];
                // Try to turn on the specified power line (e.g., 5V, 12V)
                if (SatelliteModes_SwitchPowerLine(pwrl_id, GPIO_PIN_SET) == HAL_OK) {
                    // If successful, reply with command code
                    reply.data[0] = SSP_CMD_SON;
                } else {
                    // If failed, send NACK
                    reply.cmd = SSP_CMD_NACK;
                    reply.data[0] = SSP_CMD_SON;
                }
            } else {
                // If length is invalid, send NACK
                reply.cmd = SSP_CMD_NACK;
                reply.data[0] = SSP_CMD_SON;
            }
            break;

        case SSP_CMD_SOF: // Switch power line off command
            // Check if data length is 1 byte
            if (frame->len == 1) {
                // Get the power line ID from data
                PowerLineID_t pwrl_id = (PowerLineID_t)frame->data[0];
                // Try to turn off the specified power line
                if (SatelliteModes_SwitchPowerLine(pwrl_id, GPIO_PIN_RESET) == HAL_OK) {
                    // If successful, reply with command code
                    reply.data[0] = SSP_CMD_SOF;
                } else {
                    // If failed, send NACK
                    reply.cmd = SSP_CMD_NACK;
                    reply.data[0] = SSP_CMD_SOF;
                }
            } else {
                // If length is invalid, send NACK
                reply.cmd = SSP_CMD_NACK;
                reply.data[0] = SSP_CMD_SOF;
            }
            break;

        case SSP_CMD_SM: // Set satellite mode command
            // Check if data length is 1 byte
            if (frame->len == 1) {
                // Get the mode ID from data
                SatelliteMode_t mode = (SatelliteMode_t)frame->data[0];
                // Try to set the satellite mode (e.g., normal, low power)
                if (SatelliteModes_SetMode(mode) == HAL_OK) {
                    // If successful, reply with command code
                    reply.data[0] = SSP_CMD_SM;
                } else {
                    // If failed, send NACK
                    reply.cmd = SSP_CMD_NACK;
                    reply.data[0] = SSP_CMD_SM;
                }
            } else {
                // If length is invalid, send NACK
                reply.cmd = SSP_CMD_NACK;
                reply.data[0] = SSP_CMD_SM;
            }
            break;

        case SSP_CMD_GM: // Get satellite mode command
            // Set reply command to ACK
            reply.cmd = SSP_CMD_ACK;
            // Set data length to 1 byte
            reply.len = 1;
            // Get the current satellite mode and store in reply
            reply.data[0] = (uint8_t)SatelliteModes_GetMode();
            break;

        case SSP_CMD_READ: // Read parameter command
            // Set reply command to ACK
            reply.cmd = SSP_CMD_ACK;
            // Set data length to 4 bytes (e.g., a uint32_t value)
            reply.len = 4; // Return 4-byte value (e.g., uint32_t)
            // Handle the target ID (subsystem | function)
            switch (target_id) {
                case EPS_F1:   // Parameter ID 0x0201
                    // Placeholder: Set reply data to 0 (needs actual implementation)
                    reply.data[0] = 0; // Example: Placeholder value
                    reply.data[1] = 0;
                    reply.data[2] = 0;
                    reply.data[3] = 0;
                    break;
                case EPS_F2:   // Parameter ID 0x0202
                    // Placeholder: Set reply data to 0
                    reply.data[0] = 0; // Example: Placeholder value
                    reply.data[1] = 0;
                    reply.data[2] = 0;
                    reply.data[3] = 0;
                    break;
                // ... Add cases up to EPS_F255 (0x02FF) as needed
                case EPS_F255: // Parameter ID 0x02FF
                    // Placeholder: Set reply data to 0
                    reply.data[0] = 0; // Example: Placeholder value
                    reply.data[1] = 0;
                    reply.data[2] = 0;
                    reply.data[3] = 0;
                    break;
                default:
                    // If target ID is unknown, send NACK
                    reply.cmd = SSP_CMD_NACK;
                    reply.len = 1;
                    reply.data[0] = frame->cmd;
                    break;
            }
            break;

        case SSP_CMD_WRITE: // Write parameter command
            // Check if data length is at least 6 bytes (2-byte ID + 4-byte value)
            if (frame->len >= 6) { // 2-byte ID + 4-byte value
                // Handle the target ID
                switch (target_id) {
                    case EPS_F1:   // Parameter ID 0x0201
                        // Placeholder: Implement write logic for EPS_F1
                        reply.data[0] = SSP_CMD_WRITE;
                        break;
                    case EPS_F2:   // Parameter ID 0x0202
                        // Placeholder: Implement write logic for EPS_F2
                        reply.data[0] = SSP_CMD_WRITE;
                        break;
                    // ... Add cases up to EPS_F255 (0x02FF) as needed
                    case EPS_F255: // Parameter ID 0x02FF
                        // Placeholder: Implement write logic for EPS_F255
                        reply.data[0] = SSP_CMD_WRITE;
                        break;
                    default:
                        // If target ID is unknown, send NACK
                        reply.cmd = SSP_CMD_NACK;
                        reply.data[0] = frame->cmd;
                        break;
                }
            } else {
                // If length is invalid, send NACK
                reply.cmd = SSP_CMD_NACK;
                reply.data[0] = frame->cmd;
            }
            break;

        case SSP_CMD_GD: // Get data (telemetry and fault logs) command
        {
            // Declare a structure to hold telemetry and timestamp from EEPROM
            EEPROM_TelemetryWithTimestamp telemetry;
            // Read the latest telemetry from EEPROM
            HAL_StatusTypeDef t_status = epspd_ReadTelemetry(ssp_i2c, &telemetry);

            // If telemetry read was successful, send it
            if (t_status == HAL_OK) {
                // Set reply command to GD
                reply.cmd = SSP_CMD_GD;
                // Set data length to size of telemetry structure
                reply.len = sizeof(EEPROM_TelemetryWithTimestamp);
                // Copy telemetry data to reply
                memcpy(reply.data, &telemetry, reply.len);
                // Send the telemetry reply
                SSP_SendCommand(huart, hdma_tx, &reply);
            }
            // Request BMS telemetry over I2C
            uint8_t bms_rx[BMS_MAX_PAYLOAD_LEN];   // Maximum 253-byte payload
            uint8_t bms_rx_len = sizeof(bms_rx);

            // Transmit CMD_READ_TELEMETRY, expect response
            if (EPS_I2C_TransmitReceiveWithRetry(ssp_i2c,
                                                 CMD_GET_TELEMETRY,
                                                 NULL, 0,  // No TX payload
                                                 bms_rx, &bms_rx_len,
                                                 EPS_BMS_I2C_ADDR) == HAL_OK)
            {
                // Build the SSP reply with BMS telemetry data
                reply.cmd = SSP_CMD_GD;
                reply.len = bms_rx_len;
                memcpy(reply.data, bms_rx, bms_rx_len);
                SSP_SendCommand(huart, hdma_tx, &reply);
            }
            else
            {
                // If BMS I2C read fails after retries, send NACK
                reply.cmd = SSP_CMD_NACK;
                reply.len = 1;
                reply.data[0] = frame->cmd;
                SSP_SendCommand(huart, hdma_tx, &reply);
                break;  // Abort this switch case path
            }


            // Get the number of fault logs stored in EEPROM
            uint8_t count = EPS_GetFaultLogCount();
            // Loop through each fault log
            for (uint8_t i = 0; i < count; i++) {
                // Declare a structure to hold a fault log
                EEPROM_FaultLog log;
                // Read the fault log from EEPROM
                if (EPS_ReadFaultLog(ssp_i2c, i, &log) != HAL_OK) break;

                // Set reply command to GD (same as telemetry)
                reply.cmd = SSP_CMD_GD; // same command code for logs
                // Set data length to size of fault log
                reply.len = sizeof(EEPROM_FaultLog);
                // Copy fault log to reply
                memcpy(reply.data, &log, reply.len);
                // Send the fault log reply
                SSP_SendCommand(huart, hdma_tx, &reply);
            }
        }
        break;

        case SSP_CMD_KEN: // Keep-alive enable command
        {

        	extern I2C_HandleTypeDef hi2c3;
            HAL_StatusTypeDef status = EPS_I2C_SendCommand(&hi2c3, CMD_DISABLE_CHARGING,
                                                           NULL, 0, NULL, 0, I2C_SLAVE_ADDR_BMS);

            if (status == HAL_OK) {
                reply.data[0] = SSP_CMD_KEN; // Echo back command
            } else {
                reply.data[0] = 0xFF; // NACK or error indicator
                EPS_Log_Message(EPS_LOG_ERROR, "SSP_CMD_KEN: Failed to disable charging\n");
            }
            break;
        }

        case SSP_CMD_KDIS: // Kill Disable – resume charging only
        {
        	extern I2C_HandleTypeDef hi2c3;
            HAL_StatusTypeDef status = EPS_I2C_SendCommand(&hi2c3, CMD_ENABLE_CHARGING,
                                                           NULL, 0, NULL, 0, I2C_SLAVE_ADDR_BMS);

            if (status == HAL_OK) {
                reply.data[0] = SSP_CMD_KDIS;  // Echo back command as ACK
                EPS_Log_Message(EPS_LOG_INFO, "SSP_CMD_KDIS: Charging re-enabled");
            } else {
                reply.data[0] = 0xFF;  // Indicate failure
                EPS_Log_Message(EPS_LOG_ERROR, "SSP_CMD_KDIS: Failed to re-enable charging");
            }
            break;
        }

        default: // Unknown command
            // Send NACK for unrecognized command
            reply.cmd = SSP_CMD_NACK;
            reply.data[0] = frame->cmd;
            break;
    }

    // Send the reply frame over the appropriate UART
    SSP_SendCommand(huart, hdma_tx, &reply);
}

// Function: HAL_UART_TxCpltCallback
// Inputs:
//   - huart: A pointer to a UART_HandleTypeDef, the UART that completed transmission
// Output:
//   - None (void), resets the RS485 driver enable pin
// Significance:
//   - Called when a UART DMA transmission completes, resets the RS485 driver enable
//     pin to allow reception, critical for RS485 communication (Reference Manual, Section 36.8).
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Check if the UART is one of the SSP interfaces (UART2 or UART3)
    if (huart == ssp_uart1 || huart == ssp_uart2) {
        // Select the correct driver enable pin based on UART
        GPIO_PinState de_pin = (huart->Instance == USART2) ? RS4852_DE_Pin : RS4851_DE_Pin;
        // Both pins are on GPIOD
        GPIO_TypeDef *de_port = (huart->Instance == USART2) ? GPIOD : GPIOD;
        // Reset the driver enable pin to disable RS485 transmission
        HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
    }
}

// Function: HAL_UART_ErrorCallback
// Inputs:
//   - huart: A pointer to a UART_HandleTypeDef, the UART where an error occurred
// Output:
//   - None (void), handles UART errors by restarting DMA reception
// Significance:
//   - Handles UART errors (e.g., framing errors) by incrementing an error counter and
//     restarting DMA reception to recover, ensuring robust communication.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Check if the UART is one of the SSP interfaces
    if (huart == ssp_uart1 || huart == ssp_uart2) {
        // Increment the framing error counter
        framing_errors++;

        // Get the correct receive buffer for this UART
        uint8_t *rx_buffer = SSP_GetRxBuffer(huart);
        // Restart DMA reception to recover from the error
        HAL_UART_Receive_DMA(huart, rx_buffer, SSP_MAX_DATA_LEN + SSP_FRAME_OVERHEAD);
    }
}

// Function: SSP_GetCRCErrors
// Inputs:
//   - None (void)
// Output:
//   - Returns a uint32_t, the number of CRC errors detected
// Significance:
//   - Provides the count of CRC errors for debugging and monitoring communication reliability.
uint32_t SSP_GetCRCErrors(void) {
    // Return the current CRC error count
    return crc_errors;
}

// Function: SSP_GetFramingErrors
// Inputs:
//   - None (void)
// Output:
//   - Returns a uint32_t, the number of framing errors detected
// Significance:
//   - Provides the count of UART framing errors for debugging and monitoring.
uint32_t SSP_GetFramingErrors(void) {
    // Return the current framing error count
    return framing_errors;
}

// Function: SSP_ResetErrorCounters
// Inputs:
//   - None (void)
// Output:
//   - None (void), resets CRC and framing error counters
// Significance:
//   - Resets error counters to zero, useful for starting fresh diagnostics or after resolving issues.
void SSP_ResetErrorCounters(void) {
    // Set CRC error counter to 0
    crc_errors = 0;
    // Set framing error counter to 0
    framing_errors = 0;
}
