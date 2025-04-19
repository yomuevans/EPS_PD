#ifndef __SSP_COMMON_H
#define __SSP_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// Common defines
#define SSP_START_END_BYTE 0xC0  // Per ICD, Page 87 (SON command example)
#define SSP_MAX_DATA_LEN 32
#define SSP_FRAME_OVERHEAD 7  // Start byte, SrcAddr, DestAddr, CmdId, DataLen, CRC (2 bytes), End byte
#define SSP_MAX_FRAME_LEN (SSP_FRAME_OVERHEAD + SSP_MAX_DATA_LEN)

// SSP Addresses (ICD, Page 55)
#define SSP_ADDR_OBC 0x01
#define SSP_ADDR_EPS 0x50
#define SSP_ADDR_CCU 0x02
#define SSP_ADDR_ALL 0xFF

// SSP Response Codes (ICD, Page 61)
#define SSP_RESP_ACK 0x02
#define SSP_RESP_NACK 0x03

// SSP Frame Structure
typedef struct {
    uint8_t StartByte;
    uint8_t SrcAddr;
    uint8_t DestAddr;
    uint8_t CmdId;
    uint8_t DataLen;
    uint8_t Data[SSP_MAX_DATA_LEN];
    uint16_t Crc;
    uint8_t EndByte;
} SSP_Frame;

#ifdef __cplusplus
}
#endif

#endif /* __SSP_COMMON_H */
