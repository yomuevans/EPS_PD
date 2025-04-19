#ifndef __SSP_FRAME_H
#define __SSP_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ssp_common.h"  // Include SSP_Frame definition

void SSP_PackFrame(SSP_Frame *frame, uint8_t src_addr, uint8_t dest_addr, uint8_t cmd_id,
                   const uint8_t *data, uint8_t data_len);
HAL_StatusTypeDef SSP_UnpackFrame(const uint8_t *buffer, uint16_t len, SSP_Frame *frame);
uint16_t SSP_CalculateCRC(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SSP_FRAME_H */
