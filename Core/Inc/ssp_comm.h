#ifndef __SSP_COMM_H
#define __SSP_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ssp_common.h"  // Include SSP_Frame definition

HAL_StatusTypeDef SSP_SendFrame(UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin, SSP_Frame *frame);
HAL_StatusTypeDef SSP_ReceiveFrame(UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin,
                                   SSP_Frame *frame, uint32_t timeout);
void SSP_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void SSP_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __SSP_COMM_H */
