#ifndef __SYNC_COUNTER_H
#define __SYNC_COUNTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim2; // Declare htim2 as extern

// Synchronization counter structure
typedef struct {
    volatile uint64_t sync_counter; // 64-bit counter incremented on PD1 rising edge
    uint32_t subtick_us;           // Microseconds since last pulse (via TIM2)
} SyncCounter_t;

// Function Prototypes
void SyncCounter_Init(void);
void SyncPulse_IRQHandler(void);
void SetSyncCounter(uint64_t val);
uint64_t GetSyncCounter(void);
uint32_t GetSubtickUs(void);
void GetSyncTimestamp(uint64_t *counter, uint32_t *subtick);

#ifdef __cplusplus
}
#endif

#endif /* __SYNC_COUNTER_H */
