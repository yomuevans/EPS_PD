#include "sync_counter.h"
#include "main.h" // For GPIO and TIM2 access

// Global sync counter state
static SyncCounter_t sync_counter = {
    .sync_counter = 0,
    .subtick_us = 0
};

void SyncCounter_Init(void) {
    // Configure PD1 (Sync_pulse) as EXTI
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = SYNC_PULSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SYNC_PULSE_GPIO_Port, &GPIO_InitStruct);

    // Enable and set PD1 interrupt
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

// Interrupt handler for PD1 (Sync_pulse) rising edge
// Note: sync_counter is incremented on each pulse, and subtick_us is reset to 0 to mark
// the start of a new second. This ensures subtick_us reflects the time elapsed since the
// last pulse, with TIM2 counting microseconds from this point. The order prioritizes
// resetting the subtick at the pulse event for interval tracking.
void SyncPulse_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(SYNC_PULSE_Pin) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(SYNC_PULSE_Pin); // Clear first to prevent re-entry
        sync_counter.sync_counter++;
        sync_counter.subtick_us = 0;
        __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset TIM2 for next subtick
    }
}

// Set sync counter to a new value
void SetSyncCounter(uint64_t val) {
    __disable_irq();
    sync_counter.sync_counter = val;
    sync_counter.subtick_us = 0; // Reset subtick on manual set
    __enable_irq();
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset TIM2
}

// Get current sync counter value with race condition protection
uint64_t GetSyncCounter(void) {
    uint64_t val;
    __disable_irq();
    val = sync_counter.sync_counter;
    __enable_irq();
    return val;
}

// Get current subtick microseconds
uint32_t GetSubtickUs(void) {
    return __HAL_TIM_GET_COUNTER(&htim2); // Current TIM2 count since last pulse
}

// Get composite timestamp with race condition protection
void GetSyncTimestamp(uint64_t *counter, uint32_t *subtick) {
    __disable_irq();
    *counter = sync_counter.sync_counter;
    *subtick = __HAL_TIM_GET_COUNTER(&htim2);
    __enable_irq();
}
