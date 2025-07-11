#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "ssp.h"
#include "satellite_modes.h"
#include "sync_counter.h"
#include "telemetry.h"
#include "eeprom.h"

extern I2C_HandleTypeDef hi2c3;



void Error_Handler(void);

/* ---------------------- FAULT DETECTION PINS ---------------------- */
#define ADCS12V_FLT_Pin        GPIO_PIN_7
#define ADCS12V_FLT_GPIO_Port  GPIOB
#define ADCS5V_FLT_Pin         GPIO_PIN_9
#define ADCS5V_FLT_GPIO_Port   GPIOB
#define ADCS_FAULT_Pin         GPIO_PIN_0
#define ADCS_FAULT_GPIO_Port   GPIOD
#define CCU5V_FLT_Pin          GPIO_PIN_11
#define CCU5V_FLT_GPIO_Port    GPIOE
#define CCU_FAULT_Pin          GPIO_PIN_9
#define CCU_FAULT_GPIO_Port    GPIOD
#define GPS_FLT_Pin            GPIO_PIN_5
#define GPS_FLT_GPIO_Port      GPIOB
#define OBC_FAULT_Pin          GPIO_PIN_2
#define OBC_FAULT_GPIO_Port    GPIOD
#define PL5V_FLT_Pin           GPIO_PIN_0
#define PL5V_FLT_GPIO_Port     GPIOE
#define PL_FLT_Pin             GPIO_PIN_14
#define PL_FLT_GPIO_Port       GPIOD
#define RS12V_FLT_Pin          GPIO_PIN_7
#define RS12V_FLT_GPIO_Port    GPIOE
#define RS3V3_FLT_Pin          GPIO_PIN_8
#define RS3V3_FLT_GPIO_Port    GPIOD
#define RS5V_FLT_Pin           GPIO_PIN_1
#define RS5V_FLT_GPIO_Port     GPIOE
#define UHF_FLT_Pin            GPIO_PIN_4
#define UHF_FLT_GPIO_Port      GPIOB
#define XB12V_FLT_Pin          GPIO_PIN_9
#define XB12V_FLT_GPIO_Port    GPIOE

/* ---------------------- POWER ENABLE PINS ---------------------- */
#define ADCS12_EN_Pin          GPIO_PIN_8
#define ADCS12_EN_GPIO_Port    GPIOB
#define ADCS5V_EN_Pin          GPIO_PIN_4
#define ADCS5V_EN_GPIO_Port    GPIOE
#define ADCS_EN_Pin            GPIO_PIN_12
#define ADCS_EN_GPIO_Port      GPIOA
#define CCU5V_EN_Pin           GPIO_PIN_10
#define CCU5V_EN_GPIO_Port     GPIOE
#define GPS_EN_Pin             GPIO_PIN_6
#define GPS_EN_GPIO_Port       GPIOB
#define PL5V_EN_Pin            GPIO_PIN_3
#define PL5V_EN_GPIO_Port      GPIOE
#define PL_EN_Pin              GPIO_PIN_13
#define PL_EN_GPIO_Port        GPIOD
#define RS12V_EN_Pin           GPIO_PIN_2
#define RS12V_EN_GPIO_Port     GPIOB
#define RS3V3_EN_Pin           GPIO_PIN_15
#define RS3V3_EN_GPIO_Port     GPIOB
#define RS5V_EN_Pin            GPIO_PIN_2
#define RS5V_EN_GPIO_Port      GPIOE
#define UHF_EN_Pin             GPIO_PIN_7
#define UHF_EN_GPIO_Port       GPIOD
#define XB12V_EN_Pin           GPIO_PIN_8
#define XB12V_EN_GPIO_Port     GPIOE

/* ---------------------- CURRENT SENSE PINS ---------------------- */
#define ADCS12V_I_Pin          GPIO_PIN_5
#define ADCS12V_I_GPIO_Port    GPIOC
#define ADCS5V_1_Pin           GPIO_PIN_2
#define ADCS5V_1_GPIO_Port     GPIOC
#define ADCS_I_Pin             GPIO_PIN_7
#define ADCS_I_GPIO_Port       GPIOA
#define CCU5V_I_Pin            GPIO_PIN_4
#define CCU5V_I_GPIO_Port      GPIOA
#define CCU_I_Pin              GPIO_PIN_5
#define CCU_I_GPIO_Port        GPIOA
#define GPS_I_Pin              GPIO_PIN_3
#define GPS_I_GPIO_Port        GPIOC
#define OBC_I_Pin              GPIO_PIN_1
#define OBC_I_GPIO_Port        GPIOB
#define PL_I_Pin               GPIO_PIN_4
#define PL_I_GPIO_Port         GPIOC
#define PS5V_I_Pin             GPIO_PIN_1
#define PS5V_I_GPIO_Port       GPIOC
#define RS3V3_I_Pin            GPIO_PIN_6
#define RS3V3_I_GPIO_Port      GPIOA
#define RS5V_I_Pin             GPIO_PIN_0
#define RS5V_I_GPIO_Port       GPIOC
#define UHF_I_Pin              GPIO_PIN_0
#define UHF_I_GPIO_Port        GPIOB
#define XB12V_I_Pin            GPIO_PIN_3
#define XB12V_I_GPIO_Port      GPIOA

/* ---------------------- OSCILLATOR PINS ---------------------- */
#define OSC32_IN_Pin           GPIO_PIN_14
#define OSC32_IN_GPIO_Port     GPIOC
#define OSC32_OUT_Pin          GPIO_PIN_15
#define OSC32_OUT_GPIO_Port    GPIOC
#define OSC_IN_Pin             GPIO_PIN_0
#define OSC_IN_GPIO_Port       GPIOH
#define OSC_OUT_Pin            GPIO_PIN_1
#define OSC_OUT_GPIO_Port      GPIOH

/* ---------------------- SPI PINS ---------------------- */
#define SPI1_MISO_Pin          GPIO_PIN_14
#define SPI1_MISO_GPIO_Port    GPIOE
#define SPI1_MOSI_Pin          GPIO_PIN_15
#define SPI1_MOSI_GPIO_Port    GPIOE
#define SPI1_SCK_Pin           GPIO_PIN_13
#define SPI1_SCK_GPIO_Port     GPIOE
#define SPI1_SS_Pin            GPIO_PIN_12
#define SPI1_SS_GPIO_Port      GPIOE
#define SPI3_MISO_Pin          GPIO_PIN_11
#define SPI3_MISO_GPIO_Port    GPIOC
#define SP13_MOSI_Pin          GPIO_PIN_12
#define SP13_MOSI_GPIO_Port    GPIOC
#define SP13_SCK_Pin           GPIO_PIN_10
#define SP13_SCK_GPIO_Port     GPIOC
#define SPI3_SS_Pin            GPIO_PIN_15
#define SPI3_SS_GPIO_Port      GPIOA

/* ---------------------- UART / RS485 ---------------------- */
#define RS4851_DE_Pin          GPIO_PIN_4
#define RS4851_DE_GPIO_Port    GPIOD
#define RS4851_RX_Pin          GPIO_PIN_6
#define RS4851_RX_GPIO_Port    GPIOD
#define RS4851_TX_Pin          GPIO_PIN_5
#define RS4851_TX_GPIO_Port    GPIOD
#define RS4852_DE_Pin          GPIO_PIN_12
#define RS4852_DE_GPIO_Port    GPIOD
#define RS4852_RX_Pin          GPIO_PIN_11
#define RS4852_RX_GPIO_Port    GPIOB
#define RS4852_TX_Pin          GPIO_PIN_10
#define RS4852_TX_GPIO_Port    GPIOB
#define USART1_RX_Pin          GPIO_PIN_10
#define USART1_RX_GPIO_Port    GPIOA
#define USART1_TX_Pin          GPIO_PIN_9
#define USART1_TX_GPIO_Port    GPIOA

/* ---------------------- I2C PINS ---------------------- */
#define I2C2_SCL_Pin           GPIO_PIN_13
#define I2C2_SCL_GPIO_Port     GPIOB
#define I2C2_SDA_Pin           GPIO_PIN_14
#define I2C2_SDA_GPIO_Port     GPIOB

/* ---------------------- SYNC / RESET / OTHER ---------------------- */
#define SYNC_PULSE_Pin         GPIO_PIN_1
#define SYNC_PULSE_GPIO_Port   GPIOD
#define SYNC_PULSE_EXTI_IRQn   EXTI1_IRQn
#define BOOT0_Pin              GPIO_PIN_3
#define BOOT0_GPIO_Port        GPIOH
#define SWCLK_Pin              GPIO_PIN_14
#define SWCLK_GPIO_Port        GPIOA
#define SWDIO_Pin              GPIO_PIN_13
#define SWDIO_GPIO_Port        GPIOA
#define SWO_Pin                GPIO_PIN_3
#define SWO_GPIO_Port          GPIOB
#define M_OUT_Pin              GPIO_PIN_0
#define M_OUT_GPIO_Port        GPIOA
#define S0_Pin                 GPIO_PIN_7
#define S0_GPIO_Port           GPIOC
#define S1_Pin                 GPIO_PIN_8
#define S1_GPIO_Port           GPIOC
#define S2_Pin                 GPIO_PIN_9
#define S2_GPIO_Port           GPIOC
#define SA2_I_Pin              GPIO_PIN_1
#define SA2_I_GPIO_Port        GPIOA
#define SA3_I_Pin              GPIO_PIN_2
#define SA3_I_GPIO_Port        GPIOA
#define SYNC_PULSE_TASK_DELAY_MS 10 //


#include "log.h"

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
