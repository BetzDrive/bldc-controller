#ifndef BAREMETAL_CONFIG_H
#define BAREMETAL_CONFIG_H

/*
 * Baremetal firmware hardware configuration.
 * All pin assignments, clock values, DMA channels, buffer sizes, etc.
 * derived from board.h and mcuconf.h.
 */

#include <stdint.h>

#ifdef STM32F405xx
#include "stm32f4xx.h"

#else
/* Host-compatible GPIO port addresses for unit testing */
#define GPIOA 0x40020000U
#define GPIOB 0x40020400U
#define GPIOC 0x40020800U
#define GPIOD 0x40020C00U
#define GPIOE 0x40021000U
#endif

/* ──────────────────────────────────────────────
 *  Clock Configuration (from mcuconf.h)
 *  HSE 8MHz → PLL → SYSCLK 168MHz
 * ────────────────────────────────────────────── */
#define SYSCLK_HZ           168000000U
#define AHB_HZ              168000000U  /* HPRE = /1 */
#define APB1_HZ              42000000U  /* PPRE1 = /4 */
#define APB1_TIMER_HZ        84000000U  /* x2 because prescaler != 1 */
#define APB2_HZ              84000000U  /* PPRE2 = /2 */
#define APB2_TIMER_HZ       168000000U  /* x2 because prescaler != 1 */

#define HSE_HZ                8000000U
#define PLL_M                 4U
#define PLL_N                 168U
#define PLL_P                 2U
#define PLL_Q                 7U
#define FLASH_LATENCY         5U        /* 5 WS for 168MHz @ 3.3V */

/* ──────────────────────────────────────────────
 *  GPIO Pin Assignments (from board.h)
 * ────────────────────────────────────────────── */

/* LEDs (active low, PWM via TIM5) */
#define LED_G_PORT           GPIOA
#define LED_G_PIN            0U   /* TIM5_CH1 AF2 */
#define LED_B_PORT           GPIOA
#define LED_B_PIN            1U   /* TIM5_CH2 AF2 */
#define LED_R_PORT           GPIOA
#define LED_R_PIN            2U   /* TIM5_CH3 AF2 */
#define LED_Y_PORT           GPIOB
#define LED_Y_PIN            8U   /* Comms activity, GPIO push-pull */

/* Motor PWM (TIM1, active low) */
#define MDRV_PWM_C_PORT      GPIOA
#define MDRV_PWM_C_PIN       8U   /* TIM1_CH1 AF1 */
#define MDRV_PWM_B_PORT      GPIOA
#define MDRV_PWM_B_PIN       9U   /* TIM1_CH2 AF1 */
#define MDRV_PWM_A_PORT      GPIOA
#define MDRV_PWM_A_PIN       10U  /* TIM1_CH3 AF1 */

/* Gate Driver (DRV8312) */
#define MDRV_RST_A_PORT      GPIOC
#define MDRV_RST_A_PIN       7U
#define MDRV_RST_B_PORT      GPIOC
#define MDRV_RST_B_PIN       8U
#define MDRV_RST_C_PORT      GPIOC
#define MDRV_RST_C_PIN       9U
#define MDRV_NFAULT_PORT     GPIOC
#define MDRV_NFAULT_PIN      6U   /* Input, active low */
#define MDRV_NOCTW_PORT      GPIOB
#define MDRV_NOCTW_PIN       15U  /* Input, active low */

/* RS485 UART (USART1 on PB6/PB7 AF7) */
#define RS485_TX_PORT        GPIOB
#define RS485_TX_PIN         6U   /* USART1_TX AF7 */
#define RS485_RX_PORT        GPIOB
#define RS485_RX_PIN         7U   /* USART1_RX AF7 */
#define RS485_DIR_PORT       GPIOD
#define RS485_DIR_PIN        2U   /* Direction control, GPIO push-pull */
#define RS485_SHUNT_EN_PORT  GPIOA
#define RS485_SHUNT_EN_PIN   12U

/* SPI3 Encoder (AS5047D) on PC10/PC11/PC12 AF6, CS on PB9 */
#define SPI_CLK_PORT         GPIOC
#define SPI_CLK_PIN          10U  /* SPI3_SCK AF6 */
#define SPI_MISO_PORT        GPIOC
#define SPI_MISO_PIN         11U  /* SPI3_MISO AF6 */
#define SPI_MOSI_PORT        GPIOC
#define SPI_MOSI_PIN         12U  /* SPI3_MOSI AF6 */
#define ENC_CSN_PORT         GPIOB
#define ENC_CSN_PIN          9U   /* GPIO output, active low */

/* I2C2 (Accelerometer + Temperature) */
#define I2C_SCL_PORT         GPIOB
#define I2C_SCL_PIN          10U  /* I2C2_SCL AF4 */
#define I2C_SDA_PORT         GPIOB
#define I2C_SDA_PIN          11U  /* I2C2_SDA AF4 */
#define TEMP_INT_PORT        GPIOB
#define TEMP_INT_PIN         13U
#define IMU_INT1_PORT        GPIOB
#define IMU_INT1_PIN         14U

/* ADC Current/Voltage Sense */
#define ISENSE_C_PORT        GPIOC
#define ISENSE_C_PIN         0U   /* ADC123_IN10 */
#define ISENSE_B_PORT        GPIOC
#define ISENSE_B_PIN         1U   /* ADC123_IN11 */
#define ISENSE_A_PORT        GPIOC
#define ISENSE_A_PIN         2U   /* ADC123_IN12 */
#define VSENSE_VIN_PORT      GPIOC
#define VSENSE_VIN_PIN       3U   /* ADC123_IN13 */

/* Disco Bus */
#define DISCO_BUS_OUT_PORT   GPIOA
#define DISCO_BUS_OUT_PIN    15U
#define DISCO_BUS_IN_PORT    GPIOB
#define DISCO_BUS_IN_PIN     4U

/* ──────────────────────────────────────────────
 *  DMA Channel Assignments (from mcuconf.h)
 * ────────────────────────────────────────────── */

/* USART1: DMA2 */
#define USART1_RX_DMA_STREAM     DMA2_Stream5
#define USART1_RX_DMA_CHANNEL    4U  /* Channel 4 for USART1_RX */
#define USART1_TX_DMA_STREAM     DMA2_Stream7
#define USART1_TX_DMA_CHANNEL    4U  /* Channel 4 for USART1_TX */

/* ADC: DMA2 */
#define ADC1_DMA_STREAM          DMA2_Stream4
#define ADC1_DMA_CHANNEL         0U  /* Channel 0 for ADC1 */
#define ADC2_DMA_STREAM          DMA2_Stream2
#define ADC2_DMA_CHANNEL         1U  /* Channel 1 for ADC2 */
#define ADC3_DMA_STREAM          DMA2_Stream1
#define ADC3_DMA_CHANNEL         2U  /* Channel 2 for ADC3 */

/* SPI3: DMA1 */
#define SPI3_RX_DMA_STREAM      DMA1_Stream0
#define SPI3_RX_DMA_CHANNEL     0U  /* Channel 0 for SPI3_RX */
#define SPI3_TX_DMA_STREAM      DMA1_Stream5
#define SPI3_TX_DMA_CHANNEL     0U  /* Channel 0 for SPI3_TX */

/* I2C2: DMA1 */
#define I2C2_RX_DMA_STREAM      DMA1_Stream2
#define I2C2_RX_DMA_CHANNEL     7U  /* Channel 7 for I2C2_RX */
#define I2C2_TX_DMA_STREAM      DMA1_Stream7
#define I2C2_TX_DMA_CHANNEL     7U  /* Channel 7 for I2C2_TX */

/* ──────────────────────────────────────────────
 *  Buffer Sizes
 * ────────────────────────────────────────────── */
#define UART_RX_BUF_SIZE     512U
#define UART_TX_BUF_SIZE     512U
#define COMMS_MAX_BYTES_PER_STEP  32U

/* ──────────────────────────────────────────────
 *  NVIC Priorities (0=highest, 15=lowest)
 *  Lower number = higher priority.
 *  Matches existing ChibiOS config.
 * ────────────────────────────────────────────── */
#define NVIC_PRIO_USART1         3U
#define NVIC_PRIO_TIM1_UP        4U   /* Motor control loop - highest after UART */
#define NVIC_PRIO_I2C2           5U
#define NVIC_PRIO_ADC            6U
#define NVIC_PRIO_DMA_USART1_RX  6U
#define NVIC_PRIO_DMA_USART1_TX  6U
#define NVIC_PRIO_DMA_ADC        6U
#define NVIC_PRIO_TIM1_CC        7U
#define NVIC_PRIO_SPI3          10U

/* ──────────────────────────────────────────────
 *  Timing Constants
 * ────────────────────────────────────────────── */
#define SENSOR_POLL_MS       100U
#define LED_STEP_MS           10U
#define COMMS_IDLE_TIMEOUT_US 5000U
#define IWDG_TIMEOUT_MS       10U

/* ──────────────────────────────────────────────
 *  Flash Memory Map (matches bootloader)
 * ────────────────────────────────────────────── */
#define FLASH_BOARD_ID_ADDR  0x0800C000U  /* Sector 3 - must match bootloader */
#define FLASH_CALIB_ADDR     0x08008000U  /* Sector 2 - must match bootloader */
#define FLASH_FW_ADDR        0x08010000U

#endif /* BAREMETAL_CONFIG_H */
