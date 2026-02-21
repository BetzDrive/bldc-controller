#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

/* ── Module selection ─────────────────────────────────────────── */
#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED

/* ── Oscillator values ────────────────────────────────────────── */
#define HSE_VALUE            8000000U
#define HSE_STARTUP_TIMEOUT  100U
#define HSI_VALUE            16000000U
#define LSI_VALUE            32000U
#define LSE_VALUE            32768U
#define LSE_STARTUP_TIMEOUT  5000U
#define EXTERNAL_CLOCK_VALUE 12288000U

/* ── System configuration ─────────────────────────────────────── */
#define VDD_VALUE            3300U
#define TICK_INT_PRIORITY    0x0FU
#define USE_RTOS             0U
#define PREFETCH_ENABLE      1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE    1U

/* ── Ethernet (not used) ──────────────────────────────────────── */
#define MAC_ADDR0            2U
#define MAC_ADDR1            0U
#define MAC_ADDR2            0U
#define MAC_ADDR3            0U
#define MAC_ADDR4            0U
#define MAC_ADDR5            0U
#define DP83848_PHY_ADDRESS  0x01U

/* ── Module includes ──────────────────────────────────────────── */
#ifdef HAL_RCC_MODULE_ENABLED
 #include "stm32f4xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
 #include "stm32f4xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
 #include "stm32f4xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
 #include "stm32f4xx_hal_cortex.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
 #include "stm32f4xx_hal_adc.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
 #include "stm32f4xx_hal_flash.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif
#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif

/* ── Assert ───────────────────────────────────────────────────── */
#ifdef USE_FULL_ASSERT
 #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
 void assert_failed(uint8_t *file, uint32_t line);
#else
 #define assert_param(expr) ((void)0U)
#endif

#endif /* STM32F4XX_HAL_CONF_H */
