#ifndef STM32F4XX_CONF_H_
#define STM32F4XX_CONF_H_

#define USE_RTOS 0

#include "misc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_wwdg.h"

#ifdef USE_FULL_ASSERT
#define assert_param(expr)                                                     \
  ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif

#endif // STM32F4XX_CONF_H_
