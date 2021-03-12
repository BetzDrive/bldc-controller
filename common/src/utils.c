#include "utils.h"

void pwmSetChannelOutputMode(PWMDriver *pwm_driver, pwmchannel_t channel,
                             uint16_t output_mode) {
  switch (channel) {
  case 0:
    pwm_driver->tim->CCMR1 = ((pwm_driver->tim->CCMR1 & ~TIM_CCMR1_OC1M) |
                              STM32_TIM_CCMR1_OC1M(output_mode));
    break;
  case 1:
    pwm_driver->tim->CCMR1 = ((pwm_driver->tim->CCMR1 & ~TIM_CCMR1_OC2M) |
                              STM32_TIM_CCMR1_OC2M(output_mode));
    break;
  case 2:
    pwm_driver->tim->CCMR2 = ((pwm_driver->tim->CCMR2 & ~TIM_CCMR2_OC3M) |
                              STM32_TIM_CCMR2_OC3M(output_mode));
    break;
  case 3:
    pwm_driver->tim->CCMR2 = ((pwm_driver->tim->CCMR2 & ~TIM_CCMR2_OC4M) |
                              STM32_TIM_CCMR2_OC4M(output_mode));
    break;
  default:
    break;
  }
}

// FIXME
void *__dso_handle = 0;
void __cxa_pure_virtual(void) {
  while (1)
    ;
}
