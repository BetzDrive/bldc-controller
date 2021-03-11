#ifndef _UTILS_H_
#define _UTILS_H_

#include "ch.h"
#include "hal.h"

struct IOPin {
  ioportid_t port;
  int pin;
};

#ifdef __cplusplus
extern "C" {
#endif

  void pwmSetChannelOutputMode(PWMDriver *pwm_driver, pwmchannel_t channel,
      uint16_t output_mode);

#ifdef __cplusplus
}
#endif

#define STM32_TIM_CCMRX_OCXM_ACTIVE_HIGH    (6U)
#define STM32_TIM_CCMRX_OCXM_ACTIVE_LOW     (7U)

#define NS2RTT(nsec) (((halGetCounterFrequency() + 999999999UL) / 1000000000UL) * (nsec))

#endif /* _UTILS_H_ */
