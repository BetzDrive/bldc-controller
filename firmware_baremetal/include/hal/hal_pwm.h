#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract PWM interface for motor and LED control. */

/* Motor PWM (TIM1) */
void hal_pwm_motor_init(void);
void hal_pwm_motor_start(void);
void hal_pwm_motor_set_duty(uint8_t channel, float duty);  /* 0.0 - 1.0 */
void hal_pwm_motor_enable_channel(uint8_t channel);
void hal_pwm_motor_disable_channel(uint8_t channel);

/* LED PWM (TIM5) */
void hal_pwm_led_init(void);
void hal_pwm_led_set(uint8_t channel, uint16_t pulse_width);

/* Register the callback invoked from the TIM1 update ISR. */
typedef void (*hal_pwm_motor_callback_t)(void);
void hal_pwm_motor_set_callback(hal_pwm_motor_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* HAL_PWM_H */
