#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract timing interface. */

/* Microsecond timestamp (wraps at 2^32). */
uint32_t hal_timer_usec(void);

/* Millisecond timestamp (wraps at 2^32). */
uint32_t hal_timer_msec(void);

/* Delay (blocking). */
void hal_timer_delay_us(uint32_t us);
void hal_timer_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* HAL_TIMER_H */
