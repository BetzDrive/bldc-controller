#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize LED state machine. */
void led_init(void);

/* Advance LED state machine (fade/blink patterns).
 * Called from the main loop. Returns true if LEDs were updated. */
bool led_step(void);

/* Record comms activity for the yellow activity LED. */
void led_set_comms_activity(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
