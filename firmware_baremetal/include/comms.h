#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the comms protocol state machine. */
void comms_init(void);

/* Process up to max_bytes from the UART RX buffer.
 * Called from the main loop. Returns number of bytes consumed. */
size_t comms_step(uint32_t max_bytes);

/* Deferred action flags - checked by main loop after comms_step. */
bool comms_should_reset(void);
uint32_t comms_get_jump_addr(void);

/* Watchdog timeout flag (reported in response flags). */
void comms_set_watchdog_timeout(bool flag);

/* Watchdog reset flag (set at startup if WDG reset was detected). */
void comms_set_wdg_reset_flag(bool flag);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_H */
