#ifndef SYSTEM_H
#define SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize clock tree, flash, peripheral clocks, SysTick, DWT.
 * Must be called first before any other init. */
void system_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_H */
