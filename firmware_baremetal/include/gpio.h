#ifndef GPIO_H
#define GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Configure all GPIO pins per board.h assignments.
 * Must be called after system_init() (GPIO clocks enabled). */
void gpio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H */
