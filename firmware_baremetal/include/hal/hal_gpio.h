#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract GPIO operations. Implementations are platform-specific. */

void hal_gpio_set(uint32_t port_base, uint32_t pin);
void hal_gpio_clear(uint32_t port_base, uint32_t pin);
void hal_gpio_write(uint32_t port_base, uint32_t pin, bool value);
bool hal_gpio_read(uint32_t port_base, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
