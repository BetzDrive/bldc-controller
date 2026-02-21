#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract flash interface for calibration storage and bootloader support. */

bool hal_flash_erase(uint32_t addr, size_t len);
bool hal_flash_write(uint32_t addr, const void *data, size_t len);
void hal_flash_read(uint32_t addr, void *buf, size_t len);
bool hal_flash_verify(uint32_t addr, const void *data, size_t len);
bool hal_flash_verify_erased(uint32_t addr, size_t len);

/* Jump to an address (for bootloader). Does not return. */
void hal_flash_jump(uint32_t addr) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_H */
