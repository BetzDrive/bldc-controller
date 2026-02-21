#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract SPI interface for encoder reads. */

void hal_spi_init(void);

/* Blocking 16-bit SPI transfer with CS management. */
uint16_t hal_spi_transfer16(uint16_t tx_data);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H */
