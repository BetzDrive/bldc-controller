#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Async I2C interface for sensor reads.
 * All transfers are non-blocking. The caller starts a transfer then polls
 * hal_i2c_busy() each main-loop iteration. When busy returns false,
 * hal_i2c_error() indicates success/failure and rx_buf contains the data. */

typedef enum {
    HAL_I2C_OK = 0,
    HAL_I2C_ERR_NACK,
    HAL_I2C_ERR_BUS,
    HAL_I2C_ERR_TIMEOUT,
    HAL_I2C_ERR_DMA,
    HAL_I2C_ERR_BUSY,
} hal_i2c_status_t;

void hal_i2c_init(void);

/* Start a write-then-read transaction (repeated START).
 * Returns false if a transfer is already in progress. */
bool hal_i2c_start_write_read(uint8_t addr,
                               const uint8_t *tx_buf, size_t tx_len,
                               uint8_t *rx_buf, size_t rx_len);

/* Start a write-only transaction.
 * Returns false if a transfer is already in progress. */
bool hal_i2c_start_write(uint8_t addr,
                          const uint8_t *tx_buf, size_t tx_len);

/* Returns true while a transfer is in progress. */
bool hal_i2c_busy(void);

/* Returns the status of the last completed transfer. */
hal_i2c_status_t hal_i2c_error(void);

/* Returns the raw internal I2C driver state (I2CState enum value).
 * 0=IDLE 1=START_SENT 2=ADDR_W_SENT 3=TX_DATA 4=RESTART_SENT
 * 5=ADDR_R_SENT 6=RX_DMA 7=COMPLETE 8=ERROR */
uint8_t hal_i2c_raw_state(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_I2C_H */
