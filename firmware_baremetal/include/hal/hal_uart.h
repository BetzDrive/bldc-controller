#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract UART interface with circular buffer semantics.
 * The implementation handles DMA setup and buffer management. */

void hal_uart_init(uint32_t baud);

/* RX operations - read from the DMA circular receive buffer */
size_t hal_uart_rx_available(void);
size_t hal_uart_rx_read(uint8_t *buf, size_t max_len);
bool hal_uart_rx_peek(uint8_t *out);
void hal_uart_rx_consume(size_t count);

/* TX operations - enqueue data into transmit buffer */
bool hal_uart_tx_send(const uint8_t *data, size_t len);
bool hal_uart_tx_busy(void);
void hal_uart_tx_wait_complete(void);

/* RS485 direction control */
void hal_uart_set_tx_mode(bool transmit);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
