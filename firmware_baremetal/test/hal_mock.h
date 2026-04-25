#ifndef HAL_MOCK_H
#define HAL_MOCK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/hal_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Reset all mock state to defaults. Call before each test. */
void mock_reset_all(void);

/* ── Mock Timer ────────────────────────────────────────── */
void mock_timer_advance_ms(uint32_t ms);
void mock_timer_advance_us(uint32_t us);
void mock_timer_set_ms(uint32_t ms);
void mock_timer_set_us(uint32_t us);

/* ── Mock GPIO ─────────────────────────────────────────── */
#define MOCK_GPIO_MAX_PORTS 6  /* A=0, B=1, C=2, D=3, E=4, F=5 */
#define MOCK_GPIO_PINS_PER_PORT 16

void mock_gpio_set_input(uint32_t port, uint32_t pin, bool value);
bool mock_gpio_get_output(uint32_t port, uint32_t pin);

/* ── Mock UART ─────────────────────────────────────────── */
#define MOCK_UART_BUF_SIZE 1024

void mock_uart_inject_rx(const uint8_t *data, size_t len);
size_t mock_uart_read_tx(uint8_t *buf, size_t max_len);

/* ── Mock PWM ──────────────────────────────────────────── */
#define MOCK_PWM_LED_CHANNELS 3
#define MOCK_PWM_MOTOR_CHANNELS 3

uint16_t mock_pwm_get_led(uint8_t channel);
float mock_pwm_get_motor_duty(uint8_t channel);

/* ── Mock I2C (async) ──────────────────────────────────── */
#define MOCK_I2C_MAX_RESPONSES 16
#define MOCK_I2C_MAX_DATA 32

/* Pre-configure a response for a given addr+reg combo.
 * When hal_i2c_start_write_read is called with tx_buf[0]==reg,
 * the mock copies this data into rx_buf on completion. */
void mock_i2c_set_response(uint8_t addr, uint8_t reg,
                           const uint8_t *data, size_t len);

/* Complete the pending async transfer (simulates ISR finishing).
 * Call this after hal_i2c_start_write/hal_i2c_start_write_read. */
void mock_i2c_complete(void);

/* Complete the pending transfer with an error. */
void mock_i2c_complete_with_error(hal_i2c_status_t err);

/* Check what was last written to an I2C address */
size_t mock_i2c_get_last_write(uint8_t addr, uint8_t *buf, size_t max_len);

/* ── Mock ADC ──────────────────────────────────────────── */
void mock_adc_set_samples(uint16_t ia, uint16_t ib, uint16_t ic, uint16_t vbus);

/* ── Mock SPI ──────────────────────────────────────────── */
void mock_spi_set_response(uint16_t value);
uint16_t mock_spi_get_last_tx(void);

/* ── Mock IWDG ─────────────────────────────────────────── */
int mock_iwdg_clear_reset_flag_count(void);

/* ── Mock Flash ────────────────────────────────────────── */
#define MOCK_FLASH_SIZE 32768
#define MOCK_FLASH_BASE 0x08008000U

uint8_t *mock_flash_get_memory(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_MOCK_H */
