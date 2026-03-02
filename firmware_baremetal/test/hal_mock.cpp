/*
 * Mock HAL implementations for unit testing.
 * All logic modules link against this instead of hal_stm32.
 */

#include "hal_mock.h"

#include <string.h>

#include "hal/hal_adc.h"
#include "hal/hal_flash.h"
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "hal/hal_iwdg.h"
#include "hal/hal_pwm.h"
#include "hal/hal_spi.h"
#include "hal/hal_timer.h"
#include "hal/hal_uart.h"

/* ════════════════════════════════════════════════════════════
 *  Timer Mock
 * ════════════════════════════════════════════════════════════ */

static uint32_t mock_time_us;
static uint32_t mock_time_ms;

void mock_timer_advance_ms(uint32_t ms) {
    mock_time_ms += ms;
    mock_time_us += ms * 1000;
}

void mock_timer_advance_us(uint32_t us) {
    mock_time_us += us;
    mock_time_ms += us / 1000;
}

void mock_timer_set_ms(uint32_t ms) {
    mock_time_ms = ms;
    mock_time_us = ms * 1000;
}

void mock_timer_set_us(uint32_t us) {
    mock_time_us = us;
    mock_time_ms = us / 1000;
}

extern "C" uint32_t hal_timer_usec(void) { return mock_time_us; }
extern "C" uint32_t hal_timer_msec(void) { return mock_time_ms; }
extern "C" void hal_timer_delay_us(uint32_t us) { mock_timer_advance_us(us); }
extern "C" void hal_timer_delay_ms(uint32_t ms) { mock_timer_advance_ms(ms); }

/* ════════════════════════════════════════════════════════════
 *  GPIO Mock
 * ════════════════════════════════════════════════════════════ */

/* Use port_base value to index: GPIOA=0x40020000, GPIOB=0x40020400, etc.
 * We store both input and output state separately. */
static bool mock_gpio_inputs[MOCK_GPIO_MAX_PORTS][MOCK_GPIO_PINS_PER_PORT];
static bool mock_gpio_outputs[MOCK_GPIO_MAX_PORTS][MOCK_GPIO_PINS_PER_PORT];

static int port_index(uint32_t port_base) {
    /* STM32F4 GPIO ports are at 0x4002_0000 + 0x400 * n */
    return (int)((port_base - 0x40020000U) / 0x400U);
}

void mock_gpio_set_input(uint32_t port, uint32_t pin, bool value) {
    int idx = port_index(port);
    if (idx >= 0 && idx < MOCK_GPIO_MAX_PORTS && pin < MOCK_GPIO_PINS_PER_PORT)
        mock_gpio_inputs[idx][pin] = value;
}

bool mock_gpio_get_output(uint32_t port, uint32_t pin) {
    int idx = port_index(port);
    if (idx >= 0 && idx < MOCK_GPIO_MAX_PORTS && pin < MOCK_GPIO_PINS_PER_PORT)
        return mock_gpio_outputs[idx][pin];
    return false;
}

extern "C" void hal_gpio_set(uint32_t port_base, uint32_t pin) {
    int idx = port_index(port_base);
    if (idx >= 0 && idx < MOCK_GPIO_MAX_PORTS && pin < MOCK_GPIO_PINS_PER_PORT)
        mock_gpio_outputs[idx][pin] = true;
}

extern "C" void hal_gpio_clear(uint32_t port_base, uint32_t pin) {
    int idx = port_index(port_base);
    if (idx >= 0 && idx < MOCK_GPIO_MAX_PORTS && pin < MOCK_GPIO_PINS_PER_PORT)
        mock_gpio_outputs[idx][pin] = false;
}

extern "C" void hal_gpio_write(uint32_t port_base, uint32_t pin, bool value) {
    if (value)
        hal_gpio_set(port_base, pin);
    else
        hal_gpio_clear(port_base, pin);
}

extern "C" bool hal_gpio_read(uint32_t port_base, uint32_t pin) {
    int idx = port_index(port_base);
    if (idx >= 0 && idx < MOCK_GPIO_MAX_PORTS && pin < MOCK_GPIO_PINS_PER_PORT)
        return mock_gpio_inputs[idx][pin];
    return false;
}

/* ════════════════════════════════════════════════════════════
 *  UART Mock
 * ════════════════════════════════════════════════════════════ */

static uint8_t mock_uart_rx_buf[MOCK_UART_BUF_SIZE];
static size_t mock_uart_rx_head;
static size_t mock_uart_rx_tail;

static uint8_t mock_uart_tx_buf[MOCK_UART_BUF_SIZE];
static size_t mock_uart_tx_count;

void mock_uart_inject_rx(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        size_t next = (mock_uart_rx_head + 1) % MOCK_UART_BUF_SIZE;
        if (next != mock_uart_rx_tail) {
            mock_uart_rx_buf[mock_uart_rx_head] = data[i];
            mock_uart_rx_head = next;
        }
    }
}

size_t mock_uart_read_tx(uint8_t *buf, size_t max_len) {
    size_t n = mock_uart_tx_count < max_len ? mock_uart_tx_count : max_len;
    memcpy(buf, mock_uart_tx_buf, n);
    if (n < mock_uart_tx_count)
        memmove(mock_uart_tx_buf, mock_uart_tx_buf + n, mock_uart_tx_count - n);
    mock_uart_tx_count -= n;
    return n;
}

extern "C" void hal_uart_init(uint32_t baud) { (void)baud; }

extern "C" size_t hal_uart_rx_available(void) {
    return (mock_uart_rx_head + MOCK_UART_BUF_SIZE - mock_uart_rx_tail) %
           MOCK_UART_BUF_SIZE;
}

extern "C" size_t hal_uart_rx_read(uint8_t *buf, size_t max_len) {
    size_t avail = hal_uart_rx_available();
    size_t n = avail < max_len ? avail : max_len;
    for (size_t i = 0; i < n; i++) {
        buf[i] = mock_uart_rx_buf[mock_uart_rx_tail];
        mock_uart_rx_tail = (mock_uart_rx_tail + 1) % MOCK_UART_BUF_SIZE;
    }
    return n;
}

extern "C" bool hal_uart_rx_peek(uint8_t *out) {
    if (mock_uart_rx_head == mock_uart_rx_tail) return false;
    *out = mock_uart_rx_buf[mock_uart_rx_tail];
    return true;
}

extern "C" void hal_uart_rx_consume(size_t count) {
    for (size_t i = 0; i < count && mock_uart_rx_head != mock_uart_rx_tail; i++)
        mock_uart_rx_tail = (mock_uart_rx_tail + 1) % MOCK_UART_BUF_SIZE;
}

extern "C" bool hal_uart_tx_send(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (mock_uart_tx_count >= MOCK_UART_BUF_SIZE) return false;
        mock_uart_tx_buf[mock_uart_tx_count++] = data[i];
    }
    return true;
}

extern "C" bool hal_uart_tx_busy(void) { return false; }

extern "C" void hal_uart_tx_wait_complete(void) {}

extern "C" void hal_uart_set_tx_mode(bool transmit) { (void)transmit; }

extern "C" void hal_uart_rx_debug(uint32_t *write_pos, uint32_t *read_pos,
                                   uint32_t *ndtr_val, uint32_t *buf_size) {
    *write_pos = 0;
    *read_pos = 0;
    *ndtr_val = 0;
    *buf_size = MOCK_UART_BUF_SIZE;
}

/* ════════════════════════════════════════════════════════════
 *  PWM Mock
 * ════════════════════════════════════════════════════════════ */

static uint16_t mock_pwm_led[MOCK_PWM_LED_CHANNELS];
static float mock_pwm_motor[MOCK_PWM_MOTOR_CHANNELS];
static hal_pwm_motor_callback_t mock_motor_cb;

uint16_t mock_pwm_get_led(uint8_t channel) {
    return (channel < MOCK_PWM_LED_CHANNELS) ? mock_pwm_led[channel] : 0;
}

float mock_pwm_get_motor_duty(uint8_t channel) {
    return (channel < MOCK_PWM_MOTOR_CHANNELS) ? mock_pwm_motor[channel] : 0.0f;
}

extern "C" void hal_pwm_motor_init(void) {}
extern "C" void hal_pwm_motor_start(void) {}

extern "C" void hal_pwm_motor_set_duty(uint8_t channel, float duty) {
    if (channel < MOCK_PWM_MOTOR_CHANNELS)
        mock_pwm_motor[channel] = duty;
}

extern "C" void hal_pwm_motor_enable_channel(uint8_t channel) { (void)channel; }
extern "C" void hal_pwm_motor_disable_channel(uint8_t channel) { (void)channel; }
extern "C" void hal_pwm_led_init(void) {}

extern "C" void hal_pwm_led_set(uint8_t channel, uint16_t pulse_width) {
    if (channel < MOCK_PWM_LED_CHANNELS)
        mock_pwm_led[channel] = pulse_width;
}

extern "C" void hal_pwm_motor_set_callback(hal_pwm_motor_callback_t cb) {
    mock_motor_cb = cb;
}

/* ════════════════════════════════════════════════════════════
 *  I2C Mock (async)
 * ════════════════════════════════════════════════════════════ */

struct MockI2CResponse {
    uint8_t addr;
    uint8_t reg;
    uint8_t data[MOCK_I2C_MAX_DATA];
    size_t len;
    bool active;
};

static struct MockI2CResponse mock_i2c_responses[MOCK_I2C_MAX_RESPONSES];

static uint8_t mock_i2c_last_write_addr;
static uint8_t mock_i2c_last_write_data[MOCK_I2C_MAX_DATA];
static size_t mock_i2c_last_write_len;

/* Async state */
static bool mock_i2c_is_busy;
static hal_i2c_status_t mock_i2c_last_error;

/* Pending transfer info for deferred completion */
static uint8_t mock_i2c_pending_addr;
static uint8_t mock_i2c_pending_reg;   /* tx_buf[0] */
static uint8_t *mock_i2c_pending_rx_buf;
static size_t mock_i2c_pending_rx_len;
static bool mock_i2c_pending_has_rx;

void mock_i2c_set_response(uint8_t addr, uint8_t reg,
                           const uint8_t *data, size_t len) {
    for (int i = 0; i < MOCK_I2C_MAX_RESPONSES; i++) {
        if (!mock_i2c_responses[i].active ||
            (mock_i2c_responses[i].addr == addr &&
             mock_i2c_responses[i].reg == reg)) {
            mock_i2c_responses[i].addr = addr;
            mock_i2c_responses[i].reg = reg;
            size_t copy_len = len < MOCK_I2C_MAX_DATA ? len : MOCK_I2C_MAX_DATA;
            memcpy(mock_i2c_responses[i].data, data, copy_len);
            mock_i2c_responses[i].len = copy_len;
            mock_i2c_responses[i].active = true;
            return;
        }
    }
}

void mock_i2c_complete(void) {
    if (!mock_i2c_is_busy) return;

    if (mock_i2c_pending_has_rx && mock_i2c_pending_rx_buf) {
        /* Look up the pre-configured response and copy data */
        for (int i = 0; i < MOCK_I2C_MAX_RESPONSES; i++) {
            if (mock_i2c_responses[i].active &&
                mock_i2c_responses[i].addr == mock_i2c_pending_addr &&
                mock_i2c_responses[i].reg == mock_i2c_pending_reg) {
                size_t n = mock_i2c_responses[i].len < mock_i2c_pending_rx_len ?
                           mock_i2c_responses[i].len : mock_i2c_pending_rx_len;
                memcpy(mock_i2c_pending_rx_buf, mock_i2c_responses[i].data, n);
                mock_i2c_last_error = HAL_I2C_OK;
                mock_i2c_is_busy = false;
                return;
            }
        }
        /* No response configured: NACK */
        mock_i2c_last_error = HAL_I2C_ERR_NACK;
    } else {
        mock_i2c_last_error = HAL_I2C_OK;
    }
    mock_i2c_is_busy = false;
}

void mock_i2c_complete_with_error(hal_i2c_status_t err) {
    mock_i2c_last_error = err;
    mock_i2c_is_busy = false;
}

size_t mock_i2c_get_last_write(uint8_t addr, uint8_t *buf, size_t max_len) {
    if (mock_i2c_last_write_addr != addr) return 0;
    size_t n = mock_i2c_last_write_len < max_len ?
               mock_i2c_last_write_len : max_len;
    memcpy(buf, mock_i2c_last_write_data, n);
    return n;
}

extern "C" void hal_i2c_init(void) {}

extern "C" bool hal_i2c_start_write_read(uint8_t addr,
                                          const uint8_t *tx_buf, size_t tx_len,
                                          uint8_t *rx_buf, size_t rx_len) {
    if (mock_i2c_is_busy) return false;
    if (tx_len == 0) return false;

    /* Record the write */
    mock_i2c_last_write_addr = addr;
    size_t copy_len = tx_len < MOCK_I2C_MAX_DATA ? tx_len : MOCK_I2C_MAX_DATA;
    memcpy(mock_i2c_last_write_data, tx_buf, copy_len);
    mock_i2c_last_write_len = copy_len;

    /* Save pending transfer info */
    mock_i2c_pending_addr = addr;
    mock_i2c_pending_reg = tx_buf[0];
    mock_i2c_pending_rx_buf = rx_buf;
    mock_i2c_pending_rx_len = rx_len;
    mock_i2c_pending_has_rx = true;

    mock_i2c_is_busy = true;
    mock_i2c_last_error = HAL_I2C_OK;
    return true;
}

extern "C" bool hal_i2c_start_write(uint8_t addr,
                                     const uint8_t *tx_buf, size_t tx_len) {
    if (mock_i2c_is_busy) return false;

    mock_i2c_last_write_addr = addr;
    size_t copy_len = tx_len < MOCK_I2C_MAX_DATA ? tx_len : MOCK_I2C_MAX_DATA;
    memcpy(mock_i2c_last_write_data, tx_buf, copy_len);
    mock_i2c_last_write_len = copy_len;

    mock_i2c_pending_has_rx = false;
    mock_i2c_pending_rx_buf = NULL;
    mock_i2c_pending_rx_len = 0;

    mock_i2c_is_busy = true;
    mock_i2c_last_error = HAL_I2C_OK;
    return true;
}

extern "C" bool hal_i2c_busy(void) {
    return mock_i2c_is_busy;
}

extern "C" hal_i2c_status_t hal_i2c_error(void) {
    return mock_i2c_last_error;
}

/* ════════════════════════════════════════════════════════════
 *  ADC Mock
 * ════════════════════════════════════════════════════════════ */

static struct AdcSamples mock_adc_samples;

void mock_adc_set_samples(uint16_t ia, uint16_t ib, uint16_t ic,
                          uint16_t vbus) {
    mock_adc_samples.ia = ia;
    mock_adc_samples.ib = ib;
    mock_adc_samples.ic = ic;
    mock_adc_samples.vbus = vbus;
}

extern "C" void hal_adc_init(void) {}
extern "C" void hal_adc_start(void) {}
extern "C" struct AdcSamples hal_adc_get_latest(void) {
    return mock_adc_samples;
}

/* ════════════════════════════════════════════════════════════
 *  SPI Mock
 * ════════════════════════════════════════════════════════════ */

static uint16_t mock_spi_response;
static uint16_t mock_spi_last_tx;

void mock_spi_set_response(uint16_t value) { mock_spi_response = value; }
uint16_t mock_spi_get_last_tx(void) { return mock_spi_last_tx; }

extern "C" void hal_spi_init(void) {}
extern "C" uint16_t hal_spi_transfer16(uint16_t tx_data) {
    mock_spi_last_tx = tx_data;
    return mock_spi_response;
}

/* ════════════════════════════════════════════════════════════
 *  Flash Mock
 * ════════════════════════════════════════════════════════════ */

static uint8_t mock_flash_mem[MOCK_FLASH_SIZE];

uint8_t *mock_flash_get_memory(void) { return mock_flash_mem; }

extern "C" bool hal_flash_erase(uint32_t addr, size_t len) {
    if (addr < MOCK_FLASH_BASE) return false;
    uint32_t offset = addr - MOCK_FLASH_BASE;
    if (offset + len > MOCK_FLASH_SIZE) return false;
    memset(mock_flash_mem + offset, 0xFF, len);
    return true;
}

extern "C" bool hal_flash_write(uint32_t addr, const void *data, size_t len) {
    if (addr < MOCK_FLASH_BASE) return false;
    uint32_t offset = addr - MOCK_FLASH_BASE;
    if (offset + len > MOCK_FLASH_SIZE) return false;
    memcpy(mock_flash_mem + offset, data, len);
    return true;
}

extern "C" void hal_flash_read(uint32_t addr, void *buf, size_t len) {
    if (addr < MOCK_FLASH_BASE) {
        memset(buf, 0xFF, len);
        return;
    }
    uint32_t offset = addr - MOCK_FLASH_BASE;
    if (offset + len > MOCK_FLASH_SIZE) {
        memset(buf, 0xFF, len);
        return;
    }
    memcpy(buf, mock_flash_mem + offset, len);
}

extern "C" bool hal_flash_verify(uint32_t addr, const void *data, size_t len) {
    if (addr < MOCK_FLASH_BASE) return false;
    uint32_t offset = addr - MOCK_FLASH_BASE;
    if (offset + len > MOCK_FLASH_SIZE) return false;
    return memcmp(mock_flash_mem + offset, data, len) == 0;
}

extern "C" bool hal_flash_verify_erased(uint32_t addr, size_t len) {
    if (addr < MOCK_FLASH_BASE) return false;
    uint32_t offset = addr - MOCK_FLASH_BASE;
    if (offset + len > MOCK_FLASH_SIZE) return false;
    for (size_t i = 0; i < len; i++) {
        if (mock_flash_mem[offset + i] != 0xFF) return false;
    }
    return true;
}

extern "C" void hal_flash_jump(uint32_t addr) {
    (void)addr;
    /* No-op in tests - would normally not return */
    while (1) {}
}

/* ════════════════════════════════════════════════════════════
 *  IWDG Mock
 * ════════════════════════════════════════════════════════════ */

extern "C" void hal_iwdg_init(void) {}
extern "C" void hal_iwdg_kick(void) {}
extern "C" void hal_iwdg_pause(void) {}
extern "C" void hal_iwdg_resume(void) {}

/* ════════════════════════════════════════════════════════════
 *  Reset All
 * ════════════════════════════════════════════════════════════ */

void mock_reset_all(void) {
    mock_time_us = 0;
    mock_time_ms = 0;

    memset(mock_gpio_inputs, 0, sizeof(mock_gpio_inputs));
    memset(mock_gpio_outputs, 0, sizeof(mock_gpio_outputs));

    mock_uart_rx_head = 0;
    mock_uart_rx_tail = 0;
    mock_uart_tx_count = 0;

    memset(mock_pwm_led, 0, sizeof(mock_pwm_led));
    memset(mock_pwm_motor, 0, sizeof(mock_pwm_motor));

    memset(mock_i2c_responses, 0, sizeof(mock_i2c_responses));
    mock_i2c_last_write_addr = 0;
    mock_i2c_last_write_len = 0;
    mock_i2c_is_busy = false;
    mock_i2c_last_error = HAL_I2C_OK;
    mock_i2c_pending_rx_buf = NULL;
    mock_i2c_pending_rx_len = 0;
    mock_i2c_pending_has_rx = false;

    memset(&mock_adc_samples, 0, sizeof(mock_adc_samples));

    mock_spi_response = 0;
    mock_spi_last_tx = 0;

    memset(mock_flash_mem, 0xFF, sizeof(mock_flash_mem));
}
