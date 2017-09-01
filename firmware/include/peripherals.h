#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#include <stdint.h>
#include "hal.h"
#include "DRV8301.h"
#include "AS5047D.h"
#include "constants.h"

extern SerialUSBDriver SDU1;

namespace motor_driver {

constexpr UARTDriver *rs485_uart_driver = &UARTD1;

extern const PWMConfig motor_pwm_config;

extern DRV8301 gate_driver;

extern const PWMConfig led_pwm_config;

extern AS5047D encoder;

extern BinarySemaphore ivsense_adc_samples_bsem;

extern volatile adcsample_t *ivsense_adc_samples_ptr;

extern volatile size_t ivsense_adc_samples_count;

extern adcsample_t ivsense_sample_buf[ivsense_channel_count * ivsense_sample_buf_depth];

void initPeripherals();

void startPeripherals();

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue);

void setStatusLEDColor(uint32_t color);

void setRS485TransmitMode(bool transmit);

} // namespace motor_driver

#endif /* _PERIPHERALS_H_ */
