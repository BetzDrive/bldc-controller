#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#include <stdint.h>
#include "hal.h"
#include "DRV8301.h"
#include "AS5047D.h"
#include "modbus/modbus.h"

extern SerialUSBDriver SDU1;

namespace motor_driver {

constexpr UARTDriver *rs485_uart_driver = &UARTD1;

extern const PWMConfig motor_pwm_config;

extern DRV8301 gate_driver;

extern const PWMConfig led_pwm_config;

extern AS5047D encoder;

void startPeripherals();

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue);

void setStatusLEDColor(uint32_t color);

void setRS485TransmitMode(bool transmit);

} // namespace motor_driver

#endif /* _PERIPHERALS_H_ */
