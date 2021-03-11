#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#include <stdint.h>
#include "hal.h"

namespace motor_driver {
  namespace peripherals {

    constexpr UARTDriver *rs485_uart_driver = &UARTD1;

    extern const PWMConfig led_pwm_config;

    void startPeripherals();

    void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue);

    void setStatusLEDColor(uint32_t color);

    void setCommsActivityLED(bool on);

    void setRS485TransmitMode(bool transmit);

  } // namespace peripherals
} // namespace motor_driver

#endif /* _PERIPHERALS_H_ */
