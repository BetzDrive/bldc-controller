#pragma once

#include "hal.h"

namespace motor_driver {
namespace peripherals {

constexpr UARTDriver *rs485_uart_driver = &UARTD1;

} // namespace peripherals
} // namespace motor_driver
