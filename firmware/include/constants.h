#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>

namespace motor_driver {

extern const uint16_t led_gamma_table[];

constexpr uint32_t rs485_baud = 115200;

} // namespace motor_driver

#endif /* _CONSTANTS_H_ */
