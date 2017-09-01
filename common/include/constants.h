#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>
#include <stddef.h>

namespace motor_driver {

extern const uint16_t led_gamma_table[];

constexpr uint32_t rs485_baud = 115200;

constexpr unsigned int motor_pwm_clock_freq = 168000000; // Hz

constexpr unsigned int motor_pwm_cycle_freq = 80000; // Hz

constexpr unsigned int ivsense_samples_per_cycle = 30;

constexpr size_t ivsense_sample_buf_depth = ivsense_samples_per_cycle * 2; // "double-buffering"

constexpr size_t ivsense_channel_count = 7; // 3 current channels, 4 voltage channels

} // namespace motor_driver

#endif /* _CONSTANTS_H_ */
