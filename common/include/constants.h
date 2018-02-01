#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>
#include <stddef.h>

namespace motor_driver {

constexpr float pi = 3.1415927410125732421875f;
constexpr float sqrt3_div_2 = 0.866025388240814208984375f;
constexpr float one_div_sqrt3 = 0.57735025882720947265625f;
constexpr float two_div_sqrt3 = 1.1547005176544189453125f;

extern const uint16_t led_gamma_table[];

constexpr uint32_t rs485_baud = 1000000;

constexpr uint16_t encoder_period = 1U << 14;

/* Need to flip the sign because encoder angle increases clockwise */
constexpr float encoder_angle_to_radians = -2.0f * pi / encoder_period;

constexpr unsigned int motor_pwm_clock_freq = 168000000; // Hz

constexpr unsigned int motor_pwm_cycle_freq = 40000; // Hz

constexpr float current_control_interval = 2.0f / motor_pwm_cycle_freq; // Current control runs every two PWM cycles

constexpr unsigned int ivsense_samples_per_cycle = 30;

constexpr size_t ivsense_sample_buf_depth = ivsense_samples_per_cycle * 2; // "double-buffering"

constexpr size_t ivsense_channel_count = 7; // 4 voltage channels, 3 current channels

constexpr size_t ivsense_channel_ia = 0;    // Phase A current channel index
constexpr size_t ivsense_channel_ib = 1;    // Phase B current channel index
constexpr size_t ivsense_channel_ic = 2;    // Phase C current channel index
constexpr size_t ivsense_channel_va = 3;    // Phase A voltage channel index
constexpr size_t ivsense_channel_vb = 4;    // Phase B voltage channel index
constexpr size_t ivsense_channel_vc = 5;    // Phase C voltage channel index
constexpr size_t ivsense_channel_vin = 6;   // Supply voltage channel index

constexpr float ivsense_voltage_ratio = (2.21e3f + 39.2e3f) / 2.21e3f;      // Ratio of actual voltage to ADC input voltage
constexpr float ivsense_current_shunt_value = 0.002f;                       // Current shunt resistor value, ohms
constexpr float ivsense_current_amp_gain = 50.0f;                           // Current shunt amplifier gain, V/V
constexpr float adc_vref_voltage = 3.3f;                                    // ADC reference voltage, volts
constexpr unsigned int adc_max_value = 1u << 12;                            // ADC maximum value

/* Maximum expected voltage measurement */
constexpr float ivsense_voltage_max = adc_vref_voltage * ivsense_voltage_ratio;

/* Maximum expected current measurement */
constexpr float ivsense_current_max = adc_vref_voltage / ivsense_current_amp_gain / ivsense_current_shunt_value;

/* Actual voltage per ADC count */
constexpr float ivsense_voltage_per_count = ivsense_voltage_max / adc_max_value;

/* Actual current per ADC count */
constexpr float ivsense_current_per_count = ivsense_current_max / adc_max_value;

constexpr size_t recorder_channel_count = 8;

constexpr size_t recorder_channel_ia = 0;           // Phase A current channel index
constexpr size_t recorder_channel_ib = 1;           // Phase B current channel index
constexpr size_t recorder_channel_ic = 2;           // Phase C current channel index
constexpr size_t recorder_channel_va = 3;           // Phase A voltage channel index
constexpr size_t recorder_channel_vb = 4;           // Phase B voltage channel index
constexpr size_t recorder_channel_vc = 5;           // Phase C voltage channel index
constexpr size_t recorder_channel_vin = 6;          // Supply voltage channel index
constexpr size_t recorder_channel_rotor_pos = 7;    // Rotor position channel index

/* Address of non-volatile parameters storage */
/* extern const void *nvparams_start; */

/* Length of non-volatile parameters storage */
constexpr size_t nvparams_len = 1u << 14; // 16 kiB

extern const uint8_t *board_id_ptr;

} // namespace motor_driver

#endif /* _CONSTANTS_H_ */
