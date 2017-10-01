#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>
#include <stddef.h>

namespace motor_driver {

extern const uint16_t led_gamma_table[];

constexpr uint32_t rs485_baud = 1000000;

constexpr unsigned int motor_pwm_clock_freq = 168000000; // Hz

constexpr unsigned int motor_pwm_cycle_freq = 80000; // Hz

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
constexpr float ivsense_current_amp_gain = 50.0f;                           // Current shunt amplifier gain
constexpr float adc_vref_voltage = 3.3f;                                    // ADC reference voltage, volts
constexpr unsigned int adc_max_value = 1u << 12;                            // ADC maximum value

/* Actual voltage per ADC count */
constexpr float ivsense_voltage_per_count = adc_vref_voltage / adc_max_value * ivsense_voltage_ratio;

/* Actual current per ADC count */
constexpr float ivsense_current_per_count = adc_vref_voltage / adc_max_value / ivsense_current_amp_gain / ivsense_current_shunt_value;

} // namespace motor_driver

#endif /* _CONSTANTS_H_ */
