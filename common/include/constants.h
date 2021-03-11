#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>
#include <stddef.h>

namespace motor_driver {
namespace consts {

constexpr float pi = 3.1415927410125732421875f;
constexpr float sqrt3_div_2 = 0.866025388240814208984375f;
constexpr float one_div_sqrt3 = 0.57735025882720947265625f;
constexpr float two_div_sqrt3 = 1.1547005176544189453125f;

extern const uint16_t led_gamma_table[];

// How long the comms activity LED stays lit after a packet, in milliseconds.
constexpr unsigned int comms_activity_led_duration = 25;

constexpr uint32_t rs485_baud = 1000000;

constexpr uint16_t encoder_period = 1U << 14;

// Need to flip the sign because encoder position increases clockwise.
constexpr float rad_per_enc_tick = -2.0f * pi / encoder_period;

constexpr unsigned int motor_pwm_clock_freq = 168000000; // Hz

constexpr unsigned int motor_pwm_cycle_freq = 40000; // Hz

// TIM3 can't handle the full 168MHz like TIM1
constexpr unsigned int adc_pwm_cycle_freq = motor_pwm_clock_freq/2; // Hz

// Current control should be a factor of the motor_pwm_cycle_freq
constexpr unsigned int current_control_freq = motor_pwm_cycle_freq / 2;
constexpr uint8_t vel_divider = 5;
constexpr unsigned int velocity_control_freq = current_control_freq / vel_divider;
constexpr uint8_t pos_divider = 5;
constexpr unsigned int position_control_freq = current_control_freq / pos_divider;

constexpr float current_control_interval = 1.0f / current_control_freq;
constexpr float velocity_control_interval = 1.0f / velocity_control_freq;
constexpr float position_control_interval = 1.0f / position_control_freq;

constexpr float position_control_alpha = 0.076f;

constexpr unsigned int current_control_count_per_motor_cycle = motor_pwm_cycle_freq / current_control_freq;

constexpr size_t ivsense_rolling_average_count = 5;

constexpr unsigned int ivsense_samples_per_cycle = 1;

constexpr size_t ivsense_sample_buf_depth = ivsense_samples_per_cycle * 2; // "double-buffering"

constexpr size_t ivsense_channel_count = 4; // 1 voltage channel, 3 current channels

constexpr size_t ivsense_channel_ia = 0;    // Phase A current channel index
constexpr size_t ivsense_channel_ib = 1;    // Phase B current channel index
constexpr size_t ivsense_channel_ic = 2;    // Phase C current channel index
constexpr size_t ivsense_channel_vin = 3;   // Supply voltage channel index


/*******      General ADC constants      *******/
// ADC reference voltage, volts.
constexpr float adc_vref_voltage = 3.3f;
// ADC maximum value.
constexpr unsigned int adc_max_value = 1u << 12;
constexpr float adc_v_per_count = adc_vref_voltage / adc_max_value;

/***********************************************/
/******* Current gain constants (isense) *******/

// Gain provided by the INA250A1.
constexpr float isense_v_per_amp = 0.200f;
constexpr float isense_amp_per_v = 1.0f/isense_v_per_amp;
// Voltage offset post-gain (to handle negative currents).
constexpr float isense_v_offset = 1.650f;
// Actual current per ADC count.
constexpr float isense_current_per_count = adc_v_per_count * isense_amp_per_v;
// Maximum expected current measurement.
constexpr float isense_current_max = (adc_vref_voltage - isense_v_offset) * isense_amp_per_v;
// Minimum expected current measurement.
constexpr float ivsense_current_min = (-isense_v_offset) * isense_amp_per_v;
// Voltage at zero current.
constexpr float isense_voltage_zero_current = isense_v_offset;
// ADC Value zero current is centered on.
constexpr float isense_count_zero_current = isense_voltage_zero_current / adc_v_per_count;

/*******  End of current gain constants  *******/
/***********************************************/

/***********************************************/
/******* Voltage gain constants (vsense) *******/

// Ratio of actual voltage to ADC input voltage
constexpr float vsense_voltage_ratio = (2.21e3f + 49.9e3f) / 2.21e3f;

// Maximum expected voltage measurement.
constexpr float vsense_voltage_max = adc_vref_voltage * vsense_voltage_ratio;

// Actual voltage per ADC count.
constexpr float vsense_voltage_per_count = vsense_voltage_max / adc_max_value;

/*******  End of voltage gain constants  *******/
/***********************************************/

constexpr size_t recorder_channel_count = 8;

constexpr size_t recorder_channel_ia = 0;         // Phase A current channel index
constexpr size_t recorder_channel_ib = 1;         // Phase B current channel index
constexpr size_t recorder_channel_ic = 2;         // Phase C current channel index
constexpr size_t recorder_channel_vin = 3;        // Supply voltage channel index
constexpr size_t recorder_channel_rotor_pos = 4;  // Rotor position channel index
constexpr size_t recorder_channel_rotor_vel = 5;  // Rotor velocity channel index
constexpr size_t recorder_channel_ex1 = 6;        // Extra 1 channel index
constexpr size_t recorder_channel_ex2 = 7;        // Extra 2 channel index

constexpr size_t recorder_max_samples = 2000;

// Control modes.
constexpr uint8_t control_mode_foc_current = 0;
constexpr uint8_t control_mode_raw_phase_pwm = 1;
constexpr uint8_t control_mode_torque = 2;
constexpr uint8_t control_mode_velocity = 3;
constexpr uint8_t control_mode_position = 4;
constexpr uint8_t control_mode_position_velocity = 5;
constexpr uint8_t control_mode_position_feed_forward = 6;
constexpr uint8_t control_mode_pwm_drive = 7;

// Maximum duty cycle before gate on-time interferes with ADC sampling.
constexpr float max_duty_cycle (0.95f);
// Minimum duty cycle before gate off-time interferes with ADC sampling.
constexpr float min_duty_cycle (0.05f);

// Encoder driver selection.
constexpr uint8_t encoder_mode_none = 0;
constexpr uint8_t encoder_mode_as5047d = 1;
constexpr uint8_t encoder_mode_mlx90363 = 2;
constexpr uint8_t encoder_mode_aeat6600 = 3;

// Encoder angle correction.
constexpr size_t enc_ang_corr_table_size = 257;

// Addresses of memory sections in flash.
extern const uint8_t *board_id_ptr;
extern const void *calibration_ptr;
extern const void *firmware_ptr;

// Length of non-volatile parameters storage.
constexpr size_t nvparams_len = 1u << 14; // 16 kiB

// Unique constant that identifies a valid calibration stored in memory.
constexpr uint16_t calib_ss = 0x5454;

} // namespace consts
} // namespace motor_driver

#endif // _CONSTANTS_H_.
