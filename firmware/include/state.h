#ifndef _STATE_H_
#define _STATE_H_

#include <stdint.h>
#include <ch.h>
#include "Recorder.h"

namespace motor_driver {

struct Results {
  int32_t encoder_pos = 0;      // Encoder position (encoder units)
  float encoder_vel = 0;        // Encoder velocity (encoder units/second)
  float foc_d_current = 0;      // Measured FOC direct current (amperes)
  float foc_q_current = 0;      // Measured FOC quadrature current (amperes)

  uint16_t encoder_angle = 0;   // Encoder angle, wraps around
  uint16_t encoder_diag = 0;

  float encoder_radian_angle = 0;
  int16_t encoder_revs = 0;

  float average_va = 0;         // Average voltage on phase A (volts)
  float average_vb = 0;         // Average voltage on phase B (volts)
  float average_vc = 0;         // Average voltage on phase C (volts)
  float average_vin = 0;        // Average supply voltage (volts)
  float average_ia = 0;         // Average current into phase A (amperes)
  float average_ib = 0;         // Average current into phase B (amperes)
  float average_ic = 0;         // Average current into phase C (amperes)

  int32_t xl_x = 0;             // X-acceleration in milli-g's
  int32_t xl_y = 0;             // Y-acceleration in milli-g's
  int32_t xl_z = 0;             // Z-acceleration in milli-g's

  float temp = 0;               // Temperature in degrees Celsius

  Results() {}
};

struct Calibration {
  uint16_t encoder_zero = 0;          // Phase-aligned encoder zero position
  uint8_t erevs_per_mrev = 1;         // Electrical revolutions per mechanical revolution
  uint8_t flip_phases = false;        // Phases A, B, C are arranged in clockwise instead of ccw order
  float foc_kp_d = 2.0f;              // Proportional gain for FOC/d PI loop
  float foc_ki_d = 0.0f;              // Integral gain for FOC/d PI loop
  float foc_kp_q = 2.0f;              // Proportional gain for FOC/q PI loop
  float foc_ki_q = 0.0f;              // Integral gain for FOC/q PI loop
  float sw_endstop_min = 0.0f;        // Software endstop minimum angle
  float sw_endstop_max = 0.0f;        // Software endstop maximum angle
  float sw_endstop_slope = 20.0f;     // Software endstop torque slope
  float motor_resistance = 17.8f;     // Motor resistance (ohm)
  float motor_inductance = 0.0f;      // Motor inductance (henries)
  float motor_vel_const = 1e3f;       // Motor velocity constant (rad/s/V)

  Calibration() {}
};

struct Parameters {
  float foc_q_current_sp = 0;         // FOC quadrature current setpoint in amperes
  float foc_d_current_sp = 0;         // FOC direct current setpoint in amperes
  bool override_led_color = false;    // Override normal status LED behavior
  uint8_t led_red_intensity = 0;      // Status LED red intensity
  uint8_t led_green_intensity = 0;    // Status LED green intensity
  uint8_t led_blue_intensity = 0;     // Status LED blue intensity
  float cmd_duty_cycle = 0;           // Duty cycle command
  uint8_t raw_pwm_mode = 1;
  float phase0 = 0;
  float phase1 = 0;
  float phase2 = 0;

  Parameters() {}
};

/**
 * Result values written by the control thread
 */
extern Results results;

/**
 * Calibration values
 */
extern Calibration calibration;

/**
 * Parameter values written by the comms thread
 */
extern Parameters parameters;

/**
 * Recorder
 */
extern Recorder recorder;

/**
 * Results synchronization was requested
 */
extern volatile bool should_copy_results;

/**
 * Parameter synchronization was requested
 */
extern volatile bool should_copy_parameters;

void initState();

} // namespace motor_driver

#endif /* _STATE_H_ */
