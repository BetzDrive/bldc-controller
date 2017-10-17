#ifndef _STATE_H_
#define _STATE_H_

#include <stdint.h>
#include <ch.h>

namespace motor_driver {

struct Results {
  int32_t encoder_pos;          // Encoder position in encoder units
  float encoder_vel;            // Encoder velocity in encoder units/second
  float foc_q_current_avg;      // Average FOC quadrature current in amperes
  float foc_d_current_avg;      // Average FOC direct current in amperes

  uint16_t encoder_angle;       // Encoder angle, wraps around
  uint16_t encoder_diag;
  float angle;                  // Encoder diagnostics

  float average_va;             // Average voltage on phase A, volts
  float average_vb;             // Average voltage on phase B, volts
  float average_vc;             // Average voltage on phase C, volts
  float average_vin;            // Average supply voltage, volts
  float average_ia;             // Average current into phase A, amperes
  float average_ib;             // Average current into phase B, amperes
  float average_ic;             // Average current into phase C, amperes

  uint16_t debug_u16;
  float debug_f;

  Results()
    : encoder_pos(0),
      encoder_vel(0),
      foc_q_current_avg(0),
      foc_d_current_avg(0),
      angle(0)  {}
};

struct Calibration {
  uint16_t encoder_zero;        // Phase-aligned encoder zero position
  uint8_t erevs_per_mrev;       // Electronic revolutions per mechanical revolution
  float winding_resistance;     // Motor winding resistance in ohms
  uint8_t flip_phases;          // Phases A, B, C are arranged in clockwise instead of ccw order
  float foc_kp_d;               // Proportional gain for FOC/d PI loop
  float foc_ki_d;               // Integral gain for FOC/d PI loop
  float foc_kp_q;               // Proportional gain for FOC/q PI loop
  float foc_ki_q;               // Integral gain for FOC/q PI loop

  Calibration()
    : encoder_zero(0),
      erevs_per_mrev(1),
      winding_resistance(17.8f),// GBM110-150T
      flip_phases(false),
      foc_kp_d(0.01f),
      foc_ki_d(100.0f),
      foc_kp_q(0.01f),
      foc_ki_q(100.0f)
  {}
};

struct Parameters {
  float foc_q_current_sp;       // FOC quadrature current setpoint in amperes
  float foc_d_current_sp;       // FOC direct current setpoint in amperes
  bool override_led_color;      // Override normal status LED behavior
  uint8_t led_red_intensity;    // Status LED red intensity
  uint8_t led_green_intensity;  // Status LED green intensity
  uint8_t led_blue_intensity;   // Status LED blue intensity
  float cmd_duty_cycle;         // Duty cycle command
  uint8_t raw_pwm_mode;
  float phase0;
  float phase1;
  float phase2;

  Parameters()
    : foc_q_current_sp(0),
      foc_d_current_sp(0),
      override_led_color(false),
      led_red_intensity(0),
      led_green_intensity(0),
      led_blue_intensity(0),
      cmd_duty_cycle(0),
      raw_pwm_mode(1),
      phase0(0),
      phase1(0),
      phase2(0){}
};

/**
 * Result values written by the control thread
 */
extern Results results;

/**
 * Calibration values
 *
 */
extern Calibration calibration;

/**
 * Parameter values written by the comms thread
 */
extern Parameters parameters;

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
