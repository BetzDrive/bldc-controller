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
  uint16_t encoder_diag;        // Encoder diagnostics

  Results()
    : encoder_pos(0),
      encoder_vel(0),
      foc_q_current_avg(0),
      foc_d_current_avg(0) {}
};

struct Parameters {
  float foc_q_current_sp;       // FOC quadrature current setpoint in amperes
  float foc_d_current_sp;       // FOC direct current setpoint in amperes
  bool override_led_color;      // Override normal status LED behavior
  uint8_t led_red_intensity;    // Status LED red intensity
  uint8_t led_green_intensity;  // Status LED green intensity
  uint8_t led_blue_intensity;   // Status LED blue intensity
  uint16_t encoder_zero;        // Phase-aligned encoder zero position
  uint8_t erpm_per_revolution;     // How many poles does our motor have?
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
      encoder_zero(0),
      erpm_per_revolution(8),
      raw_pwm_mode(0),
      phase0(0),
      phase1(0),
      phase2(0) {}
};

/**
 * Result values written by the control thread
 */
extern Results active_results;

/**
 * Result values read by the comms thread
 */
extern Results sync_results;

/**
 * Parameter values written by the comms thread
 */
extern Parameters active_parameters;

/**
 * Parameter values read by the control thread
 */
extern Parameters sync_parameters;

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
