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

  Parameters()
    : foc_q_current_sp(0),
      foc_d_current_sp(0),
      override_led_color(false),
      led_red_intensity(0),
      led_green_intensity(0),
      led_blue_intensity(0) {}
};

/**
 * Result values written by the control thread
 */
extern Results results;

/**
 * Result values read by the comms thread
 */
extern Results sync_results;

/**
 * Parameter values written by the comms thread
 */
extern Parameters parameters;

/**
 * Parameter values read by the control thread
 */
extern Parameters sync_parameters;

extern Mutex state_mutex;

void initState();

} // namespace motor_driver

#endif /* _STATE_H_ */
