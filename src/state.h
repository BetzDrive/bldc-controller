#ifndef _STATE_H_
#define _STATE_H_

#include <stdint.h>

namespace motor_driver {

struct RegisteredResults {
	int32_t encoder_pos; 			// Encoder position in encoder units
	float encoder_vel; 				// Encoder velocity in encoder units/second
	float foc_q_current_avg; 		// Average FOC quadrature current in amperes
	float foc_d_current_avg;		// Average FOC direct current in amperes
};

struct RegisteredParameters {
	float foc_q_current_sp;			// FOC quadrature current setpoint in amperes
	float foc_d_current_sp;			// FOC direct current setpoint in amperes
};

struct ImmediateParameters {
	bool override_led_color;		// Override normal status LED behavior
	uint8_t led_red_intensity;		// Status LED red intensity
	uint8_t led_green_intensity;	// Status LED green intensity
	uint8_t led_blue_intensity;		// Status LED blue intensity
};

} // namespace motor_driver

#endif /* _STATE_H_ */
