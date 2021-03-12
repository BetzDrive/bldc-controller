#ifndef _STATE_H_
#define _STATE_H_

#include "Recorder.h"
#include "ch.h"
#include "constants.h"

#include <stdint.h>

namespace motor_driver {
namespace state {

struct Results {
  // Measured FOC direct current (amperes).
  float foc_d_current = 0;
  // Measured FOC quadrature current (amperes).
  float foc_q_current = 0;
  // Measured FOC direct voltage (volts).
  float foc_d_voltage = 0;
  // Measured FOC quadrature voltage (volts).
  float foc_q_voltage = 0;

  // id output from PID loop to motor.
  float id_output = 0;
  // iq output from PID loop to motor.
  float iq_output = 0;

  // Calculated duty cycle for phase A.
  float duty_a = 0;
  // Calculated duty cycle for phase B.
  float duty_b = 0;
  // Calculated duty cycle for phase C.
  float duty_c = 0;

  // Encoder mode.
  uint8_t encoder_mode = consts::encoder_mode_none;
  // Raw encoder value, wraps around.
  uint16_t raw_enc_value = 0;
  // Corrected encoder position, wraps around (radians).
  float enc_pos = 0;
  // Encoder diagnostics.
  uint32_t encoder_diag = 0;

  // Total number of rotor revolutions.
  int16_t rotor_revs = 0;
  // Rotor position (radians).
  float rotor_pos = 0;
  // Rotor velocity High Frequency Estimate (radians/second).
  float hf_rotor_vel = 0;
  // Rotor velocity Low Frequency Estimate (radians/second).
  float lf_rotor_vel = 0;

  // voltage on phase A (volts).
  float va = 0;
  // voltage on phase B (volts).
  float vb = 0;
  // voltage on phase C (volts).
  float vc = 0;
  // supply voltage (volts).
  float vin = 0;
  // current into phase A (amperes).
  float ia = 0;
  // current into phase B (amperes).
  float ib = 0;
  // current into phase C (amperes).
  float ic = 0;

  // X-acceleration in milli-g's.
  int16_t xl_x = 0;
  // Y-acceleration in milli-g's.
  int16_t xl_y = 0;
  // Z-acceleration in milli-g's.
  int16_t xl_z = 0;

  // Temperature in degrees Celsius.
  float temperature = 0;

  Results() {}
};

struct Calibration {
  // Start sequence to determine whether this is a valid calibration.
  uint16_t start_sequence = consts::calib_ss;
  // Encoder reading at the start of an electrical revolution.
  uint16_t erev_start = 0;
  // Electrical revolutions per mechanical revolution.
  uint8_t erevs_per_mrev = 1;
  // Phases A, B, C are arranged in clockwise instead of ccw order.
  uint8_t flip_phases = false;
  // Proportional gain for FOC/d PI loop.
  float foc_kp_d = 0.5f;
  // Integral gain for FOC/d PI loop.
  float foc_ki_d = 0.1f;
  // Proportional gain for FOC/q PI loop.
  float foc_kp_q = 1.0f;
  // Integral gain for FOC/q PI loop.
  float foc_ki_q = 0.2f;
  // Proportional gain for velocity PI loop.
  float velocity_kp = 0.1f;
  // Integral gain for velocity PI loop.
  float velocity_kd = 1e-3f;
  // Proportional gain for position PI loop.
  float position_kp = 5.0f;
  // Integral gain for position PI loop.
  float position_kd = 0.0f;
  // Current limit (A).
  float current_limit = 2.0f;
  // Torque limit (N*m).
  float torque_limit = 3.0f;
  // Velocity limit (rad/s).
  float velocity_limit = 10.0f;
  // Position lower limit (rad).
  float position_lower_limit = 0.0f;
  // Position upper limit (rad).
  float position_upper_limit = 0.0f;
  // Motor resistance (ohm).
  float motor_resistance = 17.8f;
  // Motor inductance (henries).
  float motor_inductance = 0.0f;
  // Motor torque constant (newton-meters per ampere).
  float motor_torque_const = 0.0f;
  // Control timeout (ms).
  uint16_t control_timeout = 0;
  // Parameter for high frequency velocity estimate.
  float hf_velocity_filter_param = 0.01f;
  // Parameter for low frequency velocity estimate.
  float lf_velocity_filter_param = (1.0 - .9975);
  // Position offset.
  float position_offset = 0.0f;
  // Current Offset for Phase A.
  float ia_offset = 0.0f;
  // Current Offset for Phase B.
  float ib_offset = 0.0f;
  // Current Offset for Phase C.
  float ic_offset = 0.0f;
  // Encoder angle correction scale (rad).
  float enc_ang_corr_scale = 0.0f;
  // Encoder angle correction offset (rad).
  float enc_ang_corr_offset = 0.0f;
  // Encoder angle correction table values.
  int8_t enc_ang_corr_table_values[consts::enc_ang_corr_table_size];

  Calibration() {}
};

struct Parameters {
  // Control mode.
  uint8_t control_mode = consts::control_mode_foc_current;
  // FOC quadrature current setpoint in amperes.
  float foc_q_current_sp = 0.0f;
  // FOC direct current setpoint in amperes.
  float foc_d_current_sp = 0.0f;
  // Override normal status LED behavior.
  bool override_led_color = false;
  // Status LED red intensity.
  uint8_t led_red_intensity = 0;
  // Status LED green intensity.
  uint8_t led_green_intensity = 0;
  // Status LED blue intensity.
  uint8_t led_blue_intensity = 0;

  // Phase 0 duty cycle.
  float phase0 = 0.0f;
  // Phase 1 duty cycle.
  float phase1 = 0.0f;
  // Phase 2 duty cycle.
  float phase2 = 0.0f;

  // Torque control setpoint (N*m).
  float torque_sp = 0.0f;
  // Velocity control setpoint (rad/s).
  float velocity_sp = 0.0f;
  // Position control setpoint (rad).
  float position_sp = 0.0f;
  // Feed forward term for load compensation (A).
  float feed_forward = 0.0f;
  float pwm_drive = 0.0f;

  // Flag for whether the gates are active or not.
  bool gate_active = false;
  // Flag for whether the gate has a fault.
  bool gate_fault = false;

  bool timeout_flag = false;
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

void storeCalibration();

void loadCalibration();

void clearCalibration();

} // namespace state
} // namespace motor_driver

#endif /* _STATE_H_ */
