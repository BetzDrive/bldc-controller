#include "control.h"
#include "transforms.h"
#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include "fast_math.h"
#include "chprintf.h"
#include "SVM.h"
#include "pid.h"
#include "constants.h"

#include <cmath>

namespace motor_driver {
namespace controller {

static Thread *control_thread_ptr = nullptr;

static SVM modulator(SVMStrategy::MIDPOINT_CLAMP);

static PID pid_id(state::calibration.foc_kp_d, state::calibration.foc_ki_d, 0.0f, consts::current_control_interval);

static PID pid_iq(state::calibration.foc_kp_q, state::calibration.foc_ki_q, 0.0f, consts::current_control_interval);

static PID pid_velocity(state::calibration.velocity_kp, state::calibration.velocity_ki, 0.0f, consts::velocity_control_interval);

static PID pid_position(state::calibration.position_kp, state::calibration.position_ki, 0.0f, consts::position_control_interval);

static systime_t last_control_timeout_reset;

static const math::LFFlipType enc_ang_corr_periodicity_flips[] = {
  math::LFFlipType::NONE
};

static const math::LFPeriodicity enc_ang_corr_periodicity = {
  1,
  enc_ang_corr_periodicity_flips
};

static math::LUTFunction<int8_t> enc_ang_corr_table(0, 2 * consts::pi, state::calibration.enc_ang_corr_table_values, consts::enc_ang_corr_table_size, enc_ang_corr_periodicity);

static float getEncoderAngleCorrection(float raw_enc_pos) {
  if (state::calibration.enc_ang_corr_scale != 0.0f) {
    return enc_ang_corr_table(raw_enc_pos) * state::calibration.enc_ang_corr_scale + state::calibration.enc_ang_corr_offset;
  } else {
    return 0.0f;
  }
}

static float clamp(float val, float min, float max) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

static float Q_rsqrt( float number )
{
  const float x2 = number * 0.5F;
  const float threehalfs = 1.5F;

  union {
    float f;
    uint32_t i;
  } conv = {number}; // member 'f' set to value of 'number'.
  conv.i  = 0x5f3759df - ( conv.i >> 1 );
  conv.f  *= ( threehalfs - ( x2 * conv.f * conv.f ) );
  return conv.f;
}

void initControl() {
  pid_id.setLimits(-consts::ivsense_current_max, consts::ivsense_current_max);
  pid_iq.setLimits(-consts::ivsense_current_max, consts::ivsense_current_max);
  last_control_timeout_reset = chTimeNow();
}

void resumeInnerControlLoop() {
  if (control_thread_ptr != nullptr) {
    chSysLockFromIsr();
    chEvtSignalI(control_thread_ptr, (flagsmask_t)1);
    chSysUnlockFromIsr();
  }
}

void runInnerControlLoop() {
  control_thread_ptr = chThdSelf();

  chSysLock();

  // getPipelinedResultI requires startPipelinedAngleReadI to be called beforehand
  peripherals::encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  while (true) {
    /*
     * Wait for resumeInnerControlLoop to be called
     */
    chEvtWaitAny((flagsmask_t)1);

    // If there is no fault, enable the motors.
    if (!state::parameters.gate_active && !state::parameters.gate_fault) {
      peripherals::gate_driver.enableGates();
      state::parameters.gate_active = true;
      chThdSleepMicroseconds(500);
    }

    // If there is a fault, disable the motors.
    if (state::parameters.gate_active && state::parameters.gate_fault) {
      peripherals::gate_driver.disableGates();
      brakeMotor();
      state::parameters.gate_active = false;
      chThdSleepMicroseconds(500);
    }

    if (state::calibration.control_timeout != 0 && 
        (chTimeNow() - last_control_timeout_reset) >= MS2ST(state::calibration.control_timeout)) {
      brakeMotor();
    } 

    chMtxLock(&peripherals::var_access_mutex);

    estimateState();

    runPositionControl();

    runVelocityControl();

    runCurrentControl();

    chMtxUnlock();
  }
}

void estimateState() {
  /*
   * Get current encoder position and velocity
   */

  uint16_t raw_enc_value;

  chSysLock(); // Required for function calls with "I" suffix

  raw_enc_value = peripherals::encoder.getPipelinedRegisterReadResultI();
  peripherals::encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  state::results.raw_enc_value = raw_enc_value;

  float raw_enc_pos = raw_enc_value * consts::rad_per_enc_tick;
  float enc_pos = raw_enc_pos + getEncoderAngleCorrection(raw_enc_pos);

  float prev_enc_pos = state::results.enc_pos;
  state::results.enc_pos = enc_pos;

  float enc_pos_diff = enc_pos - prev_enc_pos;
  if (enc_pos_diff < -consts::pi) {
    state::results.rotor_revs += 1;
    enc_pos_diff += 2 * consts::pi; // Normalize to (-pi, pi) range
  } else if (enc_pos_diff > consts::pi) {
    state::results.rotor_revs -= 1;
    enc_pos_diff -= 2 * consts::pi; // Normalize to (-pi, pi) range
  }

  state::results.rotor_pos = enc_pos + state::results.rotor_revs * 2 * consts::pi - state::calibration.position_offset;

  float rotor_vel_update = enc_pos_diff * consts::current_control_freq;
  // High frequency estimate used for on-board commutation
  float hf_alpha = state::calibration.hf_velocity_filter_param;
  state::results.hf_rotor_vel = hf_alpha * rotor_vel_update + (1.0f - hf_alpha) * state::results.hf_rotor_vel;
  // Low frequency estimate sent to host
  float lf_alpha = state::calibration.lf_velocity_filter_param;
  state::results.lf_rotor_vel = lf_alpha * rotor_vel_update + (1.0f - lf_alpha) * state::results.lf_rotor_vel;

  /*
   * Calculate average voltages and currents
   * This will have odd behavior if the following conditions are not met:
   * 1) count starts at zero 
   * 2) average arrays are not initialized to zero
   */

  // Subtract old values before storing/adding new values
  // Start doing this after rolling over
  if (state::rolladc.vin[state::rolladc.count] != 0) {
    state::results.raw_average_ia  -= state::rolladc.ia [state::rolladc.count];
    state::results.raw_average_ib  -= state::rolladc.ib [state::rolladc.count];
    state::results.raw_average_ic  -= state::rolladc.ic [state::rolladc.count];
    state::results.raw_average_va  -= state::rolladc.va [state::rolladc.count];
    state::results.raw_average_vb  -= state::rolladc.vb [state::rolladc.count];
    state::results.raw_average_vc  -= state::rolladc.vc [state::rolladc.count];
    state::results.raw_average_vin -= state::rolladc.vin[state::rolladc.count];
  }

  state::rolladc.ia [state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_ia ];
  state::rolladc.ib [state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_ib ];
  state::rolladc.ic [state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_ic ];
  state::rolladc.va [state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_va ];
  state::rolladc.vb [state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_vb ];
  state::rolladc.vc [state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_vc ];
  state::rolladc.vin[state::rolladc.count] = peripherals::ivsense_adc_samples_ptr[consts::ivsense_channel_vin];

  // The new average is equal to the addition of the old value minus the last value.
  // For the first (ivsense_rolling_average_count) values, the average will be wrong.
  state::results.raw_average_ia  += state::rolladc.ia [state::rolladc.count];
  state::results.raw_average_ib  += state::rolladc.ib [state::rolladc.count];
  state::results.raw_average_ic  += state::rolladc.ic [state::rolladc.count];
  state::results.raw_average_va  += state::rolladc.va [state::rolladc.count];
  state::results.raw_average_vb  += state::rolladc.vb [state::rolladc.count];
  state::results.raw_average_vc  += state::rolladc.vc [state::rolladc.count];
  state::results.raw_average_vin += state::rolladc.vin[state::rolladc.count];

  state::rolladc.count = (state::rolladc.count + 1) % consts::ivsense_rolling_average_count;

  state::results.average_ia  = peripherals::adcValueToCurrent(state::results.raw_average_ia  / consts::ivsense_rolling_average_count);
  state::results.average_ib  = peripherals::adcValueToCurrent(state::results.raw_average_ib  / consts::ivsense_rolling_average_count);
  state::results.average_ic  = peripherals::adcValueToCurrent(state::results.raw_average_ic  / consts::ivsense_rolling_average_count);
  state::results.average_va  = peripherals::adcValueToVoltage(state::results.raw_average_va  / consts::ivsense_rolling_average_count);
  state::results.average_vb  = peripherals::adcValueToVoltage(state::results.raw_average_vb  / consts::ivsense_rolling_average_count);
  state::results.average_vc  = peripherals::adcValueToVoltage(state::results.raw_average_vc  / consts::ivsense_rolling_average_count);
  state::results.average_vin = peripherals::adcValueToVoltage(state::results.raw_average_vin / consts::ivsense_rolling_average_count);

  state::results.corrected_ia = state::results.average_ia - state::calibration.ia_offset;
  state::results.corrected_ib = state::results.average_ib - state::calibration.ib_offset;
  state::results.corrected_ic = state::results.average_ic - state::calibration.ic_offset;

  //if (results.duty_a > results.duty_b && results.duty_a > results.duty_c) {
  //  results.corrected_ia = -(results.corrected_ib + results.corrected_ic);
  //} else if (results.duty_b > results.duty_c) {
  //  results.corrected_ib = -(results.corrected_ia + results.corrected_ic);
  //} else {
  //  results.corrected_ic = -(results.corrected_ia + results.corrected_ib);
  //}

  /*
   * Record data
   */
  if (state::rolladc.count == 0) {
    float recorder_new_data[consts::recorder_channel_count];

    recorder_new_data[consts::recorder_channel_ia]        = state::results.corrected_ia;
    recorder_new_data[consts::recorder_channel_ib]        = state::results.corrected_ib;
    recorder_new_data[consts::recorder_channel_ic]        = state::results.corrected_ic;
    recorder_new_data[consts::recorder_channel_va]        = state::results.average_va;
    recorder_new_data[consts::recorder_channel_vb]        = state::results.average_vb;
    recorder_new_data[consts::recorder_channel_vc]        = state::results.average_vc;
    recorder_new_data[consts::recorder_channel_vin]       = state::results.average_vin;
    recorder_new_data[consts::recorder_channel_rotor_pos] = state::results.rotor_pos;
    recorder_new_data[consts::recorder_channel_rotor_vel] = state::results.hf_rotor_vel;
    recorder_new_data[consts::recorder_channel_ex1]       = state::results.foc_q_current;
    recorder_new_data[consts::recorder_channel_ex2]       = state::results.foc_d_current;

    state::recorder.recordSample(recorder_new_data);
  }
  
}

void runPositionControl() {
  if (state::parameters.control_mode == consts::control_mode_position || 
      state::parameters.control_mode == consts::control_mode_position_velocity) {
    pid_position.setGains(state::calibration.position_kp, state::calibration.position_ki, 0.0f);
    pid_position.setLimits(-state::calibration.velocity_limit, state::calibration.velocity_limit);
    pid_position.setTarget(state::parameters.position_sp);
    state::parameters.velocity_sp = pid_position.compute(state::results.rotor_pos);
  }
}

void runVelocityControl() {
  if (state::parameters.control_mode == consts::control_mode_velocity || 
      state::parameters.control_mode == consts::control_mode_position || 
      state::parameters.control_mode == consts::control_mode_position_velocity) {
    pid_velocity.setGains(state::calibration.velocity_kp, state::calibration.velocity_ki, 0.0f);
    // float velocity_max = results.average_vin / calibration.motor_torque_const;
    float velocity_max = 40.0f;
    pid_velocity.setLimits(-state::calibration.torque_limit, state::calibration.torque_limit);
    pid_velocity.setTarget(state::parameters.velocity_sp);
    state::parameters.torque_sp = pid_velocity.compute(state::results.hf_rotor_vel);
  }
}

void runCurrentControl() {
  /*
   * Compute phase duty cycles
   */

  if (state::parameters.control_mode == consts::control_mode_raw_phase_pwm) {
    /*
     * Directly set PWM duty cycles
     */

    peripherals::gate_driver.setPWMDutyCycle(0, state::parameters.phase0 * consts::max_duty_cycle);
    peripherals::gate_driver.setPWMDutyCycle(1, state::parameters.phase1 * consts::max_duty_cycle);
    peripherals::gate_driver.setPWMDutyCycle(2, state::parameters.phase2 * consts::max_duty_cycle);
  } else {
    /*
     * Run field-oriented control
     */
    float ialpha, ibeta;
    math::transformClarke(state::results.corrected_ia, 
                    state::results.corrected_ib, 
                    state::results.corrected_ic, ialpha, ibeta);

    if (state::calibration.flip_phases) {
      ibeta = -ibeta;
    }

    float mech_pos = state::results.enc_pos - state::calibration.erev_start * consts::rad_per_enc_tick;
    float elec_pos = mech_pos * state::calibration.erevs_per_mrev;

    float cos_theta = math::fast_cos(elec_pos);
    float sin_theta = math::fast_sin(elec_pos);

    float id, iq;
    math::transformPark(ialpha, ibeta, cos_theta, sin_theta, id, iq);

    pid_id.setGains(state::calibration.foc_kp_d, state::calibration.foc_ki_d, 0.0f);
    pid_iq.setGains(state::calibration.foc_kp_q, state::calibration.foc_ki_q, 0.0f);

    pid_id.setLimits(-state::calibration.current_limit, state::calibration.current_limit);
    pid_iq.setLimits(-state::calibration.current_limit, state::calibration.current_limit);

    float id_sp, iq_sp;
    if (state::parameters.control_mode == consts::control_mode_foc_current) {
      // Use the provided FOC current setpoints
      id_sp = state::parameters.foc_d_current_sp;
      iq_sp = state::parameters.foc_q_current_sp;
    } else {
      // Generate FOC current setpoints from the reference torque
      id_sp = 0.0f;
      iq_sp = state::parameters.torque_sp / state::calibration.motor_torque_const;
    }
    
    float vd = 0.0;
    float vq = 0.0;
    if (state::parameters.control_mode == consts::control_mode_pwm_drive) {
      vd = 0;
      vq = state::parameters.pwm_drive;
    } else {
      pid_id.setTarget(id_sp);
      pid_iq.setTarget(iq_sp);

      state::results.id_output = pid_id.compute(id);
      state::results.iq_output = pid_iq.compute(iq);

      vd = state::results.id_output * state::calibration.motor_resistance;
      vq = state::results.iq_output * state::calibration.motor_resistance; // + state::results.hf_rotor_vel * state::calibration.motor_torque_const;
    }

    // Normalize the vectors
    float mag = Q_rsqrt(vd*vd + vq*vq);
    float div = std::min(1.0/state::results.average_vin, mag);
    float vd_norm = vd * div;
    float vq_norm = vq * div;

    float valpha_norm, vbeta_norm;
    math::transformInversePark(vd_norm, vq_norm, cos_theta, sin_theta, valpha_norm, vbeta_norm);

    if (state::calibration.flip_phases) {
      vbeta_norm = -vbeta_norm;
    }

    modulator.computeDutyCycles(valpha_norm, vbeta_norm, 
                                state::results.duty_a, 
                                state::results.duty_b, 
                                state::results.duty_c);

    if (state::parameters.gate_active) {
      state::results.duty_a = state::results.duty_a * consts::max_duty_cycle;
      state::results.duty_b = state::results.duty_b * consts::max_duty_cycle;
      state::results.duty_c = state::results.duty_c * consts::max_duty_cycle;
    } else {
      state::results.duty_a = 0.0f;
      state::results.duty_b = 0.0f;
      state::results.duty_c = 0.0f;
    }

    peripherals::gate_driver.setPWMDutyCycle(0, state::results.duty_a);
    peripherals::gate_driver.setPWMDutyCycle(1, state::results.duty_b);
    peripherals::gate_driver.setPWMDutyCycle(2, state::results.duty_c);

    state::results.foc_d_current = id;
    state::results.foc_q_current = iq;
    state::results.foc_d_voltage = vd;
    state::results.foc_q_voltage = vq;
  }
}

void resetControlTimeout() {
  last_control_timeout_reset = chTimeNow();
}

void brakeMotor() {
  state::parameters.foc_d_current_sp = 0.0f;
  state::parameters.foc_q_current_sp = 0.0f;
  state::parameters.control_mode = consts::control_mode_foc_current;
}

} // namespace controller
} // namespace motor_driver
