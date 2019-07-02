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

static Thread *control_thread_ptr = nullptr;

static SVM modulator(SVMStrategy::MIDPOINT_CLAMP);

static PID pid_id(calibration.foc_kp_d, calibration.foc_ki_d, 0.0f, current_control_interval);

static PID pid_iq(calibration.foc_kp_q, calibration.foc_ki_q, 0.0f, current_control_interval);

static PID pid_velocity(calibration.velocity_kp, calibration.velocity_ki, 0.0f, velocity_control_interval);

static PID pid_position(calibration.position_kp, calibration.position_ki, 0.0f, position_control_interval);

static systime_t last_control_timeout_reset;

static const LFFlipType enc_ang_corr_periodicity_flips[] = {
  LFFlipType::NONE
};

static const LFPeriodicity enc_ang_corr_periodicity = {
  1,
  enc_ang_corr_periodicity_flips
};

static LUTFunction<int8_t> enc_ang_corr_table(0, 2 * pi, calibration.enc_ang_corr_table_values, enc_ang_corr_table_size, enc_ang_corr_periodicity);

static float getEncoderAngleCorrection(float raw_enc_pos) {
  if (calibration.enc_ang_corr_scale != 0.0f) {
    return enc_ang_corr_table(raw_enc_pos) * calibration.enc_ang_corr_scale + calibration.enc_ang_corr_offset;
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
  pid_id.setLimits(-ivsense_current_max, ivsense_current_max);
  pid_iq.setLimits(-ivsense_current_max, ivsense_current_max);
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
  encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  while (true) {
    /*
     * Wait for resumeInnerControlLoop to be called
     */
    chEvtWaitAny((flagsmask_t)1);

    // If there is no fault, enable the motors.
    if (!parameters.gate_active && !parameters.gate_fault) {
      gate_driver.enableGates();
      parameters.gate_active = true;
      chThdSleepMicroseconds(500);
    }

    // If there is a fault, disable the motors.
    if (parameters.gate_active && parameters.gate_fault) {
      gate_driver.disableGates();
      brakeMotor();
      parameters.gate_active = false;
      chThdSleepMicroseconds(500);
    }

    if (calibration.control_timeout != 0 && (chTimeNow() - last_control_timeout_reset) >= MS2ST(calibration.control_timeout)) {
      brakeMotor();
    } 

    estimateState();

    runPositionControl();

    runVelocityControl();

    runCurrentControl();
  }
}

void estimateState() {
  /*
   * Get current encoder position and velocity
   */

  uint16_t raw_enc_value;

  chSysLock(); // Required for function calls with "I" suffix

  raw_enc_value = encoder.getPipelinedRegisterReadResultI();
  encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  results.raw_enc_value = raw_enc_value;

  float raw_enc_pos = raw_enc_value * rad_per_enc_tick;
  float enc_pos = raw_enc_pos + getEncoderAngleCorrection(raw_enc_pos);

  float prev_enc_pos = results.enc_pos;
  results.enc_pos = enc_pos;

  float enc_pos_diff = enc_pos - prev_enc_pos;
  if (enc_pos_diff < -pi) {
    results.rotor_revs += 1;
    enc_pos_diff += 2 * pi; // Normalize to (-pi, pi) range
  } else if (enc_pos_diff > pi) {
    results.rotor_revs -= 1;
    enc_pos_diff -= 2 * pi; // Normalize to (-pi, pi) range
  }

  results.rotor_pos = enc_pos + results.rotor_revs * 2 * pi - calibration.position_offset;

  float rotor_vel_update = enc_pos_diff * current_control_freq;
  // High frequency estimate used for on-board commutation
  float hf_alpha = calibration.hf_velocity_filter_param;
  results.hf_rotor_vel = hf_alpha * rotor_vel_update + (1.0f - hf_alpha) * results.hf_rotor_vel;
  // Low frequency estimate sent to host
  float lf_alpha = calibration.lf_velocity_filter_param;
  results.lf_rotor_vel = lf_alpha * rotor_vel_update + (1.0f - lf_alpha) * results.lf_rotor_vel;

  /*
   * Calculate average voltages and currents
   * This will have odd behavior if the following conditions are not met:
   * 1) count starts at zero 
   * 2) average arrays are not initialized to zero
   */

  // Subtract old values before storing/adding new values
  // Start doing this after rolling over
  if (rolladc.vin[rolladc.count] != 0) {
    results.raw_average_ia  -= rolladc.ia [rolladc.count];
    results.raw_average_ib  -= rolladc.ib [rolladc.count];
    results.raw_average_ic  -= rolladc.ic [rolladc.count];
    results.raw_average_va  -= rolladc.va [rolladc.count];
    results.raw_average_vb  -= rolladc.vb [rolladc.count];
    results.raw_average_vc  -= rolladc.vc [rolladc.count];
    results.raw_average_vin -= rolladc.vin[rolladc.count];
  }

  rolladc.ia [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_ia ];
  rolladc.ib [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_ib ];
  rolladc.ic [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_ic ];
  rolladc.va [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_va ];
  rolladc.vb [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_vb ];
  rolladc.vc [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_vc ];
  rolladc.vin[rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_vin];

  // The new average is equal to the addition of the old value minus the last value.
  // For the first (ivsense_rolling_average_count) values, the average will be wrong.
  results.raw_average_ia  += rolladc.ia [rolladc.count];
  results.raw_average_ib  += rolladc.ib [rolladc.count];
  results.raw_average_ic  += rolladc.ic [rolladc.count];
  results.raw_average_va  += rolladc.va [rolladc.count];
  results.raw_average_vb  += rolladc.vb [rolladc.count];
  results.raw_average_vc  += rolladc.vc [rolladc.count];
  results.raw_average_vin += rolladc.vin[rolladc.count];

  rolladc.count = (rolladc.count + 1) % ivsense_rolling_average_count;

  results.average_ia  = adcValueToCurrent(results.raw_average_ia  / ivsense_rolling_average_count);
  results.average_ib  = adcValueToCurrent(results.raw_average_ib  / ivsense_rolling_average_count);
  results.average_ic  = adcValueToCurrent(results.raw_average_ic  / ivsense_rolling_average_count);
  results.average_va  = adcValueToVoltage(results.raw_average_va  / ivsense_rolling_average_count);
  results.average_vb  = adcValueToVoltage(results.raw_average_vb  / ivsense_rolling_average_count);
  results.average_vc  = adcValueToVoltage(results.raw_average_vc  / ivsense_rolling_average_count);
  results.average_vin = adcValueToVoltage(results.raw_average_vin / ivsense_rolling_average_count);

  results.corrected_ia = results.average_ia - calibration.ia_offset;
  results.corrected_ib = results.average_ib - calibration.ib_offset;
  results.corrected_ic = results.average_ic - calibration.ic_offset;

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
  if (rolladc.count == 0) {
    float recorder_new_data[recorder_channel_count];

    recorder_new_data[recorder_channel_ia] = results.corrected_ia;
    recorder_new_data[recorder_channel_ib] = results.corrected_ib;
    recorder_new_data[recorder_channel_ic] = results.corrected_ic;
    recorder_new_data[recorder_channel_va] = results.average_va;
    recorder_new_data[recorder_channel_vb] = results.average_vb;
    recorder_new_data[recorder_channel_vc] = results.average_vc;
    recorder_new_data[recorder_channel_vin] = results.average_vin;
    recorder_new_data[recorder_channel_rotor_pos] = results.rotor_pos;
    recorder_new_data[recorder_channel_rotor_vel] = results.hf_rotor_vel;
    recorder_new_data[recorder_channel_ex1] = results.foc_q_current;
    recorder_new_data[recorder_channel_ex2] = results.foc_d_current;

    recorder.recordSample(recorder_new_data);
  }
  
}

void runPositionControl() {
  if (parameters.control_mode == control_mode_position || parameters.control_mode == control_mode_position_velocity) {
    pid_position.setGains(calibration.position_kp, calibration.position_ki, 0.0f);
    pid_position.setLimits(-calibration.velocity_limit, calibration.velocity_limit);
    pid_position.setTarget(parameters.position_sp);
    parameters.velocity_sp = pid_position.compute(results.rotor_pos);
  }
}

void runVelocityControl() {
  if (parameters.control_mode == control_mode_velocity || parameters.control_mode == control_mode_position || parameters.control_mode == control_mode_position_velocity) {
    pid_velocity.setGains(calibration.velocity_kp, calibration.velocity_ki, 0.0f);
    // float velocity_max = results.average_vin / calibration.motor_torque_const;
    float velocity_max = 40.0f;
    pid_velocity.setLimits(-calibration.torque_limit, calibration.torque_limit);
    pid_velocity.setTarget(parameters.velocity_sp);
    parameters.torque_sp = pid_velocity.compute(results.hf_rotor_vel);
  }
}

void runCurrentControl() {
  /*
   * Compute phase duty cycles
   */

  if (parameters.control_mode == control_mode_raw_phase_pwm) {
    /*
     * Directly set PWM duty cycles
     */

    gate_driver.setPWMDutyCycle(0, parameters.phase0 * max_duty_cycle);
    gate_driver.setPWMDutyCycle(1, parameters.phase1 * max_duty_cycle);
    gate_driver.setPWMDutyCycle(2, parameters.phase2 * max_duty_cycle);
  } else {
    /*
     * Run field-oriented control
     */
    float ialpha, ibeta;
    transformClarke(results.corrected_ia, 
                    results.corrected_ib, 
                    results.corrected_ic, ialpha, ibeta);

    if (calibration.flip_phases) {
      ibeta = -ibeta;
    }

    float mech_pos = results.enc_pos - calibration.erev_start * rad_per_enc_tick;
    float elec_pos = mech_pos * calibration.erevs_per_mrev;

    float cos_theta = fast_cos(elec_pos);
    float sin_theta = fast_sin(elec_pos);

    float id, iq;
    transformPark(ialpha, ibeta, cos_theta, sin_theta, id, iq);

    pid_id.setGains(calibration.foc_kp_d, calibration.foc_ki_d, 0.0f);
    pid_iq.setGains(calibration.foc_kp_q, calibration.foc_ki_q, 0.0f);

    pid_id.setLimits(-calibration.current_limit, calibration.current_limit);
    pid_iq.setLimits(-calibration.current_limit, calibration.current_limit);

    float id_sp, iq_sp;
    if (parameters.control_mode == control_mode_foc_current) {
      // Use the provided FOC current setpoints
      id_sp = parameters.foc_d_current_sp;
      iq_sp = parameters.foc_q_current_sp;
    } else {
      // Generate FOC current setpoints from the reference torque
      id_sp = 0.0f;
      iq_sp = parameters.torque_sp / calibration.motor_torque_const;
    }
    
    float vd = 0.0;
    float vq = 0.0;
    if (parameters.control_mode == control_mode_pwm_drive) {
      vd = 0;
      vq = parameters.pwm_drive;
    } else {
      pid_id.setTarget(id_sp);
      pid_iq.setTarget(iq_sp);

      results.id_output = pid_id.compute(id);
      results.iq_output = pid_iq.compute(iq);

      vd = results.id_output * calibration.motor_resistance;
      vq = results.iq_output * calibration.motor_resistance; // + results.hf_rotor_vel * calibration.motor_torque_const;
    }

    // Normalize the vectors
    float mag = Q_rsqrt(vd*vd + vq*vq);
    float div = std::min(1.0/results.average_vin, mag);
    float vd_norm = vd * div;
    float vq_norm = vq * div;

    float valpha_norm, vbeta_norm;
    transformInversePark(vd_norm, vq_norm, cos_theta, sin_theta, valpha_norm, vbeta_norm);

    if (calibration.flip_phases) {
      vbeta_norm = -vbeta_norm;
    }

    modulator.computeDutyCycles(valpha_norm, vbeta_norm, 
                                results.duty_a, results.duty_b, results.duty_c);

    if (parameters.gate_active) {
      results.duty_a = results.duty_a * max_duty_cycle;
      results.duty_b = results.duty_b * max_duty_cycle;
      results.duty_c = results.duty_c * max_duty_cycle;
    } else {
      results.duty_a = 0.0f;
      results.duty_b = 0.0f;
      results.duty_c = 0.0f;
    }

    gate_driver.setPWMDutyCycle(0, results.duty_a);
    gate_driver.setPWMDutyCycle(1, results.duty_b);
    gate_driver.setPWMDutyCycle(2, results.duty_c);

    results.foc_d_current = id;
    results.foc_q_current = iq;
    results.foc_d_voltage = vd;
    results.foc_q_voltage = vq;
  }
}

void resetControlTimeout() {
  last_control_timeout_reset = chTimeNow();
}

void brakeMotor() {
  parameters.foc_d_current_sp = 0.0f;
  parameters.foc_q_current_sp = 0.0f;
  parameters.control_mode = control_mode_foc_current;
}

} // namespace motor_driver
