#include "control.h"
#include "transforms.h"
#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include "fast_math.h"
#include "chprintf.h"
#include "SVM.h"
#include "PID.h"
#include "constants.h"

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

void initControl() {
  pid_id.setInputLimits(-ivsense_current_max, ivsense_current_max);
  pid_iq.setInputLimits(-ivsense_current_max, ivsense_current_max);
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
  float alpha = calibration.velocity_filter_param;
  results.rotor_vel = alpha * rotor_vel_update + (1.0f - alpha) * results.rotor_vel;

  /*
   * Calculate average voltages and currents
   * This will have odd behavior if the following conditions are not met:
   * 1) count starts at zero 
   * 2) average arrays are not initialized to zero
   */

  // TODO: should this be RMS voltage and current?
  // Subtract old values before storing/adding new values
  // Start doing this after rolling over
  if (rolladc.vin[rolladc.count] != 0) {
    results.average_ia  -= adcValueToCurrent((float)(rolladc.ia [rolladc.count])) / ivsense_rolling_average_count;
    results.average_ib  -= adcValueToCurrent((float)(rolladc.ib [rolladc.count])) / ivsense_rolling_average_count;
    results.average_ic  -= adcValueToCurrent((float)(rolladc.ic [rolladc.count])) / ivsense_rolling_average_count;
    results.average_va  -= adcValueToVoltage((float)(rolladc.va [rolladc.count])) / ivsense_rolling_average_count;
    results.average_vb  -= adcValueToVoltage((float)(rolladc.vb [rolladc.count])) / ivsense_rolling_average_count;
    results.average_vc  -= adcValueToVoltage((float)(rolladc.vc [rolladc.count])) / ivsense_rolling_average_count;
    results.average_vin -= adcValueToVoltage((float)(rolladc.vin[rolladc.count])) / ivsense_rolling_average_count;
  }

  rolladc.ia [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_ia ];
  rolladc.ib [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_ib ];
  rolladc.ic [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_ic ];
  rolladc.va [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_va ];
  rolladc.vb [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_vb ];
  rolladc.vc [rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_vc ];
  rolladc.vin[rolladc.count] = ivsense_adc_samples_ptr[ivsense_channel_vin];

  // The new average is equal to the addition of the old value minus the last value (delta) over the count and then converted to a current.
  // For the first (ivsense_rolling_average_count) values, the average will be wrong.
  results.average_ia  += adcValueToCurrent((float)(rolladc.ia [rolladc.count])) / ivsense_rolling_average_count;
  results.average_ib  += adcValueToCurrent((float)(rolladc.ib [rolladc.count])) / ivsense_rolling_average_count;
  results.average_ic  += adcValueToCurrent((float)(rolladc.ic [rolladc.count])) / ivsense_rolling_average_count;
  results.average_va  += adcValueToVoltage((float)(rolladc.va [rolladc.count])) / ivsense_rolling_average_count;
  results.average_vb  += adcValueToVoltage((float)(rolladc.vb [rolladc.count])) / ivsense_rolling_average_count;
  results.average_vc  += adcValueToVoltage((float)(rolladc.vc [rolladc.count])) / ivsense_rolling_average_count;
  results.average_vin += adcValueToVoltage((float)(rolladc.vin[rolladc.count])) / ivsense_rolling_average_count;

  rolladc.count = (rolladc.count + 1) % ivsense_rolling_average_count;

  /*
   * Record data
   */
  if (rolladc.count == 0) {
    float recorder_new_data[recorder_channel_count];

    recorder_new_data[recorder_channel_ia] = results.average_ia;
    recorder_new_data[recorder_channel_ib] = results.average_ib;
    recorder_new_data[recorder_channel_ic] = results.average_ic;
    recorder_new_data[recorder_channel_va] = results.average_va;
    recorder_new_data[recorder_channel_vb] = results.average_vb;
    recorder_new_data[recorder_channel_vc] = results.average_vc;
    recorder_new_data[recorder_channel_vin] = results.average_vin;
    recorder_new_data[recorder_channel_rotor_pos] = results.rotor_pos;
    recorder_new_data[recorder_channel_rotor_vel] = results.rotor_vel;
    recorder_new_data[recorder_channel_iq] = results.foc_q_current;
    recorder_new_data[recorder_channel_vq] = results.foc_d_current;

    recorder.recordSample(recorder_new_data);
  }
  
}

void runPositionControl() {
  if (parameters.control_mode == control_mode_position || parameters.control_mode == control_mode_position_velocity) {
    pid_position.setMode(AUTO_MODE);
    pid_position.setTunings(calibration.position_kp, calibration.position_ki, 0.0f);
    pid_position.setInputLimits(-1.0f, 1.0f);
    pid_position.setOutputLimits(-calibration.velocity_limit, calibration.velocity_limit);
    pid_position.setSetPoint(0.0f);
    pid_position.setProcessValue(results.rotor_pos - parameters.position_sp);
    pid_position.setBias(0.0f);
    parameters.velocity_sp = pid_position.compute();
  } else {
    pid_position.setMode(MANUAL_MODE);
  }
}

void runVelocityControl() {
  if (parameters.control_mode == control_mode_velocity || parameters.control_mode == control_mode_position || parameters.control_mode == control_mode_position_velocity) {
    pid_velocity.setMode(AUTO_MODE);
    pid_velocity.setTunings(calibration.velocity_kp, calibration.velocity_ki, 0.0f);
    // float velocity_max = results.average_vin / calibration.motor_torque_const;
    float velocity_max = 40.0f;
    pid_velocity.setInputLimits(-velocity_max, velocity_max);
    pid_velocity.setOutputLimits(-calibration.torque_limit, calibration.torque_limit);
    pid_velocity.setSetPoint(parameters.velocity_sp);
    pid_velocity.setProcessValue(results.rotor_vel);
    pid_velocity.setBias(0.0f);
    parameters.torque_sp = pid_velocity.compute();
  } else {
    pid_velocity.setMode(MANUAL_MODE);
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

    gate_driver.setPWMDutyCycle(0, parameters.phase0);
    gate_driver.setPWMDutyCycle(1, parameters.phase1);
    gate_driver.setPWMDutyCycle(2, parameters.phase2);
  } else {
    /*
     * Run field-oriented control
     */

    float ialpha, ibeta;
    transformClarke(results.average_ia, results.average_ib, results.average_ic, ialpha, ibeta);

    if (calibration.flip_phases) {
      ibeta = -ibeta;
    }

    float mech_pos = results.enc_pos - calibration.erev_start * rad_per_enc_tick;
    float elec_pos = mech_pos * calibration.erevs_per_mrev;

    float cos_theta = fast_cos(elec_pos);
    float sin_theta = fast_sin(elec_pos);

    float id, iq;
    transformPark(ialpha, ibeta, cos_theta, sin_theta, id, iq);

    pid_id.setMode(AUTO_MODE);
    pid_iq.setMode(AUTO_MODE);

    pid_id.setTunings(calibration.foc_kp_d, calibration.foc_ki_d, 0.0f);
    pid_iq.setTunings(calibration.foc_kp_q, calibration.foc_ki_q, 0.0f);

    pid_id.setOutputLimits(-results.average_vin, results.average_vin);
    pid_iq.setOutputLimits(-results.average_vin, results.average_vin);

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

    //pid_id.setSetPoint(id_sp);
    //pid_id.setProcessValue(id);
    //pid_id.setBias(id_sp * calibration.motor_resistance);

    //pid_iq.setSetPoint(iq_sp);
    //pid_iq.setProcessValue(iq);
    //pid_iq.setBias(iq_sp * calibration.motor_resistance + results.rotor_vel * calibration.motor_torque_const);

    //float vd = pid_id.compute();
    //float vq = pid_iq.compute();
    float vd = 0;
    float vq = iq_sp * calibration.motor_resistance + results.rotor_vel * calibration.motor_torque_const;

    float vd_norm = vd / results.average_vin;
    float vq_norm = vq / results.average_vin;

    float valpha_norm, vbeta_norm;
    transformInversePark(vd_norm, vq_norm, cos_theta, sin_theta, valpha_norm, vbeta_norm);

    if (calibration.flip_phases) {
      vbeta_norm = -vbeta_norm;
    }

    float duty0, duty1, duty2;
    modulator.computeDutyCycles(valpha_norm, vbeta_norm, duty0, duty1, duty2);

    gate_driver.setPWMDutyCycle(0, duty0);
    gate_driver.setPWMDutyCycle(1, duty1);
    gate_driver.setPWMDutyCycle(2, duty2);

    results.foc_d_current = id;
    results.foc_q_current = iq;
    results.foc_d_voltage = vd_norm;
    results.foc_q_voltage = vq_norm;

  }
}

void resetControlTimeout() {
  last_control_timeout_reset = chTimeNow();
}

void brakeMotor() {
  parameters.foc_d_current_sp = 0.0f;
  parameters.foc_q_current_sp = 0.0f;
  calibration.motor_torque_const = 0.0f; // Damps the motor to prevent a voltage spike
  parameters.control_mode = control_mode_raw_phase_pwm;
  parameters.phase0 = 0.0f;
  parameters.phase1 = 0.0f;
  parameters.phase2 = 0.0f;
}

} // namespace motor_driver
