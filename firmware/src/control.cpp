#include "control.hpp"

#include <cmath>

#include "hal.h"

#include "SVM.hpp"
#include "ch.h"
#include "chprintf.h"
#include "constants.hpp"
#include "fast_math.hpp"
#include "peripherals.hpp"
#include "pid.hpp"
#include "state.hpp"
#include "transforms.hpp"

namespace motor_driver {
namespace controller {

static Thread *control_thread_ptr = nullptr;

static SVM modulator(SVMStrategy::MIDPOINT_CLAMP);

static PID pid_id(state::calibration_pb.foc_kp_d,
                  state::calibration_pb.foc_ki_d, 0.0f,
                  consts::current_control_interval);
static PID pid_iq(state::calibration_pb.foc_kp_q,
                  state::calibration_pb.foc_ki_q, 0.0f,
                  consts::current_control_interval);
static PID pid_velocity(state::calibration_pb.velocity_kp, 0.0f,
                        state::calibration_pb.velocity_kd,
                        consts::velocity_control_interval);
static PID pid_position(state::calibration_pb.position_kp, 0.0f,
                        state::calibration_pb.position_kd,
                        consts::position_control_interval);

static systime_t last_control_timeout_reset;

static const math::LFFlipType enc_ang_corr_periodicity_flips[] = {
    math::LFFlipType::NONE};

static const math::LFPeriodicity enc_ang_corr_periodicity = {
    1, enc_ang_corr_periodicity_flips};

static math::LUTFunction<int8_t> enc_ang_corr_table(
    0, 2 * consts::pi,
    reinterpret_cast<int8_t *>(
        state::calibration_pb.enc_ang_corr_table_values.bytes),
    state::calibration_pb.enc_ang_corr_table_values.size,
    enc_ang_corr_periodicity);

static float getEncoderAngleCorrection(float raw_enc_pos) {
  if (state::calibration_pb.enc_ang_corr_scale != 0.0f) {
    return ((enc_ang_corr_table(raw_enc_pos) *
             state::calibration_pb.enc_ang_corr_scale) +
            state::calibration_pb.enc_ang_corr_offset);
  } else {
    return 0.0f;
  }
}

inline static float clamp(float val, float min, float max) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

static float Q_rsqrt(float number) {
  const float x2 = number * 0.5F;
  const float threehalfs = 1.5F;

  union {
    float f;
    uint32_t i;
  } conv = {number}; // member 'f' set to value of 'number'.
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  return conv.f;
}

void initControl() {
  pid_id.setLimits(-consts::isense_current_max, consts::isense_current_max);
  pid_iq.setLimits(-consts::isense_current_max, consts::isense_current_max);
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

  // getPipelinedResultI requires startPipelinedAngleReadI to be called
  // beforehand.
  peripherals::encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  unsigned int count = 0;

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

    // Put motor into braking mode if the communication line times out
    // Timeout flag notifies host of this. The flag is cleared when a motor
    //   related command arrives (in fw_comms.cpp)
    // TODO(gbalke): The flag clear is placed in a bad location... Figure out a
    // cleaner solution.
    if (state::calibration_pb.control_timeout != 0 &&
        ((chTimeNow() - last_control_timeout_reset) >=
         MS2ST(state::calibration_pb.control_timeout))) {
      brakeMotor();
      state::parameters.timeout_flag = true;
    }

    chMtxLock(&peripherals::var_access_mutex);

    estimateState();

    if (count % consts::pos_divider == 0) {
      runPositionControl();
    }

    if (count % consts::vel_divider == 0) {
      runVelocityControl();
    }

    runCurrentControl();

    chMtxUnlock();

    count = (count + 1) % consts::current_control_freq;
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
  } else if (enc_pos_diff > consts::pi) {
    state::results.rotor_revs -= 1;
  }

  float prev_rotor_pos = state::results.rotor_pos;
  state::results.rotor_pos =
      (enc_pos + (state::results.rotor_revs * 2 * consts::pi) -
       state::calibration_pb.position_offset);
  float rotor_pos_diff = state::results.rotor_pos - prev_rotor_pos;

  // TODO(greg): Use system clock to calculate dt rather than loop freq.
  // NOTE: The actual loop frequency is closer to 10kHz. The gains have been
  // tuned to the current setup so it'll be easier to just fix them again once
  // this code is switched to use the true dt rather than re-tune twice.
  float dt_inverse = 10000; // consts::current_control_freq;
  float rotor_vel_update = rotor_pos_diff * dt_inverse;
  // High frequency estimate used for on-board commutation
  float hf_alpha = state::calibration_pb.hf_velocity_filter_param;
  state::results.hf_rotor_vel =
      (hf_alpha * rotor_vel_update) +
      ((1.0f - hf_alpha) * state::results.hf_rotor_vel);
  // Low frequency estimate sent to host
  float lf_alpha = state::calibration_pb.lf_velocity_filter_param;
  state::results.lf_rotor_vel =
      (lf_alpha * rotor_vel_update) +
      ((1.0f - lf_alpha) * state::results.lf_rotor_vel);

  /*
   * Calculate average voltages and currents
   * This will have odd behavior if the following conditions are not met:
   * 1) count starts at zero
   * 2) average arrays are not initialized to zero
   */

  static RolledADC rolladc;

  static uint32_t raw_avg_ia = 0, raw_avg_ib = 0, raw_avg_ic = 0;
  static uint32_t raw_avg_vin = 0;

  // Subtract old values before storing/adding new values
  // Start doing this after rolling over
  if (rolladc.vin[rolladc.count] != 0) {
    raw_avg_ia -= rolladc.ia[rolladc.count];
    raw_avg_ib -= rolladc.ib[rolladc.count];
    raw_avg_ic -= rolladc.ic[rolladc.count];
    raw_avg_vin -= rolladc.vin[rolladc.count];
  }

  rolladc.ia[rolladc.count] = *peripherals::curra_adc_samples_ptr;
  rolladc.ib[rolladc.count] = *peripherals::currb_adc_samples_ptr;
  rolladc.ic[rolladc.count] = *peripherals::currc_adc_samples_ptr;
  rolladc.vin[rolladc.count] = *peripherals::vsense_adc_samples_ptr;

  // The new average is equal to the addition of the old value minus the
  // last value. For the first (ivsense_rolling_average_count) values, the
  // average will be wrong.
  raw_avg_ia += rolladc.ia[rolladc.count];
  raw_avg_ib += rolladc.ib[rolladc.count];
  raw_avg_ic += rolladc.ic[rolladc.count];
  raw_avg_vin += rolladc.vin[rolladc.count];

  rolladc.count = ((rolladc.count + 1) % consts::ivsense_rolling_average_count);

  static float avg_ia, avg_ib, avg_ic;
  static float avg_vin;

  avg_ia = peripherals::adcValueToCurrent(
      raw_avg_ia / consts::ivsense_rolling_average_count);
  avg_ib = peripherals::adcValueToCurrent(
      raw_avg_ib / consts::ivsense_rolling_average_count);
  avg_ic = peripherals::adcValueToCurrent(
      raw_avg_ic / consts::ivsense_rolling_average_count);
  avg_vin = peripherals::adcValueToVoltage(
      raw_avg_vin / consts::ivsense_rolling_average_count);

  state::results.ia = avg_ia - state::calibration_pb.ia_offset;
  state::results.ib = avg_ib - state::calibration_pb.ib_offset;
  state::results.ic = avg_ic - state::calibration_pb.ic_offset;
  state::results.vin = avg_vin;

  // if (results.duty_a > results.duty_b && results.duty_a > results.duty_c) {
  //  results.ia = -(results.ib + results.ic);
  //} else if (results.duty_b > results.duty_c) {
  //  results.ib = -(results.ia + results.ic);
  //} else {
  //  results.ic = -(results.ia + results.ib);
  //}

  /*
   * Record data
   */
  if (rolladc.count == 0) {
    float recorder_new_data[consts::recorder_channel_count];

    recorder_new_data[consts::recorder_channel_ia] = state::results.ia;
    recorder_new_data[consts::recorder_channel_ib] = state::results.ib;
    recorder_new_data[consts::recorder_channel_ic] = state::results.ic;

    // recorder_new_data[consts::recorder_channel_ia]        = raw_avg_ia /
    // consts::ivsense_rolling_average_count;
    // recorder_new_data[consts::recorder_channel_ib]        = raw_avg_ib /
    // consts::ivsense_rolling_average_count;
    // recorder_new_data[consts::recorder_channel_ic]        = raw_avg_ic /
    // consts::ivsense_rolling_average_count;

    recorder_new_data[consts::recorder_channel_vin] = state::results.vin;
    recorder_new_data[consts::recorder_channel_rotor_pos] =
        (state::results.rotor_pos);
    recorder_new_data[consts::recorder_channel_rotor_vel] =
        (state::results.hf_rotor_vel);
    recorder_new_data[consts::recorder_channel_ex1] =
        (state::results.foc_q_current);
    recorder_new_data[consts::recorder_channel_ex2] =
        (state::results.foc_d_current);

    state::recorder.recordSample(recorder_new_data);
  }
  state::results.estimation_loops++;
}

void runPositionControl() {
  if (state::parameters.control_mode == consts::control_mode_position ||
      (state::parameters.control_mode ==
       consts::control_mode_position_velocity) ||
      (state::parameters.control_mode ==
       consts::control_mode_position_feed_forward)) {
    pid_position.setGains(state::calibration_pb.position_kp, 0.0f,
                          state::calibration_pb.position_kd);
    pid_position.setAlpha(consts::position_control_alpha);
    pid_position.setLimits(-state::calibration_pb.torque_limit,
                           state::calibration_pb.torque_limit);
    pid_position.setTarget(state::parameters.position_sp);
    state::parameters.torque_sp =
        pid_position.compute(state::results.rotor_pos);
  }
}

void runVelocityControl() {
  if (state::parameters.control_mode == consts::control_mode_velocity ||
      (state::parameters.control_mode ==
       consts::control_mode_position_velocity)) {
    pid_velocity.setGains(state::calibration_pb.velocity_kp, 0.0f,
                          state::calibration_pb.velocity_kd);
    // float velocity_max = state::results.vin /
    // state::calibration_pb.motor_torque_const;
    pid_velocity.setLimits(-state::calibration_pb.torque_limit,
                           state::calibration_pb.torque_limit);
    pid_velocity.setTarget(state::parameters.velocity_sp);
    state::parameters.torque_sp =
        pid_velocity.compute(state::results.hf_rotor_vel);
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

    peripherals::gate_driver.setPWMDutyCycle(0, state::parameters.phase0 *
                                                    consts::max_duty_cycle);
    peripherals::gate_driver.setPWMDutyCycle(1, state::parameters.phase1 *
                                                    consts::max_duty_cycle);
    peripherals::gate_driver.setPWMDutyCycle(2, state::parameters.phase2 *
                                                    consts::max_duty_cycle);
  } else {
    /*
     * Run field-oriented control
     */
    float ialpha, ibeta;
    math::transformClarke(state::results.ia, state::results.ib,
                          state::results.ic, ialpha, ibeta);

    if (state::calibration_pb.flip_phases) {
      ibeta = -ibeta;
    }

    float mech_pos =
        (state::results.enc_pos -
         state::calibration_pb.erev_start * consts::rad_per_enc_tick);
    float elec_pos = mech_pos * state::calibration_pb.erevs_per_mrev;

    float cos_theta = math::fast_cos(elec_pos);
    float sin_theta = math::fast_sin(elec_pos);

    float id, iq;
    math::transformPark(ialpha, ibeta, cos_theta, sin_theta, id, iq);

    pid_id.setGains(state::calibration_pb.foc_kp_d,
                    state::calibration_pb.foc_ki_d, 0.0f);
    pid_iq.setGains(state::calibration_pb.foc_kp_q,
                    state::calibration_pb.foc_ki_q, 0.0f);

    pid_id.setLimits(-state::calibration_pb.current_limit,
                     state::calibration_pb.current_limit);
    pid_iq.setLimits(-state::calibration_pb.current_limit,
                     state::calibration_pb.current_limit);

    float id_sp, iq_sp;
    if (state::parameters.control_mode == consts::control_mode_foc_current) {
      // Use the provided FOC current setpoints
      id_sp = state::parameters.foc_d_current_sp;
      iq_sp = state::parameters.foc_q_current_sp;
    } else if (state::parameters.control_mode ==
               consts::control_mode_position_feed_forward) {
      id_sp = 0.0f;
      iq_sp = ((state::parameters.torque_sp /
                state::calibration_pb.motor_torque_const) +
               state::parameters.feed_forward);
    } else {
      // Generate FOC current setpoints from the reference torque
      id_sp = 0.0f;
      iq_sp = (state::parameters.torque_sp /
               state::calibration_pb.motor_torque_const);
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

      vd = state::results.id_output * state::calibration_pb.motor_resistance;
      vq = state::results.iq_output * state::calibration_pb.motor_resistance;
    }

    // Normalize the vectors
    float mag = Q_rsqrt(vd * vd + vq * vq);
    float div = std::min(1.0 / state::results.vin, mag);
    float vd_norm = vd * div;
    float vq_norm = vq * div;

    float valpha_norm, vbeta_norm;
    math::transformInversePark(vd_norm, vq_norm, cos_theta, sin_theta,
                               valpha_norm, vbeta_norm);

    if (state::calibration_pb.flip_phases) {
      vbeta_norm = -vbeta_norm;
    }

    modulator.computeDutyCycles(valpha_norm, vbeta_norm, state::results.duty_a,
                                state::results.duty_b, state::results.duty_c);

    if (state::parameters.gate_active) {
      state::results.duty_a =
          clamp(state::results.duty_a, consts::min_duty_cycle,
                consts::max_duty_cycle);
      state::results.duty_b =
          clamp(state::results.duty_b, consts::min_duty_cycle,
                consts::max_duty_cycle);
      state::results.duty_c =
          clamp(state::results.duty_c, consts::min_duty_cycle,
                consts::max_duty_cycle);
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

void resetControlTimeout() { last_control_timeout_reset = chTimeNow(); }

void brakeMotor() {
  state::parameters.phase0 = 0;
  state::parameters.phase1 = 0;
  state::parameters.phase2 = 0;
  state::parameters.control_mode = consts::control_mode_raw_phase_pwm;
}

} // namespace controller
} // namespace motor_driver
