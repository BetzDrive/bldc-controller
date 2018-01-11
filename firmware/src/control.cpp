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

static Thread *control_thread_ptr;

static SVM modulator(SVMStrategy::TOP_BOTTOM_CLAMP);

static PID pid_id(calibration.foc_kp_d, calibration.foc_ki_d, 0.0f, current_control_interval);

static PID pid_iq(calibration.foc_kp_q, calibration.foc_ki_q, 0.0f, current_control_interval);

void initControl() {
  pid_id.setInputLimits(-ivsense_current_max, ivsense_current_max);
  pid_iq.setInputLimits(-ivsense_current_max, ivsense_current_max);
}

void resumeInnerControlLoop() {
  if (control_thread_ptr != NULL) {
    chSysLockFromIsr();
    chEvtSignalI(control_thread_ptr, (flagsmask_t)1);
    chSysUnlockFromIsr();
  }
}

void runInnerControlLoop() {
  control_thread_ptr = chThdSelf();

  chSysLock();

  /*
   * getPipelinedRegisterReadResultI requires startPipelinedRegisterReadI to be called beforehand
   */
  encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  while (true) {
    /*
     * Wait for resumeInnerControlLoop to be called
     */
    chEvtWaitAny((flagsmask_t)1);

    runCurrentControl();
  }
}

void runCurrentControl() {
  palClearPad(GPIOA, GPIOA_LED_Y);

  /*
   * Get current encoder angle
   */

  uint16_t raw_encoder_angle;

  chSysLock(); // Required for function calls with "I" suffix

  raw_encoder_angle = encoder.getPipelinedRegisterReadResultI();
  encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  uint16_t prev_raw_encoder_angle = results.encoder_angle;
  float threshold = pi;
  float diff = ((int16_t)prev_raw_encoder_angle - (int16_t)raw_encoder_angle) * encoder_angle_to_radians;
  if (diff > threshold) {
    results.encoder_revs += 1;
  } else if (diff < -threshold) {
    results.encoder_revs -= 1;
  }
  results.encoder_radian_angle = raw_encoder_angle * encoder_angle_to_radians + results.encoder_revs * 2 * pi;
  results.encoder_angle = raw_encoder_angle;

  /*
   * Calculate average voltages and currents
   */

  // TODO: should this be RMS voltage and current?

  unsigned int adc_ia_sum = 0;
  unsigned int adc_ib_sum = 0;
  unsigned int adc_ic_sum = 0;
  // unsigned int adc_va_sum = 0;
  // unsigned int adc_vb_sum = 0;
  // unsigned int adc_vc_sum = 0;
  unsigned int adc_vin_sum = 0;

  for (size_t i = 0; i < ivsense_samples_per_cycle; i++) {
    adc_ia_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_ia];
    adc_ib_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_ib];
    adc_ic_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_ic];
    // adc_va_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_va];
    // adc_vb_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_vb];
    // adc_vc_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_vc];
    adc_vin_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_vin];
  }

  results.average_ia = adcValueToCurrent((float)adc_ia_sum / ivsense_samples_per_cycle);
  results.average_ib = adcValueToCurrent((float)adc_ib_sum / ivsense_samples_per_cycle);
  results.average_ic = adcValueToCurrent((float)adc_ic_sum / ivsense_samples_per_cycle);
  // results.average_va = adcValueToVoltage((float)adc_va_sum / ivsense_samples_per_cycle);
  // results.average_vb = adcValueToVoltage((float)adc_vb_sum / ivsense_samples_per_cycle);
  // results.average_vc = adcValueToVoltage((float)adc_vc_sum / ivsense_samples_per_cycle);
  results.average_vin = adcValueToVoltage((float)adc_vin_sum / ivsense_samples_per_cycle);

  /*
   * Compute phase duty cycles
   */

  if (parameters.raw_pwm_mode) {
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

    uint16_t zeroed_encoder_angle = (raw_encoder_angle - calibration.encoder_zero + encoder_period) % encoder_period;
    float elec_angle_radians = zeroed_encoder_angle * encoder_angle_to_radians * calibration.erevs_per_mrev;

    float cos_theta = fast_cos(elec_angle_radians);
    float sin_theta = fast_sin(elec_angle_radians);

    float id, iq;
    transformPark(ialpha, ibeta, cos_theta, sin_theta, id, iq);

    pid_id.setMode(AUTO_MODE);
    pid_iq.setMode(AUTO_MODE);

    pid_id.setTunings(calibration.foc_kp_d, calibration.foc_ki_d, 0.0f);
    pid_iq.setTunings(calibration.foc_kp_q, calibration.foc_ki_q, 0.0f);

    pid_id.setOutputLimits(-results.average_vin, results.average_vin);
    pid_iq.setOutputLimits(-results.average_vin, results.average_vin);

    pid_id.setSetPoint(0.0f);
    pid_id.setProcessValue(id);
    pid_id.setBias(0.0f);

    float torque_command = parameters.cmd_duty_cycle;

    if (calibration.sw_endstop_min < calibration.sw_endstop_max) {
      // Software endstops are only active if they have different values

      if (torque_command >= 0) {
        // Positive torque command, only check the maximum endstop

        float torque_limit = std::max(0.0f, (calibration.sw_endstop_max - results.encoder_radian_angle) * calibration.sw_endstop_slope);
        torque_command = std::min(torque_command, torque_limit);
      } else {
        // Negative torque command, only check the minimum endstop

        float torque_limit = std::min(0.0f, (calibration.sw_endstop_min - results.encoder_radian_angle) * calibration.sw_endstop_slope);
        torque_command = std::max(torque_command, torque_limit);
      }
    }

    pid_iq.setSetPoint(parameters.cmd_duty_cycle);
    pid_iq.setProcessValue(iq);
    pid_iq.setBias(parameters.cmd_duty_cycle * calibration.winding_resistance);

    float vd = pid_id.compute();
    float vq = pid_iq.compute();

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
  }

  palSetPad(GPIOA, GPIOA_LED_Y);
}

} // namespace motor_driver
