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

namespace motor_driver {

static Thread *control_thread_ptr;

static SVM modulator(SVMStrategy::TOP_BOTTOM_CLAMP);

static PID pid_id(10.0f, 0.0f, 0.0f, current_control_interval);

static PID pid_iq(10.0f, 0.0f, 0.0f, current_control_interval);

void initControl() {
  pid_id.setInputLimits(-ivsense_current_max, ivsense_current_max);
  pid_id.setOutputLimits(-ivsense_voltage_max, ivsense_voltage_max);

  pid_iq.setInputLimits(-ivsense_current_max, ivsense_current_max);
  pid_iq.setOutputLimits(-ivsense_voltage_max, ivsense_voltage_max);
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

  pid_id.setMode(AUTO_MODE);
  pid_iq.setMode(AUTO_MODE);

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

  active_results.encoder_angle = raw_encoder_angle;

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

  active_results.average_ia = adcValueToCurrent((float)adc_ia_sum / ivsense_samples_per_cycle);
  active_results.average_ib = adcValueToCurrent((float)adc_ib_sum / ivsense_samples_per_cycle);
  active_results.average_ic = adcValueToCurrent((float)adc_ic_sum / ivsense_samples_per_cycle);
  // active_results.average_va = adcValueToVoltage((float)adc_va_sum / ivsense_samples_per_cycle);
  // active_results.average_vb = adcValueToVoltage((float)adc_vb_sum / ivsense_samples_per_cycle);
  // active_results.average_vc = adcValueToVoltage((float)adc_vc_sum / ivsense_samples_per_cycle);
  active_results.average_vin = adcValueToVoltage((float)adc_vin_sum / ivsense_samples_per_cycle);

  /*
   * Compute phase duty cycles
   */

  if (active_parameters.raw_pwm_mode) {
    /*
     * Directly set PWM duty cycles
     */

    gate_driver.setPWMDutyCycle(0, active_parameters.phase0);
    gate_driver.setPWMDutyCycle(1, active_parameters.phase1);
    gate_driver.setPWMDutyCycle(2, active_parameters.phase2);
  } else {
    /*
     * Run field-oriented control
     */

    float ialpha, ibeta;
    transforms_clarke(active_results.average_ia, active_results.average_ib, active_results.average_ic, &ialpha, &ibeta);
    
    uint16_t zeroed_encoder_angle = (raw_encoder_angle - active_parameters.encoder_zero + encoder_period) % encoder_period;
    // float elec_angle_radians = zeroed_encoder_angle * encoder_angle_to_radians * active_parameters.erpm_per_revolution;
    float elec_angle_radians = zeroed_encoder_angle * encoder_angle_to_radians * 14.0f;

    float id, iq;
    transforms_park(ialpha, ibeta, elec_angle_radians, &id, &iq);

    pid_id.setSetPoint(0.0f);
    pid_id.setProcessValue(id);

    pid_iq.setSetPoint(active_parameters.cmd_duty_cycle);
    pid_iq.setProcessValue(iq);
    pid_iq.setBias(active_parameters.cmd_duty_cycle * active_parameters.winding_resistance);

    float vd = pid_id.compute();
    float vq = pid_iq.compute();

    float vd_norm = vd / active_results.average_vin;
    float vq_norm = vq / active_results.average_vin;

    float valpha_norm, vbeta_norm;
    transforms_inverse_park(vd_norm, vq_norm, elec_angle_radians, &valpha_norm, &vbeta_norm);

    if (active_parameters.flip_phases) {
      vbeta_norm = -vbeta_norm;
    }

    float duty0, duty1, duty2;
    modulator.computeDutyCycles(valpha_norm, vbeta_norm, duty0, duty1, duty2);

    gate_driver.setPWMDutyCycle(0, duty0);
    gate_driver.setPWMDutyCycle(1, duty1);
    gate_driver.setPWMDutyCycle(2, duty2);
  }

  palSetPad(GPIOA, GPIOA_LED_Y);
}

} // namespace motor_driver
