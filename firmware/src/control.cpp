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

float ang = 0.0;
PID pi_controller_d(100, 10);
PID pi_controller_p(100, 10);
float dt = 1.0 / (motor_pwm_clock_freq / motor_pwm_cycle_freq);

void runCurrentControl() {
  uint16_t raw_encoder_angle;

  chSysLock(); // Required for function calls with "I" suffix

  raw_encoder_angle = encoder.getPipelinedRegisterReadResultI();
  encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  active_results.encoder_angle = raw_encoder_angle;

  if (active_parameters.raw_pwm_mode) {
    gate_driver.setPWMDutyCycle(0, active_parameters.phase0);
    gate_driver.setPWMDutyCycle(1, active_parameters.phase1);
    gate_driver.setPWMDutyCycle(2, active_parameters.phase2);
  } else {
    // TODO: optimize this
    // float ang = (((float) raw_encoder_angle - (float) active_parameters.encoder_zero) * 2.0 * pi / 16384.0) * 14.0 + pi / 2.0;
    // active_results.angle = ang;

    // SVM svm_func(SVMStrategy::TOP_BOTTOM_CLAMP);

    // float duty1;
    // float duty2;
    // float duty3;
    // float d = active_parameters.cmd_duty_cycle * svm_func.getMaxAmplitude();
    // svm_func.computeDutyCycles(d * fast_cos(ang), d * fast_sin(ang), duty1, duty2, duty3);

    // gate_driver.setPWMDutyCycle(0, duty1);
    // gate_driver.setPWMDutyCycle(1, duty2);
    // gate_driver.setPWMDutyCycle(2, duty3);
    /***********************************************************************************************/
    /*
     * Calculate average voltage and current
     */

    // TODO: should this be RMS voltage and current?

    unsigned int adc_ia_sum = 0;
    unsigned int adc_ib_sum = 0;
    unsigned int adc_ic_sum = 0;
    unsigned int adc_va_sum = 0;
    unsigned int adc_vb_sum = 0;
    unsigned int adc_vc_sum = 0;
    unsigned int adc_vin_sum = 0;

    for (size_t i = 0; i < ivsense_samples_per_cycle; i++) {
      adc_ia_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_ia];
      adc_ib_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_ib];
      adc_ic_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_ic];
      adc_va_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_va];
      adc_vb_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_vb];
      adc_vc_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_vc];
      adc_vin_sum += ivsense_adc_samples_ptr[i * ivsense_channel_count + ivsense_channel_vin];
    }

    active_results.average_ia = adcValueToCurrent((float)adc_ia_sum / ivsense_samples_per_cycle);
    active_results.average_ib = adcValueToCurrent((float)adc_ib_sum / ivsense_samples_per_cycle);
    active_results.average_ic = adcValueToCurrent((float)adc_ic_sum / ivsense_samples_per_cycle);
    active_results.average_va = adcValueToVoltage((float)adc_va_sum / ivsense_samples_per_cycle);
    active_results.average_vb = adcValueToVoltage((float)adc_vb_sum / ivsense_samples_per_cycle);
    active_results.average_vc = adcValueToVoltage((float)adc_vc_sum / ivsense_samples_per_cycle);
    active_results.average_vin = adcValueToVoltage((float)adc_vin_sum / ivsense_samples_per_cycle);

    /*
     * Run field-oriented control
     */

    // do the clark and other transform
    float alpha;
    float beta;
    transforms_clarke(active_results.average_ia, active_results.average_ib, active_results.average_ic, &alpha, &beta);
    float d;
    float q; 
    
    float ang = (((float) raw_encoder_angle - (float) active_parameters.encoder_zero) * 2.0 * pi / 16384.0) * 14.0 + pi / 2.0;
    active_results.angle = ang;
    
    transforms_park(alpha, beta, ang, &d, &q);
    pi_controller_d.update(dt, d, &d);
    pi_controller_p.update(dt, q, &q);


	float d_norm = d / ((2.0 / 3.0) * active_results.average_vin);
    float q_norm = q / ((2.0 / 3.0) * active_results.average_vin);

    float alpha_norm;
    float beta_norm;
    transforms_inverse_park(d_norm, q_norm, ang, &alpha_norm, &beta_norm);

    
    SVM svm_func(SVMStrategy::TOP_BOTTOM_CLAMP);
    float duty1;
    float duty2;
    float duty3;
    d = active_parameters.cmd_duty_cycle * svm_func.getMaxAmplitude();
    svm_func.computeDutyCycles(alpha_norm, beta_norm, duty1, duty2, duty3);
    gate_driver.setPWMDutyCycle(0, duty1);
    gate_driver.setPWMDutyCycle(1, duty2);
    gate_driver.setPWMDutyCycle(2, duty3);

    //float a_norm
    //float b_norm
    //float c_norm
    //transforms_inverse_clarke(alpha_norm, beta_norm, &a_norm, &b_norm, &c_norm);
   
    //apply_zsm(&motor_state.v_a_norm, &motor_state.v_b_norm, &motor_state.v_c_norm);
    //SET_DUTY(motor_state.v_a_norm, motor_state.v_b_norm, motor_state.v_c_norm);

    // pass new values into svm and set duty cycles

  }

  SVM svm;
  float dc_a, dc_b, dc_c;
  svm.computeDutyCycles(0.5f, 0.5f, dc_a, dc_b, dc_c);

}

} // namespace motor_driver
