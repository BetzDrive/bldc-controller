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
//pi_controller_d.setPoint(0);
PID pi_controller_p(100, 10);
//pi_controller_p.setPoint(0);

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
    float ang = (((float) raw_encoder_angle - (float) active_parameters.encoder_zero) * 2.0 * pi / 16384.0) * 14.0 + pi / 2.0;
    active_results.angle = ang;

    SVM svm_func(SVMStrategy::TOP_BOTTOM_CLAMP);

    float duty1;
    float duty2;
    float duty3;
    float d = active_parameters.cmd_duty_cycle * svm_func.getMaxAmplitude();
    svm_func.computeDutyCycles(d * fast_cos(ang), d * fast_sin(ang), duty1, duty2, duty3);

    gate_driver.setPWMDutyCycle(0, duty1);
    gate_driver.setPWMDutyCycle(1, duty2);
    gate_driver.setPWMDutyCycle(2, duty3);

    // // get raw adc readings
    // curr1 = ivsense_adc_samples_ptr[0];
    // curr2 = ivsense_adc_samples_ptr[1];
    // curr3 = ivsense_adc_samples_ptr[2];
    // // convert raw values to amps
    //   // to do
    // // do the clark and other transform
    // float alpha;
    // float beta;
    // transforms_clarke(curr1, curr2, curr3, &alpha, &beta);
    // float d;
    // float q; 
    // transforms_park(alpha, beta, ang, &d, &q);
    // pi_controller_d.update(dt, d, &d);
    // pi_controller_p.update(dt, q, &q);
    // transforms_inverse_park(&d, &q, ang, &alpha, &beta);
    // transforms_inverse_clarke(alpha, beta, &curr1, &curr2, &curr3 );

    // // pass new values into svm and set duty cycles
    // todo
  }



}

} // namespace motor_driver
