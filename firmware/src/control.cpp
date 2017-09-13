#include "control.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include "fast_math.h"

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
    float ang = ((float) raw_encoder_angle - (float) active_parameters.encoder_zero) * 2.0 * pi / 16384.0 + pi / 2.0;
    active_results.angle = ang;
    float d = active_parameters.cmd_duty_cycle;
    gate_driver.setPWMDutyCycle(0, 0.5f + d * fast_cos(ang));
    gate_driver.setPWMDutyCycle(1, 0.5f + d * fast_cos(ang - 2 / 3.0f * pi));
    gate_driver.setPWMDutyCycle(2, 0.5f + d * fast_cos(ang - 4 / 3.0f * pi));
  }



}

} // namespace motor_driver
