#include "control.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"

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
    gate_driver.setPWMDutyCycle(0, 0);
    gate_driver.setPWMDutyCycle(1, 0);
    gate_driver.setPWMDutyCycle(2, 0.4);
  }



}

} // namespace motor_driver
