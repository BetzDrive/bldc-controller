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

  chSysLock();

  results.encoder_angle = encoder.getPipelinedRegisterReadResultI();
  encoder.startPipelinedRegisterReadI(0x3fff);

  chSysUnlock();

  // results.encoder_angle = encoder.readRegister(0x3fff);

  // results.encoder_diag = encoder.readRegister(0x3ffc);

  palSetPad(GPIOA, GPIOA_LED_Y);
}

} // namespace motor_driver
