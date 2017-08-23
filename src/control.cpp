#include "control.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"

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
  // encoder.readRegister(0x3fff);
}

} // namespace motor_driver
