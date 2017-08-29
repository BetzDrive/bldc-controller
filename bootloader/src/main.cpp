#include "ch.h"
#include "hal.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "peripherals.h"
#include "bl_comms.h"

namespace motor_driver {

/*
 * LED blinker thread
 */

static WORKING_AREA(blinker_thread_wa, 128);
static msg_t blinkerThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("blinker");

  int t = 0;

  // setStatusLEDColor(0, 255, 128);

  while (true) {

    // palSetPad(GPIOA, GPIOA_LED_B);
   //  palClearPad(GPIOA, GPIOA_LED_R);
   //  chThdSleepMilliseconds(250);
   //  palSetPad(GPIOA, GPIOA_LED_R);
   //  palClearPad(GPIOA, GPIOA_LED_G);
   //  chThdSleepMilliseconds(250);
   //  palSetPad(GPIOA, GPIOA_LED_G);
   //  palClearPad(GPIOA, GPIOA_LED_B);
   //  chThdSleepMilliseconds(250);
    // palClearPad(GPIOA, GPIOA_LED_G);
    // chThdSleepMilliseconds(250);
    // palSetPad(GPIOA, GPIOA_LED_G);
    // chThdSleepMilliseconds(250);

    uint8_t g = ::abs(t - 255);
    setStatusLEDColor(0, 0, g);
    t = (t + 10) % 510;
    chThdSleepMilliseconds(10);
  }

  return CH_SUCCESS; // Should never get here
}

/*
 * Communications thread
 */

static WORKING_AREA(comms_thread_wa, 128);
static msg_t commsThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("comms");

  startComms();

  while (true) {
    runComms();
  }

  return CH_SUCCESS; // Should never get here
}

int main(void) {
  // Start RTOS
  halInit();
  chSysInit();

  // Start peripherals
  startPeripherals();

  // Start threads
  chThdCreateStatic(blinker_thread_wa, sizeof(blinker_thread_wa), LOWPRIO, blinkerThreadRun, NULL);
  chThdCreateStatic(comms_thread_wa, sizeof(comms_thread_wa), NORMALPRIO, commsThreadRun, NULL);

  while (true) {
    chThdSleepMilliseconds(1000);
  }

  return CH_SUCCESS; // Should never get here
}

} // namespace motor_driver

// FIXME: hack
int main(void) {
  return motor_driver::main();
}
