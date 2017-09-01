#include "ch.h"
#include "hal.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "chprintf.h"
#include "peripherals.h"
#include "control.h"
#include "fw_comms.h"
#include "fast_math.h"
#include "state.h"

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
    if (gate_driver.hasFault() || gate_driver.hasOCTW()) {
      setStatusLEDColor(g < 50 ? 255 : 0, g, 0);
    } else {
      setStatusLEDColor(g, 0, g);
    }
    t = (t + 20) % 510;
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

  // Initialize state
  initState();

  // Initialize peripherals
  initPeripherals();

  // Start peripherals
  startPeripherals();

  // Start threads
  chThdCreateStatic(blinker_thread_wa, sizeof(blinker_thread_wa), LOWPRIO, blinkerThreadRun, NULL);
  chThdCreateStatic(comms_thread_wa, sizeof(comms_thread_wa), NORMALPRIO, commsThreadRun, NULL);

  chThdSetPriority(HIGHPRIO);
  runInnerControlLoop();

  // chThdSleepMilliseconds(3000);

  // float d = 0.5f;
  // int u = 0;
  // int v = 1;
  // int w = 2;
  // uint16_t t = 0;
  // uint16_t speed = 0;
  // while (true) {
  //   float arg = (float)t / 65536.0f * 2.0f * pi;
  //   gate_driver.setPWMDutyCycle(u, 0.5f + d * fast_cos(arg));
  //   gate_driver.setPWMDutyCycle(v, 0.5f + d * fast_cos(arg - 2 / 3.0f * pi));
  //   gate_driver.setPWMDutyCycle(w, 0.5f + d * fast_cos(arg - 4 / 3.0f * pi));
  //   chThdSleepMilliseconds(1);
  //   t += speed;
  //   if (speed < 2000) {
  //     speed += 10;
  //   }
  // }

  return CH_SUCCESS; // Should never get here
}

} // namespace motor_driver

// FIXME: hack
int main(void) {
  return motor_driver::main();
}
