#include "ch.h"
#include "hal.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "chprintf.h"
#include "peripherals.h"
#include "control.h"
#include "comms.h"
#include "fast_math.h"
#include "state.h"
#include "constants.h"

namespace motor_driver {

static systime_t last_comms_activity_time = 0;

static void comms_activity_callback() {
  resetControlTimeout();
  last_comms_activity_time = chTimeNow();
}

/*
 * LED blinker thread
 */

static WORKING_AREA(blinker_thread_wa, 512);
static msg_t blinkerThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("blinker");

  int t = 0;

  setCommsActivityLED(false);

  while (true) {
    uint8_t g = ::abs(t - 255);
    uint8_t r = 0;
    uint8_t b = 0;
    if (gate_driver.hasFault()) {
      r = g < 50 ? 255 : r;
      g = g < 50 ? 0 : g;
    } 
    if (gate_driver.hasOCTW()) {
      r = g > 200 ? 255 : r;
      g = g > 200 ? 255 : g;
    }

    setStatusLEDColor(r,g,b);

    systime_t time_now = chTimeNow();

    setCommsActivityLED(time_now - last_comms_activity_time < MS2ST(comms_activity_led_duration) &&
                        last_comms_activity_time != 0);

    t = (t + 10) % 510;
    chThdSleepMilliseconds(10);
  }

  return CH_SUCCESS; // Should never get here
}

/*
 * Communications thread
 */

static WORKING_AREA(comms_thread_wa, 512);
static msg_t commsThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("comms");

  startComms();

  while (true) {
    runComms();
  }

  return CH_SUCCESS; // Should never get here
}

/*
 * Sensor thread
 */

static WORKING_AREA(sensor_thread_wa, 512);
static msg_t sensorThreadRun(void *arg) {
  (void)arg;
  
  chRegSetThreadName("sensor");

  while (true) {
    int32_t xl[3];
    float temperature;
    acc_gyr.Get_Acc(xl);
    temp_sensor.getTemperature(&temperature);
    results.xl_x = xl[0];
    results.xl_y = xl[1];
    results.xl_z = xl[2];
    results.temperature = temperature;
    chThdSleepMilliseconds(100);
  }

  return CH_SUCCESS;
}

/*
 * Control thread
 */

static WORKING_AREA(control_thread_wa, 512);
static msg_t controlThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("control");

  runInnerControlLoop();

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

  // Initialize control
  initControl();

  // Start peripherals
  startPeripherals();

  // Set comms activity callback
  comms_protocol_fsm.setActivityCallback(&comms_activity_callback);

  // Start threads
  chThdCreateStatic(blinker_thread_wa, sizeof(blinker_thread_wa), LOWPRIO, blinkerThreadRun, NULL);
  chThdCreateStatic(comms_thread_wa, sizeof(comms_thread_wa), NORMALPRIO, commsThreadRun, NULL);
  chThdCreateStatic(sensor_thread_wa, sizeof(sensor_thread_wa), LOWPRIO, sensorThreadRun, NULL);
  chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), HIGHPRIO, controlThreadRun, NULL);

  // Wait forever
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
