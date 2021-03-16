#include "hal.h"

#include "ch.h"
#include "chprintf.h"
#include "comms.h"
#include "constants.h"
#include "control.h"
#include "fast_math.h"
#include "helper.h"
#include "peripherals.h"
#include "state.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "string.h"

namespace motor_driver {

static systime_t last_comms_activity_time = 0;

static void comms_activity_callback() {
  controller::resetControlTimeout();
  last_comms_activity_time = chTimeNow();
}

/*
 * LED blinker thread
 */
bool acc_success = false;
static WORKING_AREA(blinker_thread_wa, 512);
static msg_t blinkerThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("blinker");

  int t = 0;

  peripherals::setCommsActivityLED(false);

  while (true) {
    uint8_t r = 0;
    uint8_t g = ::abs(t - 255);
    uint8_t b = 0;
    bool fault = peripherals::gate_driver.hasFault();
    bool OCTW = peripherals::gate_driver.hasOCTW();
    // bool valid_acc = peripherals::acc.checkID();

    chMtxLock(&peripherals::var_access_mutex);
    if (fault or not acc_success) {
      r = g < 50 ? 255 : r;
      g = g < 50 ? 0 : g;
      state::parameters.gate_fault = true;
    }
    if (OCTW) {
      b = g > 200 ? 255 : b;
      g = g > 200 ? 0 : g;
      state::parameters.gate_fault = true;
    }
    if (not(fault or OCTW)) {
      state::parameters.gate_fault = false;
    }
    chMtxUnlock();

    peripherals::setStatusLEDColor(r, g, b);

    systime_t time_diff = chTimeNow() - last_comms_activity_time;

    peripherals::setCommsActivityLED(
        time_diff < MS2ST(consts::comms_activity_led_duration) &&
        last_comms_activity_time != 0);

    t = (t + 10) % 510;
    chThdSleepMilliseconds(10);
  }

  // Should never get here.
  return CH_SUCCESS;
}

/*
 * Communications thread
 */

static WORKING_AREA(comms_thread_wa, 512);
static msg_t commsThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("comms");

  comms::startComms();

  while (true) {
    comms::runComms();
    if (state::parameters.timeout_flag)
      comms::setWDGTimeout();
    else
      comms::clearWDGTimeout();
  }

  // Should never get here.
  return CH_SUCCESS;
}

/*
 * Sensor thread
 */

static WORKING_AREA(sensor_thread_wa, 512);
static msg_t sensorThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("sensor");

  int16_t xl[3];
  float temperature;
  while (true) {
    acc_success = peripherals::acc.getAccel(xl);
    peripherals::temp_sensor.getTemperature(&temperature);

    chMtxLock(&peripherals::var_access_mutex);
    state::results.xl_x = xl[0];
    state::results.xl_y = xl[1];
    state::results.xl_z = xl[2];
    state::results.temperature = temperature;
    chMtxUnlock();

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

  controller::runInnerControlLoop();

  // Should never get here.
  return CH_SUCCESS;
}

/*
 * Independent Watchdog thread
 */

static WORKING_AREA(watchdog_thread_wa, 512);
static msg_t watchdogThreadRun(void *arg) {
  (void)arg;

  /*
   * The CSR WDG RST flag is handled in the comms code which
   *   notifies the host/enables them to clear the flag.
   */

  chRegSetThreadName("watchdog");

  // Refer to table 107 in reference manual for configuration.
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  // 0.125 ms per count.
  IWDG_SetPrescaler(IWDG_Prescaler_4);
  // 10 ms timeout.
  IWDG_SetReload(80);
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);

  IWDG_ReloadCounter();
  IWDG_Enable();

  while (true) {
    IWDG_ReloadCounter();
    chThdSleepMilliseconds(5);
  }

  // Should never get here.
  return CH_SUCCESS;
}

int main(void) {
  // Start RTOS
  halInit();
  chSysInit();

  // Start Watchdog Immediately.
  chThdCreateStatic(watchdog_thread_wa, sizeof(watchdog_thread_wa), HIGHPRIO,
                    watchdogThreadRun, NULL);

  // Initialize state
  state::initState();

  // Initialize peripherals
  peripherals::initPeripherals();

  // Initialize control
  controller::initControl();

  // Start peripherals
  peripherals::startPeripherals();

  // Load Calibrations from Flash
  state::loadCalibration();

  // Set comms activity callback
  comms::comms_protocol_fsm.setActivityCallback(&comms_activity_callback);

  // Start threads
  chThdCreateStatic(blinker_thread_wa, sizeof(blinker_thread_wa), LOWPRIO,
                    blinkerThreadRun, NULL);
  chThdCreateStatic(sensor_thread_wa, sizeof(sensor_thread_wa), LOWPRIO,
                    sensorThreadRun, NULL);
  chThdCreateStatic(comms_thread_wa, sizeof(comms_thread_wa), NORMALPRIO,
                    commsThreadRun, NULL);
  chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), HIGHPRIO,
                    controlThreadRun, NULL);

  // Wait forever
  while (true) {
    chThdSleepMilliseconds(1000);
  }

  // Should never get here.
  return CH_SUCCESS;
}

} // namespace motor_driver

extern "C" void HardFault_Handler(void) {
  flashJumpApplication((uint32_t)motor_driver::consts::firmware_ptr);
}

// FIXME: hack
int main(void) { return motor_driver::main(); }
