#include "ch.h"
#include "hal.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "stdbool.h"
#include "chprintf.h"
#include "peripherals.h"
#include "control.h"
#include "comms.h"
#include "fast_math.h"
#include "state.h"
#include "constants.h"
#include "helper.h"

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
    bool fault = gate_driver.hasFault();
    bool OCTW = gate_driver.hasOCTW();
    if (fault) {
      r = g < 50 ? 255 : r;
      g = g < 50 ? 0 : g;
      parameters.gate_fault = true;
    }
    if (OCTW) {
      b = g > 200 ? 255 : b;
      g = g > 200 ? 0 : g;
      parameters.gate_fault = true;
    }
    if (not (fault or OCTW)) {
      parameters.gate_fault = false;
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

/*
 * Independent Watchdog thread
 */

static WORKING_AREA(watchdog_thread_wa, 512);
static msg_t watchdogThreadRun(void *arg) {
  (void)arg;

  chRegSetThreadName("watchdog");

  RCC->CSR |= RCC_CSR_RMVF;

  // Refer to table 107 in reference manual for configuration.
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_4); // 0.125 ms per count
  IWDG_SetReload(80);                  // 10 ms timeout
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);

  IWDG_ReloadCounter();
  IWDG_Enable();

  while (true) {
    IWDG_ReloadCounter();
    chThdSleepMilliseconds(5);
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

  // Initialize control
  initControl();

  // Start peripherals
  startPeripherals();

  // Load Calibrations from Flash
  loadCalibration();

  // Set comms activity callback
  comms_protocol_fsm.setActivityCallback(&comms_activity_callback);

  // Start threads
  chThdCreateStatic(blinker_thread_wa, sizeof(blinker_thread_wa), LOWPRIO, blinkerThreadRun, NULL);
  chThdCreateStatic(comms_thread_wa, sizeof(comms_thread_wa), HIGHPRIO, commsThreadRun, NULL);
  chThdCreateStatic(sensor_thread_wa, sizeof(sensor_thread_wa), LOWPRIO, sensorThreadRun, NULL);
  chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), NORMALPRIO, controlThreadRun, NULL);
  chThdCreateStatic(watchdog_thread_wa, sizeof(watchdog_thread_wa), HIGHPRIO, watchdogThreadRun, NULL);

  // Wait forever
  while (true) {
    chThdSleepMilliseconds(1000);
  }

  return CH_SUCCESS; // Should never get here
}

} // namespace motor_driver

extern "C" void HardFault_Handler(void) {
  flashJumpApplication((uint32_t)motor_driver::firmware_ptr);
}

// FIXME: hack
int main(void) {
  return motor_driver::main();
}
