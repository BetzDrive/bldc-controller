/*
 * Baremetal BLDC controller main entry point.
 * Cooperative loop: no RTOS, all logic runs in the main loop
 * with ISR-driven motor control (TIM1 update).
 */

#include "baremetal_config.h"
#include "comms.h"
#include "control.h"
#include "gpio.h"
#include "hal/hal_adc.h"
#include "hal/hal_flash.h"
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "hal/hal_iwdg.h"
#include "hal/hal_pwm.h"
#include "hal/hal_spi.h"
#include "hal/hal_timer.h"
#include "hal/hal_uart.h"
#include "led.h"
#include "sensor.h"
#include "state.h"
#include "system.h"

int main(void) {
  system_init();
  gpio_init();

  hal_iwdg_init();
  hal_pwm_motor_init();
  hal_pwm_led_init();
  hal_adc_init();
  hal_spi_init();
  hal_i2c_init();
  hal_uart_init(1000000); /* 1 Mbit/s RS485 */

  state_init();
  state_load_calibration();
  control_init();
  comms_init();
  sensor_init();
  led_init();

  hal_adc_start();
  hal_pwm_motor_start();

  /* Cooperative main loop */
  while (1) {
    hal_iwdg_kick();
    comms_step(COMMS_MAX_BYTES_PER_STEP);

    /* Handle deferred actions from comms */
    uint32_t jump = comms_get_jump_addr();
    if (jump != 0) {
      hal_flash_jump(jump);
    }
    if (comms_should_reset()) {
      NVIC_SystemReset();
    }

    sensor_step();
    control_step();
    led_step();
  }

  return 0;
}
