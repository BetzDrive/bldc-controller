#include "peripherals.h"

#include "constants.h"

namespace motor_driver {
namespace peripherals {

constexpr unsigned int led_pwm_clock_freq = 84000000; // Hz
constexpr unsigned int led_pwm_period = 52500; // clock cycles

const PWMConfig led_pwm_config = {
  led_pwm_clock_freq,
  led_pwm_period,
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

void startPeripherals() {
  // Start LED PWM
  pwmStart(&PWMD5, &led_pwm_config);
}

static uint16_t ledPWMPulseWidthFromIntensity(uint8_t intensity) {
  return consts::led_gamma_table[intensity];
}

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue) {
  pwmEnableChannel(&PWMD5, 0, ledPWMPulseWidthFromIntensity(red));
  pwmEnableChannel(&PWMD5, 2, ledPWMPulseWidthFromIntensity(green));
  pwmEnableChannel(&PWMD5, 1, ledPWMPulseWidthFromIntensity(blue));
}

void setStatusLEDColor(uint32_t color) {
  setStatusLEDColor(color >> 16, color >> 8, color);
}

void setCommsActivityLED(bool on) {
  palWritePad(GPIOA, GPIOA_LED_Y, !on);
}

void setRS485TransmitMode(bool transmit) {
  palWritePad(GPIOD, GPIOD_RS485_DIR, transmit);
}

} // namespace peripherals
} // namespace motor_driver
