#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#include <stdint.h>
#include "hal.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"

/* Include Hardware Drivers */
#include "DRV8312.h"
#include "AS5047D.h"
#include "MCP9808.h"
#include "LSM6DS3Sensor.h"

/* Include Config Files */
#include "constants.h"
#include "hw_conf.h"

namespace motor_driver {

constexpr UARTDriver *rs485_uart_driver = &UARTD1;

extern PWMConfig motor_pwm_config;

extern DRV8312 gate_driver;

extern const PWMConfig led_pwm_config;

extern AS5047D encoder;

extern MCP9808 temp_sensor;

extern LSM6DS3Sensor acc_gyr;

void initPeripherals();

void startEncoder();

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue);

void setStatusLEDColor(uint32_t color);

void setADCOn();

void setCommsActivityLED(bool on);

void setRS485TransmitMode(bool transmit);

/**
 * Converts an ADC value to voltage (in volts)
 */
inline float adcValueToVoltage(float adc_value) {
  return adc_value * ivsense_voltage_per_count;
}

/**
 * Converts an ADC value to current (in amperes)
 */
inline float adcValueToCurrent(float adc_value) {
  return (adc_value - ivsense_count_zero_current) * ivsense_current_per_count;
}

} // namespace motor_driver

#endif /* _PERIPHERALS_H_ */
