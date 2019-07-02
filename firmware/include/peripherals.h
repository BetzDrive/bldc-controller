#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#include <stdint.h>
#include <cstring>
#include "hal.h"
#include "flash.h"
#include "DRV8312.h"
#include "AS5047D.h"
#include "MCP9808.h"
#include "LSM6DS3Sensor.h"
#include "constants.h"

extern SerialUSBDriver SDU1;

namespace motor_driver {

constexpr UARTDriver *rs485_uart_driver = &UARTD1;

extern PWMConfig motor_pwm_config;

extern DRV8312 gate_driver;

extern const PWMConfig led_pwm_config;

extern AS5047D encoder;

extern MCP9808 temp_sensor;

extern LSM6DS3Sensor acc_gyr;

extern BinarySemaphore ivsense_adc_samples_bsem;

extern volatile adcsample_t *ivsense_adc_samples_ptr;

extern volatile size_t ivsense_adc_samples_count;

extern adcsample_t ivsense_sample_buf[ivsense_channel_count * ivsense_sample_buf_depth];

void initPeripherals();

void startPeripherals();

void startEncoder();

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue);

void setStatusLEDColor(uint32_t color);

void setADCOn();

void setCommsActivityLED(bool on);

void setRS485TransmitMode(bool transmit);

void storeCalibration();

void loadCalibration();

void clearCalibration();

/**
 * Converts an ADC value to voltage (in volts)
 */
inline float adcValueToVoltage(uint16_t adc_value) {
  return static_cast<float>(adc_value) * ivsense_voltage_per_count;
}

/**
 * Converts an ADC value to current (in amperes)
 */
inline float adcValueToCurrent(uint16_t adc_value) {
  return (ivsense_count_zero_current - static_cast<float>(adc_value)) * ivsense_current_per_count;
}

} // namespace motor_driver

#endif /* _PERIPHERALS_H_ */
