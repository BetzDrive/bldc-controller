#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#include <stdint.h>
#include "hal.h"
#include "DRV8312.h"
#include "AS5047D.h"
#include "MCP9808.h"
#include "IIS328DQ.h"
#include "constants.h"

namespace motor_driver {
  namespace controller {
    extern void resumeInnerControlLoop();
  }
  namespace peripherals {

    constexpr UARTDriver *rs485_uart_driver = &UARTD1;

    extern PWMConfig motor_pwm_config;

    extern DRV8312 gate_driver;

    extern const PWMConfig led_pwm_config;

    extern AS5047D encoder;

    extern MCP9808 temp_sensor;

    extern IIS328DQ acc;

    extern BinarySemaphore ivsense_adc_samples_bsem;

    extern volatile adcsample_t *vsense_adc_samples_ptr;
    extern volatile adcsample_t *curra_adc_samples_ptr;
    extern volatile adcsample_t *currb_adc_samples_ptr;
    extern volatile adcsample_t *currc_adc_samples_ptr;

    //extern volatile size_t ivsense_adc_samples_count;

    extern adcsample_t ivsense_sample_buf[(
        consts::ivsense_channel_count * consts::ivsense_sample_buf_depth)];

    extern Mutex var_access_mutex;

    void initPeripherals();

    void startPeripherals();

    void startEncoder();

    void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue);

    void setStatusLEDColor(uint32_t color);

    void setADCOn();

    void setCommsActivityLED(bool on);

    void setRS485TransmitMode(bool transmit);

    /**
     * Converts an ADC value to voltage (in volts)
     */
    inline float adcValueToVoltage(uint16_t adc_value) {
      return static_cast<float>(adc_value) * consts::vsense_voltage_per_count;
    }

    /**
     * Converts an ADC value to current (in amperes)
     */
    inline float adcValueToCurrent(uint16_t adc_value) {
      return (
          (static_cast<float>(adc_value) - consts::isense_count_zero_current) *
          consts::isense_current_per_count);
    }

  } // namespace peripherals
} // namespace motor_driver

#endif /* _PERIPHERALS_H_ */
