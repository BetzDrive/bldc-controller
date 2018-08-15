#ifndef _DRV8312_H_
#define _DRV8312_H_

#include <algorithm>
#include <cmath>
#include "ch.h"
#include "hal.h"
#include "utils.h"

namespace motor_driver {

/**
 * Texas Instruments DRV8312 Three-Phase Gate Driver With Dual Current Shunt Amplifiers and Buck Regulator
 */
class DRV8312 {
public:
  DRV8312(SPIDriver& spi_driver, PWMDriver& pwm_driver,
             const pwmchannel_t ch_a, const pwmchannel_t ch_b, const pwmchannel_t ch_c,
             const IOPin nscs, const IOPin en_gate, const IOPin nfault, const IOPin noctw)
    : spi_driver_(&spi_driver),
      pwm_driver_(&pwm_driver),
      ch_a_(ch_a),
      ch_b_(ch_b),
      ch_c_(ch_c),
      nscs_(nscs),
      en_gate_(en_gate),
      nfault_(nfault),
      noctw_(noctw) {
    spi_config_.end_cb = NULL;
    spi_config_.ssport = nscs.port;
    spi_config_.sspad = nscs.pin;
    spi_config_.cr1 = SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_CPHA; // f_PCLK/16, master mode, capture data on second edge
  }
  void start();
  uint16_t readRegister(uint8_t addr);
  void writeRegister(uint8_t addr, uint16_t value);
  bool hasFault();
  bool hasOCTW();
  void setPWMPulseWidth(int phase, int32_t pulse_width);
  void setPWMDutyCycle(int phase, float duty_cycle);

private:
  SPIDriver * const spi_driver_;
  SPIConfig spi_config_;
  PWMDriver * const pwm_driver_;
  const pwmchannel_t ch_a_;
  const pwmchannel_t ch_b_;
  const pwmchannel_t ch_c_;
  const IOPin nscs_;
  const IOPin en_gate_;
  const IOPin nfault_;
  const IOPin noctw_;
};

} // namespace motor_driver

#endif /* _DRV8312_H_ */
