#ifndef _AS5047D_H_
#define _AS5047D_H_

#include "hal.h"
#include "utils.h"

namespace motor_driver {

class AS5047D;

struct AS5047DSPIConfig : SPIConfig {
  AS5047D *as5047d;
};

class AS5047D {
public:
  AS5047D(SPIDriver& spi_driver, IOPin csn)
    : spi_driver_(&spi_driver),
      csn_(csn) {
    spi_config_.end_cb = spiEndCallbackStatic;
    spi_config_.ssport = csn.port;
    spi_config_.sspad = csn.pin;
    spi_config_.cr1 = SPI_CR1_BR_1 | SPI_CR1_MSTR | SPI_CR1_CPHA; // f_PCLK/8
    spi_config_.as5047d = this;
  }
  void start();
  uint16_t readRegister(uint16_t addr);
  // void writeRegister(uint16_t addr, uint16_t value);
  uint16_t getAngle();
  void startPipelinedRegisterReadI(uint16_t addr);
  uint16_t getPipelinedRegisterReadResultI();

private:
  SPIDriver * const spi_driver_;
  AS5047DSPIConfig spi_config_;
  const IOPin csn_;
  uint8_t pipeline_txbuf_[2];
  uint8_t pipeline_rxbuf_[2];

  static void spiEndCallbackStatic(SPIDriver *spi_driver);
  void spiEndCallback(SPIDriver *spi_driver);
};

} // namespace motor_driver

#endif /* _AS5047D_H_ */
