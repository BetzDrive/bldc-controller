#ifndef _AEAT6600_H_
#define _AEAT6600_H_

#include "hal.h"
#include "utils.h"

namespace motor_driver {

class AEAT6600;

struct AEAT6600SPIConfig : SPIConfig {
  AEAT6600 *aeat6600;
};

class AEAT6600 {
public:
  AEAT6600(SPIDriver &spi_driver, IOPin csn)
      : spi_driver_(&spi_driver), csn_(csn) {
    spi_config_.end_cb = spiEndCallbackStatic;
    spi_config_.ssport = csn.port;
    spi_config_.sspad = csn.pin;
    spi_config_.cr1 = SPI_CR1_BR_1 | // SPI_CR1_BR_1 | //SPI_CR1_BR_0 |
                      SPI_CR1_MSTR | SPI_CR1_CPOL;
    spi_config_.aeat6600 = this;
  }
  void start();
  uint16_t getAngle();
  void startPipelinedAngleReadI();
  uint16_t getPipelinedResultI();

private:
  SPIDriver *const spi_driver_;
  AEAT6600SPIConfig spi_config_;
  const IOPin csn_;
  uint8_t pipeline_rxbuf_[2];

  static void spiEndCallbackStatic(SPIDriver *spi_driver);
  void spiEndCallback(SPIDriver *spi_driver);
};

} // namespace motor_driver

#endif /* _AEAT6600_H_ */
