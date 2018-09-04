#include "AEAT6600.h"

static uint16_t getResultFromRxbuf(uint8_t *rxbuf) {
  return ((uint16_t)(rxbuf[0] & 0x3f) << 8) | (uint16_t)rxbuf[1];
}

namespace motor_driver {

void AEAT6600::start() {
  spiStart(spi_driver_, &spi_config_);
}

uint16_t AEAT6600::getAngle() {
  uint8_t rxbuf[2];

  spiSelect(spi_driver_);
  halPolledDelay(NS2RTT(500));
  spiReceive(spi_driver_, 2, rxbuf);
  spiUnselect(spi_driver_);

  return getResultFromRxbuf(rxbuf);
}

void AEAT6600::startPipelinedAngleReadI() {
  spiSelectI(spi_driver_);
  spiStartReceiveI(spi_driver_, 2, pipeline_rxbuf_);
}

uint16_t AEAT6600::getPipelinedResultI() {
  return getResultFromRxbuf(pipeline_rxbuf_);
}

void AEAT6600::spiEndCallbackStatic(SPIDriver *spi_driver) {
  AEAT6600SPIConfig *spi_config = (AEAT6600SPIConfig *)spi_driver->config;
  spi_config->aeat6600->spiEndCallback(spi_driver);
}

void AEAT6600::spiEndCallback(SPIDriver *spi_driver) {
  (void)spi_driver;

  chSysLockFromIsr();
  spiUnselectI(spi_driver_);
  chSysUnlockFromIsr();
}

} // namespace motor_driver
