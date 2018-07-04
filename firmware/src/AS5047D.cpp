#include "AS5047D.h"

static int hasEvenParity(uint8_t *buf, size_t len) {
  uint8_t acc = 0;

  for (size_t i = 0; i < len; i++) {
    acc ^= buf[i];
  }

  acc ^= acc >> 4;
  acc ^= acc >> 2;
  acc ^= acc >> 1;
  return acc & 1;
}

static void prepareTxbufForRead(uint8_t *txbuf, uint16_t addr) {
  txbuf[0] = (uint8_t)(0x40 | ((addr >> 8) & 0x3f));
  txbuf[1] = (uint8_t)(addr & 0xff);
  txbuf[0] |= (uint8_t)hasEvenParity(txbuf, 2) << 7;
}

static uint16_t getResultFromRxbuf(uint8_t *rxbuf) {
  return ((uint16_t)(rxbuf[0] & 0x3f) << 8) | (uint16_t)rxbuf[1];
}

namespace motor_driver {

void AS5047D::start() {
  spiStart(spi_driver_, &spi_config_);
}

uint16_t AS5047D::readRegister(uint16_t addr) {
  uint8_t txbuf[2];
  uint8_t rxbuf[2];

  prepareTxbufForRead(txbuf, addr);

  spiSelect(spi_driver_);
  spiSend(spi_driver_, 2, txbuf);
  spiUnselect(spi_driver_);

  halPolledDelay(NS2RTT(350));

  // Do a dummy read
  txbuf[0] = 0xc0;
  txbuf[1] = 0x00;
  spiSelect(spi_driver_);
  spiExchange(spi_driver_, 2, txbuf, rxbuf);
  spiUnselect(spi_driver_);

  return getResultFromRxbuf(rxbuf);
}

uint16_t AS5047D::getAngle() {
  return readRegister(0x3fff);
}

uint16_t AS5047D::getDiagnostics() {
  return readRegister(0x3ffc);
}

void AS5047D::startPipelinedRegisterReadI(uint16_t addr) {
  prepareTxbufForRead(pipeline_txbuf_, addr);

  spiSelectI(spi_driver_);
  spiStartExchangeI(spi_driver_, 2, pipeline_txbuf_, pipeline_rxbuf_);
}

uint16_t AS5047D::getPipelinedRegisterReadResultI() {
  return getResultFromRxbuf(pipeline_rxbuf_);
}

void AS5047D::spiEndCallbackStatic(SPIDriver *spi_driver) {
  AS5047DSPIConfig *spi_config = (AS5047DSPIConfig *)spi_driver->config;
  spi_config->as5047d->spiEndCallback(spi_driver);
}

void AS5047D::spiEndCallback(SPIDriver *spi_driver) {
  (void)spi_driver;

  chSysLockFromIsr();
  spiUnselectI(spi_driver_);
  chSysUnlockFromIsr();
}

} // namespace motor_driver
