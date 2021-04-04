#include "MLX90363.h"

#include "crc_mlx.h"
#include <algorithm>

namespace motor_driver {

void MLX90363::start() {
  spi_config_.end_cb = NULL;
  spiStart(spi_driver_, &spi_config_);
}

void MLX90363::startAsync() {
  spi_config_.end_cb = spiEndCallbackStatic;
  spiStart(spi_driver_, &spi_config_);
}

void MLX90363::sendMessage(const uint8_t *txbuf) {
  spiSelect(spi_driver_);
  spiSend(spi_driver_, 8, txbuf);
  spiUnselect(spi_driver_);
}

void MLX90363::receiveMessage(uint8_t *rxbuf) {
  // Need to send a dummy command
  uint8_t txbuf[8];
  createNopMessage(txbuf, 0);

  spiSelect(spi_driver_);
  spiExchange(spi_driver_, 8, txbuf, rxbuf);
  spiUnselect(spi_driver_);
}

void MLX90363::exchangeMessage(const uint8_t *txbuf, uint8_t *rxbuf) {
  spiSelect(spi_driver_);
  spiExchange(spi_driver_, 8, txbuf, rxbuf);
  spiUnselect(spi_driver_);
}

void MLX90363::startAsyncExchangeMessageI(const uint8_t *txbuf) {
  std::copy(txbuf, txbuf + 8, async_txbuf_);

  spiSelectI(spi_driver_);
  spiStartExchangeI(spi_driver_, 8, async_txbuf_, async_rxbuf_);
}

void MLX90363::getAsyncExchangeMessageResultI(uint8_t *rxbuf) {
  std::copy(async_rxbuf_, async_rxbuf_ + 8, rxbuf);
}

void MLX90363::createNopMessage(uint8_t *txbuf, uint16_t key) {
  txbuf[0] = 0;
  txbuf[1] = 0;
  txbuf[2] = key & 0xff;
  txbuf[3] = (key >> 8) & 0xff;
  txbuf[4] = 0;
  txbuf[5] = 0;
  txbuf[6] = (3 << 6) | MLX90363_OPCODE_NOP;
  txbuf[7] = computeMessageCRC(txbuf);
}

void MLX90363::createGet1AlphaMessage(uint8_t *txbuf, uint16_t timeout) {
  txbuf[0] = 0;
  txbuf[1] = 0;
  txbuf[2] = timeout & 0xff;
  txbuf[3] = (timeout >> 8) & 0xff;
  txbuf[4] = 0;
  txbuf[5] = 0;
  txbuf[6] = (0 << 6) | MLX90363_OPCODE_GET1;
  txbuf[7] = computeMessageCRC(txbuf);
}

void MLX90363::createDiagnosticDetailsMessage(uint8_t *txbuf) {
  txbuf[0] = 0;
  txbuf[1] = 0;
  txbuf[2] = 0;
  txbuf[3] = 0;
  txbuf[4] = 0;
  txbuf[5] = 0;
  txbuf[6] = (3 << 6) | MLX90363_OPCODE_DIAGDETAILS;
  txbuf[7] = computeMessageCRC(txbuf);
}

mlx90363_status_t MLX90363::parseEchoMessage(const uint8_t *rxbuf,
                                             uint16_t *key_echo) {
  mlx90363_status_t status = MLX90363_STATUS_OK;

  if (rxbuf[7] != computeMessageCRC(rxbuf)) {
    status |= MLX90363_STATUS_WRONG_CRC;
    return status;
  }

  if (rxbuf[6] != ((3 << 6) | MLX90363_OPCODE_ECHO)) {
    status |= MLX90363_STATUS_WRONG_OPCODE;
    return status;
  }

  if (key_echo != nullptr) {
    *key_echo = ((uint16_t)rxbuf[3] << 8) | (uint16_t)rxbuf[2];
  }

  return status;
}

mlx90363_status_t MLX90363::parseReadyMessage(const uint8_t *rxbuf,
                                              uint8_t *fw_version,
                                              uint8_t *hw_version) {
  mlx90363_status_t status = MLX90363_STATUS_OK;

  if (rxbuf[7] != computeMessageCRC(rxbuf)) {
    status |= MLX90363_STATUS_WRONG_CRC;
    return status;
  }

  if (rxbuf[6] != ((3 << 6) | MLX90363_OPCODE_READY)) {
    status |= MLX90363_STATUS_WRONG_OPCODE;
    return status;
  }

  if (fw_version != nullptr) {
    *fw_version = rxbuf[1];
  }

  if (hw_version != nullptr) {
    *hw_version = rxbuf[0];
  }

  return status;
}

mlx90363_status_t MLX90363::parseAlphaMessage(const uint8_t *rxbuf,
                                              uint16_t *alpha, uint8_t *vg) {
  mlx90363_status_t status = MLX90363_STATUS_OK;

  if (rxbuf[7] != computeMessageCRC(rxbuf)) {
    status |= MLX90363_STATUS_WRONG_CRC;
    return status;
  }

  if (alpha != nullptr) {
    *alpha = ((uint16_t)(rxbuf[1] & 0x3f) << 8) | (uint16_t)rxbuf[0];
  }

  if (vg != nullptr) {
    *vg = rxbuf[4];
  }

  return status;
}

mlx90363_status_t MLX90363::parseDiagnosticsAnswerMessage(const uint8_t *rxbuf,
                                                          uint32_t *diag_bits,
                                                          uint8_t *fsmerc,
                                                          uint8_t *anadiagcnt) {
  mlx90363_status_t status = MLX90363_STATUS_OK;

  if (rxbuf[7] != computeMessageCRC(rxbuf)) {
    status |= MLX90363_STATUS_WRONG_CRC;
    return status;
  }

  if (rxbuf[6] != ((3 << 6) | MLX90363_OPCODE_DIAGANSWER)) {
    status |= MLX90363_STATUS_WRONG_OPCODE;
    return status;
  }

  if (diag_bits != nullptr) {
    *diag_bits = ((uint32_t)rxbuf[2] << 16) | ((uint32_t)rxbuf[1] << 8) |
                 (uint32_t)rxbuf[0];
  }

  if (fsmerc != nullptr) {
    *fsmerc = rxbuf[3] >> 6;
  }

  if (anadiagcnt != nullptr) {
    *anadiagcnt = rxbuf[3] & 0x3f;
  }

  return status;
}

uint8_t MLX90363::computeMessageCRC(const uint8_t *buf) {
  uint8_t crc = crc_mlx_init();
  crc = crc_mlx_update(crc, buf, 7);
  return crc_mlx_finalize(crc);
}

void MLX90363::spiEndCallbackStatic(SPIDriver *spi_driver) {
  MLX90363SPIConfig *spi_config = (MLX90363SPIConfig *)spi_driver->config;
  spi_config->mlx90363->spiEndCallback(spi_driver);
}

void MLX90363::spiEndCallback(SPIDriver *spi_driver) {
  (void)spi_driver;

  chSysLockFromIsr();
  spiUnselectI(spi_driver_);
  chSysUnlockFromIsr();
}

} // namespace motor_driver
