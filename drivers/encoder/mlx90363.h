#ifndef _MLX90363_H_
#define _MLX90363_H_

#include "hal.h"
#include "utils.h"

namespace motor_driver {

class MLX90363;

struct MLX90363SPIConfig : SPIConfig {
  MLX90363 *mlx90363;
};

using mlx90363_status_t = int;
constexpr mlx90363_status_t MLX90363_STATUS_OK = 0;
constexpr mlx90363_status_t MLX90363_STATUS_WRONG_CRC = 1;
constexpr mlx90363_status_t MLX90363_STATUS_WRONG_OPCODE = 2;

using mlx90363_opcode_t = uint8_t;
constexpr mlx90363_opcode_t MLX90363_OPCODE_NOP = 16;
constexpr mlx90363_opcode_t MLX90363_OPCODE_ECHO = 17;
constexpr mlx90363_opcode_t MLX90363_OPCODE_DIAGDETAILS = 22;
constexpr mlx90363_opcode_t MLX90363_OPCODE_DIAGANSWER = 23;
constexpr mlx90363_opcode_t MLX90363_OPCODE_GET1 = 19;
constexpr mlx90363_opcode_t MLX90363_OPCODE_READY = 44;

class MLX90363 {
public:
  MLX90363(SPIDriver &spi_driver, IOPin csn)
      : spi_driver_(&spi_driver), csn_(csn) {
    spi_config_.ssport = csn.port;
    spi_config_.sspad = csn.pin;
    spi_config_.cr1 = SPI_CR1_BR_2 | SPI_CR1_MSTR | SPI_CR1_CPHA; // f_PCLK/32
    spi_config_.mlx90363 = this;
  }
  void start();
  void startAsync();
  void sendMessage(const uint8_t *txbuf);
  void receiveMessage(uint8_t *rxbuf);
  void exchangeMessage(const uint8_t *txbuf, uint8_t *rxbuf);
  void startAsyncExchangeMessageI(const uint8_t *txbuf);
  void getAsyncExchangeMessageResultI(uint8_t *rxbuf);
  void createNopMessage(uint8_t *txbuf, uint16_t key);
  void createGet1AlphaMessage(uint8_t *txbuf, uint16_t timeout);
  void createDiagnosticDetailsMessage(uint8_t *txbuf);
  mlx90363_status_t parseEchoMessage(const uint8_t *rxbuf, uint16_t *key_echo);
  mlx90363_status_t parseReadyMessage(const uint8_t *rxbuf, uint8_t *fw_version,
                                      uint8_t *hw_version);
  mlx90363_status_t parseAlphaMessage(const uint8_t *rxbuf, uint16_t *alpha,
                                      uint8_t *vg);
  mlx90363_status_t parseDiagnosticsAnswerMessage(const uint8_t *rxbuf,
                                                  uint32_t *diag_bits,
                                                  uint8_t *fsmerc,
                                                  uint8_t *anadiagcnt);
  uint8_t computeMessageCRC(const uint8_t *buf);

private:
  SPIDriver *const spi_driver_;
  MLX90363SPIConfig spi_config_;
  const IOPin csn_;
  uint8_t async_txbuf_[8];
  uint8_t async_rxbuf_[8];

  static void spiEndCallbackStatic(SPIDriver *spi_driver);
  void spiEndCallback(SPIDriver *spi_driver);
};

} // namespace motor_driver

#endif /* _MLX90363_H_ */
