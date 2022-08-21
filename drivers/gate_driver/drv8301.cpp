#include "DRV8301.h"

namespace motor_driver {

void DRV8301::start() {
  spiStart(spi_driver_, &spi_config_);

  // Set control pins to default values
  pwmEnableChannel(pwm_driver_, ch_a_, 0);
  pwmEnableChannel(pwm_driver_, ch_b_, 0);
  pwmEnableChannel(pwm_driver_, ch_c_, 0);
  palSetPad(nscs_.port, nscs_.pin);

  // Enable gate driver
  palSetPad(en_gate_.port, en_gate_.pin);

  // Configure parameters over SPI
  writeRegister(0x02, 0x0400); // Set gate drive peak current to 1.7 A
}

uint16_t DRV8301::readRegister(uint8_t addr) {
  uint8_t txbuf[2] = {(uint8_t)(0x80 | ((addr << 3) & 0x78)), 0};
  uint8_t rxbuf[2];

  palClearPad(nscs_.port, nscs_.pin);
  spiExchange(spi_driver_, 2, txbuf, rxbuf);
  palSetPad(nscs_.port, nscs_.pin);
  return ((uint16_t)(rxbuf[0] & 0x07) << 8) | (uint16_t)rxbuf[1];
}

void DRV8301::writeRegister(uint8_t addr, uint16_t value) {
  uint8_t buf[2] = {(uint8_t)(((addr << 3) & 0x78) | ((value >> 8) & 0x07)),
                    (uint8_t)(value & 0xff)};

  palClearPad(nscs_.port, nscs_.pin);
  spiSend(spi_driver_, 2, buf);
  palSetPad(nscs_.port, nscs_.pin);
}

bool DRV8301::hasFault() { return !palReadPad(nfault_.port, nfault_.pin); }

bool DRV8301::hasOCTW() { return !palReadPad(noctw_.port, noctw_.pin); }

void DRV8301::setPWMPulseWidth(int phase, int32_t pulse_width) {
  pwmchannel_t ch;

  switch (phase) {
  case 0:
    ch = ch_a_;
    break;
  case 1:
    ch = ch_b_;
    break;
  case 2:
    ch = ch_c_;
    break;
  default:
    return;
  }

  pwmEnableChannel(pwm_driver_, ch, pulse_width);
}

void DRV8301::setPWMDutyCycle(int phase, float duty_cycle) {
  int32_t pulse_width = ::lround(duty_cycle * pwm_driver_->config->period);
  pulse_width = std::max(
      (int32_t)0, std::min((int32_t)pwm_driver_->config->period, pulse_width));
  setPWMPulseWidth(phase, pulse_width);
}

} // namespace motor_driver
