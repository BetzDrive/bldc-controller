#include "DRV8312.h"

namespace motor_driver {

void DRV8312::start() {

  // Set control pins to default values
  pwmEnableChannel(pwm_driver_, ch_a_, 0);
  pwmEnableChannel(pwm_driver_, ch_b_, 0);
  pwmEnableChannel(pwm_driver_, ch_c_, 0);

  // Enable control of gates.
  enableGates();

  // Enable gate driver
  palSetPad(en_gate_.port, en_gate_.pin);
}

bool DRV8312::hasFault() {
  return !palReadPad(nfault_.port, nfault_.pin);
}

bool DRV8312::hasOCTW() {
  return !palReadPad(noctw_.port, noctw_.pin);
}

void DRV8312::setPWMPulseWidth(int phase, int32_t pulse_width) {
  pwmchannel_t ch;

  switch(phase) {
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

void DRV8312::setPWMDutyCycle(int phase, float duty_cycle) {
  int32_t pulse_width = ::lround((1-duty_cycle) * pwm_driver_->config->period);
  pulse_width = std::max((int32_t)0, std::min((int32_t)pwm_driver_->config->period, pulse_width));
  setPWMPulseWidth(phase, pulse_width);
}

void DRV8312::enableGates()
{
  // Set reset pins to high to allow for control of motors (active low)!
  palSetPad(rst_a_.port, rst_a_.pin);
  palSetPad(rst_b_.port, rst_b_.pin);
  palSetPad(rst_c_.port, rst_c_.pin);
}

void DRV8312::disableGates()
{
  // Set reset pins to low to stop control of motors (active low)!
  palClearPad(rst_a_.port, rst_a_.pin);
  palClearPad(rst_b_.port, rst_b_.pin);
  palClearPad(rst_c_.port, rst_c_.pin);
}

} // namespace motor_driver
