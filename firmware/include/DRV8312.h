#ifndef _DRV8312_H_
#define _DRV8312_H_

#include "hal.h"

#include "ch.h"
#include "utils.h"

#include <algorithm>
#include <cmath>

namespace motor_driver {
namespace peripherals {

// Texas Instruments DRV8312 Three-Phase Gate Driver With Dual Current
// Shunt Amplifiers and Buck Regulator
class DRV8312 {
public:
  DRV8312(PWMDriver &pwm_driver, const pwmchannel_t ch_a,
          const pwmchannel_t ch_b, const pwmchannel_t ch_c, const IOPin rst_a,
          const IOPin rst_b, const IOPin rst_c, const IOPin nfault,
          const IOPin noctw)
      : pwm_driver_(&pwm_driver), ch_a_(ch_a), ch_b_(ch_b), ch_c_(ch_c),
        rst_a_(rst_a), rst_b_(rst_b), rst_c_(rst_c), nfault_(nfault),
        noctw_(noctw) {}
  void start();
  bool hasFault();
  bool hasOCTW();
  void setPWMPulseWidth(int phase, int32_t pulse_width);
  void setPWMDutyCycle(int phase, float duty_cycle);
  void enableGates();
  void disableGates();

private:
  PWMDriver *const pwm_driver_;
  const pwmchannel_t ch_a_;
  const pwmchannel_t ch_b_;
  const pwmchannel_t ch_c_;
  const IOPin rst_a_;
  const IOPin rst_b_;
  const IOPin rst_c_;
  const IOPin nfault_;
  const IOPin noctw_;
};

} // namespace peripherals
} // namespace motor_driver

#endif /* _DRV8312_H_ */
