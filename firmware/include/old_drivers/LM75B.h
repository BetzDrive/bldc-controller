#ifndef _LM75B_H_
#define _LM75B_H_

#include "hal.h"

namespace motor_driver {

class LM75B {
public:
  LM75B(I2CDriver& i2c_driver): i2c_driver_(&i2c_driver) {
    i2c_config_.op_mode = OPMODE_I2C;
    i2c_config_.clock_speed = 400000;
    i2c_config_.duty_cycle = FAST_DUTY_CYCLE_2;
  }
  void start();
  bool receive(uint16_t addr, uint8_t* data, size_t size);
  bool getTemperature(float* temp);

private:
  I2CDriver * const i2c_driver_;
  I2CConfig i2c_config_;
};

} // namespace motor_driver


#endif /* _LM75B_H_ */
