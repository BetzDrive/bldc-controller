#ifndef _MCP9808_H_
#define _MCP9808_H_

#include "hal.h"

namespace motor_driver {
namespace peripherals {

const static uint8_t MCP9808_DEFAULT_ADDRESS = 0b0011000;
const static uint8_t MCP9808_TEMP_AMBIENT = 0x05;

class MCP9808 {
public:
  MCP9808(I2CDriver& i2c_driver): i2c_driver_(&i2c_driver) {
    i2c_config_.op_mode = OPMODE_I2C;
    i2c_config_.clock_speed = 400000;
    i2c_config_.duty_cycle = FAST_DUTY_CYCLE_2;
  }
  void start();
  bool receive(uint16_t addr, uint8_t reg, uint8_t* data, size_t size);
  bool getTemperature(float* temp);
  bool checkID();

private:
  I2CDriver * const i2c_driver_;
  I2CConfig i2c_config_;
};

} // namespace peripherals
} // namespace motor_driver


#endif /* _MCP9808_H_ */
