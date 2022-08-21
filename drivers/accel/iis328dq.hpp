#pragma once

#include "hal.h"

namespace motor_driver {
namespace peripherals {

constexpr uint8_t IIS328DQ_DEFAULT_ADDRESS = 0b0011000;

constexpr uint8_t IIS328DQ_WHO_AM_I_ADDRESS = 0x0F;
constexpr uint8_t IIS328DQ_WHO_AM_I = 0b00110010;

constexpr uint8_t IIS328DQ_CTRL_REG4 = 0x23;

constexpr uint8_t IIS328DQ_OUT_X_L = 0x28;
constexpr uint8_t IIS328DQ_OUT_X_H = 0x29;

constexpr uint8_t IIS328DQ_OUT_Y_L = 0x2A;
constexpr uint8_t IIS328DQ_OUT_Y_H = 0x2B;

constexpr uint8_t IIS328DQ_OUT_Z_L = 0x2C;
constexpr uint8_t IIS328DQ_OUT_Z_H = 0x2D;

constexpr uint8_t IIS328DQ_MASK_SUB = 0x80;

class IIS328DQ {
public:
  explicit IIS328DQ(I2CDriver &i2c_driver) : i2c_driver_(&i2c_driver) {
    i2c_config_.op_mode = OPMODE_I2C;
    i2c_config_.clock_speed = 400000;
    i2c_config_.duty_cycle = FAST_DUTY_CYCLE_2;
  }
  void start();
  bool receive(uint8_t reg, uint8_t *data, size_t size);
  bool getAccel(int16_t *accel_arr);
  bool checkID();

private:
  I2CDriver *const i2c_driver_;
  I2CConfig i2c_config_;
};

} // namespace peripherals
} // namespace motor_driver
