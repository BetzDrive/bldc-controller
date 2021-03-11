#ifndef _IIS328DQ_H_
#define _IIS328DQ_H_

#include "hal.h"

namespace motor_driver {
  namespace peripherals {

    const static uint8_t IIS328DQ_DEFAULT_ADDRESS = 0b0011000;

    const static uint8_t IIS328DQ_WHO_AM_I_ADDRESS = 0x0F;
    const static uint8_t IIS328DQ_WHO_AM_I = 0b00110010;

    const static uint8_t IIS328DQ_CTRL_REG4 = 0x23;

    const static uint8_t IIS328DQ_OUT_X_L = 0x28;
    const static uint8_t IIS328DQ_OUT_X_H = 0x29;

    const static uint8_t IIS328DQ_OUT_Y_L = 0x2A;
    const static uint8_t IIS328DQ_OUT_Y_H = 0x2B;

    const static uint8_t IIS328DQ_OUT_Z_L = 0x2C;
    const static uint8_t IIS328DQ_OUT_Z_H = 0x2D;

    const static uint8_t IIS328DQ_MASK_SUB = 0x80;

    class IIS328DQ {
      public:
        IIS328DQ(I2CDriver& i2c_driver): i2c_driver_(&i2c_driver) {
          i2c_config_.op_mode = OPMODE_I2C;
          i2c_config_.clock_speed = 400000;
          i2c_config_.duty_cycle = FAST_DUTY_CYCLE_2;
        }
        void start();
        bool receive(uint8_t reg, uint8_t* data, size_t size);
        bool getAccel(int16_t* accel_arr);
        bool checkID();

      private:
        I2CDriver * const i2c_driver_;
        I2CConfig i2c_config_;
    };

  } // namespace peripherals
} // namespace motor_driver


#endif /* _IIS328DQ_H_ */
