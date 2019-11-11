#include "IIS328DQ.h"

namespace motor_driver {
namespace peripherals {

void IIS328DQ::start() {
  uint8_t config[3];
  systime_t tmo = MS2ST(4); // 4 millisecond timeout

  config[0] = 0x01;
  config[1] = 0x00;
  config[2] = 0x08;

  i2cStart(i2c_driver_, &i2c_config_);

  i2cAcquireBus(i2c_driver_);

  i2cMasterTransmitTimeout(i2c_driver_, 
      IIS328DQ_DEFAULT_ADDRESS,         // Address
      config, 3,                        // TX Buffer, Len 
      NULL, 0,                          // RX Buffer, Len
      tmo);                             // Timeout

  i2cReleaseBus(i2c_driver_);
}

bool IIS328DQ::checkID() {
  uint8_t id = 0;

  bool success = IIS328DQ::receive(IIS328DQ_WHO_AM_I_ADDRESS, &id, 1);

  return (success && (id == IIS328DQ_WHO_AM_I));
}

bool IIS328DQ::receive(uint8_t reg, uint8_t* data, size_t size) {
  systime_t tmo = MS2ST(4); // 4 millisecond timeout
  i2cAcquireBus(i2c_driver_);

  msg_t status = i2cMasterTransmitTimeout(i2c_driver_, IIS328DQ_DEFAULT_ADDRESS, &reg, 1, data, size, tmo);

  i2cReleaseBus(i2c_driver_);
  return status == RDY_OK;
}

/* Expects accel_arr to be of size 3 and stores x,y,z in that order */
bool IIS328DQ::getAccel(int16_t* accel_arr) {
  return IIS328DQ::receive(IIS328DQ_OUT_X_L, (uint8_t*) accel_arr, 6);
}

} // namespace peripherals
} // namespace motor_driver
