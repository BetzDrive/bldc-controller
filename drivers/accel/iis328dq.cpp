// Datasheet: https://www.st.com/resource/en/datasheet/iis328dq.pdf

#include "drivers/accel/iis328dq.hpp"

namespace motor_driver {
namespace peripherals {

void IIS328DQ::start() {
  uint8_t config[2];
  systime_t tmo = MS2ST(4); // 4 millisecond timeout

  config[0] = 0x20;
  config[1] = 0x27;

  // i2cStart should only be called once; for now called on temperature
  // sensor init.
  // i2cStart(i2c_driver_, &i2c_config_);

  i2cAcquireBus(i2c_driver_);

  i2cMasterTransmitTimeout(i2c_driver_,
                           IIS328DQ_DEFAULT_ADDRESS, // Address
                           config, 2,                // TX Buffer, Len
                           NULL, 0,                  // RX Buffer, Len
                           tmo);                     // Timeout

  /* Select FS from Table 3 (CTRL REG4 section 7.5) */
  config[0] = IIS328DQ_CTRL_REG4;
  /* Configure sensitivity for +-4g */
  config[1] = 0b00010000;
  i2cMasterTransmitTimeout(i2c_driver_, IIS328DQ_DEFAULT_ADDRESS, config, 2,
                           NULL, 0, tmo);

  i2cReleaseBus(i2c_driver_);
}

bool IIS328DQ::checkID() {
  uint8_t id = 0;

  bool success = IIS328DQ::receive(IIS328DQ_WHO_AM_I_ADDRESS, &id, 1);

  return (success && (id == IIS328DQ_WHO_AM_I));
}

bool IIS328DQ::receive(uint8_t reg, uint8_t *data, size_t size) {
  systime_t tmo = MS2ST(4); // 4 millisecond timeout
  i2cAcquireBus(i2c_driver_);

  msg_t status = i2cMasterTransmitTimeout(i2c_driver_, IIS328DQ_DEFAULT_ADDRESS,
                                          &reg, 1, data, size, tmo);

  i2cReleaseBus(i2c_driver_);
  return status == RDY_OK;
}

/* Expects accel_arr to be of size 3 and stores x,y,z in that order */
bool IIS328DQ::getAccel(int16_t *accel_arr) {
  // Set the register address msb (SUB) to autoincrement register address
  return IIS328DQ::receive(IIS328DQ_OUT_X_L | IIS328DQ_MASK_SUB,
                           reinterpret_cast<uint8_t *>(accel_arr), 6);
}

} // namespace peripherals
} // namespace motor_driver
