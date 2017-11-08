#include "LM75B.h"
#include "state.h"

namespace motor_driver {

void LM75B::start() {
  i2cStart(i2c_driver_, &i2c_config_);
}

bool LM75B::receive(uint16_t addr, uint8_t* data, size_t size) {
  systime_t tmo = MS2ST(4);
  msg_t status = RDY_OK;
  /* i2cAcquireBus(i2c_driver_); */
  status = i2cMasterReceiveTimeout(i2c_driver_, addr, data, size, tmo);
  /* i2cReleaseBus(i2c_driver_); */
  i2cflags_t errors = 0;
  if (status != RDY_OK) {
    errors = i2cGetErrors(i2c_driver_);
  }
  results.debug_u16 = (uint16_t) errors;
  return status == RDY_OK;
}

bool LM75B::getTemperature(float* temp) {
  uint8_t data[2];
  bool success = LM75B::receive(0x48, data, 2);
  if (success) {
    int16_t bits = (data[1] << 8) + data[0];
    *temp = (float) (bits >> 5) * 0.125;
  }

  return success;
}

} // namespace motor_driver

