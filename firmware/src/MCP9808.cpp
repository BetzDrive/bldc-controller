#include "MCP9808.h"

namespace motor_driver {
namespace peripherals {

void MCP9808::start() {
  uint8_t config[3];
  // 4 millisecond timeout.
  systime_t tmo = MS2ST(4);

  config[0] = 0x01;
  config[1] = 0x00;
  config[2] = 0x08;

  i2cStart(i2c_driver_, &i2c_config_);

  i2cAcquireBus(i2c_driver_);

  i2cMasterTransmitTimeout(i2c_driver_,
                           // Address.
                           MCP9808_DEFAULT_ADDRESS,
                           // TX Buffer, Len.
                           config, 3,
                           // RX Buffer, Len.
                           NULL, 0,
                           // Timeout.
                           tmo);

  i2cReleaseBus(i2c_driver_);
}

bool MCP9808::checkID() { return true; }

bool MCP9808::receive(uint8_t reg, uint8_t *data, size_t size) {
  // 4 millisecond timeout.
  systime_t tmo = MS2ST(4);
  i2cAcquireBus(i2c_driver_);

  msg_t status = i2cMasterTransmitTimeout(i2c_driver_, MCP9808_DEFAULT_ADDRESS,
                                          &reg, 1, data, size, tmo);

  i2cReleaseBus(i2c_driver_);
  return status == RDY_OK;
}

/*               **** Temperature byte packet structure ****             *
 *    | MSB(7) |   (6) |   (5) |   (4) |   (3) |   (2) |   (1) |   (0) | *
 * Up |Ta vs Tc|Ta v Tu|Ta v Tl|  Sign |  2^7C |  2^6C |  2^5C |  2^4C | *
 * Lo |  2^3C  |  2^2C |  2^1C |  2^0C | 2^-1C | 2^-2C | 2^-3C | 2^-4C | */
bool MCP9808::getTemperature(float *temp) {
  uint8_t data[2];
  bool success = MCP9808::receive(MCP9808_TEMP_AMBIENT, data, 2);

  if (success) {
    uint16_t bits = (((uint16_t)data[0] & 0x0F) << 8 | data[1]);
    // Mult by 2^-4.
    *temp = (float)bits * 0.0625f;
  }

  return success;
}

} // namespace peripherals
} // namespace motor_driver
