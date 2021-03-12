#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "constants.h"
#include "stdint.h"

namespace motor_driver {
namespace controller {

struct RolledADC {
  uint16_t count = 0;

  uint16_t ia[consts::ivsense_rolling_average_count] = {0};
  uint16_t ib[consts::ivsense_rolling_average_count] = {0};
  uint16_t ic[consts::ivsense_rolling_average_count] = {0};
  uint16_t vin[consts::ivsense_rolling_average_count] = {0};

  RolledADC() {}
};

void initControl();

void resumeInnerControlLoop();

void runInnerControlLoop();

void estimateState();

void runPositionControl();

void runVelocityControl();

void runCurrentControl();

void resetControlTimeout();

void brakeMotor();

} // namespace controller
} // namespace motor_driver

#endif /* _CONTROL_H_ */
