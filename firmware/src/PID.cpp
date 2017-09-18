#include "pid.h"

#include <math.h>
#include <algorithm>

namespace motor_driver {

void PID::update(float dt, float curr_value, float& out) {
    error = setpoint - curr_value;
    integral += error * ki * dt;
    out = integral + kp * error * dt;
}


} // namespace motor_driver
