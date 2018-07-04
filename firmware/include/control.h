#ifndef _CONTROL_H_
#define _CONTROL_H_

namespace motor_driver {

void initControl();

void resumeInnerControlLoop();

void runInnerControlLoop();

void estimateState();

void runPositionControl();

void runVelocityControl();

void runCurrentControl();

void resetControlTimeout();

void brakeMotor();

} // namespace motor_driver

#endif /* _CONTROL_H_ */
