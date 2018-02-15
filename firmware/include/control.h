#ifndef _CONTROL_H_
#define _CONTROL_H_

namespace motor_driver {

void initControl();

void resumeInnerControlLoop();

void runInnerControlLoop();

void estimateState();

void runVelocityControl();

void runCurrentControl();

} // namespace motor_driver

#endif /* _CONTROL_H_ */
