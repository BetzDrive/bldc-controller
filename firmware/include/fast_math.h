#ifndef _FAST_MATH_H_
#define _FAST_MATH_H_

#include "LUTFunction.h"

namespace motor_driver {

constexpr float pi = 3.1415927410125732421875f;

extern const LUTFunction<float> fast_sin;
extern const LUTFunction<float> fast_cos;

} // namespace motor_driver

#endif /* _FAST_MATH_H_ */
