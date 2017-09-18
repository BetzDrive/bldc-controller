#ifndef _FAST_MATH_H_
#define _FAST_MATH_H_

#include "LUTFunction.h"

namespace motor_driver {

// TODO: do these constants belong here? should they be in caps? - brent
constexpr float pi = 3.1415927410125732421875f;
constexpr float sqrt3_by_2 = 0.866025388240814208984375f;
constexpr float one_by_sqrt3 = 0.57735025882720947265625f;
constexpr float two_by_sqrt3 = 1.1547005176544189453125f;

extern const LUTFunction<float> fast_sin;
extern const LUTFunction<float> fast_cos;

} // namespace motor_driver

#endif /* _FAST_MATH_H_ */
