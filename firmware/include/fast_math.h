#ifndef _FAST_MATH_H_
#define _FAST_MATH_H_

#include "LUTFunction.h"

namespace motor_driver {
  namespace math {

    extern const LUTFunction<float> fast_sin;
    extern const LUTFunction<float> fast_cos;

  } // namespace math
} // namespace motor_driver

#endif /* _FAST_MATH_H_ */
