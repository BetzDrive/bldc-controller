#ifndef FAST_MATH_HPP_
#define FAST_MATH_HPP_

#include "LUTFunction.hpp"

namespace motor_driver {
namespace math {

extern const LUTFunction<float> fast_sin;
extern const LUTFunction<float> fast_cos;

} // namespace math
} // namespace motor_driver

#endif // FAST_MATH_HPP_
