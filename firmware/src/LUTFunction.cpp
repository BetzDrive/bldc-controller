#include "LUTFunction.h"

#include <cmath>

namespace motor_driver {

template <typename T>
float LUTFunction<T>::lookup(float arg) const {
  float norm_arg = (arg - x_first_) / (x_last_ - x_first_);
  float norm_arg_integral = std::floor(norm_arg);
  float norm_arg_fraction = norm_arg - norm_arg_integral;
  size_t flip_index = ((int)norm_arg_integral % periodicity_.repetition_count + periodicity_.repetition_count)
      % periodicity_.repetition_count;

  switch (periodicity_.repetition_flips[flip_index]) {
    case LFFlipType::NONE:
    default:
      return lookupReduced(norm_arg_fraction);
    case LFFlipType::HORIZONTAL:
      return lookupReduced(1.0f - norm_arg_fraction);
    case LFFlipType::VERTICAL:
      return -lookupReduced(norm_arg_fraction);
    case LFFlipType::BOTH:
      return -lookupReduced(1.0f - norm_arg_fraction);
  }
}

template <typename T>
float LUTFunction<T>::lookupReduced(float reduced_arg) const {
  if (reduced_arg <= 0.0f) {
    return y_[0];
  } else if (reduced_arg >= 1.0f) {
    return y_[y_len_ - 1];
  } else {
    float integral = std::floor(reduced_arg * (y_len_ - 1));
    float fraction = reduced_arg * (y_len_ - 1) - integral;
    size_t index_before = (size_t)integral;
    return y_[index_before] * (1.0f - fraction) + y_[index_before + 1] * fraction; // Linear interpolation
  }
}

template class LUTFunction<float>;

} // namespace motor_driver
