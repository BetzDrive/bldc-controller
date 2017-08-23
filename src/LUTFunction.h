#ifndef _LUT_FUNCTION_H_
#define _LUT_FUNCTION_H_

#include <stdlib.h>

namespace motor_driver {

enum class LFFlipType {
  NONE,
  HORIZONTAL,
  VERTICAL,
  BOTH
};

struct LFPeriodicity {
  size_t repetition_count;
  const LFFlipType *repetition_flips;
};

template <typename T>
class LUTFunction {
public:
  LUTFunction(float x_first, float x_last, const T *y, size_t y_len, LFPeriodicity periodicity)
    : x_first_(x_first),
      x_last_(x_last),
      y_(y),
      y_len_(y_len),
      periodicity_(periodicity) {}
  T lookup(float arg) const;
  T operator()(float arg) const { return lookup(arg); }

private:
  const float x_first_;
  const float x_last_;
  const T * const y_;
  const size_t y_len_;
  const LFPeriodicity periodicity_;

  T lookupReduced(float reduced_arg) const;
};

} // namespace motor_driver

#endif /* _LUT_FUNCTION_H_ */
