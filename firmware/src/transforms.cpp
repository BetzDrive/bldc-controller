#include "transforms.h"

#include "fast_math.h"
#include "constants.h"
#include <math.h>

namespace motor_driver {
  namespace math {

    void transformPark(
        float alpha, float beta, float cos_theta, float sin_theta,
        float& d, float& q) {
      d = cos_theta * alpha + sin_theta * beta;
      q = -sin_theta * alpha + cos_theta * beta;
    }

    void transformInversePark(
        float d, float q, float cos_theta, float sin_theta,
        float& alpha, float& beta) {
      alpha = cos_theta * d - sin_theta * q;
      beta = sin_theta * d + cos_theta * q;
    }

    void transformClarke(
        float a, float b, float c, float& alpha, float& beta) {
      alpha = (2.0f * a - b - c) / 3.0f;
      beta = consts::one_div_sqrt3 * (b - c);
    }

    void transformInverseClarke(
        float alpha, float beta, float& a, float& b, float& c) {
      a = alpha;
      b = -alpha / 2.0f + (consts::sqrt3_div_2 * beta);
      c = -alpha / 2.0f - (consts::sqrt3_div_2 * beta);
      // Multiply by 1/sqrt(3) because phase voltages have amplitude 1/sqrt(3)
      // of bus voltage.
      a *= consts::one_div_sqrt3;
      b *= consts::one_div_sqrt3;
      c *= consts::one_div_sqrt3;
    }

  } // namespace math
} // namespace motor_driver
