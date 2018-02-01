#include "SVM.h"

#include <math.h>
#include <algorithm>
#include "constants.h"

namespace motor_driver {

void SVM::computeDutyCycles(float v_alpha, float v_beta, float& dc_a, float& dc_b, float& dc_c) {
  dc_a = (v_alpha) * one_div_sqrt3 + 0.5f;
  dc_b = (-0.5f * v_alpha + sqrt3_div_2 * v_beta) * one_div_sqrt3 + 0.5f;
  dc_c = (-0.5f * v_alpha - sqrt3_div_2 * v_beta) * one_div_sqrt3 + 0.5f;

  switch (strategy_) {
    case SVMStrategy::SINUSOIDAL:
    default:
      /*
       * Sinusoidal commutation
       */

      // Duty cycles are already in the right form

      break;

    case SVMStrategy::TOP_BOTTOM_CLAMP:
      /*
       * Top and bottom clamping ZSM
       */

      {
        float top_shift = 1.0f - std::max({dc_a, dc_b, dc_c});
        float bottom_shift = std::min({dc_a, dc_b, dc_c});

        if (top_shift < bottom_shift) {
          dc_a += top_shift;
          dc_b += top_shift;
          dc_c += top_shift;
        } else {
          dc_a -= bottom_shift;
          dc_b -= bottom_shift;
          dc_c -= bottom_shift;
        }
      }

      break;

    case SVMStrategy::MIDPOINT_CLAMP:
      /*
       * Midpoint clamping ZSM
       */

      {
        float shift = 0.5f * (1.0f - std::min({dc_a, dc_b, dc_c}) - std::max({dc_a, dc_b, dc_c}));

        dc_a += shift;
        dc_b += shift;
        dc_c += shift;
      }

      break;
  }
}

float SVM::getMaxAmplitude() const {
  switch (strategy_) {
    case SVMStrategy::SINUSOIDAL:
    default:

      return sqrt(3.0f) / 2.0f;

    case SVMStrategy::TOP_BOTTOM_CLAMP:

      return 1.0f;
  }
}

} // namespace motor_driver
