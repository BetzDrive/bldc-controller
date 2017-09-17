#include "SVM.h"

#include <math.h>
#include <algorithm>

namespace motor_driver {

void SVM::computeDutyCycles(float v_alpha, float v_beta, float& dc_a, float& dc_b, float& dc_c) {
  constexpr float s3d2 = sqrt(3.0f) / 2.0f;
  constexpr float scale = 1.0 / sqrt(3.0f);

  dc_a = (v_alpha) * scale + 0.5f;
  dc_b = (-0.5f * v_alpha + s3d2 * v_beta) * scale + 0.5f;
  dc_c = (-0.5f * v_alpha - s3d2 * v_beta) * scale + 0.5f;

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
        float top_shift = 1.0 - std::max({dc_a, dc_b, dc_c});
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
