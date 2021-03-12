#ifndef SVM_HPP_
#define SVM_HPP_

namespace motor_driver {
namespace controller {

enum class SVMStrategy { SINUSOIDAL, TOP_BOTTOM_CLAMP, MIDPOINT_CLAMP };

class SVM {
public:
  explicit SVM(SVMStrategy strategy) : strategy_(strategy) {}

  void computeDutyCycles(float v_alpha, float v_beta, float &dc_a, float &dc_b,
                         float &dc_c);

  float getMaxAmplitude() const;

  void setStrategy(SVMStrategy strategy) { strategy_ = strategy; }

private:
  SVMStrategy strategy_;
};

} // namespace controller
} // namespace motor_driver

#endif // SVM_HPP_
