#ifndef PID_HPP_
#define PID_HPP_

namespace motor_driver {
namespace controller {

class PID {
private:
  float kp_, ki_, kd_;
  float min_, max_;
  float target_;

  float interval_;
  float windup_;

  float alpha_;
  float deriv_;
  float val_prev_;

  float computeIntegral(float err);
  float computeDerivative(float val);

public:
  PID(float kp, float ki, float kd, float interval);

  void setGains(float kp, float ki, float kd);
  void setLimits(float min, float max);
  void setTarget(float target);
  void setAlpha(float alpha);

  float compute(float val);
};

} // namespace controller
} // namespace motor_driver

#endif // PID_HPP_
