#ifndef __PID_H__
#define __PID_H__

class PID {
private:
  float kp_, ki_, kd_;
  float min_, max_;
  float target_;

  
  float interval_;
  float windup_;

  float alpha_;
  float deriv_;

  float computeIntegral(float err, float p_out);
  float computeDerivative(float err);

public:
  PID(float kp, float ki, float kd, float interval);

  void setGains(float kp, float ki, float kd);
  void setLimits(float min, float max);
  void setTarget(float target);
  void setAlpha(float alpha);

  float compute(float val);
};

#endif //__PID_H__
