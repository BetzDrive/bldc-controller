#ifndef _PID_H_
#define _PID_H_

namespace motor_driver {

class PID {
public:
  PID(float p, float i) : kp(p), ki(i), integral(0), setpoint(0) {}

  void update(float dt, float value, float& out);

  void setPoint(float sp) {
    setpoint = sp;
  }

private:
  float kp;
  float ki;
  float integral;
  float setpoint;
};

} // namespace motor_driver

#endif /* _SVM_H_ */
