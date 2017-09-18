#ifndef _TRANSFORMS_H_
#define _TRANSFORMS_H_

namespace motor_driver {

void transforms_park(volatile float alpha, volatile float beta, volatile float theta, volatile float *d, volatile float *q);
void transforms_inverse_park(volatile float d, volatile float q, volatile float theta, volatile float *alpha, volatile float *beta);
void transforms_clarke(volatile float a, volatile float b, volatile float c, volatile float *alpha, volatile float *beta);
void transforms_inverse_clarke(volatile float alpha, volatile float beta, volatile float *a, volatile float *b, volatile float *c);

} // namespace motor_driver

#endif /* _TRANSFORMS_H_ */
