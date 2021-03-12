#ifndef _TRANSFORMS_H_
#define _TRANSFORMS_H_

namespace motor_driver {
  namespace math {

    void transformPark(
        float alpha, float beta, float cos_theta, float sin_theta,
        float& d, float& q);

    void transformInversePark(
        float d, float q, float cos_theta, float sin_theta,
        float& alpha, float& beta);

    void transformClarke(float a, float b, float c, float& alpha, float& beta);

    void transformInverseClarke(
        float alpha, float beta, float& a, float& b, float& c);

  } // namespace math
} // namespace motor_driver

#endif /* _TRANSFORMS_H_ */
