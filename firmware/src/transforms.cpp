#include "transforms.h"
#include "fast_math.h"
#include "constants.h"
#include <math.h>

namespace motor_driver {

void transforms_park(volatile float alpha, volatile float beta, volatile float theta, volatile float *d, volatile float *q)
{
    float sin = fast_sin(theta);
    float cos = fast_cos(theta);
    *d = alpha * cos + beta * sin;
    *q = beta * cos - alpha * sin;
}

void transforms_inverse_park(volatile float d, volatile float q, volatile float theta, volatile float *alpha, volatile float *beta)
{
    float sin = fast_sin(theta);
    float cos = fast_cos(theta);
    *alpha = d * cos - q * sin;
    *beta = d * sin + q * cos;
}

void transforms_clarke(volatile float a, volatile float b, volatile float c, volatile float *alpha, volatile float *beta)
{
    *alpha = (2.0f / 3.0f) * a - (1.0f / 3.0f) * b - (1.0f / 3.0f) * c;
    *beta = sqrt3_by_3 * b - sqrt3_by_3 * c;
}

void transforms_inverse_clarke(volatile float alpha, volatile float beta, volatile float *a, volatile float *b, volatile float *c)
{
    *a = alpha;
    *b = -alpha / 2.0 + (sqrt3_by_2 * beta);
    *c = -alpha / 2.0 - (sqrt3_by_2 * beta);
    // Multiply by 1/sqrt(3) because phase voltages have amplitude 1/sqrt(3) of bus voltage
    *a = (one_by_sqrt3 * (*a));
    *b = (one_by_sqrt3 * (*b));
    *c = (one_by_sqrt3 * (*c));
}

} // namespace motor_driver
