#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract ADC interface for current/voltage sensing. */

struct AdcSamples {
    uint16_t ia;   /* Phase A current */
    uint16_t ib;   /* Phase B current */
    uint16_t ic;   /* Phase C current */
    uint16_t vbus; /* Supply voltage */
};

void hal_adc_init(void);
void hal_adc_start(void);

/* Get latest ADC samples. Returns the most recent complete sample set. */
struct AdcSamples hal_adc_get_latest(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H */
