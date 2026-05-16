#ifndef __FLEXSPLINE_MATH_H__
#define __FLEXSPLINE_MATH_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ref_voltage;
    uint8_t pga;
    float excitation_v;
    float gauge_k;
    float elastic_modulus;
    float raw_to_mv_scale;
    float mv_to_microstrain_scale;
    float microstrain_to_stress_scale;
    bool calibrated;
} flexspline_params_t;

typedef struct {
    float voltage;
    float strain;
    float stress;
} flexspline_result_t;

void flexspline_params_set_default(flexspline_params_t *p);
void flexspline_params_set(flexspline_params_t *p,
                           float ref_v, uint8_t pga,
                           float v_ex, float gauge_k,
                           float elastic_modulus);

void flexspline_calculate(int32_t raw, const flexspline_params_t *params,
                          flexspline_result_t *result);

#endif // __FLEXSPLINE_MATH_H__
