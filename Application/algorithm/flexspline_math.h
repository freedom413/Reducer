#ifndef __FLEXSPLINE_MATH_H__
#define __FLEXSPLINE_MATH_H__

#include <stdbool.h>
#include <stdint.h>

#define FLEXSPLINE_GAUGE_RESISTANCE_OHM  350U
#define FLEXSPLINE_GAUGE_FACTOR          2.11f
#define FLEXSPLINE_MAX_STRAIN_UE         20000.0f
#define FLEXSPLINE_ADC_REF_VOLTAGE       2.5f
#define FLEXSPLINE_ADC_PGA_GAIN          16U
#define FLEXSPLINE_BRIDGE_EXCITATION_V   5.0f
#define FLEXSPLINE_ELASTIC_MODULUS_MPA   210000.0f

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
