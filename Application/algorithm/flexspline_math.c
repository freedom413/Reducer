#include "flexspline_math.h"
#include <math.h>

static void flexspline_params_update_scales(flexspline_params_t *p)
{
    p->raw_to_mv_scale = (p->ref_voltage / (float)p->pga) * (1000.0f / 8388608.0f);
    if (fabsf(p->excitation_v) < 1e-9f || fabsf(p->gauge_k) < 1e-9f) {
        p->mv_to_microstrain_scale = 0.0f;
    } else {
        p->mv_to_microstrain_scale = 4.0f / (p->excitation_v * p->gauge_k);
    }
    p->microstrain_to_stress_scale = p->elastic_modulus / 1000000.0f;
}

void flexspline_params_set_default(flexspline_params_t *p)
{
    p->ref_voltage = 3.0f;
    p->pga = 64;
    p->excitation_v = 3.3f;
    p->gauge_k = 2.0f;
    p->elastic_modulus = 210000.0f;
    p->calibrated = false;
    flexspline_params_update_scales(p);
}

void flexspline_params_set(flexspline_params_t *p,
                           float ref_v, uint8_t pga,
                           float v_ex, float gauge_k,
                           float elastic_modulus)
{
    p->ref_voltage = ref_v;
    p->pga = pga;
    p->excitation_v = v_ex;
    p->gauge_k = gauge_k;
    p->elastic_modulus = elastic_modulus;
    p->calibrated = true;
    flexspline_params_update_scales(p);
}

void flexspline_calculate(int32_t raw, const flexspline_params_t *params,
                          flexspline_result_t *result)
{
    result->voltage = (float)raw * params->raw_to_mv_scale;
    result->strain = result->voltage * params->mv_to_microstrain_scale;
    result->stress = result->strain * params->microstrain_to_stress_scale;
}
