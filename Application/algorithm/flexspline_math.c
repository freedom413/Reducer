#include "flexspline_math.h"
#include <math.h>

static bool flexspline_abs_less_than(float value, float limit)
{
    return (value < limit) && (value > -limit);
}

static float flexspline_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void flexspline_params_update_scales(flexspline_params_t *p)
{
    if (p == NULL) {
        return;
    }

    if (p->pga == 0U) {
        p->raw_to_mv_scale = 0.0f;
    } else {
        p->raw_to_mv_scale = ((2.0f * p->ref_voltage) / (float)p->pga) *
                             (1000.0f / 8388608.0f);
    }

    if (flexspline_abs_less_than(p->excitation_v, 1e-9f) ||
        flexspline_abs_less_than(p->gauge_k, 1e-9f)) {
        p->mv_to_microstrain_scale = 0.0f;
    } else {
        /* Four active gauges: Vout / Vex = gauge_k * strain. */
        p->mv_to_microstrain_scale = 1000.0f / (p->excitation_v * p->gauge_k);
    }
    p->microstrain_to_stress_scale = p->elastic_modulus / 1000000.0f;
}

void flexspline_params_set_default(flexspline_params_t *p)
{
    if (p == NULL) {
        return;
    }

    p->ref_voltage = FLEXSPLINE_ADC_REF_VOLTAGE;
    p->pga = FLEXSPLINE_ADC_PGA_GAIN;
    p->excitation_v = FLEXSPLINE_BRIDGE_EXCITATION_V;
    p->gauge_k = FLEXSPLINE_GAUGE_FACTOR;
    p->elastic_modulus = FLEXSPLINE_ELASTIC_MODULUS_MPA;
    p->calibrated = false;
    flexspline_params_update_scales(p);
}

void flexspline_params_set(flexspline_params_t *p,
                           float ref_v, uint8_t pga,
                           float v_ex, float gauge_k,
                           float elastic_modulus)
{
    if (p == NULL) {
        return;
    }

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
    if (params == NULL || result == NULL) {
        return;
    }

    result->voltage = (float)raw * params->raw_to_mv_scale;
    result->strain = result->voltage * params->mv_to_microstrain_scale;
    result->strain = flexspline_clamp(result->strain,
                                      -FLEXSPLINE_MAX_STRAIN_UE,
                                      FLEXSPLINE_MAX_STRAIN_UE);
    result->stress = result->strain * params->microstrain_to_stress_scale;
}
