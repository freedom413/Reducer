#include "flexspline_math.h"
#include <math.h>

void flexspline_params_set_default(flexspline_params_t *p)
{
    p->ref_voltage = 3.0f;      // 3.0V reference
    p->pga = 64;                // PGA gain = 64
    p->excitation_v = 3.3f;     // 3.3V excitation
    p->gauge_k = 2.0f;          // Typical gauge factor for metal strain gauges
    p->elastic_modulus = 210000.0f; // Steel ~210 GPa = 210000 MPa
    p->flexspline_area = 100.0f;    // Placeholder mm^2
    p->moment_of_inertia = 1000.0f;  // Placeholder mm^4
    p->calibrated = false;
}

void flexspline_params_set(flexspline_params_t *p,
                           float ref_v, uint8_t pga,
                           float v_ex, float gauge_k,
                           float E, float area, float I)
{
    p->ref_voltage = ref_v;
    p->pga = pga;
    p->excitation_v = v_ex;
    p->gauge_k = gauge_k;
    p->elastic_modulus = E;
    p->flexspline_area = area;
    p->moment_of_inertia = I;
    p->calibrated = true;
}

float flexspline_raw_to_voltage(int32_t raw, float ref_voltage, uint8_t pga)
{
    // ADS1256: 24-bit signed, range = [-2^23, 2^23-1] for bipolar
    // Full scale = ref_voltage / pga
    // Voltage = raw / 8388608 * (ref_v / pga)
    return ((float)raw / 8388608.0f) * (ref_voltage / (float)pga);
}

float flexspline_voltage_to_strain(float voltage, float v_ex, float gauge_k)
{
    // Wheatstone bridge output: V_bridge = V_ex * K * ε / 4
    // Therefore: ε = V_bridge * 4 / (V_ex * K)
    if (fabsf(v_ex) < 1e-9f || fabsf(gauge_k) < 1e-9f) {
        return 0.0f;
    }
    return voltage * 4.0f / (v_ex * gauge_k);
}

float flexspline_strain_to_stress(float strain, float E)
{
    // Hooke's law: σ = E * ε (for uniaxial stress)
    return E * strain;
}

float flexspline_stress_to_moment(float stress, float I, float y_max)
{
    // Bending moment from stress: M = σ * I / y
    if (fabsf(y_max) < 1e-9f) {
        return 0.0f;
    }
    return stress * I / y_max;
}

float flexspline_voltage_to_microstrain(float voltage, float v_ex, float gauge_k)
{
    // Convert strain (ε) to micro-strain (µε = ε * 10^6)
    float strain = flexspline_voltage_to_strain(voltage, v_ex, gauge_k);
    return strain * 1000000.0f;
}

float flexspline_microstrain_to_stress(float microstrain, float E)
{
    // Convert µε back to strain, then to stress
    float strain = microstrain / 1000000.0f;
    return flexspline_strain_to_stress(strain, E);
}

float flexspline_stress_to_displacement(float stress, float I, float y_max, float L)
{
    // Simplified beam deflection formula for cantilever:
    // δ = σ * L^2 / (3 * E * y_max) ... this is approximate
    // Or use moment: δ = M * L^2 / (3 * E * I)
    // For now, return moment as proxy for displacement
    (void)I;
    (void)L;
    // Actually calculate displacement based on bending stress
    // δ = σ * L^2 / (6 * E * y_max) for simply supported beam with uniform load
    // We'll return the moment for now since we don't have L (beam length)
    (void)y_max;
    return 0.0f; // Requires L parameter - placeholder
}

void flexspline_calculate(int32_t raw, const flexspline_params_t *params,
                          flexspline_result_t *result)
{
    // Step 1: Raw to voltage
    float voltage = flexspline_raw_to_voltage(raw, params->ref_voltage, params->pga);
    result->voltage = voltage * 1000.0f; // Convert to mV for display

    // Step 2: Voltage to strain (micro-strain)
    float strain = flexspline_voltage_to_strain(voltage, params->excitation_v, params->gauge_k);
    result->strain = strain * 1000000.0f; // Convert to µε

    // Step 3: Strain to stress (MPa)
    result->stress = flexspline_strain_to_stress(strain, params->elastic_modulus);

    // Step 4: Displacement (requires geometry - using stress as proxy for now)
    // In practice, displacement depends on the harmonic drive geometry:
    // δ = f(σ, I, y_max, L, boundary conditions)
    // Placeholder: just use stress * a constant factor for demonstration
    result->displacement = result->stress * 0.01f; // Placeholder scaling
}
