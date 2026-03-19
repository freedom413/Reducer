#ifndef __FLEXSPLINE_MATH_H__
#define __FLEXSPLINE_MATH_H__

#include <stdint.h>
#include <stdbool.h>

// Physical constants - should be configured per sensor
typedef struct {
    float ref_voltage;       // ADC reference voltage (V)
    uint8_t pga;             // Programmable Gain Amplifier
    float excitation_v;      // Strain gauge excitation voltage (V)
    float gauge_k;           // Strain gauge sensitivity factor
    float elastic_modulus;   // Young's modulus (MPa)
    float flexspline_area;   // Cross-sectional area (mm^2)
    float moment_of_inertia; // Second moment of area (mm^4)
    bool calibrated;         // Calibration flag
} flexspline_params_t;

void flexspline_params_set_default(flexspline_params_t *p);
void flexspline_params_set(flexspline_params_t *p,
                           float ref_v, uint8_t pga,
                           float v_ex, float k,
                           float E, float area, float I);

float flexspline_raw_to_voltage(int32_t raw, float ref_voltage, uint8_t pga);
float flexspline_voltage_to_strain(float voltage, float v_ex, float gauge_k);
float flexspline_strain_to_stress(float strain, float E);
float flexspline_stress_to_moment(float stress, float I, float y_max);

float flexspline_voltage_to_microstrain(float voltage, float v_ex, float gauge_k);
float flexspline_microstrain_to_stress(float microstrain, float E);
float flexspline_stress_to_displacement(float stress, float I, float y_max, float L);

typedef struct {
    float voltage;       // Voltage in mV
    float strain;        // Strain in micro-strain (µε)
    float stress;        // Stress in MPa
    float displacement;  // Displacement in µm
} flexspline_result_t;

void flexspline_calculate(int32_t raw, const flexspline_params_t *params,
                          flexspline_result_t *result);

#endif // __FLEXSPLINE_MATH_H__
