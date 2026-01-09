#include "math.h"

float inline ads1256_raw_to_voltage(int32_t raw, float ref_voltage, uint8_t pga)
{
    float voltage = ((float)raw / 8388608.0f) * (ref_voltage / pga);
    return voltage;
}

float inline voltage_to_strain(float voltage, float v_ex, float k)
{
    float strain = voltage * 4.0f / (v_ex * k);
    return strain;
}

float inline strain_to_microstrain(float strain)
{
    float microstrain = strain * 1000000.f;
    return microstrain;
}

float inline strain_to_stress_MPa(float strain, float E) 
{

    float stress = E * strain;
    return stress;
}

float inline calculate_force_basic(float stress, float area) {
    return stress * area;
}

float inline calculate_pressure_basic(float force, float area) 
{
    if (fabs(area) < 1e-12f) {
        return 0.0f;
    }
    return force / area;
}


float get_pressure_basic(int32_t raw, float ref_voltage, uint8_t pga,
                         float v_ex, float k,
                         float E,
                         float s_area,
                         float f_area)
{
    float voltage = ads1256_raw_to_voltage(raw, ref_voltage, pga);
    float strain = voltage_to_strain(voltage, v_ex, k);
    float stress = strain_to_stress_MPa(strain, E);
    float force = calculate_force_basic(stress, s_area);
    float pressure = calculate_pressure_basic(force, f_area);
    return pressure;
}