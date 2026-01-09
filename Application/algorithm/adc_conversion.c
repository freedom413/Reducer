#include "adc_conversion.h"

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