#ifndef __ADC_CONVERSION_H__
#define __ADC_CONVERSION_H__

/**
 * @brief ads1256 原始值转电压
 * @param raw ads1256 原始值
 * @param ref_voltage 参考电压
 * @param pga PGA 增益
 * @return 电压值，单位：伏特 (V)
 */
extern float inline ads1256_raw_to_voltage(int32_t raw, float ref_voltage, uint8_t pga);


/**
 * @brief 应变片电压转应变 (ε)
 * @param voltage 电压值，单位：伏特 (V)
 * @param v_ex 应变片电压偏移
 * @param k 应变片灵敏度
 * @return 应变值 (ε)
 */
extern float inline voltage_to_strain(float voltage, float v_ex, float k);

/**
 * @brief 应变转微应变 (µε)
 * @param strain 应变值 (ε)
 * @return 微应变值 (µε)
 */
extern float inline strain_to_microstrain(float strain);

/**
 * @brief 将应变转换为应力 (基于胡克定律)
 * @param strain 应变值 (ε)，可以是正(拉伸)或负(压缩)
 * @param E 材料的弹性模量，单位: MPa (兆帕)
 * @return 应力值，单位: MPa (兆帕)
 */
extern float inline strain_to_stress_MPa(float strain, float E);

/**
 * @brief 从应力和面积计算作用力 (基础版本，单位需匹配)
 * @param stress 应力值
 * @param area 受力截面积
 * @return 作用力 (单位取决于输入单位)
 * @note 确保应力与面积单位匹配：
 *       - 应力(MPa) × 面积(mm²) = 力(N)
 *       - 应力(Pa) × 面积(m²) = 力(N)
 */
extern float inline calculate_force_basic(float stress, float area);

/**
 * @brief 从力和面积计算压力（基础版本，单位需匹配）
 * @param force 作用力，单位：牛顿 (N)
 * @param area 受力面积，单位：平方米 (m²)
 * @return 压力，单位：帕斯卡 (Pa)
 * @note 确保单位匹配：力(N) ÷ 面积(m²) = 压力(Pa)
 */
extern float inline calculate_pressure_basic(float force, float area);

/**
 * @brief 计算基本压力（基于胡克定律）
 * @param raw ads1256 原始值
 * @param ref_voltage 参考电压
 * @param pga PGA 增益
 * @param v_ex 应变片电压偏移
 * @param k 应变片灵敏度
 * @param E 材料弹性模量
 * @param s_area 受力截面积
 * @param f_area 压力计算受力面积
 * @return 压力值，单位：帕斯卡 (Pa)
 */
float get_pressure_basic(int32_t raw, float ref_voltage, uint8_t pga,
                         float v_ex, float k,
                         float E,
                         float s_area,
                         float f_area);

#endif /* __ADC_CONVERSION_H__ */