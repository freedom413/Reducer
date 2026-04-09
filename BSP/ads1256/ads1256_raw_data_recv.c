
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "ads1256_raw_data_recv.h"
#include "lwrb.h"

int adc_ads1256_init(void);

static lwrb_t ads1256_data_rb;
static char ads1256_data_buf[ADS1256_DATA_BUFF_SIZE];
static volatile uint8_t ads1256_drdy_pending_mask = 0;
static uint8_t adc_a_index = 0;
static uint8_t adc_b_index = 0;
static uint16_t ads1256_sample_rate_sps = 100;

/* ADS1235_A 通道转换序列 */
const ads1256_ch_t ads1235_a_ch[] = {
    {.p = ADS1256_AIN0, .n = ADS1256_AIN1},
    {.p = ADS1256_AIN2, .n = ADS1256_AIN3},
    {.p = ADS1256_AIN4, .n = ADS1256_AIN5},
};

/* ADS1235_B 通道转换序列 */
const ads1256_ch_t ads1235_b_ch[] = {
    {.p = ADS1256_AIN0, .n = ADS1256_AIN1},
    {.p = ADS1256_AIN2, .n = ADS1256_AIN3},
    {.p = ADS1256_AIN4, .n = ADS1256_AIN5},
};

typedef struct {
    uint16_t sps;
    ads1256_sps_t rate;
} ads1256_rate_map_t;

static const ads1256_rate_map_t ads1256_rate_table[] = {
    {5, ADS1256_SPS_5},
    {10, ADS1256_SPS_10},
    {15, ADS1256_SPS_15},
    {25, ADS1256_SPS_25},
    {30, ADS1256_SPS_30},
    {50, ADS1256_SPS_50},
    {60, ADS1256_SPS_60},
    {100, ADS1256_SPS_100},
    {500, ADS1256_SPS_500},
    {1000, ADS1256_SPS_1000},
    {2000, ADS1256_SPS_2000},
    {3750, ADS1256_SPS_3750},
    {7500, ADS1256_SPS_7500},
    {15000, ADS1256_SPS_15000},
    {30000, ADS1256_SPS_30000},
};

static bool ads1256_lookup_rate(uint16_t sps, ads1256_sps_t *rate)
{
    for (uint32_t i = 0; i < ARR_LEN(ads1256_rate_table); i++) {
        if (ads1256_rate_table[i].sps == sps) {
            if (rate != NULL) {
                *rate = ads1256_rate_table[i].rate;
            }
            return true;
        }
    }
    return false;
}

int adc_ads1256_restart(void)
{
    adc_a_index = 0;
    adc_b_index = 0;
    lwrb_reset(&ads1256_data_rb);
    ads1256_set_ain_pin(&ads1256_a, ads1235_a_ch[adc_a_index].p, ads1235_a_ch[adc_a_index].n);
    ads1256_set_ain_pin(&ads1256_b, ads1235_b_ch[adc_b_index].p, ads1235_b_ch[adc_b_index].n);
    ads1256_sync(&ads1256_a);
    ads1256_sync(&ads1256_b);
    ads1256_wakeup(&ads1256_a);
    ads1256_wakeup(&ads1256_b);
    ads1256_drdy_pending_mask = 0;
    return 0;
}

void adc_ads1256_start(void)
{   
    lwrb_init(&ads1256_data_rb, ads1256_data_buf, ADS1256_DATA_BUFF_SIZE);
    //开始首次通道转换
    adc_ads1256_init();
    adc_ads1256_restart();
}

static inline void ads1256_data_set_ch(ads1256_data_t *data, ads1256_ch_t ch)
{
    data->ch = (ch.p << 4)| ch.n;
}

void ads1256_data_get_ch(ads1256_data_t *data, ads1256_ch_t *ch)
{
    ch->p = (data->ch >> 4) & 0x0F;
    ch->n = data->ch & 0x0F;
}


int adc_ads1256_get_data(ads1256_data_t *data , uint32_t max_count)
{   
    uint32_t count = lwrb_get_full(&ads1256_data_rb) / sizeof(ads1256_data_t);
    if (count > max_count) {
        count = max_count;
    }
    return lwrb_read(&ads1256_data_rb, 
                     (char *)data, 
                     sizeof(ads1256_data_t) * count) / 
                     sizeof(ads1256_data_t);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ADC1_DRDY_Pin) {
        ads1256_drdy_pending_mask |= 0x01U;
    } else if (GPIO_Pin == ADC2_DRDY_Pin) {
        ads1256_drdy_pending_mask |= 0x02U;
    }
}

bool ads1256_take_drdy_pending(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t pending;

    __disable_irq();
    pending = ads1256_drdy_pending_mask;
    ads1256_drdy_pending_mask = 0;
    if (primask == 0U) {
        __enable_irq();
    }

    return pending != 0U;
}

void ads1256_drdy_callback(void)
{
    static int32_t raw_value = 0;
    static ads1256_ch_t ch = {0};
    static ads1256_data_t adc_data = {0};

    if (ads1256_is_data_ready(&ads1256_a)) {
        HAL_GPIO_WritePin(OUT_GPIO_Port, OUT_Pin, GPIO_PIN_SET);
        /* 先获取当前转换完成的通道号 */
        ads1256_get_ain_pin(&ads1256_a, &ch.p, &ch.n);
        /* 切换到下一组通道 */
        adc_a_index = (adc_a_index + 1) % ARR_LEN(ads1235_a_ch);
        /* 配置下一个待检测ain通道 */
        ads1256_set_ain_pin(&ads1256_a, ads1235_a_ch[adc_a_index].p, ads1235_a_ch[adc_a_index].n);
        /* 开始下一次转换 */
        ads1256_start_sync_conv(&ads1256_a);
        /* 读取上一次转换结果 */
        ads1256_read_data(&ads1256_a, &raw_value);
        /* 汇总通道信息 */
        ads1256_data_set_ch(&adc_data, ch);
        adc_data.raw_value = raw_value;
        adc_data.pid = ADS1256_A;
        /* 写入数据缓冲区，满则覆盖旧数据 */
        lwrb_overwrite(&ads1256_data_rb, (const char *)&adc_data, sizeof(adc_data));
        HAL_GPIO_WritePin(OUT_GPIO_Port, OUT_Pin, GPIO_PIN_RESET);
    }

    if (ads1256_is_data_ready(&ads1256_b)) {
        /* 先获取当前转换完成的通道号 */
        ads1256_get_ain_pin(&ads1256_b, &ch.p, &ch.n);
        /* 切换到下一组通道 */
        adc_b_index = (adc_b_index + 1) % ARR_LEN(ads1235_b_ch);
        /* 配置下一个待检测ain通道 */
        ads1256_set_ain_pin(&ads1256_b, ads1235_b_ch[adc_b_index].p, ads1235_b_ch[adc_b_index].n);
        /* 开始下一次转换 */
        ads1256_start_sync_conv(&ads1256_b);
        /* 读取上一次转换结果 */
        ads1256_read_data(&ads1256_b, &raw_value);
        /* 汇总通道信息 */
        ads1256_data_set_ch(&adc_data, ch);
        adc_data.raw_value = raw_value;
        adc_data.pid = ADS1256_B;
        /* 写入数据缓冲区，满则覆盖旧数据 */
        lwrb_overwrite(&ads1256_data_rb, (const char *)&adc_data, sizeof(adc_data));
    }
}

int adc_ads1256_calibrate(void)
{
    int ret_a, ret_b;

    // Perform self-calibration on both ADCs
    ret_a = ads1256_calibration(&ads1256_a, ADS1256_CAL_SELF);
    ret_b = ads1256_calibration(&ads1256_b, ADS1256_CAL_SELF);

    // Return 0 if both succeeded, or first error code
    if (ret_a != 0) return ret_a;
    if (ret_b != 0) return ret_b;
    return 0;
}

int adc_ads1256_set_sample_rate(uint16_t sps)
{
    ads1256_sps_t rate;

    if (!ads1256_lookup_rate(sps, &rate)) {
        return -1;
    }
    if (ads1256_set_sps(&ads1256_a, rate) != 0) {
        return -2;
    }
    if (ads1256_set_sps(&ads1256_b, rate) != 0) {
        return -3;
    }

    ads1256_sample_rate_sps = sps;
    return adc_ads1256_restart();
}

uint16_t adc_ads1256_get_sample_rate(void)
{
    return ads1256_sample_rate_sps;
}
