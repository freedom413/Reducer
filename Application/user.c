#include "ads1256.h"
#include "stdio.h"
#include <stdint.h>
#include "dbg.h"
#include "delay.h"
#include "can.h"

extern ADS1256_t ads1256_a;
extern ADS1256_t ads1256_b;
int adc_ads1256_init(void);

typedef struct {
    ads1256_ain_t p;
    ads1256_ain_t n;
} adc_ch_t;

static const adc_ch_t adc_ch[6] = {
    {.p = ADS1256_AIN0, .n = ADS1256_AIN1},
    {.p = ADS1256_AIN2, .n = ADS1256_AIN3},
    {.p = ADS1256_AIN4, .n = ADS1256_AIN5},
    {.p = ADS1256_AIN0, .n = ADS1256_AIN1},
    {.p = ADS1256_AIN2, .n = ADS1256_AIN3},
    {.p = ADS1256_AIN4, .n = ADS1256_AIN5},
};

static inline int adc_ch_index_inc(int index,int s, int e)
{   
    return ((index + 1) % (e - s + 1)) + s;
}

static uint8_t adc_all_ch_mask = 0x00;

static int adc_a_index = 0;
static int adc_b_index = 3;

int32_t adc_raw_value[6] = {0};

void setup(void)
{   
    int ret;
    delay_init();
    can_init();
    ret = adc_ads1256_init();
    if (ret < 0) {
        dbg_printf("adc_ads1256_init failed, ret = %d\n", ret);
    }
    //开始通道转换
    ads1256_set_ain_pin(&ads1256_a, adc_ch[adc_a_index].p, adc_ch[adc_a_index].n);
    ads1256_set_ain_pin(&ads1256_b, adc_ch[adc_b_index].p, adc_ch[adc_b_index].n);
    ads1256_sync(&ads1256_a);
    ads1256_sync(&ads1256_b);
    ads1256_wakeup(&ads1256_a);
    ads1256_wakeup(&ads1256_b);
}


void loop(void)
{
    int32_t adc_a_value = 0;
    int32_t adc_b_value = 0;
    if (ads1256_is_data_ready(&ads1256_a)) {
        adc_a_index = adc_ch_index_inc(adc_a_index, 0, 2);
        ads1256_set_ain_pin(&ads1256_a, adc_ch[adc_a_index].p, adc_ch[adc_a_index].n);
        ads1256_sync(&ads1256_a);
        ads1256_wakeup(&ads1256_a);
        ads1256_read_data(&ads1256_a, &adc_a_value);
        adc_raw_value[adc_a_index] = adc_a_value;
        adc_all_ch_mask |= 1 << adc_a_index;
    }

    if (ads1256_is_data_ready(&ads1256_b)) {
        adc_b_index = adc_ch_index_inc(adc_b_index, 3, 5);
        ads1256_set_ain_pin(&ads1256_b, adc_ch[adc_b_index].p, adc_ch[adc_b_index].n);
        ads1256_sync(&ads1256_b);
        ads1256_wakeup(&ads1256_b);
        ads1256_read_data(&ads1256_b, &adc_b_value);
        adc_raw_value[adc_b_index] = adc_b_value;
        adc_all_ch_mask |= 1 << adc_b_index;
    }

    if (adc_all_ch_mask == 0x3F) {
        adc_all_ch_mask = 0x00;
        dbg_printf("adc_raw_value: %d, %d, %d, %d, %d, %d\n", 
            adc_raw_value[0], adc_raw_value[1], adc_raw_value[2], 
            adc_raw_value[3], adc_raw_value[4], adc_raw_value[5]);
    }
}