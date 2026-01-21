#include <stdint.h>
#include <sys/_intsup.h>
#include "dbg.h"
#include "delay.h"
#include "can.h"
#include "adc_conversion.h"
#include "fdcan.h"
#include "ads1256_raw_data_recv.h"
#include "user.h"

void ads1256_int_enable(void);

static uint8_t adc_all_ch_mask = 0x00;

static int adc_raw_value[6] = {0};

static ads1256_data_t adc_ads1256_data[6] = {0};

void setup(void)
{   
    delay_init();
    can_init();
    adc_ads1256_start();
    // ads1256_int_enable(); 
}


void loop(void)
{   
    ads1256_drdy_callback();
    int recv_count = 0;
    ads1256_ch_t ch = {0};
    int a = 0;
    int b = 0;
    int i;
    int j;
    a++;
    b = a + b;
    recv_count = adc_ads1256_get_data(adc_ads1256_data ,6);
    for (i = 0; i < recv_count; i++) {
        ads1256_data_get_ch(&adc_ads1256_data[i], &ch);
        if (adc_ads1256_data[i].pid == ADS1256_A) {
            for (j = 0; j < ARR_LEN(ads1235_a_ch); j++) {
                if (ads1235_a_ch[j].p == ch.p && ads1235_a_ch[j].n == ch.n) {
                    adc_raw_value[j] = adc_ads1256_data[i].raw_value;
                    adc_all_ch_mask |= (0x01 << j);
                    break;
                }
            }
        }
        else if (adc_ads1256_data[i].pid == ADS1256_B) {
            for (j = 0; j < ARR_LEN(ads1235_b_ch); j++) {
                if (ads1235_b_ch[j].p == ch.p && ads1235_b_ch[j].n == ch.n) {
                    adc_raw_value[j + ARR_LEN(ads1235_a_ch)] = adc_ads1256_data[i].raw_value;
                    adc_all_ch_mask |= (0x01 << (j + ARR_LEN(ads1235_a_ch)));
                    break;
                }
            }
        }
        else {
            // error
        }
    }
    /* 所有通道转换完成 */
    if (adc_all_ch_mask == 0x3F) {
        adc_all_ch_mask = 0x00;           
        can_classic_data_frame_send(0x55, (uint8_t *)&adc_raw_value[0], sizeof(adc_raw_value[0]));
            dbg_printf("adc_raw_value: %d, %d, %d, %d, %d, %d\n", 
            adc_raw_value[0], adc_raw_value[1], adc_raw_value[2], 
            adc_raw_value[3], adc_raw_value[4], adc_raw_value[5]);
    }
}