#include "ads1256.h"
#include "stdio.h"

extern ADS1256_t ads1256r;
int adc_ads1256_init(void);


void setup(void)
{
    adc_ads1256_init();
}


void loop(void)
{
    int32_t adc_value = 0;
    if (ads1256_is_data_ready(&ads1256r)) {
        ads1256_read_data(&ads1256r, &adc_value);
        // printf("adc_value: %d\n", adc_value);
    }

}