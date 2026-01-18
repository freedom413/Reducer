#ifndef __ADS1256_RAW_DATA_RECV_H__
#define __ADS1256_RAW_DATA_RECV_H__

#include "ads1256.h"
#include "user.h"

typedef struct ads1256_data{
    uint8_t pid;
    uint8_t ch;
    int32_t raw_value;
} ads1256_data_t;

typedef struct ads1256_ch {
    ads1256_ain_t p;
    ads1256_ain_t n;
} ads1256_ch_t;

#define ADS1256_DATA_BUFF_SIZE (128 * sizeof(ads1256_data_t))


#define ADS1256_A  (0x11)
#define ADS1256_B  (0x22)

extern const ads1256_ch_t ads1235_a_ch[3];
extern const ads1256_ch_t ads1235_b_ch[3];

void adc_ads1256_start(void);
int adc_ads1256_get_data(ads1256_data_t *data , uint32_t max_count);
void ads1256_data_get_ch(ads1256_data_t *data, ads1256_ch_t *ch);

#endif /* __ADS1256_RAW_DATA_RECV_H__ */