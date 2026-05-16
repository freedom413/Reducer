#ifndef __ADS1256_RAW_DATA_RECV_H__
#define __ADS1256_RAW_DATA_RECV_H__

#include "ads1256.h"
#include <stdbool.h>
#include "user.h"

typedef struct ads1256_data {
    uint8_t pid;
    uint8_t ch;
    int32_t raw_value;
} ads1256_data_t;

typedef struct ads1256_ch {
    ads1256_ain_t p;
    ads1256_ain_t n;
} ads1256_ch_t;

// LwRB reserves one byte internally, so add one byte to keep capacity record-aligned.
#define ADS1256_DATA_BUFF_RECORD_COUNT  (128U)
#define ADS1256_DATA_BUFF_SIZE          ((ADS1256_DATA_BUFF_RECORD_COUNT * sizeof(ads1256_data_t)) + 1U)


#define ADS1256_A  (0x11)
#define ADS1256_B  (0x22)

extern const ads1256_ch_t ads1235_a_ch[3];
extern const ads1256_ch_t ads1235_b_ch[3];

// ADS1256 objects (defined in ads1256_port.c)
extern ADS1256_t ads1256_a;
extern ADS1256_t ads1256_b;

void adc_ads1256_start(void);
int adc_ads1256_get_data(ads1256_data_t *data , uint32_t max_count);
void ads1256_data_get_ch(const ads1256_data_t *data, ads1256_ch_t *ch);

void adc_ads1256_poll(void);

// Calibration function - triggers ADS1256 self-calibration
int adc_ads1256_calibrate(void);
int adc_ads1256_set_sample_rate(uint16_t sps);
uint16_t adc_ads1256_get_sample_rate(void);
int adc_ads1256_restart(void);

#endif /* __ADS1256_RAW_DATA_RECV_H__ */
