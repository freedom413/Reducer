#ifndef __ADS1256_RAW_DATA_RECV_H__
#define __ADS1256_RAW_DATA_RECV_H__

#include "ads1256_config.h"
#include <stdint.h>

#define ADS1256_CHANNELS_PER_DEVICE   4U
#define ADS1256_LOGICAL_CHANNEL_COUNT \
    ((ADS1256_ENABLE_A * ADS1256_CHANNELS_PER_DEVICE) + \
     (ADS1256_ENABLE_B * ADS1256_CHANNELS_PER_DEVICE))
#define ADS1256_ALL_CHANNEL_MASK \
    ((1U << ADS1256_LOGICAL_CHANNEL_COUNT) - 1U)

typedef struct ads1256_data {
    uint8_t channel;
    int32_t raw_value;
} ads1256_data_t;

// LwRB reserves one byte internally, so add one byte to keep capacity record-aligned.
#define ADS1256_DATA_BUFF_RECORD_COUNT  (128U)
#define ADS1256_DATA_BUFF_SIZE          ((ADS1256_DATA_BUFF_RECORD_COUNT * sizeof(ads1256_data_t)) + 1U)

void adc_ads1256_start(void);
int adc_ads1256_get_data(ads1256_data_t *data , uint32_t max_count);
void adc_ads1256_poll(void);

// Calibration function - triggers ADS1256 self-calibration
int adc_ads1256_calibrate(void);
int adc_ads1256_set_sample_rate(uint16_t sps);
uint16_t adc_ads1256_get_sample_rate(void);
int adc_ads1256_set_channel_mask(uint16_t channel_mask);
uint16_t adc_ads1256_get_channel_mask(void);
int adc_ads1256_restart(void);

#endif /* __ADS1256_RAW_DATA_RECV_H__ */
