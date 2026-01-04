#ifndef __ADS1256_H__
#define __ADS1256_H__
#include "stdbool.h"
#include "stdint.h"

#define  FCLk  7680000UL  // 7.68MHz crystal oscillator frequency

typedef int (*pfn_ads1256_io_t)(uint8_t *p_data, uint8_t nbytes);
typedef int (*pfn_ads1256_pin_t)(bool state);
typedef int (*pfn_ads1256_delay_us_t)(uint32_t us);

typedef enum ads1256_mod{
    READ_ONECE_MODE,
    READ_CONTINUE_MODE,
} ads1256_mod_t;

typedef enum ads1256_ain{
    ADS1256_AIN0 = 0,
    ADS1256_AIN1,
    ADS1256_AIN2,
    ADS1256_AIN3,
    ADS1256_AIN4,
    ADS1256_AIN5,
    ADS1256_AIN6,
    ADS1256_AIN7,
    ADS1256_AINCOM,
} ads1256_ain_t;

typedef enum ads1256_gpa{
    ADS1256_GPA_1 = 0,
    ADS1256_GPA_2,
    ADS1256_GPA_4,
    ADS1256_GPA_8,
    ADS1256_GPA_16,
    ADS1256_GPA_32,
    ADS1256_GPA_64,
} ads1256_gpa_t;

typedef enum ads1256_sps{
    ADS1256_SPS_30000 = 0,
    ADS1256_SPS_15000,
    ADS1256_SPS_7500,
    ADS1256_SPS_3750,
    ADS1256_SPS_2000,
    ADS1256_SPS_1000,
    ADS1256_SPS_500,
    ADS1256_SPS_100,
    ADS1256_SPS_60,
    ADS1256_SPS_50,
    ADS1256_SPS_30,
    ADS1256_SPS_25,
    ADS1256_SPS_15,
    ADS1256_SPS_10,
    ADS1256_SPS_5,
    ADS1256_SPS_2_5,
} ads1256_sps_t;

typedef struct ads1256{
    /* must set */
    pfn_ads1256_io_t  read;
    pfn_ads1256_io_t  write;
    pfn_ads1256_pin_t cs;
    pfn_ads1256_delay_us_t delay_us;
    ads1256_mod_t mod;
    /* optional set */
    pfn_ads1256_pin_t reset;
    pfn_ads1256_pin_t sync_down;
    /* private data */
    bool is_data_ready;
    bool is_init;
}ADS1256_t;



#endif /* __ADS1256_H__ */
