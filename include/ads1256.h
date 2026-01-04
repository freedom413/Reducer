#ifndef __ADS1256_H__
#define __ADS1256_H__
#include "stdbool.h"

typedef int (*pfn_ads1256_io_t)(uint8_t reg, uint8_t *p_data, uint8_t nbytes);
typedef int (*pfn_ads1256_cs_t)(bool state);

typedef struct ads1256{
    pfn_ads1256_io_t read;
    pfn_ads1256_io_t write;
    pfn_ads1256_cs_t cs;
    bool initialized;
}ADS1256_t;



#endif /* __ADS1256_H__ */
