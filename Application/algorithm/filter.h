#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>

#define FILTER_WINDOW_SIZE  16
#define FILTER_CHANNEL_COUNT 6

typedef struct {
    int32_t buffer[FILTER_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
    int32_t sum;
} moving_avg_filter_t;

void filter_init(void);
int32_t filter_apply(uint8_t ch, int32_t raw);
void filter_reset(uint8_t ch);
void filter_reset_all(void);

#endif // __FILTER_H__
