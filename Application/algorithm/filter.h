#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdbool.h>
#include <stdint.h>

// Default filter window size
#define FILTER_WINDOW_SIZE_DEFAULT  16
#define FILTER_CHANNEL_COUNT 8

// Filter window size constraints
#define FILTER_WINDOW_SIZE_MIN  2
#define FILTER_WINDOW_SIZE_MAX  64

typedef struct {
    int32_t buffer[FILTER_WINDOW_SIZE_MAX];
    uint8_t index;
    uint8_t count;
    int64_t sum;
    uint8_t window_size;  // Current window size
    int32_t zero_offset;  // Zero offset for this channel
} moving_avg_filter_t;

void filter_init(void);
int32_t filter_apply(uint8_t ch, int32_t raw);
void filter_reset(uint8_t ch);
void filter_reset_all(void);

// Configurable filter functions
void filter_set_window_size(uint8_t size);
uint8_t filter_get_window_size(void);

// Zero offset functions
void filter_set_zero_offset(uint8_t ch, int32_t offset);
void filter_get_zero_offset(uint8_t ch, int32_t *offset);
int filter_save_zero_to_flash(void);
int filter_load_zero_from_flash(void);
int32_t filter_get_raw_filtered(uint8_t ch);
bool filter_has_samples(uint8_t ch);

#endif // __FILTER_H__
