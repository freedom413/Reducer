#include "filter.h"

static moving_avg_filter_t filters[FILTER_CHANNEL_COUNT];

void filter_init(void)
{
    for (uint8_t i = 0; i < FILTER_CHANNEL_COUNT; i++) {
        filter_reset(i);
    }
}

void filter_reset(uint8_t ch)
{
    if (ch >= FILTER_CHANNEL_COUNT) return;
    moving_avg_filter_t *f = &filters[ch];
    f->index = 0;
    f->count = 0;
    f->sum = 0;
    for (uint8_t i = 0; i < FILTER_WINDOW_SIZE; i++) {
        f->buffer[i] = 0;
    }
}

void filter_reset_all(void)
{
    for (uint8_t i = 0; i < FILTER_CHANNEL_COUNT; i++) {
        filter_reset(i);
    }
}

int32_t filter_apply(uint8_t ch, int32_t raw)
{
    if (ch >= FILTER_CHANNEL_COUNT) return raw;

    moving_avg_filter_t *f = &filters[ch];

    // Remove oldest value from sum
    f->sum -= f->buffer[f->index];

    // Store new value
    f->buffer[f->index] = raw;
    f->sum += raw;

    // Advance index
    f->index = (f->index + 1) % FILTER_WINDOW_SIZE;

    // Update count (only needed once at start)
    if (f->count < FILTER_WINDOW_SIZE) {
        f->count++;
    }

    // Return average
    return f->sum / f->count;
}
