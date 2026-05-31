#include "filter.h"
#include "flash_storage.h"
#include <stddef.h>

static moving_avg_filter_t filters[FILTER_CHANNEL_COUNT];
static uint8_t current_window_size = FILTER_WINDOW_SIZE_DEFAULT;

void filter_init(void)
{
    for (uint8_t i = 0; i < FILTER_CHANNEL_COUNT; i++) {
        filter_reset(i);
    }
    // Load zero offset from Flash
    filter_load_zero_from_flash();
}

void filter_reset(uint8_t ch)
{
    if (ch >= FILTER_CHANNEL_COUNT) return;
    moving_avg_filter_t *f = &filters[ch];
    f->index = 0;
    f->count = 0;
    f->sum = 0;
    f->window_size = current_window_size;
    for (uint8_t i = 0; i < f->window_size; i++) {
        f->buffer[i] = 0;
    }
    // Note: zero_offset is NOT reset here to preserve calibration
}

void filter_reset_all(void)
{
    for (uint8_t i = 0; i < FILTER_CHANNEL_COUNT; i++) {
        filter_reset(i);
    }
}

void filter_set_window_size(uint8_t size)
{
    // Clamp to valid range
    if (size < FILTER_WINDOW_SIZE_MIN) {
        size = FILTER_WINDOW_SIZE_MIN;
    }
    if (size > FILTER_WINDOW_SIZE_MAX) {
        size = FILTER_WINDOW_SIZE_MAX;
    }

    if (size == current_window_size) {
        return;  // No change needed
    }

    current_window_size = size;

    // Reset all filters with new size
    filter_reset_all();
}

uint8_t filter_get_window_size(void)
{
    return current_window_size;
}

int32_t filter_apply(uint8_t ch, int32_t raw)
{
    if (ch >= FILTER_CHANNEL_COUNT) return raw;

    moving_avg_filter_t *f = &filters[ch];
    uint8_t win_size = f->window_size;

    // Remove oldest value from sum (only if buffer is filled)
    if (f->count >= win_size) {
        f->sum -= f->buffer[f->index];
    }

    // Store new value
    f->buffer[f->index] = raw;
    f->sum += raw;

    f->index++;
    if (f->index >= win_size) {
        f->index = 0;
    }

    // Update count (only needed at start)
    if (f->count < win_size) {
        f->count++;
    }

    // Return average - use fixed point for precision
    // Subtract zero offset to get calibrated value
    int32_t filtered = (int32_t)(f->sum / (int64_t)f->count);
    return filtered - f->zero_offset;
}

void filter_set_zero_offset(uint8_t ch, int32_t offset)
{
    if (ch >= FILTER_CHANNEL_COUNT) return;
    filters[ch].zero_offset = offset;
}

void filter_get_zero_offset(uint8_t ch, int32_t *offset)
{
    if (ch >= FILTER_CHANNEL_COUNT || offset == NULL) return;
    *offset = filters[ch].zero_offset;
}

int filter_save_zero_to_flash(void)
{
    int32_t offsets[FILTER_CHANNEL_COUNT];
    for (uint8_t i = 0; i < FILTER_CHANNEL_COUNT; i++) {
        offsets[i] = filters[i].zero_offset;
    }
    return flash_storage_save_zero(offsets);
}

int filter_load_zero_from_flash(void)
{
    int32_t offsets[FILTER_CHANNEL_COUNT];
    if (flash_storage_load_zero(offsets) == 0) {
        for (uint8_t i = 0; i < FILTER_CHANNEL_COUNT; i++) {
            filters[i].zero_offset = offsets[i];
        }
        return 0;
    }
    return -1;
}

int32_t filter_get_raw_filtered(uint8_t ch)
{
    if (ch >= FILTER_CHANNEL_COUNT) return 0;

    moving_avg_filter_t *f = &filters[ch];
    if (f->count == 0) {
        return 0;
    }
    return (int32_t)(f->sum / (int64_t)f->count);
}

bool filter_has_samples(uint8_t ch)
{
    return ch < FILTER_CHANNEL_COUNT && filters[ch].count > 0U;
}
