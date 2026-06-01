#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "ads1256.h"
#include "ads1256_raw_data_recv.h"
#include "lwrb.h"
#include "stm32g4xx_hal_gpio.h"

#define ADS1256_ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))
#define ADS1256_POLL_ERROR_LIMIT       3U
#define ADS1256_RECOVERY_INTERVAL_MS   1000U

int adc_ads1256_init(void);
extern ADS1256_t ads1256_a;
extern ADS1256_t ads1256_b;

typedef struct {
    ads1256_ain_t p;
    ads1256_ain_t n;
} ads1256_ch_t;

typedef struct {
    ADS1256_t *adc;
    const ads1256_ch_t *channels;
    uint8_t channel_count;
    uint8_t logical_base;
    uint8_t current;
    bool enabled;
} ads1256_scan_device_t;

typedef struct {
    uint16_t sps;
    ads1256_sps_t rate;
} ads1256_rate_map_t;

static lwrb_t ads1256_data_rb;
static char ads1256_data_buf[ADS1256_DATA_BUFF_SIZE];
static uint16_t ads1256_sample_rate_sps = 100;
static uint16_t ads1256_channel_mask = ADS1256_ALL_CHANNEL_MASK;
static bool ads1256_started = false;
static uint8_t ads1256_poll_error_count = 0;
static uint32_t ads1256_last_recovery_tick = 0;

static const ads1256_ch_t ads1256_a_channels[ADS1256_CHANNELS_PER_DEVICE] = {
    {.p = ADS1256_AIN0, .n = ADS1256_AIN1},
    {.p = ADS1256_AIN2, .n = ADS1256_AIN3},
    {.p = ADS1256_AIN4, .n = ADS1256_AIN5},
    {.p = ADS1256_AIN6, .n = ADS1256_AIN7},
};

static const ads1256_ch_t ads1256_b_channels[ADS1256_CHANNELS_PER_DEVICE] = {
    {.p = ADS1256_AIN0, .n = ADS1256_AIN1},
    {.p = ADS1256_AIN2, .n = ADS1256_AIN3},
    {.p = ADS1256_AIN4, .n = ADS1256_AIN5},
    {.p = ADS1256_AIN6, .n = ADS1256_AIN7},
};

static ads1256_scan_device_t ads1256_devices[] = {
    {
        .adc = &ads1256_a,
        .channels = ads1256_a_channels,
        .channel_count = ADS1256_ARRAY_SIZE(ads1256_a_channels),
        .logical_base = 0,
        .current = 0,
        .enabled = ADS1256_ENABLE_A != 0,
    },
    {
        .adc = &ads1256_b,
        .channels = ads1256_b_channels,
        .channel_count = ADS1256_ARRAY_SIZE(ads1256_b_channels),
        .logical_base = ADS1256_ENABLE_A * ADS1256_CHANNELS_PER_DEVICE,
        .current = 0,
        .enabled = ADS1256_ENABLE_B != 0,
    },
};

static const ads1256_rate_map_t ads1256_rate_table[] = {
    {5, ADS1256_SPS_5},
    {10, ADS1256_SPS_10},
    {15, ADS1256_SPS_15},
    {25, ADS1256_SPS_25},
    {30, ADS1256_SPS_30},
    {50, ADS1256_SPS_50},
    {60, ADS1256_SPS_60},
    {100, ADS1256_SPS_100},
    {500, ADS1256_SPS_500},
    {1000, ADS1256_SPS_1000},
};

static bool ads1256_lookup_rate(uint16_t sps, ads1256_sps_t *rate)
{
    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_rate_table); i++) {
        if (ads1256_rate_table[i].sps == sps) {
            if (rate != NULL) {
                *rate = ads1256_rate_table[i].rate;
            }
            return true;
        }
    }

    return false;
}

static void ads1256_data_push(uint8_t logical_channel, int32_t raw_value)
{
    ads1256_data_t data = {
        .channel = logical_channel,
        .raw_value = raw_value,
    };
    const uint32_t record_size = sizeof(data);

    if (lwrb_get_free(&ads1256_data_rb) < record_size) {
        (void)lwrb_skip(&ads1256_data_rb, record_size);
    }
    (void)lwrb_write(&ads1256_data_rb, &data, record_size);
}

static int ads1256_device_select_channel(ads1256_scan_device_t *dev, uint8_t channel)
{
    if (channel >= dev->channel_count) {
        return -1;
    }

    const ads1256_ch_t ch = dev->channels[channel];
    int ret = ads1256_set_ain_pin(dev->adc, ch.p, ch.n);
    if (ret < 0) {
        return ret;
    }

    return ads1256_start_sync_conv(dev->adc);
}

static int ads1256_device_select_current(ads1256_scan_device_t *dev)
{
    return ads1256_device_select_channel(dev, dev->current);
}

static bool ads1256_device_channel_enabled(const ads1256_scan_device_t *dev,
                                           uint8_t channel)
{
    uint8_t logical_channel = (uint8_t)(dev->logical_base + channel);
    return (ads1256_channel_mask & (1U << logical_channel)) != 0U;
}

static bool ads1256_device_find_enabled_channel(const ads1256_scan_device_t *dev,
                                                uint8_t start,
                                                uint8_t *channel)
{
    for (uint8_t offset = 0; offset < dev->channel_count; offset++) {
        uint8_t candidate = (uint8_t)(start + offset);
        if (candidate >= dev->channel_count) {
            candidate = (uint8_t)(candidate - dev->channel_count);
        }
        if (ads1256_device_channel_enabled(dev, candidate)) {
            *channel = candidate;
            return true;
        }
    }

    return false;
}

static int ads1256_device_restart(ads1256_scan_device_t *dev)
{
    if (!ads1256_device_find_enabled_channel(dev, 0, &dev->current)) {
        return 0;
    }
    return ads1256_device_select_current(dev);
}

static int ads1256_device_poll(ads1256_scan_device_t *dev)
{
    if (!ads1256_device_channel_enabled(dev, dev->current)) {
        return 0;
    }

    int ready = ads1256_is_data_ready(dev->adc);
    if (ready <= 0) {
        return ready;
    }

    uint8_t ready_channel = dev->current;
    uint8_t next_channel = 0;
    if (!ads1256_device_find_enabled_channel(
            dev, (uint8_t)((ready_channel + 1U) % dev->channel_count),
            &next_channel)) {
        return 0;
    }

    int ret = ads1256_device_select_channel(dev, next_channel);
    if (ret < 0) {
        return ret;
    }
    dev->current = next_channel;

    int32_t raw_value = 0;
    ret = ads1256_read_data_nowait(dev->adc, &raw_value);
    if (ret < 0) {
        return ret;
    }

    ads1256_data_push((uint8_t)(dev->logical_base + ready_channel), raw_value);

    return 1;
}

int adc_ads1256_restart(void)
{
    lwrb_reset(&ads1256_data_rb);

    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }

        int ret = ads1256_device_restart(dev);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

void adc_ads1256_start(void)
{
    lwrb_init(&ads1256_data_rb, ads1256_data_buf, ADS1256_DATA_BUFF_SIZE);
    ads1256_started = false;
    ads1256_poll_error_count = 0;

    int ret = adc_ads1256_init();
    if (ret < 0) {
        ads1256_last_recovery_tick = HAL_GetTick();
        return;
    }

    if (ads1256_sample_rate_sps == 100U) {
        ret = adc_ads1256_restart();
    } else {
        ret = adc_ads1256_set_sample_rate(ads1256_sample_rate_sps);
    }
    if (ret < 0) {
        ads1256_last_recovery_tick = HAL_GetTick();
        return;
    }

    ads1256_started = true;
}

int adc_ads1256_get_data(ads1256_data_t *data, uint32_t max_count)
{
    if (data == NULL || max_count == 0U) {
        return 0;
    }

    uint32_t count = lwrb_get_full(&ads1256_data_rb) / sizeof(ads1256_data_t);
    if (count > max_count) {
        count = max_count;
    }

    return lwrb_read(&ads1256_data_rb,
                     (char *)data,
                     sizeof(ads1256_data_t) * count) /
           sizeof(ads1256_data_t);
}

void adc_ads1256_poll(void)
{
    if (!ads1256_started) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - ads1256_last_recovery_tick) >=
            ADS1256_RECOVERY_INTERVAL_MS) {
            adc_ads1256_start();
        }
        return;
    }

    bool sample_read = false;
    bool poll_error = false;

    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }

        int ret = ads1256_device_poll(dev);
        if (ret > 0) {
            sample_read = true;
        } else if (ret < 0) {
            poll_error = true;
        }
    }

    if (poll_error) {
        if (ads1256_poll_error_count < UINT8_MAX) {
            ads1256_poll_error_count++;
        }
        if (ads1256_poll_error_count >= ADS1256_POLL_ERROR_LIMIT) {
            ads1256_started = false;
            ads1256_last_recovery_tick =
                HAL_GetTick() - ADS1256_RECOVERY_INTERVAL_MS;
        }
        return;
    }

    ads1256_poll_error_count = 0;

    if (sample_read) {
        HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
    }
}

int adc_ads1256_calibrate(void)
{
    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }

        int ret = ads1256_calibration(dev->adc, ADS1256_CAL_SELF);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

int adc_ads1256_set_sample_rate(uint16_t sps)
{
    ads1256_sps_t rate;

    if (!ads1256_lookup_rate(sps, &rate)) {
        return -1;
    }

    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }

        int ret = ads1256_set_sps(dev->adc, rate);
        if (ret < 0) {
            return ret;
        }
    }

    ads1256_sample_rate_sps = sps;
    return adc_ads1256_restart();
}

uint16_t adc_ads1256_get_sample_rate(void)
{
    return ads1256_sample_rate_sps;
}

int adc_ads1256_set_channel_mask(uint16_t channel_mask)
{
    if ((channel_mask & ~ADS1256_ALL_CHANNEL_MASK) != 0U) {
        return -1;
    }

    uint16_t previous_mask = ads1256_channel_mask;
    ads1256_channel_mask = channel_mask;
    if (adc_ads1256_restart() < 0) {
        ads1256_channel_mask = previous_mask;
        (void)adc_ads1256_restart();
        return -1;
    }

    return 0;
}

uint16_t adc_ads1256_get_channel_mask(void)
{
    return ads1256_channel_mask;
}
