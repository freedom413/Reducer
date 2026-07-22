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
    uint32_t sps_x10;
    uint32_t cycling_rate_x10;
    ads1256_sps_t rate;
} ads1256_rate_map_t;

static lwrb_t ads1256_data_rb;
static char ads1256_data_buf[ADS1256_DATA_BUFF_SIZE];
static uint32_t ads1256_sample_rate_x10 = 1000U;
static uint16_t ads1256_channel_mask = ADS1256_ALL_CHANNEL_MASK;
static uint8_t ads1256_pga_gain = 16U;
static uint32_t ads1256_vref_uv = 2500000U;
static bool ads1256_started = false;
static bool ads1256_ring_ready = false;
static uint8_t ads1256_poll_error_count = 0;
static uint32_t ads1256_last_recovery_tick = 0;
static uint16_t ads1256_overflow_count = 0;
static uint16_t ads1256_recovery_count = 0;

static bool ads1256_gain_to_enum(uint8_t gain, ads1256_pga_t *pga);

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
    {25U, 25U, ADS1256_SPS_2_5},
    {50U, 50U, ADS1256_SPS_5},
    {100U, 100U, ADS1256_SPS_10},
    {150U, 150U, ADS1256_SPS_15},
    {250U, 250U, ADS1256_SPS_25},
    {300U, 300U, ADS1256_SPS_30},
    {500U, 500U, ADS1256_SPS_50},
    {600U, 590U, ADS1256_SPS_60},
    {1000U, 980U, ADS1256_SPS_100},
    {5000U, 4560U, ADS1256_SPS_500},
    {10000U, 8370U, ADS1256_SPS_1000},
    {20000U, 14380U, ADS1256_SPS_2000},
    {37500U, 21650U, ADS1256_SPS_3750},
    {75000U, 30430U, ADS1256_SPS_7500},
    {150000U, 38170U, ADS1256_SPS_15000},
    {300000U, 43740U, ADS1256_SPS_30000},
};

static bool ads1256_lookup_rate(uint32_t sps_x10, ads1256_sps_t *rate)
{
    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_rate_table); i++) {
        if (ads1256_rate_table[i].sps_x10 == sps_x10) {
            if (rate != NULL) {
                *rate = ads1256_rate_table[i].rate;
            }
            return true;
        }
    }

    return false;
}

int adc_ads1256_configure_startup(uint32_t vref_uv, uint8_t pga_gain,
                                  uint32_t sps_x10, uint16_t channel_mask)
{
    ads1256_pga_t pga;
    if (vref_uv < 1000000U || vref_uv > 5000000U ||
        !ads1256_gain_to_enum(pga_gain, &pga) ||
        !ads1256_lookup_rate(sps_x10, NULL) ||
        (channel_mask & ~ADS1256_ALL_CHANNEL_MASK) != 0U) {
        return -1;
    }

    ads1256_vref_uv = vref_uv;
    ads1256_pga_gain = pga_gain;
    ads1256_sample_rate_x10 = sps_x10;
    ads1256_channel_mask = channel_mask;
    return 0;
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
        if (ads1256_overflow_count < UINT16_MAX) {
            ads1256_overflow_count++;
        }
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

void adc_ads1256_prepare(void)
{
    lwrb_init(&ads1256_data_rb, ads1256_data_buf, ADS1256_DATA_BUFF_SIZE);
    ads1256_ring_ready = true;
    ads1256_started = false;
    ads1256_poll_error_count = 0;
    ads1256_last_recovery_tick = HAL_GetTick() - ADS1256_RECOVERY_INTERVAL_MS;
}

void adc_ads1256_start(void)
{
    if (!ads1256_ring_ready) {
        adc_ads1256_prepare();
    } else {
        lwrb_reset(&ads1256_data_rb);
        ads1256_started = false;
        ads1256_poll_error_count = 0;
    }

    int ret = adc_ads1256_init();
    if (ret < 0) {
        ads1256_last_recovery_tick = HAL_GetTick();
        return;
    }
    ret = adc_ads1256_set_vref_uv(ads1256_vref_uv);
    if (ret < 0) {
        ads1256_last_recovery_tick = HAL_GetTick();
        return;
    }
    ret = adc_ads1256_set_pga_gain(ads1256_pga_gain);
    if (ret < 0) {
        ads1256_last_recovery_tick = HAL_GetTick();
        return;
    }

    ret = adc_ads1256_set_sample_rate_x10(ads1256_sample_rate_x10);
    if (ret < 0 || adc_ads1256_calibrate() != 0 ||
        adc_ads1256_restart() != 0) {
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
            if (ads1256_started && ads1256_recovery_count < UINT16_MAX) {
                ads1256_recovery_count++;
            }
        }
        return;
    }

    bool poll_error = false;

    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }

        int ret = ads1256_device_poll(dev);
        if (ret < 0) {
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
    return adc_ads1256_set_sample_rate_x10((uint32_t)sps * 10U);
}

uint16_t adc_ads1256_get_sample_rate(void)
{
    return (uint16_t)(ads1256_sample_rate_x10 / 10U);
}

int adc_ads1256_set_sample_rate_x10(uint32_t sps_x10)
{
    ads1256_sps_t previous_rate;
    ads1256_sps_t rate;

    if (!ads1256_lookup_rate(sps_x10, &rate) ||
        !ads1256_lookup_rate(ads1256_sample_rate_x10, &previous_rate)) {
        return -1;
    }

    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }

        int ret = ads1256_set_sps(dev->adc, rate);
        if (ret < 0) {
            for (uint32_t rollback = 0; rollback < i; rollback++) {
                ads1256_scan_device_t *rollback_dev = &ads1256_devices[rollback];
                if (rollback_dev->enabled) {
                    (void)ads1256_set_sps(rollback_dev->adc, previous_rate);
                }
            }
            (void)adc_ads1256_restart();
            return ret;
        }
    }

    ads1256_sample_rate_x10 = sps_x10;
    return adc_ads1256_restart();
}

uint32_t adc_ads1256_get_sample_rate_x10(void)
{
    return ads1256_sample_rate_x10;
}

uint32_t adc_ads1256_get_cycling_rate_x10(void)
{
    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_rate_table); i++) {
        if (ads1256_rate_table[i].sps_x10 == ads1256_sample_rate_x10) {
            return ads1256_rate_table[i].cycling_rate_x10;
        }
    }
    return 0U;
}

uint16_t adc_ads1256_get_overflow_count(void)
{
    return ads1256_overflow_count;
}

uint16_t adc_ads1256_get_recovery_count(void)
{
    return ads1256_recovery_count;
}

uint8_t adc_ads1256_is_running(void)
{
    return ads1256_started ? 1U : 0U;
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

static bool ads1256_gain_to_enum(uint8_t gain, ads1256_pga_t *pga)
{
    uint8_t value = gain;
    uint8_t exponent = 0U;
    if (gain == 0U || gain > 64U || (gain & (gain - 1U)) != 0U) {
        return false;
    }
    while (value > 1U) {
        value >>= 1U;
        exponent++;
    }
    *pga = (ads1256_pga_t)exponent;
    return true;
}

int adc_ads1256_set_pga_gain(uint8_t gain)
{
    ads1256_pga_t pga;
    if (!ads1256_gain_to_enum(gain, &pga)) {
        return -1;
    }
    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (!dev->enabled) {
            continue;
        }
        ads1256_pga_t applied;
        if (ads1256_set_pga(dev->adc, pga) < 0 ||
            ads1256_get_pga(dev->adc, &applied) < 0 || applied != pga) {
            return -1;
        }
    }
    ads1256_pga_gain = gain;
    return 0;
}

uint8_t adc_ads1256_get_pga_gain(void)
{
    return ads1256_pga_gain;
}

int adc_ads1256_set_vref_uv(uint32_t vref_uv)
{
    if (vref_uv < 1000000U || vref_uv > 5000000U) {
        return -1;
    }
    float vref = (float)vref_uv / 1000000.0f;
    for (uint32_t i = 0; i < ADS1256_ARRAY_SIZE(ads1256_devices); i++) {
        ads1256_scan_device_t *dev = &ads1256_devices[i];
        if (dev->enabled && ads1256_set_vref(dev->adc, vref) < 0) {
            return -1;
        }
    }
    ads1256_vref_uv = vref_uv;
    return 0;
}

uint32_t adc_ads1256_get_vref_uv(void)
{
    return ads1256_vref_uv;
}
