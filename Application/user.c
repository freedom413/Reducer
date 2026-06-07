#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "delay.h"
#include "can.h"
#include "fdcan.h"
#include "ads1256_raw_data_recv.h"
#include "can_data.h"
#include "filter.h"
#include "flash_storage.h"
#include "flexspline_math.h"

// ============================================================================
// Constants and Configuration
// ============================================================================
#define ADC_CHANNEL_COUNT   ADS1256_LOGICAL_CHANNEL_COUNT
#define ADC_SAMPLE_BATCH_COUNT 32U
#define CAN_COMMANDS_PER_LOOP 4
#define CAN_INTERVAL_TEST_ENABLED    0U
#if CAN_INTERVAL_TEST_ENABLED
#define CAN_INTERVAL_TEST_ID         0x123U
#define CAN_INTERVAL_TEST_PERIOD_MS  100U
#endif
#define CAN_TX_WAIT_TIMEOUT_MS       5U
#define CAN_TELEMETRY_QUEUE_RECORD_COUNT      128U
#define CAN_TELEMETRY_FLUSH_PERIOD_MIN_MS     2U
#define CAN_TELEMETRY_FLUSH_PERIOD_MAX_MS     50U
#define CAN_HEALTH_PERIOD_MS         1000U
#define CONFIG_SAVE_DELAY_MS         750U

typedef struct {
    uint8_t channel;
    int32_t raw_filtered;
    int32_t voltage_uv;
    int16_t strain_ue;
    int16_t stress_qmpa;
} can_telemetry_sample_t;

// ============================================================================
// Module State
// ============================================================================
static ads1256_data_t adc_ads1256_data[ADC_SAMPLE_BATCH_COUNT] = {0};
static bool can_ready = false;
static uint8_t can_telemetry_mode = CAN_TELEMETRY_MODE_RAW;
static uint8_t can_telemetry_sequence = 0U;
static can_telemetry_sample_t
    can_telemetry_queue[CAN_TELEMETRY_QUEUE_RECORD_COUNT] = {0};
static uint16_t can_telemetry_queue_read = 0U;
static uint16_t can_telemetry_queue_write = 0U;
static uint16_t can_telemetry_queue_count = 0U;
static uint32_t can_telemetry_queue_first_tick = 0U;
static uint32_t can_telemetry_flush_period_ms =
    CAN_TELEMETRY_FLUSH_PERIOD_MAX_MS;
static uint16_t can_tx_drop_count = 0U;
static uint16_t can_tx_drop_reported_count = 0U;
static uint32_t can_health_last_tx_tick = 0U;
static uint16_t can_telemetry_samples_since_health = 0U;
static uint16_t can_telemetry_frames_since_health = 0U;
static persistent_config_t persistent_config;
static bool config_dirty = false;
static bool config_snapshot_pending = false;
static uint32_t config_save_deadline = 0U;
#if CAN_INTERVAL_TEST_ENABLED
static uint32_t can_test_last_tx_tick = 0;
static uint8_t can_test_sequence = 0;
#endif

// Physical parameters for flexspline calculation
static flexspline_params_t flexspline_params;

static void reset_can_telemetry_queue(void);
static void flush_can_telemetry(void);

#if CAN_INTERVAL_TEST_ENABLED
static void send_can_interval_test(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - can_test_last_tx_tick) < CAN_INTERVAL_TEST_PERIOD_MS) {
        return;
    }
    can_test_last_tx_tick = now;

    const uint8_t frame[8] = {
        0xAAU, 0x55U, can_test_sequence, (uint8_t)~can_test_sequence,
        0x00U, 0xFFU, 0x5AU, 0xA5U,
    };

    if (can_fd_data_frame_send(CAN_INTERVAL_TEST_ID, frame, sizeof(frame)) ==
        (int)sizeof(frame)) {
        can_test_sequence++;
    }
}
#endif

void setup(void)
{
    delay_init();
    can_ready = (can_init() == 0);

#if CAN_INTERVAL_TEST_ENABLED
    can_test_last_tx_tick = HAL_GetTick() - CAN_INTERVAL_TEST_PERIOD_MS;
    return;
#endif

    flash_storage_register_user_ops();
    if (flash_storage_load_config(&persistent_config) != 0) {
        flash_storage_config_defaults(&persistent_config);
        (void)flash_storage_save_config(&persistent_config);
    }

    adc_ads1256_start();
    if (adc_ads1256_set_vref_uv(persistent_config.vref_uv) != 0 ||
        adc_ads1256_set_pga_gain(persistent_config.pga_gain) != 0 ||
        adc_ads1256_set_sample_rate_x10(persistent_config.sample_rate_x10) != 0 ||
        adc_ads1256_set_channel_mask(persistent_config.channel_mask) != 0 ||
        persistent_config.filter_length < FILTER_WINDOW_SIZE_MIN ||
        persistent_config.filter_length > FILTER_WINDOW_SIZE_MAX ||
        persistent_config.telemetry_mode > CAN_TELEMETRY_MODE_PHYSICAL) {
        flash_storage_config_defaults(&persistent_config);
        (void)adc_ads1256_set_vref_uv(persistent_config.vref_uv);
        (void)adc_ads1256_set_pga_gain(persistent_config.pga_gain);
        (void)adc_ads1256_set_sample_rate_x10(persistent_config.sample_rate_x10);
        (void)adc_ads1256_set_channel_mask(persistent_config.channel_mask);
        (void)flash_storage_save_config(&persistent_config);
    }
    (void)adc_ads1256_calibrate();
    (void)adc_ads1256_restart();
    filter_init();
    filter_set_window_size(persistent_config.filter_length);
    can_telemetry_mode = persistent_config.telemetry_mode;
    reset_can_telemetry_queue();

    flexspline_params_set(
        &flexspline_params,
        (float)persistent_config.vref_uv / 1000000.0f,
        persistent_config.pga_gain,
        FLEXSPLINE_BRIDGE_EXCITATION_V,
        FLEXSPLINE_GAUGE_FACTOR,
        FLEXSPLINE_ELASTIC_MODULUS_MPA);
    config_snapshot_pending = true;
}

static void reset_channel_statistics(uint8_t ch)
{
    (void)ch;
}

static uint8_t active_ads1256_count(void)
{
    uint16_t channel_mask = adc_ads1256_get_channel_mask();
    uint8_t active_count = 0;

    for (uint8_t base = 0; base < ADC_CHANNEL_COUNT;
         base = (uint8_t)(base + ADS1256_CHANNELS_PER_DEVICE)) {
        uint16_t device_mask =
            (uint16_t)(((1U << ADS1256_CHANNELS_PER_DEVICE) - 1U) << base);
        if ((channel_mask & device_mask) != 0U) {
            active_count++;
        }
    }

    return active_count;
}

static uint8_t active_ads1256_channel_count(void)
{
    uint16_t mask = adc_ads1256_get_channel_mask();
    uint8_t count = 0U;

    while (mask != 0U) {
        count = (uint8_t)(count + (mask & 1U));
        mask >>= 1U;
    }
    return count;
}

static uint32_t calculate_can_telemetry_flush_period_ms(uint8_t max_records)
{
    uint32_t aggregate_rate_x10 =
        adc_ads1256_get_cycling_rate_x10() * active_ads1256_count();
    uint32_t active_channels = active_ads1256_channel_count();

    if (aggregate_rate_x10 == 0U || active_channels == 0U) {
        return CAN_TELEMETRY_FLUSH_PERIOD_MAX_MS;
    }

    /*
     * Prefer a full frame, but do not add more than roughly half of the
     * moving-average window latency. The aggregate source rate includes all
     * active ADCs; dividing it across enabled channels estimates the
     * per-channel filter-window duration.
     */
    uint32_t fill_ms =
        ((uint32_t)max_records * 10000U + aggregate_rate_x10 - 1U) /
        aggregate_rate_x10;
    uint32_t half_filter_window_ms =
        ((uint32_t)filter_get_window_size() * active_channels * 5000U +
         aggregate_rate_x10 - 1U) /
        aggregate_rate_x10;
    uint32_t minimum_batch_ms =
        (((uint32_t)max_records + 1U) / 2U * 10000U +
         aggregate_rate_x10 - 1U) /
        aggregate_rate_x10;
    uint32_t period_ms =
        fill_ms < half_filter_window_ms ? fill_ms : half_filter_window_ms;
    if (period_ms < minimum_batch_ms) {
        period_ms = minimum_batch_ms;
    }

    if (period_ms < CAN_TELEMETRY_FLUSH_PERIOD_MIN_MS) {
        return CAN_TELEMETRY_FLUSH_PERIOD_MIN_MS;
    }
    if (period_ms > CAN_TELEMETRY_FLUSH_PERIOD_MAX_MS) {
        return CAN_TELEMETRY_FLUSH_PERIOD_MAX_MS;
    }
    return period_ms;
}

static void reset_can_telemetry_queue(void)
{
    uint8_t max_records =
        can_telemetry_mode == CAN_TELEMETRY_MODE_RAW ?
            CAN_TELEMETRY_RAW_MAX_RECORDS :
            CAN_TELEMETRY_PHYSICAL_MAX_RECORDS;
    can_telemetry_flush_period_ms =
        calculate_can_telemetry_flush_period_ms(max_records);
    can_telemetry_queue_read = 0U;
    can_telemetry_queue_write = 0U;
    can_telemetry_queue_count = 0U;
    can_telemetry_queue_first_tick = HAL_GetTick();
}

static void send_can_status(uint8_t sequence, uint8_t cmd_type, uint8_t status,
                            uint32_t value, uint8_t detail)
{
    if (!can_ready) {
        return;
    }

    can_tx_status_frame_t frame;
    can_build_status_frame(&frame, sequence, cmd_type, status, value, detail);
    uint32_t start = HAL_GetTick();
    while (can_fd_data_frame_send(CAN_ID_TX_STATUS, (const uint8_t *)&frame,
                                  sizeof(frame)) == -3) {
        if ((uint32_t)(HAL_GetTick() - start) >= CAN_TX_WAIT_TIMEOUT_MS) {
            break;
        }
    }
}

static void count_can_tx_drops(uint16_t count)
{
    uint32_t total = (uint32_t)can_tx_drop_count + count;
    can_tx_drop_count = total > UINT16_MAX ? UINT16_MAX : (uint16_t)total;
}

static uint16_t can_tx_drop_delta(void)
{
    if (can_tx_drop_count < can_tx_drop_reported_count) {
        return 0U;
    }
    return (uint16_t)(can_tx_drop_count - can_tx_drop_reported_count);
}

static void pop_can_telemetry_records(uint8_t record_count)
{
    can_telemetry_queue_read =
        (uint16_t)((can_telemetry_queue_read + record_count) &
                   (CAN_TELEMETRY_QUEUE_RECORD_COUNT - 1U));
    can_telemetry_queue_count -= record_count;
    if (can_telemetry_queue_count > 0U) {
        can_telemetry_queue_first_tick = HAL_GetTick();
    }
}

static void queue_can_telemetry(uint8_t channel, int32_t raw_filtered,
                                int32_t voltage_uv, int16_t strain_ue,
                                int16_t stress_qmpa)
{
    if (!can_ready || channel >= ADC_CHANNEL_COUNT) {
        return;
    }

    if (can_telemetry_queue_count == CAN_TELEMETRY_QUEUE_RECORD_COUNT) {
        pop_can_telemetry_records(1U);
        count_can_tx_drops(1U);
    }
    if (can_telemetry_queue_count == 0U) {
        can_telemetry_queue_first_tick = HAL_GetTick();
    }
    can_telemetry_queue[can_telemetry_queue_write].channel = channel;
    can_telemetry_queue[can_telemetry_queue_write].raw_filtered = raw_filtered;
    can_telemetry_queue[can_telemetry_queue_write].voltage_uv = voltage_uv;
    can_telemetry_queue[can_telemetry_queue_write].strain_ue = strain_ue;
    can_telemetry_queue[can_telemetry_queue_write].stress_qmpa = stress_qmpa;
    can_telemetry_queue_write =
        (uint16_t)((can_telemetry_queue_write + 1U) &
                   (CAN_TELEMETRY_QUEUE_RECORD_COUNT - 1U));
    can_telemetry_queue_count++;
}

static void flush_can_telemetry(void)
{
    if (!can_ready) {
        return;
    }

    while (can_telemetry_queue_count > 0U) {
        uint8_t max_records =
            can_telemetry_mode == CAN_TELEMETRY_MODE_RAW ?
                CAN_TELEMETRY_RAW_MAX_RECORDS :
                CAN_TELEMETRY_PHYSICAL_MAX_RECORDS;
        uint32_t now = HAL_GetTick();
        if (can_telemetry_queue_count < max_records &&
            (uint32_t)(now - can_telemetry_queue_first_tick) <
                can_telemetry_flush_period_ms) {
            return;
        }

        uint8_t record_count =
            can_telemetry_queue_count > max_records ?
                max_records : (uint8_t)can_telemetry_queue_count;
        uint16_t drop_delta = can_tx_drop_delta();
        int ret;
        if (can_telemetry_mode == CAN_TELEMETRY_MODE_RAW) {
            can_tx_raw_telemetry_batch_frame_t frame = {0};
            frame.frame_type = CAN_FRAME_TYPE_TELEMETRY_RAW_BATCH;
            frame.version = CAN_PROTOCOL_VERSION;
            frame.telemetry_mode = CAN_TELEMETRY_MODE_RAW;
            frame.sequence = can_telemetry_sequence;
            frame.record_count = record_count;
            frame.drop_delta_le[0] = (uint8_t)(drop_delta & 0xFFU);
            frame.drop_delta_le[1] = (uint8_t)((drop_delta >> 8) & 0xFFU);
            for (uint8_t i = 0U; i < record_count; i++) {
                uint16_t queue_index =
                    (uint16_t)((can_telemetry_queue_read + i) &
                               (CAN_TELEMETRY_QUEUE_RECORD_COUNT - 1U));
                const can_telemetry_sample_t *sample =
                    &can_telemetry_queue[queue_index];
                can_build_raw_telemetry_record(&frame.records[i],
                                               sample->channel,
                                               sample->raw_filtered);
            }
            ret = can_fd_data_frame_send_low_priority(
                CAN_ID_TX_TELEMETRY,
                (const uint8_t *)&frame,
                sizeof(frame));
        } else {
            can_tx_physical_telemetry_batch_frame_t frame = {0};
            frame.frame_type = CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH;
            frame.version = CAN_PROTOCOL_VERSION;
            frame.telemetry_mode = CAN_TELEMETRY_MODE_PHYSICAL;
            frame.sequence = can_telemetry_sequence;
            frame.record_count = record_count;
            frame.drop_delta_le[0] = (uint8_t)(drop_delta & 0xFFU);
            frame.drop_delta_le[1] = (uint8_t)((drop_delta >> 8) & 0xFFU);
            for (uint8_t i = 0U; i < record_count; i++) {
                uint16_t queue_index =
                    (uint16_t)((can_telemetry_queue_read + i) &
                               (CAN_TELEMETRY_QUEUE_RECORD_COUNT - 1U));
                const can_telemetry_sample_t *sample =
                    &can_telemetry_queue[queue_index];
                can_build_physical_telemetry_record(&frame.records[i],
                                                    sample->channel,
                                                    sample->voltage_uv,
                                                    sample->strain_ue,
                                                    sample->stress_qmpa);
            }
            ret = can_fd_data_frame_send_low_priority(
                CAN_ID_TX_TELEMETRY,
                (const uint8_t *)&frame,
                sizeof(frame));
        }
        if (ret == -3) {
            return;
        }
        if (ret != (int)CAN_TELEMETRY_BATCH_FRAME_LEN) {
            count_can_tx_drops(record_count);
        } else {
            can_tx_drop_reported_count = can_tx_drop_count;
            can_telemetry_sequence++;
            uint32_t samples_total =
                (uint32_t)can_telemetry_samples_since_health + record_count;
            uint32_t frames_total =
                (uint32_t)can_telemetry_frames_since_health + 1U;
            can_telemetry_samples_since_health =
                samples_total > UINT16_MAX ? UINT16_MAX : (uint16_t)samples_total;
            can_telemetry_frames_since_health =
                frames_total > UINT16_MAX ? UINT16_MAX : (uint16_t)frames_total;
        }
        pop_can_telemetry_records(record_count);
    }
}

static void send_can_health(void)
{
    uint32_t now = HAL_GetTick();
    if (!can_ready ||
        (uint32_t)(now - can_health_last_tx_tick) < CAN_HEALTH_PERIOD_MS) {
        return;
    }
    can_health_last_tx_tick = now;

    uint8_t flags = adc_ads1256_is_running() != 0U ? 0x01U : 0x00U;
    flags |= config_dirty ? 0x02U : 0U;
    flags |= (persistent_config.flags & PERSISTENT_CONFIG_FLAG_ZERO_VALID) != 0U ?
             0x04U : 0U;
    can_tx_health_frame_t frame;
    can_build_health_frame(&frame,
                           adc_ads1256_get_sample_rate_x10(),
                           can_tx_drop_count,
                           adc_ads1256_get_overflow_count(),
                           adc_ads1256_get_recovery_count(),
                           can_telemetry_samples_since_health,
                           can_telemetry_frames_since_health,
                           active_ads1256_count(),
                           can_telemetry_mode,
                           flags);
    if (can_fd_data_frame_send(CAN_ID_TX_HEALTH,
                               (const uint8_t *)&frame, sizeof(frame)) !=
        (int)sizeof(frame)) {
        count_can_tx_drops(1U);
    } else {
        can_telemetry_samples_since_health = 0U;
        can_telemetry_frames_since_health = 0U;
    }
}

static void u32_le_store(uint8_t value_le[4], uint32_t value)
{
    value_le[0] = (uint8_t)value;
    value_le[1] = (uint8_t)(value >> 8);
    value_le[2] = (uint8_t)(value >> 16);
    value_le[3] = (uint8_t)(value >> 24);
}

static void sync_persistent_config_from_runtime(void)
{
    persistent_config.vref_uv = adc_ads1256_get_vref_uv();
    persistent_config.sample_rate_x10 = adc_ads1256_get_sample_rate_x10();
    persistent_config.channel_mask = adc_ads1256_get_channel_mask();
    persistent_config.pga_gain = adc_ads1256_get_pga_gain();
    persistent_config.filter_length = filter_get_window_size();
    persistent_config.telemetry_mode = can_telemetry_mode;
    for (uint8_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++) {
        int32_t offset;
        filter_get_zero_offset(channel, &offset);
        persistent_config.zero_offset[channel] = offset;
    }
}

static void request_config_save(bool immediate)
{
    sync_persistent_config_from_runtime();
    config_dirty = true;
    config_snapshot_pending = true;
    config_save_deadline = immediate ? HAL_GetTick() :
        HAL_GetTick() + CONFIG_SAVE_DELAY_MS;
}

static int save_config_now(void)
{
    sync_persistent_config_from_runtime();
    if (flash_storage_save_config(&persistent_config) != 0) {
        config_dirty = true;
        config_snapshot_pending = true;
        config_save_deadline = HAL_GetTick() + CONFIG_SAVE_DELAY_MS;
        return -1;
    }
    config_dirty = false;
    config_snapshot_pending = true;
    return 0;
}

static void service_config_save(void)
{
    if (!config_dirty ||
        (int32_t)(HAL_GetTick() - config_save_deadline) < 0) {
        return;
    }
    sync_persistent_config_from_runtime();
    if (flash_storage_save_config(&persistent_config) == 0) {
        config_dirty = false;
        config_snapshot_pending = true;
    } else {
        config_save_deadline = HAL_GetTick() + CONFIG_SAVE_DELAY_MS;
    }
}

static void send_config_snapshot(void)
{
    if (!can_ready || !config_snapshot_pending) {
        return;
    }
    sync_persistent_config_from_runtime();
    can_tx_config_frame_t frame = {0};
    frame.frame_type = CAN_FRAME_TYPE_CONFIG;
    frame.version = CAN_PROTOCOL_VERSION;
    frame.flags = (config_dirty ? 0U : 0x01U) |
                  ((persistent_config.flags &
                    PERSISTENT_CONFIG_FLAG_ZERO_VALID) != 0U ? 0x02U : 0U);
    frame.pga_gain = persistent_config.pga_gain;
    frame.filter_length = persistent_config.filter_length;
    frame.telemetry_mode = persistent_config.telemetry_mode;
    frame.channel_mask_le[0] = (uint8_t)persistent_config.channel_mask;
    frame.channel_mask_le[1] = (uint8_t)(persistent_config.channel_mask >> 8);
    u32_le_store(frame.sample_rate_x10_le, persistent_config.sample_rate_x10);
    u32_le_store(frame.vref_uv_le, persistent_config.vref_uv);
    u32_le_store(frame.config_sequence_le, persistent_config.sequence);
    for (uint8_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++) {
        u32_le_store(frame.zero_offset_le[channel],
                     (uint32_t)persistent_config.zero_offset[channel]);
    }
    if (can_fd_data_frame_send(CAN_ID_TX_CONFIG, (const uint8_t *)&frame,
                               sizeof(frame)) == (int)sizeof(frame)) {
        config_snapshot_pending = false;
    }
}

static int16_t clamp_i16_from_float(float value)
{
    long rounded = lrintf(value);
    if (rounded > INT16_MAX) {
        return INT16_MAX;
    }
    if (rounded < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)rounded;
}

static void process_adc_sample(uint8_t channel, int32_t raw_value)
{
    int32_t filtered = filter_apply(channel, raw_value);

    if (can_telemetry_mode == CAN_TELEMETRY_MODE_RAW) {
        queue_can_telemetry(channel, filtered, 0, 0, 0);
        return;
    }

    flexspline_result_t result;
    flexspline_calculate(filtered, &flexspline_params, &result);

    int32_t voltage_uv = (int32_t)lroundf(result.voltage * 1000.0f);
    int16_t strain_ue = clamp_i16_from_float(result.strain);
    int16_t stress_qmpa = clamp_i16_from_float(result.stress * 4.0f);
    queue_can_telemetry(channel, filtered, voltage_uv, strain_ue, stress_qmpa);
}

static uint8_t process_can_command(uint8_t cmd_type, uint8_t param, uint32_t value,
                                   uint32_t *applied_value, uint8_t *detail)
{
    if (applied_value != NULL) {
        *applied_value = value;
    }
    if (detail != NULL) {
        *detail = 0;
    }

    switch (cmd_type) {
        case CAN_CMD_ZERO_DATUM: {
            // Save current filtered values as zero offset, then reset
            uint16_t channel_mask = adc_ads1256_get_channel_mask();
            if (channel_mask == 0U) {
                return CAN_STATUS_BAD_VALUE;
            }
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                if ((channel_mask & (1U << i)) == 0U) {
                    continue;
                }
                if (!filter_has_samples(i)) {
                    if (detail != NULL) {
                        *detail = i;
                    }
                    return CAN_STATUS_BAD_VALUE;
                }
            }
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                if ((channel_mask & (1U << i)) == 0U) {
                    continue;
                }
                int32_t raw_filtered = filter_get_raw_filtered(i);
                filter_set_zero_offset(i, raw_filtered);
            }
            persistent_config.flags |= PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            if (save_config_now() != 0) {
                return CAN_STATUS_STORAGE_ERROR;
            }
            filter_reset_all();
            reset_can_telemetry_queue();
            return CAN_STATUS_OK;
        }

        case CAN_CMD_SET_ZERO_OFFSET:
            if (param >= ADC_CHANNEL_COUNT) {
                if (detail != NULL) {
                    *detail = param;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            filter_set_zero_offset(param, (int32_t)value);
            persistent_config.flags |= PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            filter_reset(param);
            reset_channel_statistics(param);
            reset_can_telemetry_queue();
            request_config_save(false);
            return CAN_STATUS_OK;

        case CAN_CMD_SET_CHANNEL_MASK: {
            if (value > ADS1256_ALL_CHANNEL_MASK) {
                return CAN_STATUS_BAD_VALUE;
            }
            uint16_t previous_mask = adc_ads1256_get_channel_mask();
            if (adc_ads1256_set_channel_mask((uint16_t)value) != 0) {
                if (detail != NULL) {
                    *detail = (uint8_t)value;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            if (applied_value != NULL) {
                *applied_value = adc_ads1256_get_channel_mask();
            }
            uint16_t changed_mask = previous_mask ^ value;
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                if ((changed_mask & (1U << i)) != 0U) {
                    filter_reset(i);
                    reset_channel_statistics(i);
                }
            }
            reset_can_telemetry_queue();
            request_config_save(false);
            return CAN_STATUS_OK;
        }

        case CAN_CMD_SET_FILTER_SIZE:
            if (value < 2U || value > 64U) {
                if (detail != NULL) {
                    *detail = (uint8_t)value;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            filter_set_window_size((uint8_t)value);
            if (applied_value != NULL) {
                *applied_value = value;
            }
            reset_can_telemetry_queue();
            request_config_save(false);
            return CAN_STATUS_OK;

        case CAN_CMD_SET_SAMPLE_RATE: {
            uint32_t requested_sps_x10;
            if (param == CAN_SAMPLE_RATE_PARAM_SPS) {
                requested_sps_x10 = (uint32_t)value * 10U;
            } else if (param == CAN_SAMPLE_RATE_PARAM_DECI_SPS) {
                requested_sps_x10 = value;
            } else {
                if (detail != NULL) {
                    *detail = param;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            if (adc_ads1256_set_sample_rate_x10(requested_sps_x10) != 0) {
                if (detail != NULL) {
                    *detail = 1;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            if (applied_value != NULL) {
                *applied_value = (uint16_t)(
                    param == CAN_SAMPLE_RATE_PARAM_DECI_SPS ?
                    adc_ads1256_get_sample_rate_x10() :
                    adc_ads1256_get_sample_rate());
            }
            filter_reset_all();
            reset_can_telemetry_queue();
            request_config_save(false);
            return CAN_STATUS_OK;
        }

        case CAN_CMD_START_CALIB:
            if (adc_ads1256_calibrate() != 0 || adc_ads1256_restart() != 0) {
                if (detail != NULL) {
                    *detail = 2;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            filter_reset_all();
            reset_can_telemetry_queue();
            request_config_save(false);
            return CAN_STATUS_OK;

        case CAN_CMD_CLEAR_ZERO:
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            persistent_config.flags &= (uint8_t)~PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            if (save_config_now() != 0) {
                return CAN_STATUS_STORAGE_ERROR;
            }
            filter_reset_all();
            reset_can_telemetry_queue();
            return CAN_STATUS_OK;

        case CAN_CMD_SET_TELEMETRY_MODE:
            if (value != CAN_TELEMETRY_MODE_RAW &&
                value != CAN_TELEMETRY_MODE_PHYSICAL) {
                if (detail != NULL) {
                    *detail = (uint8_t)value;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            can_telemetry_mode = (uint8_t)value;
            reset_can_telemetry_queue();
            if (applied_value != NULL) {
                *applied_value = can_telemetry_mode;
            }
            request_config_save(false);
            return CAN_STATUS_OK;

        case CAN_CMD_GET_CONFIG:
            config_snapshot_pending = true;
            return CAN_STATUS_OK;

        case CAN_CMD_SET_VREF_UV:
            if (adc_ads1256_set_vref_uv(value) != 0) {
                return CAN_STATUS_BAD_VALUE;
            }
            flexspline_params_set(
                &flexspline_params, (float)value / 1000000.0f,
                adc_ads1256_get_pga_gain(), FLEXSPLINE_BRIDGE_EXCITATION_V,
                FLEXSPLINE_GAUGE_FACTOR, FLEXSPLINE_ELASTIC_MODULUS_MPA);
            request_config_save(false);
            return CAN_STATUS_OK;

        case CAN_CMD_SET_PGA:
            if (value > UINT8_MAX) {
                return CAN_STATUS_BAD_VALUE;
            }
            {
                uint8_t previous_gain = adc_ads1256_get_pga_gain();
                if (adc_ads1256_set_pga_gain((uint8_t)value) != 0) {
                    return CAN_STATUS_BAD_VALUE;
                }
                if (adc_ads1256_calibrate() != 0 || adc_ads1256_restart() != 0) {
                    (void)adc_ads1256_set_pga_gain(previous_gain);
                    (void)adc_ads1256_restart();
                    return CAN_STATUS_BAD_VALUE;
                }
            }
            for (uint8_t i = 0U; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            persistent_config.flags &= (uint8_t)~PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            flexspline_params_set(
                &flexspline_params,
                (float)adc_ads1256_get_vref_uv() / 1000000.0f,
                adc_ads1256_get_pga_gain(), FLEXSPLINE_BRIDGE_EXCITATION_V,
                FLEXSPLINE_GAUGE_FACTOR, FLEXSPLINE_ELASTIC_MODULUS_MPA);
            filter_reset_all();
            reset_can_telemetry_queue();
            if (save_config_now() != 0) {
                return CAN_STATUS_STORAGE_ERROR;
            }
            return CAN_STATUS_OK;

        case CAN_CMD_RESTORE_DEFAULTS:
            flash_storage_config_defaults(&persistent_config);
            for (uint8_t i = 0U; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            (void)adc_ads1256_set_vref_uv(persistent_config.vref_uv);
            (void)adc_ads1256_set_pga_gain(persistent_config.pga_gain);
            (void)adc_ads1256_set_sample_rate_x10(persistent_config.sample_rate_x10);
            (void)adc_ads1256_set_channel_mask(persistent_config.channel_mask);
            (void)adc_ads1256_calibrate();
            (void)adc_ads1256_restart();
            filter_set_window_size(persistent_config.filter_length);
            can_telemetry_mode = persistent_config.telemetry_mode;
            flexspline_params_set_default(&flexspline_params);
            filter_reset_all();
            reset_can_telemetry_queue();
            if (save_config_now() != 0) {
                return CAN_STATUS_STORAGE_ERROR;
            }
            return CAN_STATUS_OK;

        default:
            if (detail != NULL) {
                *detail = cmd_type;
            }
            return CAN_STATUS_BAD_CMD;
    }
}

static void process_can_commands(void)
{
    if (!can_ready) {
        return;
    }

    can_msg_t msg;
    for (uint8_t cmd_count = 0; cmd_count < CAN_COMMANDS_PER_LOOP; cmd_count++) {
        if (can_recv(&msg, 1) != 1) {
            break;
        }

        int dlc_bytes = can_data_len_get(msg.RxHeader.DataLength);
        if (msg.RxHeader.Identifier != CAN_ID_RX_COMMAND ||
            msg.RxHeader.FDFormat != FDCAN_FD_CAN ||
            msg.RxHeader.BitRateSwitch != FDCAN_BRS_ON ||
            dlc_bytes != (int)sizeof(can_rx_command_frame_t)) {
            continue;
        }

        const can_rx_command_frame_t *frame = (const can_rx_command_frame_t *)msg.data;
        uint32_t value = can_frame_u32_le_get(frame->value_le);
        uint8_t detail = 0;

        if (frame->frame_type != CAN_FRAME_TYPE_COMMAND) {
            send_can_status(frame->sequence, frame->cmd_type, CAN_STATUS_BAD_TYPE,
                            value, frame->frame_type);
        } else if (frame->version != CAN_PROTOCOL_VERSION) {
            send_can_status(frame->sequence, frame->cmd_type, CAN_STATUS_BAD_TYPE,
                            value, frame->version);
        } else {
            uint32_t applied_value = value;
            uint8_t status = process_can_command(frame->cmd_type, frame->param, value,
                                                 &applied_value, &detail);
            send_can_status(frame->sequence, frame->cmd_type, status,
                            applied_value, detail);
        }
    }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop(void)
{
#if CAN_INTERVAL_TEST_ENABLED
    if (can_ready) {
        while(1)
        send_can_interval_test();
    }
    return;
#endif

    process_can_commands();
    service_config_save();
    send_config_snapshot();
    send_can_health();
    flush_can_telemetry();

    adc_ads1256_poll();

    int recv_count = adc_ads1256_get_data(adc_ads1256_data, ADC_SAMPLE_BATCH_COUNT);

    for (int i = 0; i < recv_count; i++) {
        uint8_t ch = adc_ads1256_data[i].channel;
        if (ch < ADC_CHANNEL_COUNT) {
            /*
             * Process channels as conversions arrive. The two ADS1256 devices
             * normally produce at most two samples per poll, which avoids a
             * multi-frame burst overflowing the three-slot FDCAN TX FIFO.
             */
            process_adc_sample(ch, adc_ads1256_data[i].raw_value);
        }
    }

    flush_can_telemetry();
}
