#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "delay.h"
#include "main.h"
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
#define CAN_TX_DRIVER_TEST_ENABLED   0U
#if CAN_TX_DRIVER_TEST_ENABLED
#define CAN_TX_DRIVER_TEST_CLASSIC_ID 0x123U
#define CAN_TX_DRIVER_TEST_PERIOD_MS  100U
#define CAN_TX_DRIVER_TEST_SEND_FD   0U
#define CAN_TX_DRIVER_TEST_GPIO_PULSE_ONLY 0U
#define CAN_TX_DRIVER_TEST_PULSE_PA12_ON_CAN_INIT_FAIL 1U
#endif
#define CAN_INTERVAL_TEST_ENABLED    0U
#if CAN_INTERVAL_TEST_ENABLED
#define CAN_INTERVAL_TEST_ID         0x123U
#define CAN_INTERVAL_TEST_PERIOD_MS  1000U
#endif
#define CAN_STATUS_QUEUE_COUNT       16U
#define CAN_TELEMETRY_QUEUE_RECORD_COUNT      128U
#define CAN_TELEMETRY_FLUSH_PERIOD_MIN_MS     2U
#define CAN_TELEMETRY_FLUSH_PERIOD_MAX_MS     50U
#define CAN_HEALTH_PERIOD_MS         1000U
#define CAN_DIAG_PERIOD_MS           250U
#define CONFIG_SAVE_DELAY_MS         750U
#define HOST_SESSION_TIMEOUT_MS      3000U
#define MCU_LED_IDLE_PERIOD_MS       1000U
#define MCU_LED_IDLE_ON_MS           75U
#define MCU_LED_STREAM_PERIOD_MS     250U
#define MCU_LED_STREAM_ON_MS         75U
#define MCU_LED_FAULT_PERIOD_MS      250U
#define MCU_LED_FAULT_ON_MS          125U
#define CAN_RESTORE_STEP_VREF         1U
#define CAN_RESTORE_STEP_PGA          2U
#define CAN_RESTORE_STEP_SAMPLE_RATE  3U
#define CAN_RESTORE_STEP_CHANNEL_MASK 4U
#define CAN_RESTORE_STEP_CALIBRATION  5U
#define CAN_RESTORE_STEP_RESTART      6U
#define CAN_RESTORE_ROLLBACK_FAILED   0x80U

typedef struct {
    uint8_t channel;
    int32_t raw_filtered;
    int32_t voltage_uv;
    int16_t strain_ue;
    int16_t stress_qmpa;
} can_telemetry_sample_t;

typedef struct {
    persistent_config_t config;
    bool dirty;
    uint32_t save_deadline;
} config_transaction_snapshot_t;

typedef enum {
    MCU_LED_PATTERN_IDLE,
    MCU_LED_PATTERN_STREAMING,
    MCU_LED_PATTERN_FAULT,
    MCU_LED_PATTERN_COUNT,
} mcu_led_pattern_t;

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
static uint32_t can_diag_last_tx_tick = 0U;
static uint16_t can_telemetry_samples_since_health = 0U;
static uint16_t can_telemetry_frames_since_health = 0U;
static uint8_t can_diag_sequence = 0U;
static bool can_main_loop_seen = false;
static bool host_session_active = false;
static uint32_t host_session_last_rx_tick = 0U;
static mcu_led_pattern_t mcu_led_pattern = MCU_LED_PATTERN_COUNT;
static uint32_t mcu_led_pattern_started_tick = 0U;
static bool mcu_led_is_on = false;
static can_tx_status_frame_t can_status_queue[CAN_STATUS_QUEUE_COUNT];
static uint8_t can_status_queue_read = 0U;
static uint8_t can_status_queue_write = 0U;
static uint8_t can_status_queue_count = 0U;
static persistent_config_t persistent_config;
static bool config_dirty = false;
static bool config_snapshot_pending = false;
static uint32_t config_save_deadline = 0U;
#if CAN_TX_DRIVER_TEST_ENABLED
static uint32_t can_tx_driver_test_last_tx_tick = 0U;
static uint8_t can_tx_driver_test_sequence = 0U;
static volatile int can_tx_driver_test_last_result = 0;
#endif
#if CAN_INTERVAL_TEST_ENABLED
static uint32_t can_test_last_tx_tick = 0;
static uint8_t can_test_sequence = 0;
#endif

// Physical parameters for flexspline calculation
static flexspline_params_t flexspline_params;

static void reset_can_telemetry_queue(void);
static void flush_can_telemetry(void);
static void service_can_status_queue(void);

/* MCU_LED is wired active-low: RESET turns it on, SET turns it off. */
static void mcu_led_set(bool on)
{
    if (mcu_led_is_on == on) {
        return;
    }

    HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    mcu_led_is_on = on;
}

#if CAN_TX_DRIVER_TEST_ENABLED
static void mcu_led_toggle(void)
{
    mcu_led_set(!mcu_led_is_on);
}
#endif

static void service_mcu_led(void)
{
    uint32_t period_ms;
    uint32_t on_time_ms;
    mcu_led_pattern_t desired_pattern;

    /* Idle: slow heartbeat; host session: fast activity; fault: 50% blink. */
    if (!can_ready || adc_ads1256_is_running() == 0U) {
        desired_pattern = MCU_LED_PATTERN_FAULT;
        period_ms = MCU_LED_FAULT_PERIOD_MS;
        on_time_ms = MCU_LED_FAULT_ON_MS;
    } else if (host_session_active) {
        desired_pattern = MCU_LED_PATTERN_STREAMING;
        period_ms = MCU_LED_STREAM_PERIOD_MS;
        on_time_ms = MCU_LED_STREAM_ON_MS;
    } else {
        desired_pattern = MCU_LED_PATTERN_IDLE;
        period_ms = MCU_LED_IDLE_PERIOD_MS;
        on_time_ms = MCU_LED_IDLE_ON_MS;
    }

    uint32_t now = HAL_GetTick();
    if (desired_pattern != mcu_led_pattern) {
        mcu_led_pattern = desired_pattern;
        mcu_led_pattern_started_tick = now;
    }

    uint32_t elapsed = (uint32_t)(now - mcu_led_pattern_started_tick);
    if (elapsed >= period_ms) {
        mcu_led_pattern_started_tick = now;
        elapsed = 0U;
    }
    mcu_led_set(elapsed < on_time_ms);
}

static void reset_can_status_queue(void)
{
    can_status_queue_read = 0U;
    can_status_queue_write = 0U;
    can_status_queue_count = 0U;
}

static void activate_host_session(void)
{
    host_session_active = true;
    host_session_last_rx_tick = HAL_GetTick();
}

static void end_host_session(void)
{
    host_session_active = false;
    reset_can_status_queue();
    reset_can_telemetry_queue();
}

static void service_host_session(void)
{
    if (!host_session_active ||
        (uint32_t)(HAL_GetTick() - host_session_last_rx_tick) <
            HOST_SESSION_TIMEOUT_MS) {
        return;
    }
    end_host_session();
}

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

#if CAN_TX_DRIVER_TEST_ENABLED
#if CAN_TX_DRIVER_TEST_GPIO_PULSE_ONLY || \
    CAN_TX_DRIVER_TEST_PULSE_PA12_ON_CAN_INIT_FAIL
static bool can_tx_driver_test_pa12_gpio_ready = false;

static void can_tx_driver_test_init_pa12_gpio(void)
{
    if (can_tx_driver_test_pa12_gpio_ready) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    can_tx_driver_test_pa12_gpio_ready = true;
}
#endif

static void send_can_tx_driver_test(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - can_tx_driver_test_last_tx_tick) <
        CAN_TX_DRIVER_TEST_PERIOD_MS) {
        return;
    }
    can_tx_driver_test_last_tx_tick = now;
    mcu_led_toggle();

#if CAN_TX_DRIVER_TEST_GPIO_PULSE_ONLY
    can_tx_driver_test_init_pa12_gpio();
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    return;
#endif

    if (!can_ready) {
#if CAN_TX_DRIVER_TEST_PULSE_PA12_ON_CAN_INIT_FAIL
        can_tx_driver_test_init_pa12_gpio();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
#endif
        return;
    }
    (void)can_recover_bus_off();

    const uint8_t frame[8] = {
        0xAAU,
        0x55U,
        can_tx_driver_test_sequence,
        (uint8_t)~can_tx_driver_test_sequence,
        (uint8_t)can_tx_driver_test_last_result,
        0x11U,
        0x22U,
        0x33U,
    };

#if CAN_TX_DRIVER_TEST_SEND_FD
    int ret = can_fd_data_frame_send(CAN_TX_DRIVER_TEST_CLASSIC_ID,
                                     frame, sizeof(frame));
#else
    int ret = can_classic_data_frame_send(CAN_TX_DRIVER_TEST_CLASSIC_ID,
                                          frame, sizeof(frame));
#endif
    can_tx_driver_test_last_result = ret;
    if (ret == (int)sizeof(frame)) {
        can_tx_driver_test_sequence++;
    }
}
#endif

void setup(void)
{
    delay_init();
    can_ready = (can_init() == 0);
    reset_can_status_queue();
    host_session_active = false;
    mcu_led_pattern = MCU_LED_PATTERN_COUNT;
    mcu_led_pattern_started_tick = HAL_GetTick();
    mcu_led_set(false);

#if CAN_TX_DRIVER_TEST_ENABLED
#if CAN_TX_DRIVER_TEST_GPIO_PULSE_ONLY
    can_tx_driver_test_init_pa12_gpio();
#endif
    can_tx_driver_test_last_tx_tick =
        HAL_GetTick() - CAN_TX_DRIVER_TEST_PERIOD_MS;
    return;
#endif

#if CAN_INTERVAL_TEST_ENABLED
    can_test_last_tx_tick = HAL_GetTick() - CAN_INTERVAL_TEST_PERIOD_MS;
    return;
#endif

    flash_storage_register_user_ops();
    if (flash_storage_load_config(&persistent_config) != 0) {
        flash_storage_config_defaults(&persistent_config);
        (void)flash_storage_save_config(&persistent_config);
    }

    bool config_valid =
        persistent_config.filter_length >= FILTER_WINDOW_SIZE_MIN &&
        persistent_config.filter_length <= FILTER_WINDOW_SIZE_MAX &&
        persistent_config.telemetry_mode <= CAN_TELEMETRY_MODE_PHYSICAL &&
        adc_ads1256_configure_startup(persistent_config.vref_uv,
                                      persistent_config.pga_gain,
                                      persistent_config.sample_rate_x10,
                                      persistent_config.channel_mask) == 0;
    if (!config_valid) {
        flash_storage_config_defaults(&persistent_config);
        (void)adc_ads1256_configure_startup(persistent_config.vref_uv,
                                            persistent_config.pga_gain,
                                            persistent_config.sample_rate_x10,
                                            persistent_config.channel_mask);
        (void)flash_storage_save_config(&persistent_config);
    }
    adc_ads1256_prepare();
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

static bool can_status_queue_has_space(void)
{
    return can_status_queue_count < CAN_STATUS_QUEUE_COUNT;
}

static void send_can_status(uint8_t sequence, uint8_t cmd_type, uint8_t status,
                            uint32_t value, uint8_t detail)
{
    if (!can_ready || !can_status_queue_has_space()) {
        return;
    }

    can_build_status_frame(&can_status_queue[can_status_queue_write],
                           sequence, cmd_type, status, value, detail);
    can_status_queue_write =
        (uint8_t)((can_status_queue_write + 1U) % CAN_STATUS_QUEUE_COUNT);
    can_status_queue_count++;
}

static void service_can_status_queue(void)
{
    while (can_ready && can_status_queue_count > 0U) {
        can_tx_status_frame_t *frame = &can_status_queue[can_status_queue_read];
        int ret = can_fd_data_frame_send(CAN_ID_TX_CONTROL,
                                         (const uint8_t *)frame,
                                         sizeof(*frame));
        if (ret != (int)sizeof(*frame)) {
            return;
        }
        can_status_queue_read =
            (uint8_t)((can_status_queue_read + 1U) % CAN_STATUS_QUEUE_COUNT);
        can_status_queue_count--;
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
    if (!can_ready || !host_session_active || channel >= ADC_CHANNEL_COUNT) {
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
    if (!can_ready || !host_session_active) {
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
    if (!can_ready || !host_session_active ||
        (uint32_t)(now - can_health_last_tx_tick) < CAN_HEALTH_PERIOD_MS) {
        return;
    }
    can_health_last_tx_tick = now;

    uint8_t flags = adc_ads1256_is_running() != 0U ? 0x01U : 0x00U;
    flags |= config_dirty ? 0x02U : 0U;
    flags |= (persistent_config.flags & PERSISTENT_CONFIG_FLAG_ZERO_VALID) != 0U ?
             0x04U : 0U;
    flags |= adc_ads1256_get_channel_mask() != 0U ? 0x08U : 0U;
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

static void send_can_diag(void)
{
    uint32_t now = HAL_GetTick();
    if (!can_ready || !host_session_active ||
        (uint32_t)(now - can_diag_last_tx_tick) < CAN_DIAG_PERIOD_MS) {
        return;
    }
    can_diag_last_tx_tick = now;

    can_diag_status_t diag = {0};
    can_diag_get(&diag);

    uint8_t flags = 0U;
    flags |= can_ready ? 0x01U : 0U;
    flags |= can_main_loop_seen ? 0x02U : 0U;
    flags |= diag.last_rx_fd != 0U ? 0x04U : 0U;
    flags |= diag.last_rx_brs != 0U ? 0x08U : 0U;
    flags |= diag.bus_off != 0U ? 0x10U : 0U;
    flags |= diag.error_passive != 0U ? 0x20U : 0U;

    can_tx_diag_frame_t frame;
    can_build_diag_frame(&frame, flags, diag.last_rx_dlc,
                         diag.last_reject_reason, diag.tx_error_count,
                         diag.rx_error_count, can_diag_sequence++);
    (void)can_classic_data_frame_send(CAN_ID_TX_DIAG,
                                      (const uint8_t *)&frame,
                                      sizeof(frame));
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

static void capture_config_transaction(config_transaction_snapshot_t *snapshot)
{
    sync_persistent_config_from_runtime();
    snapshot->config = persistent_config;
    snapshot->dirty = config_dirty;
    snapshot->save_deadline = config_save_deadline;
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

static int apply_ads_config(const persistent_config_t *config,
                            uint8_t *failed_step)
{
    if (failed_step != NULL) {
        *failed_step = 0U;
    }
    if (adc_ads1256_set_vref_uv(config->vref_uv) != 0) {
        if (failed_step != NULL) {
            *failed_step = CAN_RESTORE_STEP_VREF;
        }
        return -1;
    }
    if (adc_ads1256_set_pga_gain(config->pga_gain) != 0) {
        if (failed_step != NULL) {
            *failed_step = CAN_RESTORE_STEP_PGA;
        }
        return -1;
    }
    if (adc_ads1256_set_sample_rate_x10(config->sample_rate_x10) != 0) {
        if (failed_step != NULL) {
            *failed_step = CAN_RESTORE_STEP_SAMPLE_RATE;
        }
        return -1;
    }
    if (adc_ads1256_set_channel_mask(config->channel_mask) != 0) {
        if (failed_step != NULL) {
            *failed_step = CAN_RESTORE_STEP_CHANNEL_MASK;
        }
        return -1;
    }
    if (adc_ads1256_calibrate() != 0) {
        if (failed_step != NULL) {
            *failed_step = CAN_RESTORE_STEP_CALIBRATION;
        }
        return -1;
    }
    if (adc_ads1256_restart() != 0) {
        if (failed_step != NULL) {
            *failed_step = CAN_RESTORE_STEP_RESTART;
        }
        return -1;
    }
    return 0;
}

static void apply_non_ads_runtime_config(const persistent_config_t *config)
{
    for (uint8_t channel = 0U; channel < ADC_CHANNEL_COUNT; channel++) {
        filter_set_zero_offset(channel, config->zero_offset[channel]);
    }
    filter_set_window_size(config->filter_length);
    can_telemetry_mode = config->telemetry_mode;
    flexspline_params_set(
        &flexspline_params, (float)config->vref_uv / 1000000.0f,
        config->pga_gain, FLEXSPLINE_BRIDGE_EXCITATION_V,
        FLEXSPLINE_GAUGE_FACTOR, FLEXSPLINE_ELASTIC_MODULUS_MPA);
    filter_reset_all();
    reset_can_telemetry_queue();
}

static int restore_runtime_config(const persistent_config_t *config)
{
    uint8_t failed_step;
    int result = apply_ads_config(config, &failed_step);

    persistent_config = *config;
    apply_non_ads_runtime_config(config);
    return result;
}

static int commit_config_candidate(persistent_config_t *candidate)
{
    if (flash_storage_save_config(candidate) != 0) {
        config_snapshot_pending = true;
        return -1;
    }
    persistent_config = *candidate;
    config_dirty = false;
    config_snapshot_pending = true;
    return 0;
}

static int rollback_config_transaction(
    const config_transaction_snapshot_t *snapshot)
{
    int result = restore_runtime_config(&snapshot->config);
    persistent_config = snapshot->config;
    config_dirty = snapshot->dirty;
    config_save_deadline = snapshot->save_deadline;
    config_snapshot_pending = true;
    return result;
}

static void send_config_snapshot(void)
{
    if (!can_ready || !host_session_active || !config_snapshot_pending) {
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
    if (can_fd_data_frame_send(CAN_ID_TX_CONTROL, (const uint8_t *)&frame,
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
            config_transaction_snapshot_t snapshot;
            persistent_config_t candidate;
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
            capture_config_transaction(&snapshot);
            candidate = snapshot.config;
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                if ((channel_mask & (1U << i)) == 0U) {
                    continue;
                }
                candidate.zero_offset[i] = filter_get_raw_filtered(i);
            }
            candidate.flags |= PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            if (commit_config_candidate(&candidate) != 0) {
                persistent_config = snapshot.config;
                config_dirty = snapshot.dirty;
                config_save_deadline = snapshot.save_deadline;
                return CAN_STATUS_STORAGE_ERROR;
            }
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, candidate.zero_offset[i]);
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
                if (value > UINT32_MAX / 10U) {
                    if (detail != NULL) {
                        *detail = 1U;
                    }
                    return CAN_STATUS_BAD_VALUE;
                }
                requested_sps_x10 = value * 10U;
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
                *applied_value =
                    param == CAN_SAMPLE_RATE_PARAM_DECI_SPS ?
                    adc_ads1256_get_sample_rate_x10() :
                    adc_ads1256_get_sample_rate();
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

        case CAN_CMD_CLEAR_ZERO: {
            config_transaction_snapshot_t snapshot;
            persistent_config_t candidate;
            capture_config_transaction(&snapshot);
            candidate = snapshot.config;
            memset(candidate.zero_offset, 0, sizeof(candidate.zero_offset));
            candidate.flags &= (uint8_t)~PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            if (commit_config_candidate(&candidate) != 0) {
                persistent_config = snapshot.config;
                config_dirty = snapshot.dirty;
                config_save_deadline = snapshot.save_deadline;
                return CAN_STATUS_STORAGE_ERROR;
            }
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            filter_reset_all();
            reset_can_telemetry_queue();
            return CAN_STATUS_OK;
        }

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

        case CAN_CMD_HOST_KEEPALIVE:
            if (param == CAN_HOST_SESSION_PARAM_STOP) {
                end_host_session();
                return CAN_STATUS_OK;
            }
            return param == CAN_HOST_SESSION_PARAM_REFRESH ?
                CAN_STATUS_OK : CAN_STATUS_BAD_VALUE;

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

        case CAN_CMD_SET_PGA: {
            config_transaction_snapshot_t snapshot;
            persistent_config_t candidate;
            uint8_t failed_step = CAN_RESTORE_STEP_PGA;
            if (value > UINT8_MAX) {
                return CAN_STATUS_BAD_VALUE;
            }
            capture_config_transaction(&snapshot);
            int apply_result = adc_ads1256_set_pga_gain((uint8_t)value);
            if (apply_result == 0) {
                failed_step = CAN_RESTORE_STEP_CALIBRATION;
                apply_result = adc_ads1256_calibrate();
            }
            if (apply_result == 0) {
                failed_step = CAN_RESTORE_STEP_RESTART;
                apply_result = adc_ads1256_restart();
            }
            if (apply_result != 0) {
                if (rollback_config_transaction(&snapshot) != 0) {
                    failed_step |= CAN_RESTORE_ROLLBACK_FAILED;
                }
                if (detail != NULL) {
                    *detail = failed_step;
                }
                return CAN_STATUS_BAD_VALUE;
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
            sync_persistent_config_from_runtime();
            candidate = persistent_config;
            candidate.flags &= (uint8_t)~PERSISTENT_CONFIG_FLAG_ZERO_VALID;
            if (commit_config_candidate(&candidate) != 0) {
                if (rollback_config_transaction(&snapshot) != 0 && detail != NULL) {
                    *detail = CAN_RESTORE_ROLLBACK_FAILED;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            return CAN_STATUS_OK;
        }

        case CAN_CMD_RESTORE_DEFAULTS: {
            config_transaction_snapshot_t snapshot;
            persistent_config_t default_config;
            uint8_t failed_step = 0U;

            capture_config_transaction(&snapshot);
            flash_storage_config_defaults(&default_config);
            if (apply_ads_config(&default_config, &failed_step) != 0) {
                if (rollback_config_transaction(&snapshot) != 0) {
                    failed_step |= CAN_RESTORE_ROLLBACK_FAILED;
                }
                if (detail != NULL) {
                    *detail = failed_step;
                }
                return CAN_STATUS_BAD_VALUE;
            }

            persistent_config = default_config;
            apply_non_ads_runtime_config(&persistent_config);
            if (commit_config_candidate(&default_config) != 0) {
                if (rollback_config_transaction(&snapshot) != 0 && detail != NULL) {
                    *detail = CAN_RESTORE_ROLLBACK_FAILED;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            return CAN_STATUS_OK;
        }

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
        if (!can_status_queue_has_space()) {
            break;
        }
        if (can_recv(&msg, 1) != 1) {
            break;
        }

        int dlc_bytes = can_data_len_get(msg.RxHeader.DataLength);
        if (msg.RxHeader.Identifier != CAN_ID_RX_COMMAND) {
            can_diag_record_reject(CAN_DIAG_REJECT_BAD_ID);
            continue;
        }
        if (msg.RxHeader.FDFormat != FDCAN_FD_CAN) {
            can_diag_record_reject(CAN_DIAG_REJECT_NOT_FD);
            continue;
        }
        if (msg.RxHeader.BitRateSwitch != FDCAN_BRS_ON) {
            can_diag_record_reject(CAN_DIAG_REJECT_NO_BRS);
            continue;
        }
        if (dlc_bytes != (int)sizeof(can_rx_command_frame_t)) {
            can_diag_record_reject(CAN_DIAG_REJECT_BAD_DLC);
            continue;
        }

        const can_rx_command_frame_t *frame = (const can_rx_command_frame_t *)msg.data;
        uint32_t value = can_frame_u32_le_get(frame->value_le);
        uint8_t detail = 0;

        if (frame->frame_type != CAN_FRAME_TYPE_COMMAND) {
            can_diag_record_reject(CAN_DIAG_REJECT_BAD_TYPE);
            send_can_status(frame->sequence, frame->cmd_type, CAN_STATUS_BAD_TYPE,
                            value, frame->frame_type);
        } else if (frame->version != CAN_PROTOCOL_VERSION) {
            can_diag_record_reject(CAN_DIAG_REJECT_BAD_VERSION);
            send_can_status(frame->sequence, frame->cmd_type, CAN_STATUS_BAD_TYPE,
                            value, frame->version);
        } else {
            can_diag_record_reject(CAN_DIAG_REJECT_NONE);
            activate_host_session();
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
#if CAN_TX_DRIVER_TEST_ENABLED
    send_can_tx_driver_test();
    return;
#endif

#if CAN_INTERVAL_TEST_ENABLED
    (void)can_recover_bus_off();
    send_can_interval_test();
    return;
#endif

    can_main_loop_seen = true;
    (void)can_recover_bus_off();
    service_host_session();
    service_can_status_queue();
    process_can_commands();
    service_can_status_queue();
    service_config_save();
    send_can_diag();
    send_config_snapshot();
    send_can_health();
    flush_can_telemetry();

    adc_ads1256_poll();
    service_mcu_led();

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
