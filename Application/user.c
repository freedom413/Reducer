#include <stdint.h>
#include <math.h>
#include <stdbool.h>
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
#define CAN_COMMANDS_PER_LOOP 4
#define CAN_INTERVAL_TEST_ENABLED    0U
#if CAN_INTERVAL_TEST_ENABLED
#define CAN_INTERVAL_TEST_ID         0x123U
#define CAN_INTERVAL_TEST_PERIOD_MS  100U
#endif
#define CAN_TX_WAIT_TIMEOUT_MS       5U

// ============================================================================
// Module State
// ============================================================================
static ads1256_data_t adc_ads1256_data[ADC_CHANNEL_COUNT] = {0};
static bool can_ready = false;
#if CAN_INTERVAL_TEST_ENABLED
static uint32_t can_test_last_tx_tick = 0;
static uint8_t can_test_sequence = 0;
#endif

// Physical parameters for flexspline calculation
static flexspline_params_t flexspline_params;

// Filtered raw values for each channel
static int32_t adc_filtered_value[ADC_CHANNEL_COUNT] = {0};

static float running_mean[ADC_CHANNEL_COUNT] = {0};
static float running_m2[ADC_CHANNEL_COUNT] = {0};
static uint16_t sample_count[ADC_CHANNEL_COUNT] = {0};

#if CAN_INTERVAL_TEST_ENABLED
static void send_can_interval_test(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - can_test_last_tx_tick) < CAN_INTERVAL_TEST_PERIOD_MS) {
        return;
    }
    can_test_last_tx_tick = now;

    const uint8_t frame[CAN_DATA_LEN] = {
        0xAAU, 0x55U, can_test_sequence, (uint8_t)~can_test_sequence,
        0x00U, 0xFFU, 0x5AU, 0xA5U,
    };

    if (can_classic_data_frame_send(CAN_INTERVAL_TEST_ID, frame, sizeof(frame)) ==
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

    adc_ads1256_start();

    // Register flash hardware operations (user provides implementation)
    flash_storage_register_user_ops();

    // Initialize filter (will load zero offset from flash if available)
    filter_init();

    // Initialize flexspline parameters with defaults
    flexspline_params_set_default(&flexspline_params);
}

static void update_statistics(uint8_t ch, int32_t value)
{
    if (sample_count[ch] == UINT16_MAX) {
        return;
    }
    sample_count[ch]++;

    // Welford's online algorithm for running variance
    float delta = (float)value - running_mean[ch];
    running_mean[ch] += delta / (float)sample_count[ch];
    float delta2 = (float)value - running_mean[ch];
    running_m2[ch] += delta * delta2;
}

static void reset_statistics(void)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
        running_mean[i] = 0;
        running_m2[i] = 0;
        adc_filtered_value[i] = 0;
        sample_count[i] = 0;
    }
}

static void send_can_status(uint8_t sequence, uint8_t cmd_type, uint8_t status,
                            uint16_t value, uint8_t detail)
{
    if (!can_ready) {
        return;
    }

    can_tx_status_frame_t frame;
    can_build_status_frame(&frame, sequence, cmd_type, status, value, detail);
    uint32_t start = HAL_GetTick();
    while (can_classic_data_frame_send(CAN_ID_TX_STATUS, (const uint8_t *)&frame,
                                       sizeof(frame)) == -3) {
        if ((uint32_t)(HAL_GetTick() - start) >= CAN_TX_WAIT_TIMEOUT_MS) {
            break;
        }
    }
}

static void send_can_telemetry(uint8_t channel, int16_t voltage_001mv,
                               int16_t strain_ue, int8_t stress_01mpa)
{
    if (!can_ready) {
        return;
    }

    can_tx_telemetry_frame_t frame;
    can_build_telemetry_frame(&frame, channel, voltage_001mv, strain_ue, stress_01mpa);
    /*
     * Telemetry is best-effort. Waiting for a congested CAN TX queue here
     * delays ADC polling and makes overload worse. Status ACKs still use a
     * short bounded wait because they are command responses.
     */
    (void)can_classic_data_frame_send(CAN_ID_TX_TELEMETRY,
                                      (const uint8_t *)&frame, sizeof(frame));
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

static int8_t clamp_i8_from_float(float value)
{
    long rounded = lrintf(value);
    if (rounded > INT8_MAX) {
        return INT8_MAX;
    }
    if (rounded < INT8_MIN) {
        return INT8_MIN;
    }
    return (int8_t)rounded;
}

static void process_adc_sample(uint8_t channel, int32_t raw_value)
{
    int32_t filtered = filter_apply(channel, raw_value);

    /*
     * Keep the moving average, but do not reject large changes here.
     * A real load step is indistinguishable from an outlier at this
     * layer and must still reach telemetry.
     */
    update_statistics(channel, filtered);
    adc_filtered_value[channel] = filtered;

    flexspline_result_t result;
    flexspline_calculate(filtered, &flexspline_params, &result);

    // voltage in 0.01 mV units (e.g., 12.34 mV -> 1234)
    int16_t voltage_001mv = clamp_i16_from_float(result.voltage * 100.0f);
    // strain in micro-strain units
    int16_t strain_ue = clamp_i16_from_float(result.strain);
    // stress preview in 0.1 MPa units, clipped to fit the compact 1-byte field
    int8_t stress_01mpa = clamp_i8_from_float(result.stress * 10.0f);
    send_can_telemetry(channel, voltage_001mv, strain_ue, stress_01mpa);
}

static uint8_t process_can_command(uint8_t cmd_type, uint8_t param, uint16_t value,
                                   uint16_t *applied_value, uint8_t *detail)
{
    (void)param;
    if (applied_value != NULL) {
        *applied_value = value;
    }
    if (detail != NULL) {
        *detail = 0;
    }

    switch (cmd_type) {
        case CAN_CMD_ZERO_DATUM: {
            // Save current filtered values as zero offset, then reset
            int32_t previous_offsets[ADC_CHANNEL_COUNT];
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                if (!filter_has_samples(i)) {
                    if (detail != NULL) {
                        *detail = i;
                    }
                    return CAN_STATUS_BAD_VALUE;
                }
                filter_get_zero_offset(i, &previous_offsets[i]);
            }
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                int32_t raw_filtered = filter_get_raw_filtered(i);
                filter_set_zero_offset(i, raw_filtered);
            }
            if (filter_save_zero_to_flash() != 0) {
                for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                    filter_set_zero_offset(i, previous_offsets[i]);
                }
                if (detail != NULL) {
                    *detail = 1;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            filter_reset_all();
            reset_statistics();
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
            return CAN_STATUS_OK;

        case CAN_CMD_SET_SAMPLE_RATE:
            if (adc_ads1256_set_sample_rate(value) != 0) {
                if (detail != NULL) {
                    *detail = 1;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            if (applied_value != NULL) {
                *applied_value = adc_ads1256_get_sample_rate();
            }
            filter_reset_all();
            reset_statistics();
            return CAN_STATUS_OK;

        case CAN_CMD_START_CALIB:
            if (adc_ads1256_calibrate() != 0 || adc_ads1256_restart() != 0) {
                if (detail != NULL) {
                    *detail = 2;
                }
                return CAN_STATUS_BAD_VALUE;
            }
            filter_reset_all();
            reset_statistics();
            return CAN_STATUS_OK;

        case CAN_CMD_SAVE_ZERO:
            if (filter_save_zero_to_flash() != 0) {
                if (detail != NULL) {
                    *detail = 1;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            return CAN_STATUS_OK;

        case CAN_CMD_LOAD_ZERO:
            if (filter_load_zero_from_flash() != 0) {
                if (detail != NULL) {
                    *detail = 2;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            filter_reset_all();
            reset_statistics();
            return CAN_STATUS_OK;

        case CAN_CMD_CLEAR_ZERO:
            if (flash_storage_clear() != 0) {
                if (detail != NULL) {
                    *detail = 3;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            filter_reset_all();
            reset_statistics();
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
            dlc_bytes != (int)sizeof(can_rx_command_frame_t)) {
            continue;
        }

        const can_rx_command_frame_t *frame = (const can_rx_command_frame_t *)msg.data;
        uint8_t expected_crc = can_calc_crc8(msg.data, 7);
        uint16_t value = can_frame_u16_le_get(frame->value_le);
        uint8_t detail = 0;

        if (frame->frame_type != CAN_FRAME_TYPE_COMMAND) {
            send_can_status(frame->sequence, frame->cmd_type, CAN_STATUS_BAD_TYPE,
                            value, frame->frame_type);
        } else if (frame->crc8 != expected_crc) {
            send_can_status(frame->sequence, frame->cmd_type, CAN_STATUS_BAD_CRC,
                            value, expected_crc);
        } else {
            uint16_t applied_value = value;
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

    adc_ads1256_poll();

    int recv_count = adc_ads1256_get_data(adc_ads1256_data, ADC_CHANNEL_COUNT);

    for (int i = 0; i < recv_count; i++) {
        uint8_t ch = adc_ads1256_data[i].channel;
        if (ch < ADC_CHANNEL_COUNT) {
            /*
             * Process channels as conversions arrive. The two ADS1256 devices
             * normally produce at most two samples per poll, which avoids a
             * six-frame burst overflowing the three-slot FDCAN TX FIFO.
             */
            process_adc_sample(ch, adc_ads1256_data[i].raw_value);
        }
    }
}
