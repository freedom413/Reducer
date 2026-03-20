#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "dbg.h"
#include "delay.h"
#include "can.h"
#include "fdcan.h"
#include "ads1256.h"
#include "ads1256_raw_data_recv.h"
#include "user.h"
#include "can_data.h"
#include "filter.h"
#include "flash_storage.h"
#include "flexspline_math.h"

void ads1256_int_enable(void);

// ============================================================================
// Flash Hardware Operations (must be provided by user)
// ============================================================================
// Example implementation for STM32G4 HAL:
// static void flash_unlock(void) { HAL_FLASH_Unlock(); }
// static void flash_lock(void) { HAL_FLASH_Lock(); }
// static int flash_erase_page(uint32_t addr) { ... }
// static int flash_program_dw(uint32_t addr, uint64_t data) { ... }
// static void flash_read(uint32_t addr, void *data, uint32_t len) { memcpy(data, (void*)addr, len); }
//
// static const flash_hw_ops_t my_flash_ops = {
//     .unlock = flash_unlock,
//     .lock = flash_lock,
//     .erase_page = flash_erase_page,
//     .program_doubleword = flash_program_dw,
//     .read = flash_read,
// };

// User must provide this function to register flash operations
// __attribute__((weak)) void flash_storage_register_user_ops(void)
// {
//     flash_storage_register_ops(&my_flash_ops);
// }
void flash_storage_register_user_ops(void) __attribute__((weak));
void flash_storage_register_user_ops(void) {}

void ads1256_int_enable(void);

// ============================================================================
// Constants and Configuration
// ============================================================================
#define ADC_CHANNEL_COUNT   6
#define ADC_ALL_CH_MASK     0x3F

// ============================================================================
// ADC Channel Mapping Optimization
// ============================================================================
// Lookup table for O(1) channel mapping
// Index: encoded channel (pid<<4 | p<<1 | n>>3) -> logical channel 0-5
// A chip channels: 0-2, B chip channels: 3-5

// Helper macro to encode channel key
#define ADC_CH_KEY(pid, p, n)  (((pid) << 4) | (((p) << 1) | ((n) >> 3)))

// Initialize lookup table at runtime for better compatibility
static int8_t adc_ch_lookup[256];

static void adc_ch_lookup_init(void)
{
    // Initialize all entries to -1 (invalid)
    for (int i = 0; i < 256; i++) {
        adc_ch_lookup[i] = -1;
    }
    // ADS1256_A channel 0: AIN0-AIN1 -> logical channel 0
    adc_ch_lookup[ADC_CH_KEY(ADS1256_A, ADS1256_AIN0, ADS1256_AIN1)] = 0;
    // ADS1256_A channel 1: AIN2-AIN3 -> logical channel 1
    adc_ch_lookup[ADC_CH_KEY(ADS1256_A, ADS1256_AIN2, ADS1256_AIN3)] = 1;
    // ADS1256_A channel 2: AIN4-AIN5 -> logical channel 2
    adc_ch_lookup[ADC_CH_KEY(ADS1256_A, ADS1256_AIN4, ADS1256_AIN5)] = 2;
    // ADS1256_B channel 0: AIN0-AIN1 -> logical channel 3
    adc_ch_lookup[ADC_CH_KEY(ADS1256_B, ADS1256_AIN0, ADS1256_AIN1)] = 3;
    // ADS1256_B channel 1: AIN2-AIN3 -> logical channel 4
    adc_ch_lookup[ADC_CH_KEY(ADS1256_B, ADS1256_AIN2, ADS1256_AIN3)] = 4;
    // ADS1256_B channel 2: AIN4-AIN5 -> logical channel 5
    adc_ch_lookup[ADC_CH_KEY(ADS1256_B, ADS1256_AIN4, ADS1256_AIN5)] = 5;
}

static inline int8_t adc_encode_ch(uint8_t pid, ads1256_ain_t p, ads1256_ain_t n)
{
    return (int8_t)((pid << 4) | (((p) << 1) | ((n) >> 3)));
}

// ============================================================================
// Module State
// ============================================================================
static uint8_t adc_all_ch_mask = 0x00;
static int32_t adc_raw_value[ADC_CHANNEL_COUNT] = {0};
static ads1256_data_t adc_ads1256_data[ADC_CHANNEL_COUNT] = {0};

// Physical parameters for flexspline calculation
static flexspline_params_t flexspline_params;

// Filtered raw values for each channel
static int32_t adc_filtered_value[ADC_CHANNEL_COUNT] = {0};

// ============================================================================
// Outlier Detection Optimization
// ============================================================================
// Use variance directly and compare squared diff to threshold * variance
// Avoids sqrtf by comparing (diff^2) > (threshold^2 * var)
// Using threshold=3, threshold^2 = 9
#define OUTLIER_THRESHOLD_SQ  9.0f   // OUTLIER_THRESHOLD^2
#define OUTLIER_MIN_SAMPLES   10     // Minimum samples before outlier detection

static float running_mean[ADC_CHANNEL_COUNT] = {0};
static float running_var[ADC_CHANNEL_COUNT] = {0};  // Running variance (not std dev)
static uint8_t sample_count = 0;

void setup(void)
{
    // Initialize ADC channel lookup table first
    adc_ch_lookup_init();

    delay_init();
    can_init();
    adc_ads1256_start();

    // Register flash hardware operations (user provides implementation)
    flash_storage_register_user_ops();

    // Initialize filter (will load zero offset from flash if available)
    filter_init();

    // Initialize flexspline parameters with defaults
    flexspline_params_set_default(&flexspline_params);
}

static inline bool is_outlier(uint8_t ch, int32_t value)
{
    if (sample_count < OUTLIER_MIN_SAMPLES) {
        return false;  // Not enough samples for outlier detection
    }

    float mean = running_mean[ch];
    float variance = running_var[ch];

    // Avoid sqrt by comparing squared values
    // outlier if diff^2 > threshold^2 * var
    // Using std dev would be: |diff| > threshold * sqrt(var)
    // Squared: diff^2 > threshold^2 * var
    float diff = (float)value - mean;

    // If variance is very small, treat any significant deviation as outlier
    if (variance < 1e-9f) {
        return fabsf(diff) > 100;  // Absolute threshold if variance negligible
    }

    return (diff * diff) > (OUTLIER_THRESHOLD_SQ * variance);
}

static void update_statistics(uint8_t ch, int32_t value)
{
    if (sample_count >= 255) {
        sample_count = 254;  // Prevent overflow, keep running
    }
    sample_count++;

    // Welford's online algorithm for running variance
    float delta = (float)value - running_mean[ch];
    running_mean[ch] += delta / (float)sample_count;
    float delta2 = (float)value - running_mean[ch];
    running_var[ch] += delta * delta2;
}

static void reset_statistics(void)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
        running_mean[i] = 0;
        running_var[i] = 0;
        adc_filtered_value[i] = 0;
    }
    sample_count = 0;
}

static void process_can_command(uint8_t cmd_type, uint8_t param, uint32_t value)
{
    (void)param;

    switch (cmd_type) {
        case CAN_CMD_ZERO_DATUM:
            // Save current filtered values as zero offset, then reset
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                int32_t raw_filtered = filter_get_raw_filtered(i);
                filter_set_zero_offset(i, raw_filtered);
            }
            filter_save_zero_to_flash();
            filter_reset_all();
            reset_statistics();
            break;

        case CAN_CMD_SET_FILTER_SIZE:
            // param: channel (unused, applies to all)
            // value: new filter window size
            filter_set_window_size((uint8_t)value);
            break;

        case CAN_CMD_SET_SAMPLE_RATE:
            // param: ADC chip (0=ADS1256_A, 1=ADS1256_B, 2=both)
            // value: SPS code (see ads1256_sps_t)
            // Note: This requires reconfiguration and is hardware dependent
            // For now, we acknowledge but don't actually change rate
            // Future: implement if needed
            break;

        case CAN_CMD_START_CALIB:
            // Trigger ADS1256 self-calibration
            adc_ads1256_calibrate();
            // Also reset filters and statistics after calibration
            filter_reset_all();
            reset_statistics();
            break;

        case CAN_CMD_SAVE_ZERO:
            // Save current zero offsets to Flash
            filter_save_zero_to_flash();
            break;

        case CAN_CMD_LOAD_ZERO:
            // Reload zero offsets from Flash
            filter_load_zero_from_flash();
            break;

        case CAN_CMD_CLEAR_ZERO:
            // Clear zero offsets from Flash and reset to 0
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            flash_storage_clear();
            break;

        default:
            break;
    }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop(void)
{
    /* Process incoming CAN commands FIRST */
    can_msg_t msg;
    // Use timeout=0 for non-blocking check
    if (can_recv(&msg, 0) > 0) {
        if (msg.RxHeader.Identifier == CAN_ID_RX_CONFIG &&
            msg.RxHeader.DataLength >= 6) {
            /* Parse command frame (matches can_rx_frame_t) */
            uint8_t cmd_type = msg.data[0];
            uint8_t param = msg.data[1];
            uint32_t value = ((uint32_t)msg.data[2]) |
                             ((uint32_t)msg.data[3] << 8) |
                             ((uint32_t)msg.data[4] << 16) |
                             ((uint32_t)msg.data[5] << 24);
            process_can_command(cmd_type, param, value);
        }
    }

    ads1256_drdy_callback();

    int recv_count = adc_ads1256_get_data(adc_ads1256_data, ADC_CHANNEL_COUNT);

    for (int i = 0; i < recv_count; i++) {
        uint8_t pid = adc_ads1256_data[i].pid;
        ads1256_ch_t ch;
        ads1256_data_get_ch(&adc_ads1256_data[i], &ch);

        // O(1) lookup instead of O(n) search
        int8_t logical_ch = adc_ch_lookup[adc_encode_ch(pid, ch.p, ch.n)];
        if (logical_ch >= 0 && logical_ch < ADC_CHANNEL_COUNT) {
            adc_raw_value[logical_ch] = adc_ads1256_data[i].raw_value;
            adc_all_ch_mask |= (1U << logical_ch);
        }
    }

    /* All 6 channels conversion complete */
    if (adc_all_ch_mask == ADC_ALL_CH_MASK) {
        adc_all_ch_mask = 0x00;

        /* Process each channel: filter -> outlier check -> calculate -> send */
        for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
            int32_t filtered = filter_apply(i, adc_raw_value[i]);

            /* Check for outliers (skip if first few samples) */
            if (!is_outlier(i, filtered)) {
                update_statistics(i, filtered);
                adc_filtered_value[i] = filtered;
            } else {
                /* Skip outlier - use previous filtered value */
                filtered = adc_filtered_value[i];
            }

            /* Calculate physical values */
            flexspline_result_t result;
            flexspline_calculate(filtered, &flexspline_params, &result);

            /* Use combined CAN frame to reduce bus load (1 frame per channel instead of 3) */
            can_tx_combined_frame_t frame;

            // voltage in 0.1 mV units (e.g., 123.4 mV -> 1234)
            int16_t voltage_01mv = (int16_t)lrintf(result.voltage * 10);
            // strain in micro-strain units
            int16_t strain_ue = (int16_t)result.strain;
            // stress in 0.1 MPa units (e.g., 12.3 MPa -> 123)
            int8_t stress_01mpa = (int8_t)lrintf(result.stress * 10);

            can_build_combined_frame(&frame, i, voltage_01mv, strain_ue, stress_01mpa);
            can_classic_data_frame_send(CAN_ID_TX_DATA, (uint8_t *)&frame, sizeof(frame));
        }
    }
}
