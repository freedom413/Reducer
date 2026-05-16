#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
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
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

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

static void flash_unlock(void)
{
    HAL_FLASH_Unlock();
}

static void flash_lock(void)
{
    HAL_FLASH_Lock();
}

static int flash_erase_page(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1;

    return (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK) ? 0 : -1;
}

static int flash_program_dw(uint32_t addr, uint64_t data)
{
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data) == HAL_OK) ? 0 : -1;
}

static void flash_read(uint32_t addr, void *data, uint32_t len)
{
    memcpy(data, (const void *)addr, len);
}

static const flash_hw_ops_t flash_hw_ops = {
    .unlock = flash_unlock,
    .lock = flash_lock,
    .erase_page = flash_erase_page,
    .program_doubleword = flash_program_dw,
    .read = flash_read,
};

void flash_storage_register_user_ops(void)
{
    flash_storage_register_ops(&flash_hw_ops);
}

// ============================================================================
// Constants and Configuration
// ============================================================================
#define ADC_CHANNEL_COUNT   6
#define ADC_ALL_CH_MASK     0x3F
#define ADC_CHIP_COUNT      2
#define CAN_COMMANDS_PER_LOOP 4

// ============================================================================
// ADC Channel Mapping
// ============================================================================
#define ADC_CH_KEY(p, n)  ((((uint8_t)(p)) << 4) | ((uint8_t)(n) & 0x0FU))

static int8_t adc_ch_lookup[ADC_CHIP_COUNT][256];

static int8_t adc_pid_to_index(uint8_t pid)
{
    switch (pid) {
        case ADS1256_A:
            return 0;
        case ADS1256_B:
            return 1;
        default:
            return -1;
    }
}

static void adc_ch_lookup_init(void)
{
    for (uint8_t chip = 0; chip < ADC_CHIP_COUNT; chip++) {
        for (uint16_t i = 0; i < 256U; i++) {
            adc_ch_lookup[chip][i] = -1;
        }
    }

    // ADS1256_A channel 0: AIN0-AIN1 -> logical channel 0
    adc_ch_lookup[0][ADC_CH_KEY(ADS1256_AIN0, ADS1256_AIN1)] = 0;
    // ADS1256_A channel 1: AIN2-AIN3 -> logical channel 1
    adc_ch_lookup[0][ADC_CH_KEY(ADS1256_AIN2, ADS1256_AIN3)] = 1;
    // ADS1256_A channel 2: AIN4-AIN5 -> logical channel 2
    adc_ch_lookup[0][ADC_CH_KEY(ADS1256_AIN4, ADS1256_AIN5)] = 2;
    // ADS1256_B channel 0: AIN0-AIN1 -> logical channel 3
    adc_ch_lookup[1][ADC_CH_KEY(ADS1256_AIN0, ADS1256_AIN1)] = 3;
    // ADS1256_B channel 1: AIN2-AIN3 -> logical channel 4
    adc_ch_lookup[1][ADC_CH_KEY(ADS1256_AIN2, ADS1256_AIN3)] = 4;
    // ADS1256_B channel 2: AIN4-AIN5 -> logical channel 5
    adc_ch_lookup[1][ADC_CH_KEY(ADS1256_AIN4, ADS1256_AIN5)] = 5;
}

static int8_t adc_logical_channel(uint8_t pid, ads1256_ain_t p, ads1256_ain_t n)
{
    int8_t chip = adc_pid_to_index(pid);
    if (chip < 0) {
        return -1;
    }
    return adc_ch_lookup[(uint8_t)chip][ADC_CH_KEY(p, n)];
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
static float running_m2[ADC_CHANNEL_COUNT] = {0};
static uint16_t sample_count[ADC_CHANNEL_COUNT] = {0};

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
    if (sample_count[ch] < OUTLIER_MIN_SAMPLES) {
        return false;  // Not enough samples for outlier detection
    }

    float mean = running_mean[ch];
    float variance = running_m2[ch] / (float)(sample_count[ch] - 1U);

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
    can_tx_status_frame_t frame;
    can_build_status_frame(&frame, sequence, cmd_type, status, value, detail);
    can_classic_data_frame_send(CAN_ID_TX_STATUS, (uint8_t *)&frame, sizeof(frame));
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
        case CAN_CMD_ZERO_DATUM:
            // Save current filtered values as zero offset, then reset
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                int32_t raw_filtered = filter_get_raw_filtered(i);
                filter_set_zero_offset(i, raw_filtered);
            }
            if (filter_save_zero_to_flash() != 0) {
                if (detail != NULL) {
                    *detail = 1;
                }
                return CAN_STATUS_STORAGE_ERROR;
            }
            filter_reset_all();
            reset_statistics();
            return CAN_STATUS_OK;

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
            for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
                filter_set_zero_offset(i, 0);
            }
            if (flash_storage_clear() != 0) {
                if (detail != NULL) {
                    *detail = 3;
                }
                return CAN_STATUS_STORAGE_ERROR;
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

// ============================================================================
// Main Loop
// ============================================================================
void loop(void)
{
    can_msg_t msg;
    for (uint8_t cmd_count = 0; cmd_count < CAN_COMMANDS_PER_LOOP; cmd_count++) {
        if (can_recv(&msg, 1) != (int)sizeof(can_msg_t)) {
            break;
        }

        int dlc_bytes = can_data_len_get(msg.RxHeader.DataLength);
        if (msg.RxHeader.Identifier == CAN_ID_RX_COMMAND &&
            dlc_bytes >= (int)sizeof(can_rx_command_frame_t)) {
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

    adc_ads1256_poll();

    int recv_count = adc_ads1256_get_data(adc_ads1256_data, ADC_CHANNEL_COUNT);

    for (int i = 0; i < recv_count; i++) {
        uint8_t pid = adc_ads1256_data[i].pid;
        ads1256_ch_t ch;
        ads1256_data_get_ch(&adc_ads1256_data[i], &ch);

        // O(1) lookup instead of O(n) search
        int8_t logical_ch = adc_logical_channel(pid, ch.p, ch.n);
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
            can_tx_telemetry_frame_t frame;

            // voltage in 0.1 mV units (e.g., 123.4 mV -> 1234)
            int16_t voltage_01mv = clamp_i16_from_float(result.voltage * 10.0f);
            // strain in micro-strain units
            int16_t strain_ue = clamp_i16_from_float(result.strain);
            // stress preview in 0.1 MPa units, clipped to fit the compact 1-byte field
            int8_t stress_01mpa = clamp_i8_from_float(result.stress * 10.0f);

            can_build_telemetry_frame(&frame, i, voltage_01mv, strain_ue, stress_01mpa);
            can_classic_data_frame_send(CAN_ID_TX_TELEMETRY, (uint8_t *)&frame, sizeof(frame));
        }
    }
}
