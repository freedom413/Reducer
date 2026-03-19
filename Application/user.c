#include <stdint.h>
#include <sys/_intsup.h>
#include <math.h>
#include "dbg.h"
#include "delay.h"
#include "can.h"
#include "fdcan.h"
#include "ads1256_raw_data_recv.h"
#include "user.h"
#include "can_data.h"
#include "filter.h"
#include "flexspline_math.h"

void ads1256_int_enable(void);

static uint8_t adc_all_ch_mask = 0x00;

static int32_t adc_raw_value[6] = {0};
static ads1256_data_t adc_ads1256_data[6] = {0};

// Physical parameters for flexspline calculation
static flexspline_params_t flexspline_params;

// Filtered raw values for each channel
static int32_t adc_filtered_value[6] = {0};

// Outlier rejection: track running statistics
#define OUTLIER_THRESHOLD  3   // Number of standard deviations
static float running_mean[6] = {0};
static float running_var[6] = {0};
static uint8_t sample_count = 0;

void setup(void)
{
    delay_init();
    can_init();
    adc_ads1256_start();

    // Initialize filter
    filter_init();

    // Initialize flexspline parameters with defaults
    flexspline_params_set_default(&flexspline_params);

    // ads1256_int_enable();
}

static inline bool is_outlier(uint8_t ch, int32_t value)
{
    if (sample_count < 10) {
        return false;  // Not enough samples for outlier detection
    }

    float mean = running_mean[ch];
    float std = sqrtf(running_var[ch]);
    float diff = (float)value - mean;

    return fabsf(diff) > (OUTLIER_THRESHOLD * std);
}

static void update_statistics(uint8_t ch, int32_t value)
{
    sample_count++;
    if (sample_count > 255) sample_count = 255;

    // Welford's online algorithm for running variance
    float delta = (float)value - running_mean[ch];
    running_mean[ch] += delta / (float)sample_count;
    float delta2 = (float)value - running_mean[ch];
    running_var[ch] += delta * delta2;
}

void loop(void)
{
    ads1256_drdy_callback();
    int recv_count = 0;
    ads1256_ch_t ch = {0};
    int i;
    int j;

    recv_count = adc_ads1256_get_data(adc_ads1256_data, 6);

    for (i = 0; i < recv_count; i++) {
        ads1256_data_get_ch(&adc_ads1256_data[i], &ch);

        if (adc_ads1256_data[i].pid == ADS1256_A) {
            for (j = 0; j < ARR_LEN(ads1235_a_ch); j++) {
                if (ads1235_a_ch[j].p == ch.p && ads1235_a_ch[j].n == ch.n) {
                    adc_raw_value[j] = adc_ads1256_data[i].raw_value;
                    adc_all_ch_mask |= (0x01 << j);
                    break;
                }
            }
        }
        else if (adc_ads1256_data[i].pid == ADS1256_B) {
            for (j = 0; j < ARR_LEN(ads1235_b_ch); j++) {
                if (ads1235_b_ch[j].p == ch.p && ads1235_b_ch[j].n == ch.n) {
                    adc_raw_value[j + ARR_LEN(ads1235_b_ch)] = adc_ads1256_data[i].raw_value;
                    adc_all_ch_mask |= (0x01 << (j + ARR_LEN(ads1235_b_ch)));
                    break;
                }
            }
        }
        else {
            // error - unknown PID
        }
    }

    /* All 6 channels conversion complete */
    if (adc_all_ch_mask == 0x3F) {
        adc_all_ch_mask = 0x00;

        /* Process each channel: filter -> outlier check -> calculate -> send */
        for (int i = 0; i < 6; i++) {
            /* Apply moving average filter */
            int32_t filtered = filter_apply((uint8_t)i, adc_raw_value[i]);

            /* Check for outliers (skip if first few samples) */
            if (!is_outlier((uint8_t)i, filtered)) {
                update_statistics((uint8_t)i, filtered);
                adc_filtered_value[i] = filtered;
            } else {
                /* Skip outlier - use previous filtered value */
                filtered = adc_filtered_value[i];
            }

            /* Calculate physical values */
            flexspline_result_t result;
            flexspline_calculate(filtered, &flexspline_params, &result);

            /* Send voltage frame (frame_type=0x01) */
            {
                can_tx_frame_t frame;
                int16_t voltage_mv = (int16_t)(result.voltage * 10); // x0.1 mV -> keep precision
                int16_t voltage_frac = (int16_t)((result.voltage * 10 - voltage_mv) * 100);
                can_build_tx_frame(&frame, CAN_FRAME_VOLTAGE, (uint8_t)i,
                                   voltage_mv, voltage_frac);
                can_classic_data_frame_send(CAN_ID_TX_DATA, (uint8_t *)&frame, sizeof(frame));
            }

            /* Send strain frame (frame_type=0x02) */
            {
                can_tx_frame_t frame;
                int16_t strain_val = (int16_t)result.strain;  // micro-strain
                can_build_tx_frame(&frame, CAN_FRAME_STRAIN, (uint8_t)i,
                                   strain_val, 0);
                can_classic_data_frame_send(CAN_ID_TX_DATA, (uint8_t *)&frame, sizeof(frame));
            }

            /* Send stress frame (frame_type=0x03) */
            {
                can_tx_frame_t frame;
                int16_t stress_val = (int16_t)(result.stress * 100); // x0.01 MPa
                int16_t stress_frac = (int16_t)((result.stress * 100 - stress_val) * 100);
                can_build_tx_frame(&frame, CAN_FRAME_STRESS, (uint8_t)i,
                                   stress_val, stress_frac);
                can_classic_data_frame_send(CAN_ID_TX_DATA, (uint8_t *)&frame, sizeof(frame));
            }
        }
    }
}
