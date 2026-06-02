#include "can_data.h"

// ============================================================================
// CRC-8 XOR checksum (for combined frames)
// ============================================================================

uint8_t can_calc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
    }
    return crc;
}

uint16_t can_frame_u16_le_get(const uint8_t value_le[2])
{
    return (uint16_t)value_le[0] | ((uint16_t)value_le[1] << 8);
}

// ============================================================================
// Frame builders
// ============================================================================

void can_build_telemetry_frame(can_tx_telemetry_frame_t *frame, uint8_t channel,
                               int16_t voltage_001mv, int16_t strain_ue, int8_t stress_01mpa)
{
    frame->frame_type = CAN_FRAME_TYPE_TELEMETRY;
    frame->channel = channel;
    frame->voltage_be[0] = (uint8_t)(((uint16_t)voltage_001mv >> 8) & 0xFF);
    frame->voltage_be[1] = (uint8_t)((uint16_t)voltage_001mv & 0xFF);
    frame->strain_be[0] = (uint8_t)(((uint16_t)strain_ue >> 8) & 0xFF);
    frame->strain_be[1] = (uint8_t)((uint16_t)strain_ue & 0xFF);
    frame->stress = stress_01mpa;
    frame->crc8 = can_calc_crc8((const uint8_t *)frame, 7);
}

void can_build_telemetry_record(can_tx_telemetry_record_t *record, uint8_t channel,
                                int16_t voltage_001mv, int16_t strain_ue,
                                int8_t stress_01mpa)
{
    record->channel = channel;
    record->voltage_be[0] = (uint8_t)(((uint16_t)voltage_001mv >> 8) & 0xFF);
    record->voltage_be[1] = (uint8_t)((uint16_t)voltage_001mv & 0xFF);
    record->strain_be[0] = (uint8_t)(((uint16_t)strain_ue >> 8) & 0xFF);
    record->strain_be[1] = (uint8_t)((uint16_t)strain_ue & 0xFF);
    record->stress = stress_01mpa;
}

void can_build_telemetry_batch_frame(
    can_tx_telemetry_batch_frame_t *frame,
    const can_tx_telemetry_record_t *records,
    uint8_t record_count)
{
    if (record_count > CAN_TELEMETRY_BATCH_MAX_RECORDS) {
        record_count = CAN_TELEMETRY_BATCH_MAX_RECORDS;
    }

    frame->frame_type = CAN_FRAME_TYPE_TELEMETRY_BATCH;
    frame->record_count = record_count;
    for (uint8_t i = 0; i < CAN_TELEMETRY_BATCH_MAX_RECORDS; i++) {
        if (i < record_count) {
            frame->records[i] = records[i];
        } else {
            can_tx_telemetry_record_t empty = {0};
            frame->records[i] = empty;
        }
    }
    frame->reserved = 0U;
    frame->crc8 = can_calc_crc8((const uint8_t *)frame,
                                CAN_TELEMETRY_BATCH_FRAME_LEN - 1U);
}

void can_build_status_frame(can_tx_status_frame_t *frame, uint8_t sequence,
                            uint8_t cmd_type, uint8_t status, uint16_t value,
                            uint8_t detail)
{
    frame->frame_type = CAN_FRAME_TYPE_STATUS;
    frame->sequence = sequence;
    frame->cmd_type = cmd_type;
    frame->status = status;
    frame->value_le[0] = (uint8_t)(value & 0xFFu);
    frame->value_le[1] = (uint8_t)((value >> 8) & 0xFFu);
    frame->detail = detail;
    frame->crc8 = can_calc_crc8((const uint8_t *)frame, 7);
}

void can_build_health_frame(can_tx_health_frame_t *frame, uint32_t sample_rate_x10,
                            uint16_t telemetry_decimation, uint16_t tx_drop_count,
                            uint16_t adc_overflow_count, uint16_t adc_recovery_count,
                            uint8_t active_adc_count, uint8_t flags)
{
    frame->frame_type = CAN_FRAME_TYPE_HEALTH;
    frame->version = CAN_HEALTH_VERSION;
    frame->sample_rate_x10_le[0] = (uint8_t)(sample_rate_x10 & 0xFFU);
    frame->sample_rate_x10_le[1] = (uint8_t)((sample_rate_x10 >> 8) & 0xFFU);
    frame->sample_rate_x10_le[2] = (uint8_t)((sample_rate_x10 >> 16) & 0xFFU);
    frame->sample_rate_x10_le[3] = (uint8_t)((sample_rate_x10 >> 24) & 0xFFU);
    frame->telemetry_decimation_le[0] = (uint8_t)(telemetry_decimation & 0xFFU);
    frame->telemetry_decimation_le[1] = (uint8_t)((telemetry_decimation >> 8) & 0xFFU);
    frame->tx_drop_count_le[0] = (uint8_t)(tx_drop_count & 0xFFU);
    frame->tx_drop_count_le[1] = (uint8_t)((tx_drop_count >> 8) & 0xFFU);
    frame->adc_overflow_count_le[0] = (uint8_t)(adc_overflow_count & 0xFFU);
    frame->adc_overflow_count_le[1] = (uint8_t)((adc_overflow_count >> 8) & 0xFFU);
    frame->adc_recovery_count_le[0] = (uint8_t)(adc_recovery_count & 0xFFU);
    frame->adc_recovery_count_le[1] = (uint8_t)((adc_recovery_count >> 8) & 0xFFU);
    frame->flags = (uint8_t)(flags | ((active_adc_count & 0x0FU) << 4));
    frame->crc8 = can_calc_crc8((const uint8_t *)frame, 15);
}
