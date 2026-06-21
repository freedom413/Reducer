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

uint32_t can_frame_u32_le_get(const uint8_t value_le[4])
{
    return (uint32_t)value_le[0] |
           ((uint32_t)value_le[1] << 8) |
           ((uint32_t)value_le[2] << 16) |
           ((uint32_t)value_le[3] << 24);
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

void can_build_raw_telemetry_record(can_tx_raw_telemetry_record_t *record,
                                    uint8_t channel, int32_t raw_value)
{
    uint32_t raw24 = (uint32_t)raw_value & 0x00FFFFFFU;
    record->channel = channel;
    record->raw_be[0] = (uint8_t)((raw24 >> 16) & 0xFFU);
    record->raw_be[1] = (uint8_t)((raw24 >> 8) & 0xFFU);
    record->raw_be[2] = (uint8_t)(raw24 & 0xFFU);
}

void can_build_physical_telemetry_record(can_tx_physical_telemetry_record_t *record,
                                         uint8_t channel, int32_t voltage_uv,
                                         int16_t strain_ue, int16_t stress_qmpa)
{
    uint32_t voltage = (uint32_t)voltage_uv;
    record->channel = channel;
    record->voltage_uv_be[0] = (uint8_t)(voltage >> 24);
    record->voltage_uv_be[1] = (uint8_t)(voltage >> 16);
    record->voltage_uv_be[2] = (uint8_t)(voltage >> 8);
    record->voltage_uv_be[3] = (uint8_t)voltage;
    record->strain_ue_be[0] = (uint8_t)((uint16_t)strain_ue >> 8);
    record->strain_ue_be[1] = (uint8_t)strain_ue;
    record->stress_qmpa_be[0] = (uint8_t)((uint16_t)stress_qmpa >> 8);
    record->stress_qmpa_be[1] = (uint8_t)stress_qmpa;
}

void can_build_raw_telemetry_batch_frame(
    can_tx_raw_telemetry_batch_frame_t *frame,
    const can_tx_raw_telemetry_record_t *records,
    uint8_t record_count,
    uint8_t sequence,
    uint16_t drop_delta)
{
    if (record_count > CAN_TELEMETRY_RAW_MAX_RECORDS) {
        record_count = CAN_TELEMETRY_RAW_MAX_RECORDS;
    }

    frame->frame_type = CAN_FRAME_TYPE_TELEMETRY_RAW_BATCH;
    frame->version = CAN_PROTOCOL_VERSION;
    frame->telemetry_mode = CAN_TELEMETRY_MODE_RAW;
    frame->sequence = sequence;
    frame->record_count = record_count;
    frame->drop_delta_le[0] = (uint8_t)(drop_delta & 0xFFU);
    frame->drop_delta_le[1] = (uint8_t)((drop_delta >> 8) & 0xFFU);
    frame->reserved = 0U;

    for (uint8_t i = 0; i < CAN_TELEMETRY_RAW_MAX_RECORDS; i++) {
        if (i < record_count) {
            frame->records[i] = records[i];
        } else {
            can_tx_raw_telemetry_record_t empty = {0};
            frame->records[i] = empty;
        }
    }
}

void can_build_physical_telemetry_batch_frame(
    can_tx_physical_telemetry_batch_frame_t *frame,
    const can_tx_physical_telemetry_record_t *records,
    uint8_t record_count,
    uint8_t sequence,
    uint16_t drop_delta)
{
    if (record_count > CAN_TELEMETRY_PHYSICAL_MAX_RECORDS) {
        record_count = CAN_TELEMETRY_PHYSICAL_MAX_RECORDS;
    }

    frame->frame_type = CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH;
    frame->version = CAN_PROTOCOL_VERSION;
    frame->telemetry_mode = CAN_TELEMETRY_MODE_PHYSICAL;
    frame->sequence = sequence;
    frame->record_count = record_count;
    frame->drop_delta_le[0] = (uint8_t)(drop_delta & 0xFFU);
    frame->drop_delta_le[1] = (uint8_t)((drop_delta >> 8) & 0xFFU);
    frame->reserved = 0U;

    for (uint8_t i = 0; i < CAN_TELEMETRY_PHYSICAL_MAX_RECORDS; i++) {
        if (i < record_count) {
            frame->records[i] = records[i];
        } else {
            can_tx_physical_telemetry_record_t empty = {0};
            frame->records[i] = empty;
        }
    }
    frame->reserved_tail[0] = 0U;
    frame->reserved_tail[1] = 0U;
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
                            uint8_t cmd_type, uint8_t status, uint32_t value,
                            uint8_t detail)
{
    frame->frame_type = CAN_FRAME_TYPE_STATUS;
    frame->version = CAN_PROTOCOL_VERSION;
    frame->sequence = sequence;
    frame->cmd_type = cmd_type;
    frame->status = status;
    frame->detail = detail;
    frame->value_le[0] = (uint8_t)(value & 0xFFu);
    frame->value_le[1] = (uint8_t)((value >> 8) & 0xFFu);
    frame->value_le[2] = (uint8_t)((value >> 16) & 0xFFu);
    frame->value_le[3] = (uint8_t)((value >> 24) & 0xFFu);
    frame->reserved[0] = 0U;
    frame->reserved[1] = 0U;
}

void can_build_health_frame(can_tx_health_frame_t *frame, uint32_t sample_rate_x10,
                            uint16_t tx_drop_count,
                            uint16_t adc_overflow_count, uint16_t adc_recovery_count,
                            uint16_t telemetry_samples_per_second,
                            uint16_t telemetry_frames_per_second,
                            uint8_t active_adc_count, uint8_t telemetry_mode,
                            uint8_t flags)
{
    frame->frame_type = CAN_FRAME_TYPE_HEALTH;
    frame->version = CAN_PROTOCOL_VERSION;
    frame->sample_rate_x10_le[0] = (uint8_t)(sample_rate_x10 & 0xFFU);
    frame->sample_rate_x10_le[1] = (uint8_t)((sample_rate_x10 >> 8) & 0xFFU);
    frame->sample_rate_x10_le[2] = (uint8_t)((sample_rate_x10 >> 16) & 0xFFU);
    frame->sample_rate_x10_le[3] = (uint8_t)((sample_rate_x10 >> 24) & 0xFFU);
    frame->tx_drop_count_le[0] = (uint8_t)(tx_drop_count & 0xFFU);
    frame->tx_drop_count_le[1] = (uint8_t)((tx_drop_count >> 8) & 0xFFU);
    frame->adc_overflow_count_le[0] = (uint8_t)(adc_overflow_count & 0xFFU);
    frame->adc_overflow_count_le[1] = (uint8_t)((adc_overflow_count >> 8) & 0xFFU);
    frame->adc_recovery_count_le[0] = (uint8_t)(adc_recovery_count & 0xFFU);
    frame->adc_recovery_count_le[1] = (uint8_t)((adc_recovery_count >> 8) & 0xFFU);
    frame->telemetry_samples_per_second_le[0] =
        (uint8_t)(telemetry_samples_per_second & 0xFFU);
    frame->telemetry_samples_per_second_le[1] =
        (uint8_t)((telemetry_samples_per_second >> 8) & 0xFFU);
    frame->telemetry_frames_per_second_le[0] =
        (uint8_t)(telemetry_frames_per_second & 0xFFU);
    frame->telemetry_frames_per_second_le[1] =
        (uint8_t)((telemetry_frames_per_second >> 8) & 0xFFU);
    frame->active_adc_count = active_adc_count;
    frame->telemetry_mode = telemetry_mode;
    frame->flags = flags;
    for (uint8_t i = 0; i < sizeof(frame->reserved); i++) {
        frame->reserved[i] = 0U;
    }
}

void can_build_diag_frame(can_tx_diag_frame_t *frame, uint8_t flags,
                          uint8_t last_rx_dlc, uint8_t last_reject_reason,
                          uint8_t tx_error_count, uint8_t rx_error_count,
                          uint8_t sequence)
{
    frame->frame_type = CAN_FRAME_TYPE_DIAG;
    frame->flags = flags;
    frame->last_rx_dlc = last_rx_dlc;
    frame->last_reject_reason = last_reject_reason;
    frame->tx_error_count = tx_error_count;
    frame->rx_error_count = rx_error_count;
    frame->sequence = sequence;
    frame->crc8 = can_calc_crc8((const uint8_t *)frame,
                                sizeof(*frame) - 1U);
}
