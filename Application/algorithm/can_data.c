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
