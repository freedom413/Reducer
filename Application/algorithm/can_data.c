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

// ============================================================================
// Combined frame builder
// ============================================================================

void can_build_combined_frame(can_tx_combined_frame_t *frame, uint8_t channel,
                              int16_t voltage_01mv, int16_t strain_ue, int8_t stress_01mpa)
{
    frame->frame_type = 0x05;  // Combined frame type
    frame->channel = channel;
    frame->voltage_be[0] = (uint8_t)(((uint16_t)voltage_01mv >> 8) & 0xFF);
    frame->voltage_be[1] = (uint8_t)((uint16_t)voltage_01mv & 0xFF);
    frame->strain_be[0] = (uint8_t)(((uint16_t)strain_ue >> 8) & 0xFF);
    frame->strain_be[1] = (uint8_t)((uint16_t)strain_ue & 0xFF);
    frame->stress = stress_01mpa;

    // Calculate XOR checksum over first 7 bytes
    frame->crc8 = can_calc_crc8((const uint8_t *)frame, 7);
}
