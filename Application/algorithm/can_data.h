#ifndef __CAN_DATA_H__
#define __CAN_DATA_H__

#include <stdint.h>

// CAN ID definitions
#define CAN_ID_TX_DATA     0x101  // Downlink: STM32 -> PC
#define CAN_ID_RX_CONFIG   0x100  // Uplink: PC -> STM32

// Frame type definitions
#define CAN_FRAME_VOLTAGE  0x01   // Raw voltage data
#define CAN_FRAME_STRAIN   0x02   // Strain data
#define CAN_FRAME_STRESS   0x03   // Stress data
#define CAN_FRAME_DISP     0x04   // Displacement data
#define CAN_FRAME_ALL      0x05   // All data in one frame (future)

// CAN TX Frame format (8 bytes):
// Byte 0: frame_type
// Byte 1: channel (0-5)
// Byte 2-3: value (int16, big-endian / network order)
// Byte 4-5: value_frac (optional fractional part, 0 if not used)
// Byte 6-7: crc16 (CRC-16-CCITT, polynomial 0x1021)

typedef struct __attribute__((packed)) {
    uint8_t  frame_type;
    uint8_t  channel;
    int16_t  value;
    int16_t  value_frac;
    uint16_t crc16;
} can_tx_frame_t;

_Static_assert(sizeof(can_tx_frame_t) == 8, "can_tx_frame_t must be 8 bytes");

// CAN RX Frame (from PC - for configuration/commands)
typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;
    uint8_t  param;
    uint32_t value;
} can_rx_frame_t;

_Static_assert(sizeof(can_rx_frame_t) == 6, "can_rx_frame_t must be 6 bytes");

// Command types for RX
#define CAN_CMD_SET_SAMPLE_RATE  0x01
#define CAN_CMD_SET_FILTER_SIZE  0x02
#define CAN_CMD_ZERO_DATUM       0x03
#define CAN_CMD_START_CALIB      0x04

// Helper functions
uint16_t can_calc_crc16(const uint8_t *data, uint8_t len);
void can_build_tx_frame(can_tx_frame_t *frame, uint8_t frame_type,
                        uint8_t channel, int16_t value, int16_t value_frac);

#endif // __CAN_DATA_H__
