#ifndef __CAN_DATA_H__
#define __CAN_DATA_H__

#include <stdint.h>

// ============================================================================
// CAN ID definitions
// ============================================================================
#define CAN_ID_TX_DATA     0x101  // Downlink: STM32 -> PC
#define CAN_ID_RX_CONFIG   0x100  // Uplink: PC -> STM32

// ============================================================================
// Combined CAN TX Frame - All data in one transmission (8 bytes)
// Reduces CAN bus load from 18 frames to 6 frames per cycle
// ============================================================================
// Layout (8 bytes):
// Byte 0: frame_type = 0x05
// Byte 1: channel (0-5)
// Bytes 2-3: voltage (int16, in 0.1 mV units, big-endian)
// Bytes 4-5: strain (int16, in micro-strain units, big-endian)
// Byte 6: stress (int8, in 0.1 MPa units, signed)
// Byte 7: crc8 (XOR checksum of bytes 0-6)

typedef struct __attribute__((packed)) {
    uint8_t  frame_type;    // 0x05
    uint8_t  channel;       // 0-5
    uint8_t  voltage_be[2]; // Voltage in 0.1 mV, big-endian
    uint8_t  strain_be[2];  // Strain in micro-strain, big-endian
    int8_t   stress;        // Stress in 0.1 MPa (signed)
    uint8_t  crc8;          // XOR checksum of bytes 0-6
} can_tx_combined_frame_t;

_Static_assert(sizeof(can_tx_combined_frame_t) == 8, "can_tx_combined_frame_t must be 8 bytes");

// ============================================================================
// CAN RX Frame (from PC - for configuration/commands)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;
    uint8_t  param;
    uint32_t value;
} can_rx_frame_t;

_Static_assert(sizeof(can_rx_frame_t) == 6, "can_rx_frame_t must be 6 bytes");

// ============================================================================
// Command types for RX
// ============================================================================
#define CAN_CMD_SET_SAMPLE_RATE  0x01
#define CAN_CMD_SET_FILTER_SIZE  0x02
#define CAN_CMD_ZERO_DATUM       0x03
#define CAN_CMD_START_CALIB      0x04
#define CAN_CMD_SAVE_ZERO        0x05  // Save zero offset to Flash
#define CAN_CMD_LOAD_ZERO        0x06  // Load zero offset from Flash
#define CAN_CMD_CLEAR_ZERO       0x07  // Clear zero offset from Flash

// ============================================================================
// Helper functions
// ============================================================================

/**
 * @brief Calculate CRC-8 XOR checksum
 * @param data Pointer to data bytes
 * @param len Number of bytes
 * @return CRC-8 value
 */
uint8_t can_calc_crc8(const uint8_t *data, uint8_t len);

/**
 * @brief Build combined CAN frame with all sensor data
 * @param frame Pointer to frame structure
 * @param channel Channel number (0-5)
 * @param voltage_01mv Voltage in 0.1 mV units (e.g., 1234 = 123.4 mV)
 * @param strain_ue Strain in micro-strain units
 * @param stress_01mpa Stress in 0.1 MPa units (signed)
 */
void can_build_combined_frame(can_tx_combined_frame_t *frame, uint8_t channel,
                              int16_t voltage_01mv, int16_t strain_ue, int8_t stress_01mpa);

#endif // __CAN_DATA_H__
