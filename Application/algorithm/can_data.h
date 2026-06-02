#ifndef __CAN_DATA_H__
#define __CAN_DATA_H__

#include <stdint.h>

// ============================================================================
// CAN ID definitions
// ============================================================================
#define CAN_ID_RX_COMMAND    0x100  // Uplink: PC -> STM32
#define CAN_ID_TX_TELEMETRY  0x101  // Downlink: STM32 -> PC telemetry
#define CAN_ID_TX_STATUS     0x102  // Downlink: STM32 -> PC command status
#define CAN_ID_TX_HEALTH     0x103  // Downlink: STM32 -> PC health summary

// ============================================================================
// Frame type definitions
// ============================================================================
#define CAN_FRAME_TYPE_TELEMETRY  0x51
#define CAN_FRAME_TYPE_TELEMETRY_BATCH  0x53
#define CAN_FRAME_TYPE_COMMAND    0xA0
#define CAN_FRAME_TYPE_STATUS     0xA1
#define CAN_FRAME_TYPE_HEALTH     0x52
#define CAN_HEALTH_VERSION        0x01

// ============================================================================
// Command status definitions
// ============================================================================
#define CAN_STATUS_OK             0x00
#define CAN_STATUS_BAD_CRC        0xE1
#define CAN_STATUS_BAD_TYPE       0xE2
#define CAN_STATUS_BAD_CMD        0xE3
#define CAN_STATUS_BAD_VALUE      0xE4
#define CAN_STATUS_STORAGE_ERROR  0xE5

// ============================================================================
// Telemetry CAN TX Frame - 8-byte CAN FD frame with bit-rate switching
// ============================================================================
// Byte 0: frame_type = 0x51
// Byte 1: channel (0-7)
// Bytes 2-3: voltage (int16, in 0.01 mV units, big-endian)
// Bytes 4-5: strain (int16, in micro-strain units, big-endian)
// Byte 6: stress preview (int8, in 0.1 MPa units, signed, clipped on overflow)
// Byte 7: crc8 (XOR checksum of bytes 0-6)
typedef struct __attribute__((packed)) {
    uint8_t  frame_type;
    uint8_t  channel;
    uint8_t  voltage_be[2];
    uint8_t  strain_be[2];
    int8_t   stress;
    uint8_t  crc8;
} can_tx_telemetry_frame_t;

_Static_assert(sizeof(can_tx_telemetry_frame_t) == 8, "can_tx_telemetry_frame_t must be 8 bytes");

// ============================================================================
// Batched telemetry CAN TX frame - 64-byte CAN FD+BRS frame
// ============================================================================
#define CAN_TELEMETRY_BATCH_MAX_RECORDS  10U
#define CAN_TELEMETRY_BATCH_FRAME_LEN    64U

// Each record uses 6 bytes. The final frame carries a record count, one
// reserved byte, and a checksum over bytes 0-62.
typedef struct __attribute__((packed)) {
    uint8_t  channel;
    uint8_t  voltage_be[2];
    uint8_t  strain_be[2];
    int8_t   stress;
} can_tx_telemetry_record_t;

_Static_assert(sizeof(can_tx_telemetry_record_t) == 6, "can_tx_telemetry_record_t must be 6 bytes");

typedef struct __attribute__((packed)) {
    uint8_t frame_type;
    uint8_t record_count;
    can_tx_telemetry_record_t records[CAN_TELEMETRY_BATCH_MAX_RECORDS];
    uint8_t reserved;
    uint8_t crc8;
} can_tx_telemetry_batch_frame_t;

_Static_assert(sizeof(can_tx_telemetry_batch_frame_t) == CAN_TELEMETRY_BATCH_FRAME_LEN,
               "can_tx_telemetry_batch_frame_t must be 64 bytes");

// ============================================================================
// Command frame (PC -> STM32) with CRC and sequence number
// ============================================================================
// Byte 0: frame_type = 0xA0
// Byte 1: sequence
// Byte 2: cmd_type
// Byte 3: param
// Bytes 4-5: value (uint16, little-endian)
// Byte 6: reserved
// Byte 7: crc8 (XOR checksum of bytes 0-6)
typedef struct __attribute__((packed)) {
    uint8_t  frame_type;
    uint8_t  sequence;
    uint8_t  cmd_type;
    uint8_t  param;
    uint8_t  value_le[2];
    uint8_t  reserved;
    uint8_t  crc8;
} can_rx_command_frame_t;

_Static_assert(sizeof(can_rx_command_frame_t) == 8, "can_rx_command_frame_t must be 8 bytes");

// ============================================================================
// Status frame (STM32 -> PC) for command acknowledgment
// ============================================================================
// Byte 0: frame_type = 0xA1
// Byte 1: sequence
// Byte 2: cmd_type
// Byte 3: status
// Bytes 4-5: value (uint16, little-endian)
// Byte 6: detail
// Byte 7: crc8 (XOR checksum of bytes 0-6)
typedef struct __attribute__((packed)) {
    uint8_t  frame_type;
    uint8_t  sequence;
    uint8_t  cmd_type;
    uint8_t  status;
    uint8_t  value_le[2];
    uint8_t  detail;
    uint8_t  crc8;
} can_tx_status_frame_t;

_Static_assert(sizeof(can_tx_status_frame_t) == 8, "can_tx_status_frame_t must be 8 bytes");

// ============================================================================
// Health frame (STM32 -> PC) - 16-byte CAN FD+BRS frame
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  frame_type;
    uint8_t  version;
    uint8_t  sample_rate_x10_le[4];
    uint8_t  telemetry_decimation_le[2];
    uint8_t  tx_drop_count_le[2];
    uint8_t  adc_overflow_count_le[2];
    uint8_t  adc_recovery_count_le[2];
    uint8_t  flags;
    uint8_t  crc8;
} can_tx_health_frame_t;

_Static_assert(sizeof(can_tx_health_frame_t) == 16, "can_tx_health_frame_t must be 16 bytes");

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
#define CAN_CMD_SET_CHANNEL_MASK 0x08  // Select ADC channels to scan

#define CAN_SAMPLE_RATE_PARAM_SPS       0x00
#define CAN_SAMPLE_RATE_PARAM_DECI_SPS  0x01

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
 * @brief Build telemetry CAN frame with all sensor data
 * @param frame Pointer to frame structure
 * @param channel Channel number (0-7)
 * @param voltage_001mv Voltage in 0.01 mV units (e.g., 1234 = 12.34 mV)
 * @param strain_ue Strain in micro-strain units
 * @param stress_01mpa Stress preview in 0.1 MPa units (signed, clipped)
 */
void can_build_telemetry_frame(can_tx_telemetry_frame_t *frame, uint8_t channel,
                               int16_t voltage_001mv, int16_t strain_ue, int8_t stress_01mpa);
void can_build_telemetry_record(can_tx_telemetry_record_t *record, uint8_t channel,
                                int16_t voltage_001mv, int16_t strain_ue,
                                int8_t stress_01mpa);
void can_build_telemetry_batch_frame(
    can_tx_telemetry_batch_frame_t *frame,
    const can_tx_telemetry_record_t *records,
    uint8_t record_count);
void can_build_status_frame(can_tx_status_frame_t *frame, uint8_t sequence,
                            uint8_t cmd_type, uint8_t status, uint16_t value,
                            uint8_t detail);
void can_build_health_frame(can_tx_health_frame_t *frame, uint32_t sample_rate_x10,
                            uint16_t telemetry_decimation, uint16_t tx_drop_count,
                            uint16_t adc_overflow_count, uint16_t adc_recovery_count,
                            uint8_t active_adc_count, uint8_t flags);
uint16_t can_frame_u16_le_get(const uint8_t value_le[2]);

#endif // __CAN_DATA_H__
