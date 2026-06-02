# User Application Flow

This document describes the current runtime behavior implemented in
`Application/user.c`.

## Runtime Overview

`setup()` initializes delay support, FDCAN, both ADS1256 converters, Flash-backed
zero storage, moving-average filters, and flexspline conversion parameters.

Each `loop()` iteration:

1. Handles up to four queued CAN FD+BRS commands.
2. Sends the one-second health frame when due.
3. Polls both ADS1256 devices.
4. Pulls completed conversion records from the ADS1256 ring buffer.
5. Processes every record immediately: moving average, zero subtraction,
   statistics, physical conversion, and optional telemetry transmission.

The application does not wait for a complete multi-channel batch. That keeps ADC
polling responsive and avoids bursts that could overflow the three-slot FDCAN TX
FIFO.

## Channels And Acquisition

Each ADS1256 scans four differential inputs:

| Device | Logical channels | Differential inputs |
|---|---|---|
| ADS1256 A | `0..3` | `AIN0-AIN1`, `AIN2-AIN3`, `AIN4-AIN5`, `AIN6-AIN7` |
| ADS1256 B | `4..7` | `AIN0-AIN1`, `AIN2-AIN3`, `AIN4-AIN5`, `AIN6-AIN7` |

The host controls the runtime scan mask with `CAN_CMD_SET_CHANNEL_MASK`.
Unsubscribed channels are not scanned. A zero mask stops ADC polling.

When `DRDY` signals a result, the ADS1256 layer switches MUX, issues
`SYNC`/`WAKEUP`, reads the latched previous result, and pushes
`{logical_channel, raw_value}` into the ring buffer. SPI1 runs at
`2.65625 Mbit/s`, below the ADS1256 serial-clock limit for this board.

## Sample Rates And Throughput

All ADS1256 DRATE settings are available:

```text
2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
3750, 7500, 15000, 30000 SPS
```

The real multiplexed conversion throughput follows ADS1256 datasheet Table 14.
For example, the `30000 SPS` setting produces about `4374` conversions/s per
active ADC while cycling channels.

Every acquired conversion is filtered and converted. Firmware packs up to ten
records into one 64-byte telemetry frame and buffers up to 128 pending records
before the three-slot FDCAN hardware TX FIFO. At the maximum dual-ADC rate this
transfers all acquired conversions without telemetry decimation. A safety
decimation factor applies only above `10000 records/s`.

## Filtering, Statistics, And Zero Storage

`filter_apply()` performs a moving average and subtracts the stored zero offset.
The filter window is configurable from `2..64`.

Large steps are not rejected. At this layer a real load step cannot be
distinguished reliably from a sensor outlier, so it must reach telemetry.
Welford running statistics are maintained internally for diagnostics.

The eight zero offsets are persisted in the final STM32 Flash page as a
versioned 40-byte record with CRC-16 and a magic value. Flash writes happen only
for explicit zero-storage commands.

## CAN FD Protocol

All business frames use standard 11-bit IDs, CAN FD+BRS, `1M / 5M`, and an XOR
checksum.

| Direction | ID | Type | Length | Purpose |
|---|---:|---:|---:|---|
| PC -> MCU | `0x100` | `0xA0` | 8 | Command |
| MCU -> PC | `0x101` | `0x53` | 64 | Up to ten telemetry records |
| MCU -> PC | `0x102` | `0xA1` | 8 | Command ACK |
| MCU -> PC | `0x103` | `0x52` | 16 | One-second health report |

Command frame:

```text
byte 0: type = 0xA0
byte 1: sequence
byte 2: command
byte 3: param
byte 4-5: value uint16 LE
byte 6: reserved
byte 7: XOR CRC over bytes 0..6
```

For `CAN_CMD_SET_SAMPLE_RATE`, `param=0` encodes integer SPS and `param=1`
encodes tenths of SPS. The latter represents `2.5 SPS` as `value=25`.

Telemetry frame:

```text
byte 0: type = 0x53
byte 1: telemetry record count, 1..10
byte 2-61: ten 6-byte record slots, unused slots are zero-filled
byte 62: reserved
byte 63: XOR CRC over bytes 0..62

record byte 0: logical channel 0..7
record byte 1-2: voltage int16 BE, 0.01 mV units
record byte 3-4: strain int16 BE, microstrain units
record byte 5: clipped stress preview int8, 0.1 MPa units
```

Health frame:

```text
byte 0: type = 0x52
byte 1: protocol version = 0x01
byte 2-5: configured sample rate x10, uint32 LE
byte 6-7: telemetry decimation, uint16 LE
byte 8-9: CAN TX drop count, uint16 LE
byte 10-11: ADC ring-buffer overflow count, uint16 LE
byte 12-13: ADC automatic recovery count, uint16 LE
byte 14: high nibble active ADC count, bit 0 acquisition running
byte 15: XOR CRC over bytes 0..14
```

## Commands

| Command | Value | Purpose |
|---|---:|---|
| `SET_SAMPLE_RATE` | `0x01` | Set ADS1256 sample rate |
| `SET_FILTER_SIZE` | `0x02` | Set moving-average window |
| `ZERO_DATUM` | `0x03` | Capture current zero and save it |
| `START_CALIB` | `0x04` | Run ADS1256 self-calibration |
| `SAVE_ZERO` | `0x05` | Save zero offsets |
| `LOAD_ZERO` | `0x06` | Reload zero offsets |
| `CLEAR_ZERO` | `0x07` | Clear RAM and Flash zero offsets |
| `SET_CHANNEL_MASK` | `0x08` | Select channels to scan |
