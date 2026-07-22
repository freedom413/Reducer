# User Application Flow

This document describes the current runtime behavior implemented in
`Application/user.c`.

## Runtime Overview

`setup()` initializes delay support, FDCAN, both ADS1256 converters, Flash-backed
zero storage, moving-average filters, and flexspline conversion parameters.

Each `loop()` iteration:

1. Handles up to four queued CAN FD+BRS commands.
2. Services delayed Flash config saves and pending config snapshots.
3. Sends the 250 ms classic CAN diagnostic heartbeat and one-second FD health
   frame when due.
4. Polls both ADS1256 devices.
5. Pulls completed conversion records from the ADS1256 ring buffer.
6. Processes every record immediately: moving average, zero subtraction,
   statistics, physical conversion, and optional telemetry transmission.

The application does not wait for a complete multi-channel batch. That keeps ADC
polling responsive and avoids bursts that could overflow the three-slot FDCAN TX
FIFO.

ADC sampling continues while no host is connected, but periodic CAN telemetry,
health, diagnostics, and config snapshots are gated by a host session. Any valid
protocol command activates the session. The GUI sends a silent keepalive every
second; after three seconds without a valid command, firmware ends the session
and clears pending ACK and telemetry queues. A subsequent `GET_CONFIG` command
reactivates transmission immediately. On a normal GUI disconnect, the host sends
an explicit session-stop parameter before closing the CAN adapter.

## MCU Status LED

`MCU_LED` is active-low: driving the pin low turns the LED on, and driving it
high turns it off. GPIO initialization therefore leaves it off. The application
uses a short heartbeat while the MCU is healthy and idle (75 ms every second),
a faster short flash while a host session is active (75 ms every 250 ms), and a
50% duty-cycle fault flash whenever CAN initialization failed or the ADS1256 is
not running. The LED is deliberately not toggled for every ADC conversion, so
high-SPS acquisition does not spend time on GPIO writes. `Error_Handler` also
uses an explicit active-low fault blink.

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
`1.328125 Mbit/s`, below the ADS1256 serial-clock limit for this board.

## Sample Rates And Throughput

All ADS1256 DRATE settings are available:

```text
2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
3750, 7500, 15000, 30000 SPS
```

The real multiplexed conversion throughput follows ADS1256 datasheet Table 14.
For example, the `30000 SPS` setting produces about `4374` conversions/s per
active ADC while cycling channels.

Every acquired conversion is filtered and converted. Firmware packs up to 14
raw records or up to 6 physical records into one 64-byte telemetry frame and
buffers up to 128 pending records before the three-slot FDCAN hardware TX FIFO.
Firmware does not decimate telemetry: every acquired record enters the queue,
and failed queue/transmit attempts increment the drop counter.

## Filtering, Statistics, And Zero Storage

`filter_apply()` performs a moving average and subtracts the stored zero offset.
The filter window is configurable from `2..64`.

Large steps are not rejected. At this layer a real load step cannot be
distinguished reliably from a sensor outlier, so it must reach telemetry.
Runtime voltage statistics are maintained by the host GUI. The MCU does not
currently maintain a Welford statistics accumulator.

The eight zero offsets and runtime configuration are persisted in the final
two STM32 Flash pages as versioned 64-byte records with CRC32 and a magic value.
Flash writes happen after explicit zero/configuration changes. Each programmed
record is read back, CRC-validated, and compared byte-for-byte before the save
is reported as successful. Records append to the active page. When it is full,
the inactive page is erased and the new record is written and verified there,
so the previous valid page survives an interrupted rollover.

## CAN FD Protocol

Business frames use standard 11-bit IDs, CAN FD+BRS, `500K / 2M`, and fixed
payload lengths. The classic diagnostic heartbeat is intentionally separate
from the business protocol and is used only for bring-up visibility.
Protocol-v3 FD frames rely on the CAN frame CRC. Application-layer XOR is used
only by the classic diagnostic frame and accepted legacy frames.

| Direction | ID | Type | Length | Purpose |
|---|---:|---:|---:|---|
| MCU -> PC | `0x0F0` | `0xA1` | 12 FD+BRS | Command ACK/status |
| MCU -> PC | `0x0F0` | `0x56` | 64 FD+BRS | Config snapshot |
| PC -> MCU | `0x0F1` | `0xA0` | 12 FD+BRS | Command |
| MCU -> PC | `0x0F2` | `0x52` | 24 FD+BRS | One-second health report |
| MCU -> PC | `0x0FF` | `0x57` | 8 classic CAN | Link diagnostic heartbeat |
| MCU -> PC | `0x110` | `0x54` | 64 FD+BRS | Raw telemetry batch |
| MCU -> PC | `0x110` | `0x55` | 64 FD+BRS | Physical telemetry batch |

Command frame:

```text
byte 0: type = 0xA0
byte 1: protocol version = 0x03
byte 2: sequence
byte 3: command
byte 4: param
byte 5: reserved
byte 6-9: value uint32 LE
byte 10-11: reserved
```

For `CAN_CMD_SET_SAMPLE_RATE`, `param=0` encodes integer SPS and `param=1`
encodes tenths of SPS. The latter represents `2.5 SPS` as `value=25`.

Raw telemetry frame:

```text
byte 0: type = 0x54
byte 1: protocol version = 0x03
byte 2: telemetry mode = raw
byte 3: sequence
byte 4: record count, 1..14
byte 5-6: drop delta uint16 LE
byte 7: reserved
byte 8-63: fourteen 4-byte raw records
```

Physical telemetry frame:

```text
byte 0: type = 0x55
byte 1: protocol version = 0x03
byte 2: telemetry mode = physical
byte 3: sequence
byte 4: record count, 1..6
byte 5-6: drop delta uint16 LE
byte 7: reserved
byte 8-61: six 9-byte physical records
byte 62-63: reserved
```

Health frame:

```text
byte 0: type = 0x52
byte 1: protocol version = 0x03
byte 2-5: configured sample rate x10, uint32 LE
byte 6-7: CAN TX drop count, uint16 LE
byte 8-9: ADC ring-buffer overflow count, uint16 LE
byte 10-11: ADC automatic recovery count, uint16 LE
byte 12-13: telemetry samples/s, uint16 LE
byte 14-15: telemetry frames/s, uint16 LE
byte 16: active ADC count
byte 17: telemetry mode
byte 18: flags, bit 0 ADC hardware running, bit 1 config dirty, bit 2 zero valid,
         bit 3 channel mask is nonzero
byte 19-23: reserved
```

Diagnostic frame:

```text
byte 0: type = 0x57
byte 1: flags, CAN ready/main loop/last RX FD/last RX BRS/bus-off/passive
byte 2: last 0x0F1 DLC in bytes
byte 3: last command reject reason
byte 4: FDCAN TX error counter
byte 5: FDCAN RX error counter
byte 6: diagnostic sequence
byte 7: XOR CRC over bytes 0..6
```

## Commands

| Command | Value | Purpose |
|---|---:|---|
| `SET_SAMPLE_RATE` | `0x01` | Set ADS1256 sample rate |
| `SET_FILTER_SIZE` | `0x02` | Set moving-average window |
| `ZERO_DATUM` | `0x03` | Capture current zero and save it |
| `START_CALIB` | `0x04` | Run ADS1256 self-calibration |
| `SAVE_ZERO` | `0x05` | Legacy reserved command; not handled or exposed by the GUI |
| `LOAD_ZERO` | `0x06` | Legacy reserved command; not handled or exposed by the GUI |
| `CLEAR_ZERO` | `0x07` | Clear RAM and Flash zero offsets |
| `SET_CHANNEL_MASK` | `0x08` | Select channels to scan |
| `SET_TELEMETRY_MODE` | `0x09` | Select Raw or Physical telemetry |
| `GET_CONFIG` | `0x0A` | Request a configuration snapshot |
| `SET_VREF_UV` | `0x0B` | Set ADC reference voltage in microvolts |
| `SET_PGA` | `0x0C` | Set ADS1256 PGA gain and recalibrate |
| `RESTORE_DEFAULTS` | `0x0D` | Apply, calibrate, restart, and save defaults |
| `SET_ZERO_OFFSET` | `0x0E` | Set one channel zero offset |
| `HOST_KEEPALIVE` | `0x0F` | Refresh the host telemetry session |

`SAVE_ZERO` and `LOAD_ZERO` are legacy reserved commands. Current zero and
configuration persistence is performed by the active commands above; the GUI
does not expose the two reserved command values.
