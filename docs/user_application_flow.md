# User Application Flow

This document describes the current application-level workflow implemented in
`Application/user.c`. The module ties together the ADS1256 acquisition task,
filtering, zero-offset storage, flexspline calculation, and CAN command/telemetry
transport.

## Runtime Overview

The application has two entry points:

- `setup()`: runs once at startup.
- `loop()`: runs repeatedly from the main program.

At a high level, every `loop()` iteration does this:

1. Handle up to four pending CAN command frames.
2. Poll both ADS1256 devices and collect any completed ADC samples.
3. Copy available ADC samples from the ADS1256 ring buffer into per-channel
   storage.
4. Wait until all enabled logical channels have fresh samples.
5. For each channel, apply filtering and zero-offset correction.
6. Reject outliers using a running mean/variance check.
7. Convert the filtered raw value to voltage, strain, and stress.
8. Send one CAN telemetry frame per channel.

## Startup Flow

`setup()` initializes modules in this order:

1. `delay_init()`

   Initializes the delay provider used by low-level drivers such as ADS1256.

2. `can_init()`

   Initializes CAN. Its result is stored in `can_ready`. CAN status and telemetry
   frames are sent only when `can_ready == true`.

3. `adc_ads1256_start()`

   Initializes ADS1256 devices, configures them, resets the ADS1256 acquisition
   ring buffer, selects the first channel on each enabled device, and starts the
   first conversion.

4. `flash_storage_register_user_ops()`

   Registers board-specific STM32 Flash operations implemented in
   `BSP/flash_storage_port.c`. This must happen before the filter tries to load
   zero offsets from Flash.

5. `filter_init()`

   Resets each moving-average filter and then calls
   `filter_load_zero_from_flash()`. If valid calibration data exists in Flash,
   each channel's zero offset is restored.

6. `flexspline_params_set_default(&flexspline_params)`

   Loads default physical conversion parameters:

   - reference voltage: `2.5 V`
   - PGA: `64`
   - bridge excitation: `5.0 V`
   - gauge factor: `2.0`
   - elastic modulus: `210000 MPa`

## Channel Model

`ADC_CHANNEL_COUNT` is derived from `ADS1256_LOGICAL_CHANNEL_COUNT`.

With the current ADS1256 raw acquisition settings:

- Each ADS1256 device contributes `ADS1256_CHANNELS_PER_DEVICE`, currently `3`.
- ADS1256 A maps to logical channels `0..2`.
- ADS1256 B maps to logical channels `3..5` when B is enabled.

The application stores the latest raw sample for each logical channel in:

```c
static int32_t adc_raw_value[ADC_CHANNEL_COUNT];
```

It tracks which channels have arrived in the current processing batch with:

```c
static uint8_t adc_all_ch_mask;
```

When all channel bits equal `ADC_ALL_CH_MASK`, a full batch is processed.

## ADS1256 Sampling Flow

`loop()` calls `adc_ads1256_poll()`.

The ADS1256 layer checks each enabled ADC device. When `DRDY` indicates that a
conversion is complete, the ADS1256 layer uses the optimized multiplexer cycle:

1. Remember the channel whose result is ready.
2. Select the next channel by writing the ADS1256 `MUX` register.
3. Issue `SYNC` and `WAKEUP` to start conversion on the next channel.
4. Read the already-latched previous result with `ads1256_read_data_nowait()`.
5. Push `{logical_channel, raw_value}` into the ring buffer.

This matches the ADS1256 datasheet's efficient channel-cycling flow: the next
channel starts converting before the previous channel's data is read.

`loop()` then pulls samples with:

```c
adc_ads1256_get_data(adc_ads1256_data, ADC_CHANNEL_COUNT);
```

Each returned record updates `adc_raw_value[ch]` and sets the channel bit in
`adc_all_ch_mask`.

## Filtering And Zero Offset

After a complete channel batch is available, each channel is passed through:

```c
filtered = filter_apply(channel, adc_raw_value[channel]);
```

`filter_apply()` does two operations:

1. Moving average over the configured window.
2. Subtract that channel's `zero_offset`.

The moving average window defaults to the value in `filter.h`, and can be changed
at runtime through `CAN_CMD_SET_FILTER_SIZE`.

Zero offsets are stored inside the filter module. They can be:

- loaded from Flash during `filter_init()`
- captured from current filtered raw values with `CAN_CMD_ZERO_DATUM`
- saved with `CAN_CMD_SAVE_ZERO`
- loaded again with `CAN_CMD_LOAD_ZERO`
- cleared with `CAN_CMD_CLEAR_ZERO`

## Outlier Rejection

After filtering and zero correction, the application checks whether the value is
an outlier.

The first `OUTLIER_MIN_SAMPLES` samples per channel are always accepted. The
current value is then compared against a running mean and variance maintained
with Welford's online algorithm.

The normal outlier rule is:

```text
(value - mean)^2 > 9 * variance
```

This is equivalent to a 3-sigma threshold, but avoids calling `sqrtf()`.

If the variance is almost zero, the code uses a fixed threshold instead:

```text
abs(value - mean) > 100
```

Accepted values update:

- `running_mean[channel]`
- `running_m2[channel]`
- `sample_count[channel]`
- `adc_filtered_value[channel]`

Rejected values do not update the statistics. The application reuses the last
accepted value from `adc_filtered_value[channel]`.

Statistics are reset when commands change acquisition/filtering assumptions:

- zero datum
- sample rate change
- ADS1256 calibration
- zero offset load
- zero offset clear

## Physical Conversion

Accepted or substituted filtered values are passed to:

```c
flexspline_calculate(filtered, &flexspline_params, &result);
```

The conversion path is:

```text
raw ADC code -> voltage in mV -> microstrain -> stress in MPa
```

Using the default parameters:

```text
voltage_mV = raw * (2 * ref_voltage / pga) * 1000 / 8388608
strain     = voltage_mV * 1000 / (excitation_v * gauge_k)
stress     = strain * elastic_modulus / 1000000
```

Before CAN transmission, values are packed into compact integer units:

- `voltage_001mv`: signed 16-bit, 0.01 mV units
- `strain_ue`: signed 16-bit, microstrain units
- `stress_01mpa`: signed 8-bit, 0.1 MPa units

The default sensor configuration is a 350-ohm four-active-gauge full bridge:

- bridge excitation: 5 V
- gauge factor: 2.11
- maximum strain: +/-20000 microstrain
- ADS1256 reference voltage: 2.5 V
- ADS1256 PGA: 16

The helper clamp functions saturate values to the target integer range before
packing.

## CAN Command Flow

Each `loop()` iteration calls `process_can_commands()`.

The function processes up to `CAN_COMMANDS_PER_LOOP`, currently `4`, command
frames per loop. This prevents command handling from monopolizing the loop when
many frames are queued.

Incoming command frame requirements:

- CAN ID: `CAN_ID_RX_COMMAND` (`0x100`)
- DLC: `8`
- byte 0 `frame_type`: `CAN_FRAME_TYPE_COMMAND` (`0xA0`)
- byte 7 CRC: XOR of bytes `0..6`

Malformed frames are ignored if the CAN ID or DLC does not match. Frames with a
bad type or CRC receive a status response.

Valid commands are passed to `process_can_command()`. A status frame is sent
after command execution.

### Command Frame Format

CAN ID: `0x100`

```text
byte 0: frame_type = 0xA0
byte 1: sequence
byte 2: cmd_type
byte 3: param
byte 4: value LSB
byte 5: value MSB
byte 6: reserved
byte 7: crc8 XOR(bytes 0..6)
```

`value` is little-endian.

### Status Frame Format

CAN ID: `0x102`

```text
byte 0: frame_type = 0xA1
byte 1: sequence copied from command
byte 2: cmd_type copied from command
byte 3: status
byte 4: value LSB
byte 5: value MSB
byte 6: detail
byte 7: crc8 XOR(bytes 0..6)
```

`value` is little-endian.

### Supported Commands

`CAN_CMD_SET_SAMPLE_RATE` (`0x01`)

Sets ADS1256 sample rate using `value`. Supported rates are defined by the
ADS1256 acquisition module. On success, the applied sample rate is returned in
the status frame's `value` field. The filters and outlier statistics are reset.

`CAN_CMD_SET_FILTER_SIZE` (`0x02`)

Sets the moving-average window size. Valid range is `2..64`. The applied window
size is echoed in the status frame's `value` field.

`CAN_CMD_ZERO_DATUM` (`0x03`)

Captures the current raw moving-average value from every channel as the zero
offset. The offsets are saved to Flash. Filters and outlier statistics are reset
after saving.

`CAN_CMD_START_CALIB` (`0x04`)

Runs ADS1256 self-calibration, restarts acquisition, and resets filters and
outlier statistics.

`CAN_CMD_SAVE_ZERO` (`0x05`)

Saves the current filter zero offsets to Flash.

`CAN_CMD_LOAD_ZERO` (`0x06`)

Loads zero offsets from Flash. Filters and outlier statistics are reset after a
successful load.

`CAN_CMD_CLEAR_ZERO` (`0x07`)

Sets all in-memory zero offsets to zero, erases the Flash calibration page, and
resets filters and outlier statistics.

`CAN_CMD_SET_CHANNEL_MASK` (`0x08`)

Sets the runtime ADS1256 scan mask using `value`. Only channels selected by the
host waveform plots are scanned. A zero mask stops ADC channel polling. Channels
whose enabled state changes have their filters and statistics reset.

## CAN Telemetry Flow

After all channels in a batch are processed, `loop()` sends one telemetry frame
per channel with:

```c
send_can_telemetry(channel, voltage_001mv, strain_ue, stress_01mpa);
```

Telemetry is skipped if `can_ready == false`.

### Telemetry Frame Format

CAN ID: `0x101`

```text
byte 0: frame_type = 0x51
byte 1: channel
byte 2: voltage MSB
byte 3: voltage LSB
byte 4: strain MSB
byte 5: strain LSB
byte 6: stress
byte 7: crc8 XOR(bytes 0..6)
```

Voltage and strain are big-endian signed 16-bit values. Stress is a signed
8-bit value.

## Flash Zero-Offset Storage

The Flash storage module stores a 32-byte calibration record at
`FLASH_STORAGE_ADDR`.

The stored record contains:

- version
- six `int32_t` zero offsets
- CRC-16 over the zero offsets
- magic number
- padding

Before loading zero offsets, the module validates:

1. Flash operations are registered.
2. Magic number matches.
3. Version matches.
4. CRC-16 matches.

If validation fails, `flash_storage_load_zero()` clears the caller's offset array
to zero and returns an error. `filter_init()` ignores that error, so startup still
continues with zero offsets.

## Error Handling Summary

CAN command status values:

- `CAN_STATUS_OK` (`0x00`): command completed successfully
- `CAN_STATUS_BAD_CRC` (`0xE1`): command CRC mismatch
- `CAN_STATUS_BAD_TYPE` (`0xE2`): wrong frame type
- `CAN_STATUS_BAD_CMD` (`0xE3`): unsupported command
- `CAN_STATUS_BAD_VALUE` (`0xE4`): invalid parameter or ADC operation failure
- `CAN_STATUS_STORAGE_ERROR` (`0xE5`): Flash save/load/clear failure

Most command handlers also fill `detail` with a small error source code. The
meaning is local to each command handler.

## Important Behavior Notes

- CAN commands are now active because `loop()` calls `process_can_commands()`.
- Telemetry transmission is now active because `send_can_telemetry()` is called
  for every processed channel.
- Processing waits for a complete set of logical channels before calculating and
  transmitting telemetry.
- ADS1256 samples can arrive at different moments, but telemetry is emitted in
  channel order after the batch is complete.
- Outlier rejection operates after moving-average filtering and zero-offset
  correction, not on raw ADC codes.
- `filter_reset_all()` does not erase zero offsets. It only clears moving-average
  buffers.
- Flash erase/write only happens on zero-offset save, zero datum, or zero clear
  commands. Normal sampling does not write Flash.
