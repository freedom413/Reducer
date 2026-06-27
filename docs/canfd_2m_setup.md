# CAN FD 500K / 2M Setup

This branch uses CAN FD with bit-rate switching (BRS) end to end:

- Arbitration phase: `500000` bit/s
- Data phase: `2000000` bit/s
- Standard 11-bit IDs: `0x0F0`, `0x0F1`, `0x0F2`, `0x0FF`, `0x110`
- Payload size: 12 bytes for command/status, 64 bytes for batched telemetry,
  24 bytes for health, 64 bytes for config, and 8 classic bytes for diag
- Frame format: v3 business frames use CAN FD+BRS; diagnostic `0x0FF` is classic CAN

The STM32G431 FDCAN kernel clock is `170 MHz`. Firmware timing is:

| Phase | Prescaler | TSEG1 | TSEG2 | SJW | Result |
|---|---:|---:|---:|---:|---:|
| Nominal | 20 | 14 | 2 | 1 | 500 kbit/s |
| Data | 5 | 14 | 2 | 1 | 2 Mbit/s |

Tx delay compensation is intentionally left disabled for this 500K / 2M timing
set. This matches the known-good bring-up profile for the board and CANable 2.0
path. Enabling a fixed TDCO value such as 15 data time quanta changes the
transmit data-phase sample timing and must be reintroduced only with analyzer
evidence for the exact transceiver, cable, and adapter setup.

## Hardware

Use a CAN FD-capable transceiver on the MCU board and a CAN FD-capable PC
adapter. Keep the two 120-ohm termination resistors, CANH/CANL wiring, and
common ground. A classic CAN-only transceiver cannot be used for this branch.

## CANable 2.0 SLCAN FD

The recommended PC adapter is CANable 2.0 running its official SLCAN firmware.
The firmware extends LAWICEL-style SLCAN with CAN FD commands. The GUI opens the
selected serial port and configures the adapter automatically:

```text
C
S6
Y2
O
```

The upstream backend may emit `O` more than once while applying timing and
opening the bus. The meaningful configuration is close, nominal rate `S6`
(`500K`), data rate `Y2` (`2M`), and open.

Application frames use the CANable 2.0 `b` command for standard-ID CAN FD+BRS
frames, for example `b0F18...` for the `0x0F1` command frame. Select
`CANable 2.0 SLCAN FD` and the correct `COMx` or `/dev/ttyACMx` port in the GUI.

Standard SLCAN adapters do not provide these CANable 2.0 FD extensions.

The GUI uses the upstream `python-can` SLCAN backend with `BitTimingFd`, so it
configures `S6` and `Y2` through the maintained library implementation.
Use `python-can>=4.6.1`: SLCAN FD support landed in `4.6.0`, and `4.6.1`
includes the follow-up SLCAN initialization fix.

Before opening python-can, the host runs a `C / S6 / Y2 / O` preflight. A BEL
response fails the connection explicitly. Adapters that return no command text
are allowed with a warning because some compatible firmware acknowledges only
through serial state. The preflight result is retained for the GUI and
`can_link_probe.py` diagnostics.

Cangaroo and the Python GUI/probe cannot own the same serial port at the same
time. Close the Cangaroo measurement before starting either Python program.

CANable 2.0 exposes a USB CDC virtual serial port. Keep the GUI's USB serial
speed at `2000000` unless bring-up proves the adapter rejects that line-coding
request; the GUI can fall back to lower speeds for connection testing, but
`115200` is not enough for the full 64-byte FD+BRS telemetry stream. At the
maximum configured load, the SLCAN ASCII stream is about 875 frames/s, or roughly
1.17 Mbit/s before any host-side margin.

## ADS1256 Performance

SPI1 runs at `1.328125 Mbit/s`, below the ADS1256 `f_CLKIN / 2` serial clock
limit for the board's `7.68 MHz` clock. All ADS1256 DRATE values are exposed:

```text
2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
3750, 7500, 15000, 30000 SPS
```

With input multiplexing, effective channel-cycling throughput follows the
ADS1256 datasheet Table 14. At the `30000 SPS` setting each converter delivers
about `4374` conversions/s. Firmware processes every acquired conversion and
packs up to 14 raw records or up to 6 physical records into one 64-byte FD+BRS
frame. At the maximum dual-ADC cycling rate of about `8748` records/s, raw mode
needs about `625` CAN frames/s. A 128-record software queue smooths bursts
before the three-slot FDCAN hardware TX FIFO. Firmware does not decimate
telemetry: every acquired record is queued, and any inability to transmit is
reported through the CAN TX drop counter.

The MCU sends a 24-byte FD+BRS health frame on `0x0F2` every second. The GUI
shows ADC state, actual telemetry rates, CAN TX drops, ADC ring-buffer overflows,
automatic recoveries, and received telemetry rate.

The MCU also sends an 8-byte classic CAN diagnostic heartbeat on `0x0FF` during
bring-up. It is not a business-protocol fallback; it only reports whether the
main loop is alive, what kind of `0x0F1` frame was last seen, why a command was
rejected, and the current FDCAN error counters.

## Linux SocketCAN

Bring up a CAN FD-capable `can0` interface before starting the GUI:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

Run the GUI and select `SocketCAN FD` / `can0`:

```bash
python host_pc/python/reducer_monitor.py
```

For a terminal smoke test:

```bash
candump can0
```

Telemetry should appear as CAN FD+BRS frames on ID `101`.

## Other Windows Adapters

The GUI also exposes CAN FD backends supported by `python-can`:

| Adapter family | GUI interface | Default channel |
|---|---|---|
| PEAK PCAN-USB FD | `PCAN FD` | `PCAN_USBBUS1` |
| HMS/IXXAT CAN FD | `IXXAT FD` | `0` |
| Vector CAN FD | `Vector FD` | `0` |

Install the vendor driver before starting the GUI. For PCAN, use a PCAN
adapter with FD support; classic PCAN-USB hardware is not sufficient.

## Build And Flash

```powershell
.\scripts\build_firmware.ps1 -Configuration Debug
.\scripts\build_firmware.ps1 -Configuration Release
```

Flash `build/Debug/Reducer.elf`.

## Bring-Up Checklist

1. With power removed, verify about 60 ohms between CANH/CANL, common ground,
   transceiver power and STB/EN, the 8 MHz HSE, both 7.68 MHz ADS1256 clocks,
   and ADC supplies.
2. Flash Debug firmware, close Cangaroo, then run
   `host_pc\python\.venv\Scripts\python.exe host_pc\python\can_link_probe.py --channel COMx`.
3. Require `0x0FF` classic diag at about 4 Hz, `0x0F2` FD health at about 1 Hz,
   GET_CONFIG ACK/config on `0x0F0`, and stable zero bus-off/passive/TEC/REC.
4. At 100 SPS Raw, verify all eight GUI channels, then masks `0x0F`, `0xF0`,
   and `0xFF` to exercise each ADC separately and together.
5. Verify calibration, filter window, Raw/Physical switching, zero, clear-zero,
   and configuration ACK behavior.
6. Zero the sensors, power-cycle, and verify `zero_valid`, offsets, and config
   sequence restore from Flash.
7. Flash Release and test `100, 500, 1000, 3750, 7500, 15000, 30000 SPS` for
   at least 10 seconds per step.
8. Accept 30 kSPS only in Raw mode: expect about 8748 samples/s within 10%,
   with zero CAN drops, ADC overflows, and recoveries.
9. Validate Physical mode at 1000 SPS and CSV recording at 500 SPS or lower.
10. Disconnect and reconnect CAN while running; diag, health, telemetry, and
    command ACK must recover within five seconds.

## References

- [CANable 2.0 firmware SLCAN FD commands](https://github.com/normaldotcom/canable2-fw)
- [CANable 2.0 USB CDC line-coding handler](https://github.com/normaldotcom/canable2-fw/blob/main/src/usbd_cdc_if.c)
- [python-can SLCAN backend](https://python-can.readthedocs.io/en/stable/interfaces/slcan.html)
- [python-can changelog](https://python-can.readthedocs.io/en/main/changelog.html)
- [TI ADS1256 datasheet](https://www.ti.com/lit/ds/symlink/ads1256.pdf)
