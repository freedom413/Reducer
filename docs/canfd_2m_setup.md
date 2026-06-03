# CAN FD 1M / 2M Setup

This branch uses CAN FD with bit-rate switching (BRS) end to end:

- Arbitration phase: `1000000` bit/s
- Data phase: `2000000` bit/s
- Standard 11-bit IDs: `0x100`, `0x101`, `0x102`, `0x103`
- Payload size: 8 bytes for command/status, 64 bytes for batched telemetry,
  16 bytes for health
- Frame format: CAN FD with BRS, not classic CAN

The STM32G431 FDCAN kernel clock is `170 MHz`. Firmware timing is:

| Phase | Prescaler | TSEG1 | TSEG2 | SJW | Result |
|---|---:|---:|---:|---:|---:|
| Nominal | 10 | 14 | 2 | 1 | 1 Mbit/s |
| Data | 5 | 14 | 2 | 1 | 2 Mbit/s |

The 2 Mbit/s data phase also enables FDCAN transmitter delay compensation:

| Setting | Value |
|---|---:|
| TDC offset | 15 data time quanta |
| TDC filter | 0 data time quanta |

With the current data timing, one data time quantum is about `29.41 ns`; the
offset places the secondary sample point near the configured data sample point.
If a specific transceiver/cable setup still reports data-phase protocol errors,
verify the physical layer first and then tune this offset with a scope or bus
analyzer.

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
S8
Y2
O
```

The upstream backend may emit `O` more than once while applying timing and
opening the bus. The meaningful configuration is close, nominal rate `S8`
(`1M`), data rate `Y2` (`2M`), and open.

Application frames use the CANable 2.0 `b` command for standard-ID CAN FD+BRS
frames, for example `b1008...` for the `0x100` command frame. Select
`CANable 2.0 SLCAN FD` and the correct `COMx` or `/dev/ttyACMx` port in the GUI.

Standard SLCAN adapters do not provide these CANable 2.0 FD extensions.

The GUI uses the upstream `python-can` SLCAN backend with `BitTimingFd`, so it
configures `S8` and `Y2` through the maintained library implementation.
Use `python-can>=4.6.1`: SLCAN FD support landed in `4.6.0`, and `4.6.1`
includes the follow-up SLCAN initialization fix.

CANable 2.0 exposes a USB CDC virtual serial port. Its official firmware ignores
the host `CDC_SET_LINE_CODING` request and reports `115200` for compatibility.
The GUI therefore does not expose a misleading UART baudrate control. USB CDC
throughput, CAN FD load, and firmware decimation are observed through the health
panel counters.

## ADS1256 Performance

SPI1 runs at `2.65625 Mbit/s`, below the ADS1256 `f_CLKIN / 2` serial clock
limit for the board's `7.68 MHz` clock. All ADS1256 DRATE values are exposed:

```text
2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
3750, 7500, 15000, 30000 SPS
```

With input multiplexing, effective channel-cycling throughput follows the
ADS1256 datasheet Table 14. At the `30000 SPS` setting each converter delivers
about `4374` conversions/s. Firmware processes every acquired conversion and
packs up to ten telemetry records into one 64-byte FD+BRS frame. At the maximum
dual-ADC cycling rate of about `8748` records/s, this needs about `875` CAN
frames/s. A 128-record software queue smooths bursts before the three-slot
FDCAN hardware TX FIFO. Outgoing records are decimated only if a future source
configuration exceeds `10000` records/s.

The MCU sends a 16-byte FD+BRS health frame on `0x103` every second. The GUI
shows ADC state, effective decimation, CAN TX drops, ADC ring-buffer overflows,
automatic recoveries, and received telemetry rate.

## Linux SocketCAN

Bring up a CAN FD-capable `can0` interface before starting the GUI:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on restart-ms 100
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
cmake -S . -B build\Debug -DCMAKE_TOOLCHAIN_FILE="cmake/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build build\Debug
```

Flash `build/Debug/Reducer.elf`.

## Bring-Up Checklist

1. Flash the firmware and power-cycle the MCU.
2. Start the GUI, select `CANable 2.0 SLCAN FD`, and choose the serial port.
3. Connect; the GUI configures `1M / 2M`, FD, and BRS automatically.
4. Confirm FD+BRS telemetry on `0x101`.
5. Send `Calibrate` and confirm the FD+BRS ACK on `0x102`.
6. Increase the sample rate gradually to `30000 SPS` and watch the GUI health
   panel for TX drops, ADC overflows, and recoveries.

## References

- [CANable 2.0 firmware SLCAN FD commands](https://github.com/normaldotcom/canable2-fw)
- [CANable 2.0 USB CDC line-coding handler](https://github.com/normaldotcom/canable2-fw/blob/main/src/usbd_cdc_if.c)
- [python-can SLCAN backend](https://python-can.readthedocs.io/en/stable/interfaces/slcan.html)
- [python-can changelog](https://python-can.readthedocs.io/en/main/changelog.html)
- [TI ADS1256 datasheet](https://www.ti.com/lit/ds/symlink/ads1256.pdf)
