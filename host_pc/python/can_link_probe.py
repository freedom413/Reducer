"""Reducer CAN FD link bring-up probe.

This tool separates adapter, nominal CAN, FD/BRS, and GUI-layer failures.
It uses the current development protocol only; no legacy ACK compatibility is
implemented here.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

from can_protocol import (
    Baudrate,
    CAN_FD_DATA_BITRATE,
    CAN_ID_RX_COMMAND,
    CAN_ID_TX_CONTROL,
    CAN_ID_TX_DIAG,
    CAN_ID_TX_HEALTH,
    CAN_ID_TX_TELEMETRY,
    DEFAULT_SLCAN_TTY_BAUDRATE,
    PythonCANInterface,
    build_command_frame,
    parse_config_frame,
    parse_diag_frame,
    parse_health_frame,
    parse_status_frame,
    parse_telemetry_frames,
)


CAN_CMD_GET_CONFIG = 0x0A


def _wait_until(deadline: float, predicate) -> bool:
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _print_probe_result(bus: PythonCANInterface) -> None:
    probe = bus.last_slcan_probe
    if probe is None:
        return
    print("SLCAN FD preflight:", "OK" if probe.ok else "FAILED")
    print("SLCAN commands:", " ".join(probe.commands))
    if probe.warning:
        print("SLCAN warning:", probe.warning)
    if probe.error:
        print("SLCAN error:", probe.error)


def _diag_summary(diag) -> str:
    if diag is None:
        return "none"
    if diag.last_rx_fd and diag.last_rx_brs:
        rx_format = "FD+BRS"
    elif diag.last_rx_fd:
        rx_format = "FD no-BRS"
    else:
        rx_format = "classic"
    return (
        f"seq={diag.sequence} main={int(diag.main_loop_alive)} "
        f"last_rx={diag.last_rx_dlc}B/{rx_format} "
        f"reject=0x{diag.last_reject_reason:02X} "
        f"bus_off={int(diag.bus_off)} passive={int(diag.error_passive)} "
        f"TEC={diag.tx_error_count} REC={diag.rx_error_count}"
    )


def _health_summary(health) -> str:
    if health is None:
        return "none"
    return (
        f"ADC={'RUN' if health.adc_running else 'STOP'} "
        f"{health.sample_rate_sps:g} SPS "
        f"mode={health.telemetry_mode} "
        f"tx={health.telemetry_samples_per_second} samples/s "
        f"drops={health.tx_drop_count}"
    )


def run_probe(args: argparse.Namespace) -> int:
    state = {
        "diag": None,
        "health": None,
        "status": None,
        "config": None,
        "telemetry_frames": 0,
        "frames": 0,
    }
    lock = threading.Lock()
    changed = threading.Event()

    def on_frame(frame) -> None:
        with lock:
            state["frames"] += 1
            diag = parse_diag_frame(frame)
            if diag is not None:
                state["diag"] = diag
                changed.set()
                return

            health = parse_health_frame(frame)
            if health is not None:
                state["health"] = health
                changed.set()
                return

            status = parse_status_frame(frame)
            if status is not None and status.cmd_type == CAN_CMD_GET_CONFIG:
                state["status"] = status
                changed.set()
                return

            config = parse_config_frame(frame)
            if config is not None:
                state["config"] = config
                changed.set()
                return

            telemetry = parse_telemetry_frames(frame)
            if telemetry is not None:
                state["telemetry_frames"] += 1
                changed.set()

    bus = PythonCANInterface()
    bus.register_rx_callback(on_frame)

    print(
        f"Opening {args.interface}:{args.channel} "
        f"at CAN FD {Baudrate.BAUD_500K.bps}/{CAN_FD_DATA_BITRATE} bps"
    )
    if not bus.connect(
        args.interface,
        args.channel,
        Baudrate.BAUD_500K,
        tty_baudrate=args.tty_baudrate,
        sleep_after_open=args.open_delay,
    ):
        _print_probe_result(bus)
        print("RESULT: adapter or python-can connection failed")
        if bus.last_error:
            print("DETAIL:", bus.last_error)
        return 1

    try:
        _print_probe_result(bus)
        first_deadline = time.monotonic() + min(args.timeout, 2.0)
        _wait_until(
            first_deadline,
            lambda: state["diag"] is not None or state["health"] is not None,
        )

        sequence = args.sequence & 0xFF
        command = build_command_frame(sequence, CAN_CMD_GET_CONFIG)
        if not bus.send_frame(command):
            print("RESULT: failed to send GET_CONFIG command")
            return 1
        print(
            f"Sent GET_CONFIG on 0x{CAN_ID_RX_COMMAND:03X} seq={sequence}; "
            f"expect control responses on 0x{CAN_ID_TX_CONTROL:03X}"
        )

        deadline = time.monotonic() + args.timeout
        _wait_until(
            deadline,
            lambda: (
                state["diag"] is not None and
                state["health"] is not None and
                state["status"] is not None and
                state["config"] is not None and
                (
                    state["health"] is None or
                    not state["health"].adc_running or
                    not state["health"].channel_mask_nonzero or
                    state["telemetry_frames"] > 0
                )
            ),
        )

        diag = state["diag"]
        health = state["health"]
        status = state["status"]
        config = state["config"]
        print("Frames observed:", state["frames"])
        print(f"Classic diag 0x{CAN_ID_TX_DIAG:03X}:", _diag_summary(diag))
        print(f"FD health 0x{CAN_ID_TX_HEALTH:03X}:", _health_summary(health))
        if status is not None:
            print(
                "ACK:",
                f"seq={status.sequence} status=0x{status.status:02X} "
                f"value={status.value}",
            )
        else:
            print("ACK: none")
        if config is not None:
            print(
                "Config:",
                f"sample_rate_x10={config.sample_rate_x10} "
                f"channel_mask=0x{config.channel_mask:02X} "
                f"telemetry_mode={config.telemetry_mode}",
            )
        else:
            print("Config: none")
        print(
            f"Telemetry 0x{CAN_ID_TX_TELEMETRY:03X}:",
            f"{state['telemetry_frames']} frame(s)",
        )

        if diag is None and health is None:
            print(
                "RESULT: no classic diag and no FD health; check MCU main "
                "loop, nominal 500K CAN, wiring, termination, and bitrate."
            )
            return 2
        if diag is not None and health is None:
            print(
                "RESULT: classic 500K CAN works, but FD/BRS traffic is "
                "missing; check CANable FD firmware, data phase timing, and TDC."
            )
            return 2
        if health is not None and (status is None or config is None):
            print(
                "RESULT: FD health is present, but GET_CONFIG ACK/config is "
                "missing; check PC command format and MCU command handling."
            )
            return 2
        if (
            health is not None and
            health.adc_running and
            health.channel_mask_nonzero and
            state["telemetry_frames"] == 0
        ):
            print(
                "RESULT: acquisition is running with enabled channels, but "
                "no telemetry was observed."
            )
            return 2

        print("RESULT: link OK; GUI queue/display layer is the next place to check.")
        return 0
    finally:
        bus.disconnect()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", required=True, help="CAN channel, e.g. COM3")
    parser.add_argument(
        "--interface",
        default="slcan",
        help="python-can interface, default: slcan",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="seconds to wait for diag/health/ACK/config",
    )
    parser.add_argument(
        "--tty-baudrate",
        type=int,
        default=DEFAULT_SLCAN_TTY_BAUDRATE,
        help="USB CDC baudrate passed to slcan backend",
    )
    parser.add_argument(
        "--open-delay",
        type=float,
        default=2.0,
        help="seconds python-can waits after opening slcan",
    )
    parser.add_argument(
        "--sequence",
        type=int,
        default=1,
        help="GET_CONFIG command sequence byte",
    )
    return run_probe(parser.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
