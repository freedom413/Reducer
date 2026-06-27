"""python-can transport and Reducer CAN FD protocol helpers."""

from __future__ import annotations

import logging
import os
import struct
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, List, Optional, Tuple

try:
    import can
except ImportError:  # pragma: no cover - handled at runtime on user machine
    can = None

try:
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - handled at runtime on user machine
    list_ports = None

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime on user machine
    serial = None

logger = logging.getLogger(__name__)


CAN_ID_TX_CONTROL = 0x0F0
CAN_ID_RX_COMMAND = 0x0F1
CAN_ID_TX_HEALTH = 0x0F2
CAN_ID_TX_DIAG = 0x0FF
CAN_ID_TX_TELEMETRY = 0x110

CAN_ID_TX_STATUS = CAN_ID_TX_CONTROL
CAN_ID_TX_CONFIG = CAN_ID_TX_CONTROL

CAN_FRAME_TYPE_TELEMETRY = 0x51
CAN_FRAME_TYPE_TELEMETRY_BATCH = 0x53
CAN_FRAME_TYPE_TELEMETRY_RAW_BATCH = 0x54
CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH = 0x55
CAN_FRAME_TYPE_COMMAND = 0xA0
CAN_FRAME_TYPE_STATUS = 0xA1
CAN_FRAME_TYPE_HEALTH = 0x52
CAN_FRAME_TYPE_CONFIG = 0x56
CAN_FRAME_TYPE_DIAG = 0x57
CAN_HEALTH_VERSION = 0x01
CAN_PROTOCOL_VERSION = 0x03

TELEMETRY_MODE_RAW = 0x00
TELEMETRY_MODE_PHYSICAL = 0x01

CAN_STATUS_OK = 0x00
CAN_STATUS_BAD_CRC = 0xE1
CAN_STATUS_BAD_TYPE = 0xE2
CAN_STATUS_BAD_CMD = 0xE3
CAN_STATUS_BAD_VALUE = 0xE4
CAN_STATUS_STORAGE_ERROR = 0xE5

COMMAND_NAMES = {
    0x01: "Set Sample Rate",
    0x02: "Set Filter Size",
    0x03: "Zero Datum",
    0x04: "Calibrate",
    0x05: "Save Zero",
    0x06: "Load Zero",
    0x07: "Clear Zero",
    0x08: "Set Channel Mask",
    0x09: "Set Telemetry Mode",
    0x0A: "Get Config",
    0x0B: "Set Vref",
    0x0C: "Set PGA",
    0x0D: "Restore Defaults",
    0x0E: "Set Zero Offset",
}

STATUS_NAMES = {
    CAN_STATUS_OK: "OK",
    CAN_STATUS_BAD_CRC: "CRC mismatch",
    CAN_STATUS_BAD_TYPE: "invalid frame type",
    CAN_STATUS_BAD_CMD: "unsupported command",
    CAN_STATUS_BAD_VALUE: "invalid value",
    CAN_STATUS_STORAGE_ERROR: "storage error",
}

DEFAULT_SLCAN_TTY_BAUDRATE = 2000000
DEFAULT_SLCAN_OPEN_DELAY = 2.0
DEFAULT_CAN_SEND_TIMEOUT_S = 0.2
CAN_FD_DATA_BITRATE = 2000000
CAN_TELEMETRY_BATCH_FRAME_LEN = 64
CAN_TELEMETRY_BATCH_MAX_RECORDS = 10
CAN_TELEMETRY_RECORD_LEN = 6
CAN_TELEMETRY_V2_HEADER_LEN = 8
CAN_TELEMETRY_RAW_RECORD_LEN = 4
CAN_TELEMETRY_RAW_MAX_RECORDS = 14
CAN_TELEMETRY_PHYSICAL_RECORD_LEN = 9
CAN_TELEMETRY_PHYSICAL_MAX_RECORDS = 6


class Baudrate(Enum):
    BAUD_125K = 125000
    BAUD_250K = 250000
    BAUD_500K = 500000
    BAUD_800K = 800000
    BAUD_1M = 1000000

    @property
    def bps(self) -> int:
        return self.value


@dataclass
class CANFrame:
    id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False
    is_fd: bool = True
    bitrate_switch: bool = True
    timestamp: Optional[float] = None


@dataclass
class CommandFrame:
    sequence: int
    cmd_type: int
    param: int
    value: int


@dataclass
class StatusFrame:
    sequence: int
    cmd_type: int
    status: int
    value: int
    detail: int


@dataclass
class TelemetryFrame:
    channel: int
    voltage_001mv: int = 0
    strain_ue: int = 0
    stress_01mpa: int = 0
    raw_value: Optional[int] = None
    voltage_uv: Optional[int] = None
    stress_qmpa: Optional[int] = None
    telemetry_mode: int = TELEMETRY_MODE_PHYSICAL


@dataclass
class HealthFrame:
    sample_rate_sps: float
    tx_drop_count: int
    adc_overflow_count: int
    adc_recovery_count: int
    active_adc_count: int
    adc_running: bool
    telemetry_decimation: int = 1
    telemetry_mode: int = TELEMETRY_MODE_RAW
    telemetry_samples_per_second: int = 0
    telemetry_frames_per_second: int = 0
    config_dirty: bool = False
    zero_valid: bool = False
    channel_mask_nonzero: bool = False


@dataclass
class DiagFrame:
    can_ready: bool
    main_loop_alive: bool
    last_rx_fd: bool
    last_rx_brs: bool
    bus_off: bool
    error_passive: bool
    last_rx_dlc: int
    last_reject_reason: int
    tx_error_count: int
    rx_error_count: int
    sequence: int


@dataclass
class ConfigFrame:
    saved: bool
    zero_valid: bool
    pga_gain: int
    filter_length: int
    telemetry_mode: int
    channel_mask: int
    sample_rate_x10: int
    vref_uv: int
    sequence: int
    zero_offsets: List[int]


@dataclass
class SlcanFdProbeResult:
    ok: bool
    commands: List[str]
    error: str = ""
    warning: str = ""


def crc8_xor(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
    return crc


def build_command_frame(sequence: int, cmd_type: int, param: int = 0, value: int = 0) -> CANFrame:
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must fit in uint8")
    if not 0 <= cmd_type <= 0xFF:
        raise ValueError("cmd_type must fit in uint8")
    if not 0 <= param <= 0xFF:
        raise ValueError("param must fit in uint8")
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("value must fit in uint32")

    payload = bytes([
        CAN_FRAME_TYPE_COMMAND,
        CAN_PROTOCOL_VERSION,
        sequence,
        cmd_type,
        param,
        0x00,
    ]) + value.to_bytes(4, "little") + b"\x00\x00"
    return CANFrame(id=CAN_ID_RX_COMMAND, data=payload)


def parse_status_frame(frame: CANFrame) -> Optional[StatusFrame]:
    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_STATUS:
        return None
    if len(frame.data) not in (8, 12):
        return None
    if frame.data[0] != CAN_FRAME_TYPE_STATUS:
        return None
    if len(frame.data) == 12 and frame.data[1] == CAN_PROTOCOL_VERSION:
        return StatusFrame(
            sequence=frame.data[2],
            cmd_type=frame.data[3],
            status=frame.data[4],
            value=int.from_bytes(frame.data[6:10], "little"),
            detail=frame.data[5],
        )
    if len(frame.data) == 8 and frame.data[1] == CAN_PROTOCOL_VERSION:
        return StatusFrame(
            sequence=frame.data[2],
            cmd_type=frame.data[3],
            status=frame.data[4],
            value=frame.data[5] | (frame.data[6] << 8),
            detail=frame.data[7],
        )
    if crc8_xor(frame.data[:7]) != frame.data[7]:
        return None
    return StatusFrame(
        sequence=frame.data[1],
        cmd_type=frame.data[2],
        status=frame.data[3],
        value=frame.data[4] | (frame.data[5] << 8),
        detail=frame.data[6],
    )


def parse_diag_frame(frame: CANFrame) -> Optional[DiagFrame]:
    if frame.is_extended or frame.is_remote:
        return None
    if frame.id != CAN_ID_TX_DIAG or len(frame.data) != 8:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_DIAG:
        return None
    if crc8_xor(frame.data[:7]) != frame.data[7]:
        return None

    flags = frame.data[1]
    return DiagFrame(
        can_ready=(flags & 0x01) != 0,
        main_loop_alive=(flags & 0x02) != 0,
        last_rx_fd=(flags & 0x04) != 0,
        last_rx_brs=(flags & 0x08) != 0,
        bus_off=(flags & 0x10) != 0,
        error_passive=(flags & 0x20) != 0,
        last_rx_dlc=frame.data[2],
        last_reject_reason=frame.data[3],
        tx_error_count=frame.data[4],
        rx_error_count=frame.data[5],
        sequence=frame.data[6],
    )


def _parse_legacy_telemetry_frame(frame: CANFrame) -> Optional[TelemetryFrame]:
    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_TELEMETRY or len(frame.data) != 8:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_TELEMETRY:
        return None
    if crc8_xor(frame.data[:7]) != frame.data[7]:
        return None
    _, channel, voltage_001mv, strain_ue, stress_01mpa, _ = struct.unpack(">BBhhbB", frame.data)
    return TelemetryFrame(
        channel=channel,
        voltage_001mv=voltage_001mv,
        strain_ue=strain_ue,
        stress_01mpa=stress_01mpa,
        telemetry_mode=TELEMETRY_MODE_PHYSICAL,
    )


def _parse_i24_be(data: bytes) -> int:
    value = int.from_bytes(data, "big", signed=False)
    if value & 0x800000:
        value -= 0x1000000
    return value


def _parse_v2_raw_telemetry_batch(frame: CANFrame) -> Optional[List[TelemetryFrame]]:
    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_TELEMETRY or len(frame.data) != CAN_TELEMETRY_BATCH_FRAME_LEN:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_TELEMETRY_RAW_BATCH:
        return None
    if frame.data[1] != CAN_PROTOCOL_VERSION or frame.data[2] != TELEMETRY_MODE_RAW:
        return None
    record_count = frame.data[4]
    if not 1 <= record_count <= CAN_TELEMETRY_RAW_MAX_RECORDS:
        return None

    records = []
    for index in range(record_count):
        offset = CAN_TELEMETRY_V2_HEADER_LEN + index * CAN_TELEMETRY_RAW_RECORD_LEN
        records.append(
            TelemetryFrame(
                channel=frame.data[offset],
                raw_value=_parse_i24_be(frame.data[offset + 1:offset + 4]),
                telemetry_mode=TELEMETRY_MODE_RAW,
            )
        )
    return records


def _parse_v2_physical_telemetry_batch(frame: CANFrame) -> Optional[List[TelemetryFrame]]:
    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_TELEMETRY or len(frame.data) != CAN_TELEMETRY_BATCH_FRAME_LEN:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH:
        return None
    if frame.data[1] != CAN_PROTOCOL_VERSION or frame.data[2] != TELEMETRY_MODE_PHYSICAL:
        return None
    record_count = frame.data[4]
    if not 1 <= record_count <= CAN_TELEMETRY_PHYSICAL_MAX_RECORDS:
        return None

    records = []
    for index in range(record_count):
        offset = CAN_TELEMETRY_V2_HEADER_LEN + index * CAN_TELEMETRY_PHYSICAL_RECORD_LEN
        channel, voltage_uv, strain_ue, stress_qmpa = struct.unpack(
            ">Bihh", frame.data[offset:offset + CAN_TELEMETRY_PHYSICAL_RECORD_LEN]
        )
        records.append(
            TelemetryFrame(
                channel=channel,
                voltage_001mv=int(round(voltage_uv / 10.0)),
                strain_ue=strain_ue,
                stress_01mpa=int(round(stress_qmpa * 2.5)),
                voltage_uv=voltage_uv,
                stress_qmpa=stress_qmpa,
                telemetry_mode=TELEMETRY_MODE_PHYSICAL,
            )
        )
    return records


def parse_telemetry_frames(frame: CANFrame) -> Optional[List[TelemetryFrame]]:
    v2_raw = _parse_v2_raw_telemetry_batch(frame)
    if v2_raw is not None:
        return v2_raw

    v2_physical = _parse_v2_physical_telemetry_batch(frame)
    if v2_physical is not None:
        return v2_physical

    legacy = _parse_legacy_telemetry_frame(frame)
    if legacy is not None:
        return [legacy]

    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_TELEMETRY or len(frame.data) != CAN_TELEMETRY_BATCH_FRAME_LEN:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_TELEMETRY_BATCH:
        return None
    record_count = frame.data[1]
    if not 1 <= record_count <= CAN_TELEMETRY_BATCH_MAX_RECORDS:
        return None
    if frame.data[62] != 0 or crc8_xor(frame.data[:63]) != frame.data[63]:
        return None

    records = []
    for index in range(record_count):
        offset = 2 + index * CAN_TELEMETRY_RECORD_LEN
        channel, voltage_001mv, strain_ue, stress_01mpa = struct.unpack(
            ">Bhhb", frame.data[offset:offset + CAN_TELEMETRY_RECORD_LEN]
        )
        records.append(
            TelemetryFrame(
                channel=channel,
                voltage_001mv=voltage_001mv,
                strain_ue=strain_ue,
                stress_01mpa=stress_01mpa,
                telemetry_mode=TELEMETRY_MODE_PHYSICAL,
            )
        )
    return records


def parse_telemetry_frame(frame: CANFrame) -> Optional[TelemetryFrame]:
    records = parse_telemetry_frames(frame)
    if records is None or len(records) != 1:
        return None
    return records[0]


def parse_health_frame(frame: CANFrame) -> Optional[HealthFrame]:
    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_HEALTH:
        return None

    if len(frame.data) == 24 and frame.data[0] == CAN_FRAME_TYPE_HEALTH and frame.data[1] == CAN_PROTOCOL_VERSION:
        sample_rate_x10 = int.from_bytes(frame.data[2:6], "little")
        tx_drop = int.from_bytes(frame.data[6:8], "little")
        overflow = int.from_bytes(frame.data[8:10], "little")
        recovery = int.from_bytes(frame.data[10:12], "little")
        telemetry_samples_per_second = int.from_bytes(frame.data[12:14], "little")
        telemetry_frames_per_second = int.from_bytes(frame.data[14:16], "little")
        flags = frame.data[18]
        return HealthFrame(
            sample_rate_sps=sample_rate_x10 / 10.0,
            tx_drop_count=tx_drop,
            adc_overflow_count=overflow,
            adc_recovery_count=recovery,
            active_adc_count=frame.data[16],
            adc_running=(flags & 0x01) != 0,
            telemetry_decimation=1,
            telemetry_mode=frame.data[17],
            telemetry_samples_per_second=telemetry_samples_per_second,
            telemetry_frames_per_second=telemetry_frames_per_second,
            config_dirty=(flags & 0x02) != 0,
            zero_valid=(flags & 0x04) != 0,
            channel_mask_nonzero=(flags & 0x08) != 0,
        )

    if len(frame.data) != 16:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_HEALTH or frame.data[1] != CAN_HEALTH_VERSION:
        return None
    if crc8_xor(frame.data[:15]) != frame.data[15]:
        return None

    sample_rate_x10, decimation, tx_drop, overflow, recovery = struct.unpack(
        "<IHHHH", frame.data[2:14]
    )
    flags = frame.data[14]
    return HealthFrame(
        sample_rate_sps=sample_rate_x10 / 10.0,
        telemetry_decimation=decimation,
        tx_drop_count=tx_drop,
        adc_overflow_count=overflow,
        adc_recovery_count=recovery,
        active_adc_count=(flags >> 4) & 0x0F,
        adc_running=(flags & 0x01) != 0,
    )


def parse_config_frame(frame: CANFrame) -> Optional[ConfigFrame]:
    if frame.is_extended or frame.is_remote or not frame.is_fd or not frame.bitrate_switch:
        return None
    if frame.id != CAN_ID_TX_CONFIG or len(frame.data) != 64:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_CONFIG or frame.data[1] != CAN_PROTOCOL_VERSION:
        return None
    flags = frame.data[2]
    return ConfigFrame(
        saved=(flags & 0x01) != 0,
        zero_valid=(flags & 0x02) != 0,
        pga_gain=frame.data[3],
        filter_length=frame.data[4],
        telemetry_mode=frame.data[5],
        channel_mask=int.from_bytes(frame.data[6:8], "little"),
        sample_rate_x10=int.from_bytes(frame.data[8:12], "little"),
        vref_uv=int.from_bytes(frame.data[12:16], "little"),
        sequence=int.from_bytes(frame.data[16:20], "little"),
        zero_offsets=[
            int.from_bytes(frame.data[20 + index * 4:24 + index * 4],
                           "little", signed=True)
            for index in range(8)
        ],
    )


class _PythonCanListener(can.Listener if can is not None else object):
    def __init__(self, callbacks: List[Callable[[CANFrame], None]]):
        super().__init__()
        self._callbacks = callbacks

    def __call__(self, msg) -> None:
        self.on_message_received(msg)

    def on_message_received(self, msg) -> None:
        frame = CANFrame(
            id=msg.arbitration_id,
            data=bytes(msg.data),
            is_extended=msg.is_extended_id,
            is_remote=msg.is_remote_frame,
            is_fd=msg.is_fd,
            bitrate_switch=msg.bitrate_switch,
            timestamp=msg.timestamp,
        )
        for callback in list(self._callbacks):
            try:
                callback(frame)
            except Exception as exc:  # pragma: no cover - defensive callback isolation
                logger.error("CAN callback error: %s", exc)


def _slcan_command_response(port, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    response = bytearray()
    while time.monotonic() < deadline:
        byte = port.read(1)
        if not byte:
            continue
        response.extend(byte)
        if byte in (b"\r", b"\a"):
            break
    return bytes(response)


def _write_slcan_probe_command(port, command: str, timeout_s: float) -> Tuple[bool, str, bool]:
    port.write(command.encode("ascii") + b"\r")
    port.flush()
    response = _slcan_command_response(port, timeout_s)
    if response == b"\a":
        return False, f"{command} returned SLCAN error", True
    if not response:
        return True, "", False
    if b"\a" in response:
        return False, f"{command} returned SLCAN error", True
    return True, "", True


def probe_slcan_fd_adapter(
    channel: str,
    tty_baudrate: int = DEFAULT_SLCAN_TTY_BAUDRATE,
    *,
    timeout_s: float = 0.2,
) -> SlcanFdProbeResult:
    if serial is None:
        return SlcanFdProbeResult(False, [], "pyserial is not installed")

    commands: List[str] = []
    silent_commands: List[str] = []
    port = None
    try:
        port = serial.serial_for_url(
            channel,
            baudrate=int(tty_baudrate),
            timeout=timeout_s,
        )
        for command in ("C", "S6", "Y2", "O"):
            commands.append(command)
            ok, error, acknowledged = _write_slcan_probe_command(
                port, command, timeout_s
            )
            if not ok:
                return SlcanFdProbeResult(False, commands, error)
            if not acknowledged:
                silent_commands.append(command)

        warning = (
            "silent SLCAN command response for " + ", ".join(silent_commands)
            if silent_commands else ""
        )
        return SlcanFdProbeResult(True, commands, "", warning)
    except Exception as exc:
        return SlcanFdProbeResult(False, commands, str(exc))
    finally:
        if port is not None:
            try:
                port.write(b"C\r")
                port.flush()
            except Exception:
                pass
            try:
                port.close()
            except Exception:
                pass


class PythonCANInterface:
    def __init__(self):
        self.bus = None
        self.notifier = None
        self.interface: Optional[str] = None
        self.channel: Optional[str] = None
        self.bitrate: Optional[Baudrate] = None
        self.tty_baudrate: Optional[int] = None
        self.last_error: Optional[str] = None
        self.last_slcan_probe: Optional[SlcanFdProbeResult] = None
        self._rx_callbacks: List[Callable[[CANFrame], None]] = []

    def connect(
        self,
        interface: str,
        channel: str,
        bitrate: Baudrate,
        *,
        tty_baudrate: int = DEFAULT_SLCAN_TTY_BAUDRATE,
        sleep_after_open: float = DEFAULT_SLCAN_OPEN_DELAY,
    ) -> bool:
        if can is None:
            raise RuntimeError("python-can is not installed")
        self.last_error = None
        self.last_slcan_probe = None

        normalized_channel = self._normalize_channel(interface, channel)
        if interface == "slcan":
            self.last_slcan_probe = probe_slcan_fd_adapter(
                normalized_channel, int(tty_baudrate)
            )
            if not self.last_slcan_probe.ok:
                self.last_error = self.last_slcan_probe.error
                logger.error(
                    "SLCAN FD preflight failed (%s): %s",
                    channel,
                    self.last_error,
                )
                return False
            if self.last_slcan_probe.warning:
                logger.warning(
                    "SLCAN FD preflight warning (%s): %s",
                    channel,
                    self.last_slcan_probe.warning,
                )

        kwargs = {
            "interface": interface,
            "channel": normalized_channel,
        }
        if interface == "socketcan":
            kwargs["fd"] = True
        elif interface == "slcan":
            kwargs["timing"] = self._can_fd_timing(bitrate)
            kwargs["tty_baudrate"] = int(tty_baudrate)
            kwargs["sleep_after_open"] = sleep_after_open
        elif interface == "pcan":
            kwargs["fd"] = True
            kwargs["timing"] = self._can_fd_timing(bitrate)
        else:
            kwargs["bitrate"] = bitrate.bps
            kwargs["data_bitrate"] = CAN_FD_DATA_BITRATE
            kwargs["fd"] = True

        try:
            self.bus = can.Bus(**kwargs)
            listener = _PythonCanListener(self._rx_callbacks)
            self.notifier = can.Notifier(self.bus, [listener])
            self.interface = interface
            self.channel = channel
            self.bitrate = bitrate
            self.tty_baudrate = int(tty_baudrate) if interface == "slcan" else None
            return True
        except Exception as exc:
            self.last_error = str(exc)
            logger.error("Failed to connect CAN bus (%s/%s): %s", interface, channel, exc)
            self.disconnect()
            return False

    def disconnect(self) -> None:
        if self.notifier is not None:
            try:
                self.notifier.stop()
            except Exception:
                pass
            self.notifier = None

        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

        self.interface = None
        self.channel = None
        self.bitrate = None
        self.tty_baudrate = None

    def send_frame(self, frame: CANFrame) -> bool:
        if can is None:
            raise RuntimeError("python-can is not installed")
        if self.bus is None:
            logger.error("CAN bus is not connected")
            return False

        try:
            message = can.Message(
                arbitration_id=frame.id,
                data=frame.data,
                is_extended_id=frame.is_extended,
                is_remote_frame=frame.is_remote,
                is_fd=frame.is_fd,
                bitrate_switch=frame.bitrate_switch,
            )
            self.bus.send(message, timeout=DEFAULT_CAN_SEND_TIMEOUT_S)
            return True
        except Exception as exc:
            logger.error("Failed to send CAN frame: %s", exc)
            return False

    def register_rx_callback(self, callback: Callable[[CANFrame], None]) -> None:
        self._rx_callbacks.append(callback)

    def unregister_rx_callback(self, callback: Callable[[CANFrame], None]) -> None:
        if callback in self._rx_callbacks:
            self._rx_callbacks.remove(callback)

    @staticmethod
    def _normalize_channel(interface: str, channel: str):
        if interface in ("ixxat", "vector") and str(channel).isdigit():
            return int(channel)
        return channel

    @staticmethod
    def _can_fd_timing(bitrate: Baudrate):
        return can.BitTimingFd.from_sample_point(
            f_clock=80000000,
            nom_bitrate=bitrate.bps,
            nom_sample_point=87.5,
            data_bitrate=CAN_FD_DATA_BITRATE,
            data_sample_point=87.5,
        )


def available_interfaces() -> List[Tuple[str, str]]:
    return [
        ("slcan", "CANable 2.0 SLCAN FD (serial USB-CAN)"),
        ("socketcan", "SocketCAN FD (Linux canX/vcanX)"),
        ("pcan", "PCAN FD (Windows PCAN-Basic)"),
        ("ixxat", "IXXAT FD (Windows VCI)"),
        ("vector", "Vector FD (Windows XL Driver)"),
    ]


def list_can_channels(interface: str) -> List[Tuple[str, str]]:
    if interface == "slcan":
        return _list_slcan_channels()
    if interface == "socketcan":
        return _list_socketcan_channels()
    if interface == "pcan":
        return [("PCAN_USBBUS1", "PCAN USB channel 1")]
    if interface in ("ixxat", "vector"):
        return [("0", "CAN channel 0")]
    return []


def _list_slcan_channels() -> List[Tuple[str, str]]:
    fallback_port = "COM1" if os.name == "nt" else "/dev/ttyUSB0"
    if list_ports is None:
        return [(fallback_port, fallback_port)]

    try:
        ports = list(list_ports.comports())
    except Exception:
        return [(fallback_port, fallback_port)]

    result = []
    for port in ports:
        device = getattr(port, "device", "") or fallback_port
        description = getattr(port, "description", "") or "Serial adapter"
        vid = getattr(port, "vid", None)
        pid = getattr(port, "pid", None)
        if vid is not None and pid is not None:
            label = f"{description} (VID:PID={vid:04X}:{pid:04X})"
        elif description and description != device:
            label = description
        else:
            label = device
        result.append((device, label))

    return result or [(fallback_port, fallback_port)]


def _list_socketcan_channels() -> List[Tuple[str, str]]:
    sys_class_net = "/sys/class/net"
    if os.path.isdir(sys_class_net):
        devices = []
        for name in sorted(os.listdir(sys_class_net)):
            if name.startswith("can") or name.startswith("vcan"):
                devices.append((name, f"{name}"))
        if devices:
            return devices
    return [("can0", "can0")]

