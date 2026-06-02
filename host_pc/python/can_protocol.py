"""
can_protocol.py - python-can transport and Reducer CAN protocol helpers.

Supports SLCAN serial adapters and keeps compatibility with other python-can
backends already used by this project.
"""

from __future__ import annotations

import logging
import os
import struct
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

logger = logging.getLogger(__name__)


CAN_ID_RX_COMMAND = 0x100
CAN_ID_TX_TELEMETRY = 0x101
CAN_ID_TX_STATUS = 0x102

CAN_FRAME_TYPE_TELEMETRY = 0x51
CAN_FRAME_TYPE_COMMAND = 0xA0
CAN_FRAME_TYPE_STATUS = 0xA1

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
}

STATUS_NAMES = {
    CAN_STATUS_OK: "OK",
    CAN_STATUS_BAD_CRC: "CRC mismatch",
    CAN_STATUS_BAD_TYPE: "invalid frame type",
    CAN_STATUS_BAD_CMD: "unsupported command",
    CAN_STATUS_BAD_VALUE: "invalid value",
    CAN_STATUS_STORAGE_ERROR: "storage error",
}

DEFAULT_SLCAN_TTY_BAUDRATE = 115200
DEFAULT_SLCAN_OPEN_DELAY = 2.0
DEFAULT_CAN_SEND_TIMEOUT_S = 0.2
SUPPORTED_SLCAN_SERIAL_BAUDRATES = (
    115200,
    230400,
    460800,
    921600,
    1000000,
    2000000,
)


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
    voltage_001mv: int
    strain_ue: int
    stress_01mpa: int


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
    if not 0 <= value <= 0xFFFF:
        raise ValueError("value must fit in uint16")

    payload = bytes([
        CAN_FRAME_TYPE_COMMAND,
        sequence,
        cmd_type,
        param,
        value & 0xFF,
        (value >> 8) & 0xFF,
        0x00,
    ])
    return CANFrame(id=CAN_ID_RX_COMMAND, data=payload + bytes([crc8_xor(payload)]))


def parse_status_frame(frame: CANFrame) -> Optional[StatusFrame]:
    if frame.is_extended or frame.is_remote:
        return None
    if frame.id != CAN_ID_TX_STATUS or len(frame.data) != 8:
        return None
    if frame.data[0] != CAN_FRAME_TYPE_STATUS:
        return None
    if crc8_xor(frame.data[:7]) != frame.data[7]:
        return None
    return StatusFrame(
        sequence=frame.data[1],
        cmd_type=frame.data[2],
        status=frame.data[3],
        value=frame.data[4] | (frame.data[5] << 8),
        detail=frame.data[6],
    )


def parse_telemetry_frame(frame: CANFrame) -> Optional[TelemetryFrame]:
    if frame.is_extended or frame.is_remote:
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
            timestamp=msg.timestamp,
        )
        for callback in list(self._callbacks):
            try:
                callback(frame)
            except Exception as exc:  # pragma: no cover - defensive callback isolation
                logger.error("CAN callback error: %s", exc)


class PythonCANInterface:
    def __init__(self):
        self.bus = None
        self.notifier = None
        self.interface: Optional[str] = None
        self.channel: Optional[str] = None
        self.bitrate: Optional[Baudrate] = None
        self.tty_baudrate: Optional[int] = None
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

        kwargs = {
            "interface": interface,
            "channel": self._normalize_channel(interface, channel),
        }
        if interface == "slcan":
            kwargs["bitrate"] = bitrate.bps
            kwargs["tty_baudrate"] = int(tty_baudrate)
            kwargs["sleep_after_open"] = sleep_after_open
        elif interface != "socketcan":
            kwargs["bitrate"] = bitrate.bps

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
        return channel


def available_interfaces() -> List[Tuple[str, str]]:
    return [
        ("slcan", "SLCAN (serial USB-CAN)"),
        ("socketcan", "SocketCAN (Linux canX/vcanX)"),
    ]


def list_can_channels(interface: str) -> List[Tuple[str, str]]:
    if interface == "slcan":
        return _list_slcan_channels()
    if interface == "socketcan":
        return _list_socketcan_channels()
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

