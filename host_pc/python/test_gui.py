"""
test_gui.py - Tests for the reducer monitor GUI and CAN protocol helpers.
"""

import csv
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QAbstractButton, QLabel
from PyQt6.QtCore import QEvent, QPointF

import reducer_monitor as reducer_monitor_module
from can_protocol import (
    CANFrame,
    ConfigFrame,
    CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH,
    CAN_FRAME_TYPE_TELEMETRY_RAW_BATCH,
    CAN_FRAME_TYPE_STATUS,
    CAN_FRAME_TYPE_TELEMETRY,
    CAN_FRAME_TYPE_TELEMETRY_BATCH,
    CAN_FRAME_TYPE_DIAG,
    CAN_FRAME_TYPE_HEALTH,
    CAN_HEALTH_VERSION,
    CAN_ID_RX_COMMAND,
    CAN_ID_TX_DIAG,
    CAN_ID_TX_HEALTH,
    CAN_ID_TX_CONFIG,
    CAN_ID_TX_STATUS,
    CAN_ID_TX_TELEMETRY,
    CAN_STATUS_BAD_VALUE,
    CAN_STATUS_OK,
    CAN_STATUS_STORAGE_ERROR,
    CAN_FD_DATA_BITRATE,
    CAN_PROTOCOL_VERSION,
    DEFAULT_CAN_SEND_TIMEOUT_S,
    DEFAULT_SLCAN_TTY_BAUDRATE,
    TELEMETRY_MODE_PHYSICAL,
    TELEMETRY_MODE_RAW,
    Baudrate,
    PythonCANInterface,
    _PythonCanListener,
    available_interfaces,
    build_command_frame,
    crc8_xor,
    list_can_channels,
    parse_diag_frame,
    parse_status_frame,
    parse_telemetry_frame,
    parse_telemetry_frames,
    parse_health_frame,
    parse_config_frame,
    probe_slcan_fd_adapter,
)
from reducer_monitor import (
    CANReceiver,
    CAN_CMD_CLEAR_ZERO,
    CAN_CMD_LOAD_ZERO,
    CAN_CMD_SAVE_ZERO,
    CAN_CMD_SET_CHANNEL_MASK,
    CAN_CMD_SET_SAMPLE_RATE,
    CAN_CMD_SET_FILTER_SIZE,
    CAN_CMD_SET_ZERO_OFFSET,
    CAN_CMD_SET_TELEMETRY_MODE,
    CAN_CMD_GET_CONFIG,
    CAN_CMD_START_CALIB,
    CAN_CMD_ZERO_DATUM,
    DEFAULT_VISIBLE_PLOTS,
    FIXED_CAN_BITRATE,
    MAX_VISIBLE_PLOTS,
    NUM_CHANNELS,
    SUPPORTED_SAMPLE_RATES,
    ChannelData,
    OfflineWaveformWindow,
    ReducerMonitorWindow,
)


def build_telemetry_payload(channel: int, voltage_001mv: int, strain_ue: int, stress_01mpa: int) -> bytes:
    payload = bytes([
        CAN_FRAME_TYPE_TELEMETRY,
        channel,
        (voltage_001mv >> 8) & 0xFF,
        voltage_001mv & 0xFF,
        (strain_ue >> 8) & 0xFF,
        strain_ue & 0xFF,
        stress_01mpa & 0xFF,
    ])
    return payload + bytes([crc8_xor(payload)])


def build_status_payload(sequence: int, cmd_type: int, status: int, value: int, detail: int = 0) -> bytes:
    payload = bytes([
        CAN_FRAME_TYPE_STATUS,
        sequence & 0xFF,
        cmd_type & 0xFF,
        status & 0xFF,
        value & 0xFF,
        (value >> 8) & 0xFF,
        detail & 0xFF,
    ])
    return payload + bytes([crc8_xor(payload)])


def build_telemetry_batch_payload(records) -> bytes:
    payload = bytearray(64)
    payload[0] = CAN_FRAME_TYPE_TELEMETRY_BATCH
    payload[1] = len(records)
    for index, (channel, voltage_001mv, strain_ue, stress_01mpa) in enumerate(records):
        struct.pack_into(
            ">Bhhb", payload, 2 + index * 6,
            channel, voltage_001mv, strain_ue, stress_01mpa,
        )
    payload[63] = crc8_xor(payload[:63])
    return bytes(payload)


def build_health_payload(
    sample_rate_x10: int = 300000,
    decimation: int = 3,
    tx_drop: int = 4,
    overflow: int = 5,
    recovery: int = 6,
    active_adc_count: int = 2,
    running: bool = True,
) -> bytes:
    flags = ((active_adc_count & 0x0F) << 4) | (1 if running else 0)
    payload = (
        bytes([CAN_FRAME_TYPE_HEALTH, CAN_HEALTH_VERSION])
        + sample_rate_x10.to_bytes(4, "little")
        + decimation.to_bytes(2, "little")
        + tx_drop.to_bytes(2, "little")
        + overflow.to_bytes(2, "little")
        + recovery.to_bytes(2, "little")
        + bytes([flags])
    )
    return payload + bytes([crc8_xor(payload)])


def build_raw_v2_payload(records, sequence: int = 0x42, drop_delta: int = 3) -> bytes:
    payload = bytearray(64)
    payload[0] = CAN_FRAME_TYPE_TELEMETRY_RAW_BATCH
    payload[1] = CAN_PROTOCOL_VERSION
    payload[2] = TELEMETRY_MODE_RAW
    payload[3] = sequence
    payload[4] = len(records)
    payload[5:7] = drop_delta.to_bytes(2, "little")
    for index, (channel, raw_value) in enumerate(records):
        offset = 8 + index * 4
        payload[offset] = channel & 0xFF
        payload[offset + 1:offset + 4] = int(raw_value).to_bytes(3, "big", signed=True)
    return bytes(payload)


def build_physical_v2_payload(records, sequence: int = 0x43, drop_delta: int = 4) -> bytes:
    payload = bytearray(64)
    payload[0] = CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH
    payload[1] = CAN_PROTOCOL_VERSION
    payload[2] = TELEMETRY_MODE_PHYSICAL
    payload[3] = sequence
    payload[4] = len(records)
    payload[5:7] = drop_delta.to_bytes(2, "little")
    for index, (channel, voltage_001mv, strain_ue, stress_01mpa) in enumerate(records):
        struct.pack_into(
            ">Bihh", payload, 8 + index * 9,
            channel, voltage_001mv * 10, strain_ue, round(stress_01mpa / 2.5),
        )
    return bytes(payload)


def build_config_v3_payload() -> bytes:
    payload = bytearray(64)
    payload[0] = 0x56
    payload[1] = CAN_PROTOCOL_VERSION
    payload[2] = 0x03
    payload[3] = 16
    payload[4] = 7
    payload[5] = TELEMETRY_MODE_RAW
    payload[6:8] = (0x000F).to_bytes(2, "little")
    payload[8:12] = (10000).to_bytes(4, "little")
    payload[12:16] = (2498500).to_bytes(4, "little")
    payload[16:20] = (42).to_bytes(4, "little")
    for channel in range(8):
        payload[20 + channel * 4:24 + channel * 4] = (
            channel * -100
        ).to_bytes(4, "little", signed=True)
    return bytes(payload)


def build_health_v2_payload(
    sample_rate_x10: int = 300000,
    tx_drop: int = 4,
    overflow: int = 5,
    recovery: int = 6,
    telemetry_samples_per_second: int = 8748,
    telemetry_frames_per_second: int = 625,
    active_adc_count: int = 2,
    mode: int = TELEMETRY_MODE_RAW,
    running: bool = True,
) -> bytes:
    payload = bytearray(24)
    payload[0] = CAN_FRAME_TYPE_HEALTH
    payload[1] = CAN_PROTOCOL_VERSION
    payload[2:6] = sample_rate_x10.to_bytes(4, "little")
    payload[6:8] = tx_drop.to_bytes(2, "little")
    payload[8:10] = overflow.to_bytes(2, "little")
    payload[10:12] = recovery.to_bytes(2, "little")
    payload[12:14] = telemetry_samples_per_second.to_bytes(2, "little")
    payload[14:16] = telemetry_frames_per_second.to_bytes(2, "little")
    payload[16] = active_adc_count
    payload[17] = mode
    payload[18] = 1 if running else 0
    return bytes(payload)


def build_diag_payload(
    flags: int = 0x3F,
    last_rx_dlc: int = 12,
    reject_reason: int = 4,
    tx_error_count: int = 5,
    rx_error_count: int = 6,
    sequence: int = 7,
) -> bytes:
    payload = bytes([
        CAN_FRAME_TYPE_DIAG,
        flags & 0xFF,
        last_rx_dlc & 0xFF,
        reject_reason & 0xFF,
        tx_error_count & 0xFF,
        rx_error_count & 0xFF,
        sequence & 0xFF,
    ])
    return payload + bytes([crc8_xor(payload)])


class TestProtocolHelpers(unittest.TestCase):
    def test_v3_command_and_status_use_uint32_value(self):
        frame = build_command_frame(
            sequence=0x12, cmd_type=0x0B, param=0, value=2_498_500
        )
        self.assertEqual(len(frame.data), 12)
        self.assertEqual(int.from_bytes(frame.data[6:10], "little"), 2_498_500)

        payload = bytearray(12)
        payload[0:6] = bytes([
            CAN_FRAME_TYPE_STATUS, CAN_PROTOCOL_VERSION, 0x12, 0x0B,
            CAN_STATUS_OK, 0,
        ])
        payload[6:10] = (2_498_500).to_bytes(4, "little")
        parsed = parse_status_frame(CANFrame(id=CAN_ID_TX_STATUS, data=bytes(payload)))
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.value, 2_498_500)

    def test_status_ack_id_has_higher_priority_than_command_and_telemetry(self):
        self.assertEqual(CAN_ID_TX_STATUS, 0x0F0)
        self.assertLess(CAN_ID_TX_STATUS, CAN_ID_RX_COMMAND)
        self.assertLess(CAN_ID_TX_STATUS, CAN_ID_TX_TELEMETRY)

    def test_diag_id_can_preempt_fd_health_config_without_preempting_ack(self):
        self.assertEqual(CAN_ID_TX_DIAG, 0x0FF)
        self.assertGreater(CAN_ID_TX_DIAG, CAN_ID_TX_STATUS)
        self.assertLess(CAN_ID_TX_DIAG, CAN_ID_RX_COMMAND)
        self.assertLess(CAN_ID_TX_DIAG, CAN_ID_TX_TELEMETRY)
        self.assertLess(CAN_ID_TX_DIAG, CAN_ID_TX_HEALTH)
        self.assertLess(CAN_ID_TX_DIAG, CAN_ID_TX_CONFIG)

    def test_legacy_status_ack_id_is_not_accepted(self):
        payload = bytes([
            CAN_FRAME_TYPE_STATUS, CAN_PROTOCOL_VERSION, 3, CAN_CMD_GET_CONFIG,
            CAN_STATUS_OK, 0, 0, 0,
        ])

        self.assertIsNone(parse_status_frame(CANFrame(id=0x102, data=payload)))

    def test_parse_classic_can_diag_frame(self):
        parsed = parse_diag_frame(CANFrame(
            id=CAN_ID_TX_DIAG,
            data=build_diag_payload(),
            is_fd=False,
            bitrate_switch=False,
        ))

        self.assertIsNotNone(parsed)
        self.assertTrue(parsed.can_ready)
        self.assertTrue(parsed.main_loop_alive)
        self.assertTrue(parsed.last_rx_fd)
        self.assertTrue(parsed.last_rx_brs)
        self.assertTrue(parsed.bus_off)
        self.assertTrue(parsed.error_passive)
        self.assertEqual(parsed.last_rx_dlc, 12)
        self.assertEqual(parsed.last_reject_reason, 4)
        self.assertEqual(parsed.tx_error_count, 5)
        self.assertEqual(parsed.rx_error_count, 6)
        self.assertEqual(parsed.sequence, 7)

    def test_parse_diag_frame_rejects_bad_crc(self):
        payload = bytearray(build_diag_payload())
        payload[7] ^= 0xFF

        self.assertIsNone(parse_diag_frame(CANFrame(
            id=CAN_ID_TX_DIAG,
            data=bytes(payload),
            is_fd=False,
            bitrate_switch=False,
        )))

    def test_parse_v3_physical_telemetry_uses_wide_units(self):
        payload = bytearray(64)
        payload[0] = CAN_FRAME_TYPE_TELEMETRY_PHYSICAL_BATCH
        payload[1] = CAN_PROTOCOL_VERSION
        payload[2] = TELEMETRY_MODE_PHYSICAL
        payload[4] = 1
        struct.pack_into(">Bihh", payload, 8, 2, -1_234_567, -456, -16800)

        parsed = parse_telemetry_frames(
            CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed[0].voltage_uv, -1_234_567)
        self.assertEqual(parsed[0].stress_qmpa, -16800)

    def test_parse_v3_config_snapshot(self):
        parsed = parse_config_frame(
            CANFrame(id=CAN_ID_TX_CONFIG, data=build_config_v3_payload())
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.vref_uv, 2_498_500)
        self.assertEqual(parsed.pga_gain, 16)
        self.assertEqual(parsed.filter_length, 7)
        self.assertEqual(parsed.channel_mask, 0x000F)
        self.assertEqual(parsed.zero_offsets[3], -300)

    def test_crc8_xor_basic(self):
        self.assertEqual(crc8_xor(bytes([1, 2, 3])), 0x00)

    def test_build_command_frame_layout(self):
        frame = build_command_frame(sequence=0x12, cmd_type=CAN_CMD_SET_FILTER_SIZE, param=0x34, value=0x5678)

        self.assertEqual(frame.id, CAN_ID_RX_COMMAND)
        self.assertEqual(frame.data[0], 0xA0)
        self.assertEqual(frame.data[1], CAN_PROTOCOL_VERSION)
        self.assertEqual(frame.data[2], 0x12)
        self.assertEqual(frame.data[3], CAN_CMD_SET_FILTER_SIZE)
        self.assertEqual(frame.data[4], 0x34)
        self.assertEqual(len(frame.data), 12)
        self.assertEqual(int.from_bytes(frame.data[6:10], "little"), 0x5678)

    def test_build_telemetry_mode_command_frame(self):
        frame = build_command_frame(
            sequence=0x34,
            cmd_type=CAN_CMD_SET_TELEMETRY_MODE,
            param=0,
            value=TELEMETRY_MODE_RAW,
        )

        self.assertEqual(frame.data[:6], bytes([
            0xA0, CAN_PROTOCOL_VERSION, 0x34, CAN_CMD_SET_TELEMETRY_MODE,
            0x00, 0x00,
        ]))
        self.assertEqual(int.from_bytes(frame.data[6:10], "little"), TELEMETRY_MODE_RAW)

    def test_parse_telemetry_frame(self):
        payload = build_telemetry_payload(channel=2, voltage_001mv=1234, strain_ue=-456, stress_01mpa=-12)
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=payload)

        parsed = parse_telemetry_frame(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.channel, 2)
        self.assertEqual(parsed.voltage_001mv, 1234)
        self.assertEqual(parsed.strain_ue, -456)
        self.assertEqual(parsed.stress_01mpa, -12)

    def test_parse_batched_telemetry_frame(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_telemetry_batch_payload([
                (0, 1234, -456, -12),
                (1, 5678, 901, 34),
            ]),
        )

        parsed = parse_telemetry_frames(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(len(parsed), 2)
        self.assertEqual(parsed[0].channel, 0)
        self.assertEqual(parsed[0].voltage_001mv, 1234)
        self.assertEqual(parsed[1].channel, 1)
        self.assertEqual(parsed[1].strain_ue, 901)

    def test_parse_v2_raw_telemetry_batch(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_raw_v2_payload([
                (0, 0x123456),
                (1, -2),
            ]),
        )

        parsed = parse_telemetry_frames(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(len(parsed), 2)
        self.assertEqual(parsed[0].channel, 0)
        self.assertEqual(parsed[0].raw_value, 0x123456)
        self.assertEqual(parsed[0].telemetry_mode, TELEMETRY_MODE_RAW)
        self.assertEqual(parsed[1].channel, 1)
        self.assertEqual(parsed[1].raw_value, -2)

    def test_parse_v2_raw_telemetry_rejects_bad_version_or_count(self):
        payload = bytearray(build_raw_v2_payload([(0, 1)]))
        payload[1] = 0x01
        self.assertIsNone(parse_telemetry_frames(CANFrame(
            id=CAN_ID_TX_TELEMETRY, data=bytes(payload)
        )))

        payload = bytearray(build_raw_v2_payload([(0, 1)]))
        payload[4] = 15
        self.assertIsNone(parse_telemetry_frames(CANFrame(
            id=CAN_ID_TX_TELEMETRY, data=bytes(payload)
        )))

    def test_parse_v2_physical_telemetry_batch(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_physical_v2_payload([
                (2, 1234, -456, -12),
            ]),
        )

        parsed = parse_telemetry_frames(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].channel, 2)
        self.assertEqual(parsed[0].voltage_001mv, 1234)
        self.assertEqual(parsed[0].strain_ue, -456)
        self.assertEqual(parsed[0].stress_01mpa, -12)
        self.assertEqual(parsed[0].telemetry_mode, TELEMETRY_MODE_PHYSICAL)

    def test_parse_rejects_batched_telemetry_with_bad_crc(self):
        payload = bytearray(build_telemetry_batch_payload([(0, 1234, -456, -12)]))
        payload[-1] ^= 0xFF

        self.assertIsNone(parse_telemetry_frames(
            CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))
        ))

    def test_parse_status_frame(self):
        payload = bytes([
            CAN_FRAME_TYPE_STATUS, CAN_PROTOCOL_VERSION, 3, CAN_CMD_ZERO_DATUM,
            CAN_STATUS_OK, 16, 0, 0x55,
        ])
        frame = CANFrame(id=CAN_ID_TX_STATUS, data=payload)

        parsed = parse_status_frame(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.sequence, 3)
        self.assertEqual(parsed.cmd_type, CAN_CMD_ZERO_DATUM)
        self.assertEqual(parsed.status, CAN_STATUS_OK)
        self.assertEqual(parsed.value, 16)
        self.assertEqual(parsed.detail, 0x55)

    def test_parse_health_frame(self):
        parsed = parse_health_frame(
            CANFrame(id=CAN_ID_TX_HEALTH, data=build_health_v2_payload())
        )

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.sample_rate_sps, 30000)
        self.assertEqual(parsed.tx_drop_count, 4)
        self.assertEqual(parsed.adc_overflow_count, 5)
        self.assertEqual(parsed.adc_recovery_count, 6)
        self.assertEqual(parsed.active_adc_count, 2)
        self.assertEqual(parsed.telemetry_mode, TELEMETRY_MODE_RAW)
        self.assertEqual(parsed.telemetry_samples_per_second, 8748)
        self.assertEqual(parsed.telemetry_frames_per_second, 625)
        self.assertTrue(parsed.adc_running)

    def test_parse_rejects_bad_crc(self):
        payload = bytearray(build_telemetry_payload(channel=0, voltage_001mv=100, strain_ue=50, stress_01mpa=5))
        payload[-1] ^= 0xFF
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))

        self.assertIsNone(parse_telemetry_frame(frame))

    def test_parse_rejects_wrong_protocol_length(self):
        payload = build_telemetry_payload(channel=0, voltage_001mv=100, strain_ue=50, stress_01mpa=5)

        self.assertIsNone(parse_telemetry_frame(CANFrame(id=CAN_ID_TX_TELEMETRY, data=payload + b"\x00")))
        self.assertIsNone(parse_status_frame(CANFrame(id=CAN_ID_TX_STATUS, data=payload[:7])))

    def test_parse_rejects_extended_and_remote_frames(self):
        telemetry = build_telemetry_payload(
            channel=0, voltage_001mv=100, strain_ue=50, stress_01mpa=5
        )
        status = build_status_payload(
            sequence=1, cmd_type=CAN_CMD_ZERO_DATUM, status=CAN_STATUS_OK, value=0
        )

        self.assertIsNone(parse_telemetry_frame(
            CANFrame(id=CAN_ID_TX_TELEMETRY, data=telemetry, is_extended=True)
        ))
        self.assertIsNone(parse_status_frame(
            CANFrame(id=CAN_ID_TX_STATUS, data=status, is_remote=True)
        ))

    def test_available_interfaces_prioritizes_slcan(self):
        interfaces = available_interfaces()

        self.assertGreaterEqual(len(interfaces), 1)
        self.assertEqual(interfaces[0][0], "slcan")

    def test_list_slcan_channels_uses_serial_ports(self):
        fake_port = MagicMock()
        fake_port.device = "COM9"
        fake_port.description = "USB-CAN Adapter"
        fake_port.vid = 0x1D50
        fake_port.pid = 0x606F

        with patch("can_protocol.list_ports.comports", return_value=[fake_port]):
            channels = list_can_channels("slcan")

        self.assertEqual(channels, [("COM9", "USB-CAN Adapter (VID:PID=1D50:606F)")])

    def test_python_can_interface_connect_slcan_passes_canable2_fd_settings(self):
        bus_instance = MagicMock()
        notifier_instance = MagicMock()

        with patch("can_protocol.can.Bus", return_value=bus_instance) as mock_bus, patch(
            "can_protocol.can.Notifier",
            return_value=notifier_instance,
        ), patch("can_protocol.serial.serial_for_url") as mock_serial:
            interface = PythonCANInterface()
            connected = interface.connect(
                "slcan",
                "COM7",
                Baudrate.BAUD_500K,
                tty_baudrate=460800,
                sleep_after_open=0.25,
            )

        self.assertTrue(connected)
        kwargs = mock_bus.call_args.kwargs
        self.assertEqual(kwargs["interface"], "slcan")
        self.assertEqual(kwargs["channel"], "COM7")
        self.assertEqual(kwargs["timing"].nom_bitrate, 500000)
        self.assertEqual(kwargs["timing"].data_bitrate, CAN_FD_DATA_BITRATE)
        self.assertEqual(kwargs["tty_baudrate"], 460800)
        self.assertEqual(kwargs["sleep_after_open"], 0.25)
        self.assertEqual(interface.tty_baudrate, 460800)
        self.assertIsNone(interface.last_error)
        mock_serial.assert_not_called()

    def test_slcan_fd_preflight_rejects_adapter_without_y2(self):
        serial_port = MagicMock()
        serial_port.read.side_effect = [b"\r", b"\r", b"\a"]

        with patch(
            "can_protocol.serial.serial_for_url",
            return_value=serial_port,
        ):
            result = probe_slcan_fd_adapter("COM7", 115200)

        self.assertFalse(result.ok)
        self.assertIn("Y2", result.error)
        self.assertEqual(result.commands, ["C", "S6", "Y2"])

    def test_slcan_fd_preflight_allows_silent_command_ack(self):
        serial_port = MagicMock()
        serial_port.read.return_value = b""

        with patch(
            "can_protocol.serial.serial_for_url",
            return_value=serial_port,
        ):
            result = probe_slcan_fd_adapter("COM7", 115200, timeout_s=0)

        self.assertTrue(result.ok)
        self.assertEqual(result.error, "")
        self.assertIn("silent", result.warning)
        self.assertEqual(result.commands[:4], ["C", "S6", "Y2", "O"])

    def test_slcan_fd_preflight_does_not_transmit_bus_probe_frame(self):
        serial_port = MagicMock()
        serial_port.read.side_effect = [b"\r", b"\r", b"\r", b"\r"]

        with patch(
            "can_protocol.serial.serial_for_url",
            return_value=serial_port,
        ):
            result = probe_slcan_fd_adapter("COM7", 115200)

        writes = [call.args[0] for call in serial_port.write.call_args_list]
        self.assertTrue(result.ok)
        self.assertEqual(result.commands, ["C", "S6", "Y2", "O"])
        self.assertIn(b"S6\r", writes)
        self.assertIn(b"Y2\r", writes)
        self.assertFalse(any(write.startswith((b"b", b"B", b"d", b"D")) for write in writes))

    def test_upstream_slcan_bus_writes_canable2_fd_commands_and_brs_frame(self):
        import can as python_can
        from can.interfaces.slcan import slcanBus

        serial_port = MagicMock()
        serial_port.write_timeout = None
        timing = PythonCANInterface._can_fd_timing(Baudrate.BAUD_500K)

        with patch(
            "can.interfaces.slcan.serial.serial_for_url",
            return_value=serial_port,
        ):
            bus = slcanBus(
                channel="COM7",
                timing=timing,
                tty_baudrate=DEFAULT_SLCAN_TTY_BAUDRATE,
                sleep_after_open=0,
            )
            bus.send(
                python_can.Message(
                    arbitration_id=0x100,
                    data=bytes([0xA0, 1, 2, 3, 4, 5, 6, 7]),
                    is_extended_id=False,
                    is_fd=True,
                    bitrate_switch=True,
                )
            )
            batch_data = bytes(range(64))
            bus.send(
                python_can.Message(
                    arbitration_id=0x101,
                    data=batch_data,
                    is_extended_id=False,
                    is_fd=True,
                    bitrate_switch=True,
                )
            )
            bus.shutdown()

        serial_writes = [call.args[0] for call in serial_port.write.call_args_list]
        self.assertEqual(serial_writes[:4], [b"C\r", b"S6\r", b"Y2\r", b"O\r"])
        self.assertIn(b"b1008A001020304050607\r", serial_writes)
        self.assertIn(b"b101F" + batch_data.hex().upper().encode() + b"\r", serial_writes)

    def test_default_slcan_tty_baudrate_covers_worst_case_fd_ascii_load(self):
        max_fd_frames_per_second = 875
        slcan_64_byte_fd_brs_chars = len("b101F" + ("00" * 64) + "\r")
        required_bps = max_fd_frames_per_second * slcan_64_byte_fd_brs_chars * 10

        self.assertGreaterEqual(DEFAULT_SLCAN_TTY_BAUDRATE, required_bps)

    def test_python_can_listener_is_callable_by_notifier(self):
        received = []
        listener = _PythonCanListener([received.append])
        msg = MagicMock()
        msg.arbitration_id = 0x123
        msg.data = bytes([1, 2, 3])
        msg.is_extended_id = False
        msg.is_remote_frame = False
        msg.is_fd = True
        msg.bitrate_switch = True
        msg.timestamp = 1.25

        listener(msg)

        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].id, 0x123)
        self.assertEqual(received[0].data, bytes([1, 2, 3]))
        self.assertTrue(received[0].is_fd)
        self.assertTrue(received[0].bitrate_switch)

    def test_python_can_send_uses_bounded_timeout(self):
        bus_instance = MagicMock()
        message = MagicMock()
        interface = PythonCANInterface()
        interface.bus = bus_instance

        with patch("can_protocol.can.Message", return_value=message):
            sent = interface.send_frame(CANFrame(id=0x123, data=b"\x01\x02"))

        self.assertTrue(sent)
        bus_instance.send.assert_called_once_with(
            message, timeout=DEFAULT_CAN_SEND_TIMEOUT_S
        )


class TestChannelData(unittest.TestCase):
    def test_default_values(self):
        data = ChannelData()
        self.assertEqual(data.voltage_mv, 0.0)
        self.assertEqual(data.strain_ue, 0.0)
        self.assertEqual(data.stress_mpa, 0.0)
        self.assertEqual(data.voltage_001mv, 0)
        self.assertEqual(data.stress_01mpa, 0)
        self.assertEqual(data.samples, 0)


class TestCANReceiver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_registers_callback_before_thread_start(self):
        can_bus = MagicMock()

        receiver = CANReceiver(can_bus)

        can_bus.register_rx_callback.assert_called_once()
        receiver.stop()

    def test_queues_control_frames_before_telemetry_frames(self):
        can_bus = MagicMock()
        receiver = CANReceiver(can_bus)
        callback = can_bus.register_rx_callback.call_args.args[0]
        telemetry_frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_raw_v2_payload([(0, 1)]),
        )
        status_frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(
                sequence=7,
                cmd_type=CAN_CMD_SET_FILTER_SIZE,
                status=CAN_STATUS_OK,
                value=32,
            ),
        )

        callback(telemetry_frame)
        callback(status_frame)

        control_frames, telemetry_frames = receiver.take_pending_frames()
        self.assertEqual(control_frames, [status_frame])
        self.assertEqual(telemetry_frames, [telemetry_frame])
        receiver.stop()

    def test_control_frame_emits_signal_even_with_telemetry_backlog(self):
        can_bus = MagicMock()
        receiver = CANReceiver(can_bus)
        callback = can_bus.register_rx_callback.call_args.args[0]
        emitted = []
        receiver.frames_available.connect(lambda: emitted.append(True))
        telemetry_frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_raw_v2_payload([(0, 1)]),
        )
        status_frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(
                sequence=7,
                cmd_type=CAN_CMD_SET_FILTER_SIZE,
                status=CAN_STATUS_OK,
                value=32,
            ),
        )

        callback(telemetry_frame)
        receiver.take_pending_frames(telemetry_limit=0)
        emitted.clear()
        callback(status_frame)

        self.assertEqual(len(emitted), 1)
        control_frames, telemetry_frames = receiver.take_pending_frames(
            telemetry_limit=0
        )
        self.assertEqual(control_frames, [status_frame])
        self.assertEqual(telemetry_frames, [])
        receiver.stop()


class TestFirmwareSource(unittest.TestCase):
    def test_channel_mask_command_requests_persistent_save(self):
        user_c = Path(__file__).resolve().parents[2] / "Application" / "user.c"
        source = user_c.read_text(encoding="utf-8")
        branch = source.split("case CAN_CMD_SET_CHANNEL_MASK:", 1)[1].split(
            "case CAN_CMD_SET_FILTER_SIZE:", 1
        )[0]

        self.assertIn("request_config_save(false);", branch)

    def test_set_zero_offset_command_updates_runtime_and_flash_config(self):
        root = Path(__file__).resolve().parents[2]
        can_header = (root / "Application" / "algorithm" / "can_data.h").read_text(
            encoding="utf-8"
        )
        user_source = (root / "Application" / "user.c").read_text(encoding="utf-8")
        branch = user_source.split("case CAN_CMD_SET_ZERO_OFFSET:", 1)[1].split(
            "case CAN_CMD_SET_CHANNEL_MASK:", 1
        )[0]

        self.assertIn("#define CAN_CMD_SET_ZERO_OFFSET", can_header)
        self.assertIn("filter_set_zero_offset(param, (int32_t)value);", branch)
        self.assertIn("PERSISTENT_CONFIG_FLAG_ZERO_VALID", branch)
        self.assertIn("request_config_save(false);", branch)

    def test_fdcan_tx_queue_mode_allows_ack_id_priority(self):
        root = Path(__file__).resolve().parents[2]
        fdcan_source = (root / "Core" / "Src" / "fdcan.c").read_text(
            encoding="utf-8"
        )
        ioc_source = (root / "Reducer.ioc").read_text(encoding="utf-8")

        self.assertIn("FDCAN_TX_QUEUE_OPERATION", fdcan_source)
        self.assertIn("FDCAN1.TxFifoQueueMode=FDCAN_TX_QUEUE_OPERATION", ioc_source)

    def test_fdcan_keeps_tx_delay_compensation_disabled_at_2m(self):
        root = Path(__file__).resolve().parents[2]
        fdcan_source = (root / "Core" / "Src" / "fdcan.c").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("HAL_FDCAN_ConfigTxDelayCompensation", fdcan_source)
        self.assertNotIn("HAL_FDCAN_EnableTxDelayCompensation", fdcan_source)

    def test_fdcan_gpio_speed_is_not_left_at_low_for_canfd(self):
        root = Path(__file__).resolve().parents[2]
        fdcan_source = (root / "Core" / "Src" / "fdcan.c").read_text(
            encoding="utf-8"
        )
        ioc_source = (root / "Reducer.ioc").read_text(encoding="utf-8")

        fdcan_gpio_block = fdcan_source.split("PA11     ------> FDCAN1_RX", 1)[
            1
        ].split("HAL_GPIO_Init(GPIOA", 1)[0]

        self.assertNotIn("GPIO_SPEED_FREQ_LOW", fdcan_gpio_block)
        self.assertRegex(
            fdcan_gpio_block,
            r"GPIO_InitStruct\.Speed = GPIO_SPEED_FREQ_(HIGH|VERY_HIGH);",
        )
        self.assertIn("PA11.GPIOParameters=GPIO_Speed", ioc_source)
        self.assertIn("PA11.GPIO_Speed=GPIO_SPEED_FREQ_VERY_HIGH", ioc_source)
        self.assertIn("PA12.GPIOParameters=GPIO_Speed", ioc_source)
        self.assertIn("PA12.GPIO_Speed=GPIO_SPEED_FREQ_VERY_HIGH", ioc_source)

    def test_firmware_emits_classic_diag_and_records_reject_reasons(self):
        root = Path(__file__).resolve().parents[2]
        can_header = (root / "Application" / "algorithm" / "can_data.h").read_text(
            encoding="utf-8"
        )
        can_source = (root / "BSP" / "can.c").read_text(encoding="utf-8")
        can_api = (root / "BSP" / "can.h").read_text(encoding="utf-8")
        user_source = (root / "Application" / "user.c").read_text(encoding="utf-8")

        self.assertIn("#define CAN_ID_TX_DIAG", can_header)
        self.assertIn("#define CAN_ID_TX_DIAG       0x0FF", can_header)
        self.assertIn("can_classic_data_frame_send", can_api)
        self.assertIn("can_classic_data_frame_send(CAN_ID_TX_DIAG", user_source)
        self.assertIn("CAN_DIAG_REJECT_BAD_DLC", user_source)
        self.assertIn("can_diag_record_reject", user_source)
        self.assertIn("HAL_FDCAN_GetProtocolStatus", can_source)
        self.assertIn("HAL_FDCAN_GetErrorCounters", can_source)

    def test_firmware_recovers_fdcan_bus_off_and_sends_diag_before_fd_frames(self):
        root = Path(__file__).resolve().parents[2]
        can_source = (root / "BSP" / "can.c").read_text(encoding="utf-8")
        can_api = (root / "BSP" / "can.h").read_text(encoding="utf-8")
        user_source = (root / "Application" / "user.c").read_text(encoding="utf-8")
        loop_body = user_source.split("void loop(void)", 1)[1].split(
            "flush_can_telemetry();", 1
        )[0]

        self.assertIn("can_recover_bus_off", can_api)
        self.assertIn("HAL_FDCAN_AbortTxRequest", can_source)
        self.assertIn("HAL_FDCAN_Stop", can_source)
        self.assertIn("HAL_FDCAN_Start", can_source)
        self.assertIn("FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2", can_source)
        self.assertLess(loop_body.index("can_recover_bus_off();"),
                        loop_body.index("send_can_diag();"))
        self.assertLess(loop_body.index("send_can_diag();"),
                        loop_body.index("send_config_snapshot();"))

    def test_can_interval_test_mode_is_not_left_enabled_in_firmware(self):
        root = Path(__file__).resolve().parents[2]
        user_source = (root / "Application" / "user.c").read_text(encoding="utf-8")

        self.assertIn("#define CAN_INTERVAL_TEST_ENABLED    0U", user_source)
        self.assertNotIn("#define CAN_INTERVAL_TEST_ENABLED    1U", user_source)

    def test_can_tx_driver_test_mode_is_classic_send_only(self):
        root = Path(__file__).resolve().parents[2]
        user_source = (root / "Application" / "user.c").read_text(encoding="utf-8")
        setup_body = user_source.split("void setup(void)", 1)[1].split(
            "static void reset_channel_statistics", 1
        )[0]
        loop_body = user_source.split("void loop(void)", 1)[1]

        self.assertIn("#define CAN_TX_DRIVER_TEST_ENABLED   1U", user_source)
        self.assertIn("#define CAN_TX_DRIVER_TEST_CLASSIC_ID 0x123U", user_source)
        self.assertIn("#define CAN_TX_DRIVER_TEST_SEND_FD   0U", user_source)
        self.assertIn("can_classic_data_frame_send(CAN_TX_DRIVER_TEST_CLASSIC_ID", user_source)
        self.assertLess(setup_body.index("can_tx_driver_test_last_tx_tick"),
                        setup_body.index("flash_storage_register_user_ops();"))
        self.assertLess(loop_body.index("send_can_tx_driver_test();"),
                        loop_body.index("process_can_commands();"))

    def test_can_send_path_recovers_bus_off_before_reporting_fifo_full(self):
        root = Path(__file__).resolve().parents[2]
        can_source = (root / "BSP" / "can.c").read_text(encoding="utf-8")
        send_helper = can_source.split(
            "static int can_data_frame_send_reserved", 1
        )[1].split("int can_fd_data_frame_send", 1)[0]
        fifo_full_branch = send_helper.split(
            "HAL_FDCAN_GetTxFifoFreeLevel", 1
        )[1].split("HAL_FDCAN_AddMessageToTxFifoQ", 1)[0]

        self.assertIn("can_recover_bus_off()", fifo_full_branch)
        self.assertLess(
            fifo_full_branch.index("can_recover_bus_off()"),
            fifo_full_branch.index("return -3;"),
        )

    def test_canfd_docs_match_500k_2m_runtime_configuration(self):
        root = Path(__file__).resolve().parents[2]
        readme = (root / "README.md").read_text(encoding="utf-8")
        setup_doc = (root / "docs" / "canfd_2m_setup.md").read_text(encoding="utf-8")

        self.assertIn("500K / 2M", readme)
        self.assertIn("500K / 2M", setup_doc)
        self.assertIn("S6", setup_doc)
        self.assertIn("Tx delay compensation is intentionally left disabled", setup_doc)
        self.assertNotIn("1M / 2M", readme)
        self.assertNotIn("S8", setup_doc)

    def test_setup_defers_blocking_ads_start_until_can_service_loop(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "Application" / "user.c").read_text(encoding="utf-8")
        setup_body = source.split("void setup(void)", 1)[1].split(
            "static void reset_channel_statistics", 1
        )[0]
        loop_body = source.split("void loop(void)", 1)[1].split(
            "adc_ads1256_poll();", 1
        )[0]

        self.assertIn("adc_ads1256_configure_startup(", setup_body)
        self.assertIn("adc_ads1256_prepare();", setup_body)
        self.assertNotIn("adc_ads1256_start();", setup_body)
        self.assertLess(loop_body.index("process_can_commands();"),
                        loop_body.index("send_can_diag();"))
        self.assertLess(loop_body.index("send_can_diag();"),
                        loop_body.index("send_config_snapshot();"))
        self.assertLess(loop_body.index("send_config_snapshot();"),
                        source.split("void loop(void)", 1)[1].index("adc_ads1256_poll();"))
        before_filter = setup_body.split("filter_init();", 1)[0]
        self.assertNotIn("adc_ads1256_calibrate()", before_filter)
        self.assertNotIn("adc_ads1256_restart()", before_filter)
        self.assertNotIn("adc_ads1256_set_vref_uv", before_filter)
        self.assertNotIn("adc_ads1256_set_pga_gain", before_filter)
        self.assertNotIn("adc_ads1256_set_sample_rate_x10", before_filter)
        self.assertNotIn("adc_ads1256_set_channel_mask", before_filter)

    def test_ads1256_port_init_leaves_runtime_config_to_scan_layer(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "BSP" / "ads1256" / "ads1256_port.c").read_text(
            encoding="utf-8"
        )
        config_one = source.split("static int ads1256_config_one", 1)[1].split(
            "int adc_ads1256_init", 1
        )[0]

        self.assertIn("ads1256_reset(ads1256)", config_one)
        self.assertNotIn("ads1256_set_pga", config_one)
        self.assertNotIn("ads1256_set_sps", config_one)
        self.assertNotIn("ads1256_calibration", config_one)


class TestReducerMonitorWindow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()
        self.window.is_connected = True
        self.window.can_bus = MagicMock()
        self.window.can_bus.send_frame = MagicMock(return_value=True)

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()

    def _switch_to_english(self):
        self.window.language_combo.setCurrentIndex(
            self.window.language_combo.findData("en")
        )

    def test_available_channel_count(self):
        self.assertEqual(len(self.window.channel_data), NUM_CHANNELS)

    def test_default_waveform_plot_count(self):
        self.assertEqual(len(self.window.plot_panels), DEFAULT_VISIBLE_PLOTS)
        self.assertEqual(self.window.plot_channels, [0, 1, 2, 3])

    def test_dark_theme_is_fixed(self):
        stylesheet = self.window.styleSheet()

        self.assertEqual(self.window.theme, "dark")
        self.assertFalse(hasattr(self.window, "theme_combo"))
        self.assertIn("background: #1e1e1e", stylesheet)
        self.assertNotIn("background: #f4f7fb", stylesheet)
        self.assertFalse(self.window.windowIcon().isNull())

    def test_chinese_language_is_default(self):
        self.assertEqual(self.window.language, "zh")
        self.assertEqual(self.window.language_combo.currentData(), "zh")
        self.assertEqual(self.window.windowTitle(), "减速器柔轮监视器")
        self.assertEqual(self.window.tabs.tabText(0), "波形")

    def test_empty_waveform_plots_have_consistent_initial_ranges(self):
        for plot in self.window.plot_widgets:
            x_range, y_range = plot.viewRange()
            self.assertAlmostEqual(x_range[0], 0.0)
            self.assertAlmostEqual(x_range[1], 100.0)
            self.assertAlmostEqual(y_range[0], -1.0)
            self.assertAlmostEqual(y_range[1], 1.0)

    def test_stream_summary_has_its_own_command_row(self):
        self.assertEqual(self.window.cmd_group.layout().count(), 2)
        self.assertIn("Raw", self.window.stream_summary_label.text())

    def test_log_button_lives_with_waveform_tools(self):
        self.assertEqual(self.window.conn_group.layout().indexOf(self.window.log_btn), -1)
        self.assertGreater(
            self.window.waveform_controls_layout.indexOf(self.window.log_btn),
            self.window.waveform_controls_layout.indexOf(self.window.add_plot_btn),
        )

    def test_opengl_plot_option_is_removed(self):
        self.assertFalse(hasattr(reducer_monitor_module, "OPENGL_PLOT_AVAILABLE"))
        self.assertFalse(hasattr(self.window, "opengl_checkbox"))

    def test_metric_color_buttons_are_global_controls_only(self):
        self.assertEqual(set(self.window.metric_color_buttons), {"voltage", "strain", "stress"})
        self.assertFalse(hasattr(self.window, "plot_metric_color_buttons"))
        for button in self.window.metric_color_buttons.values():
            self.assertFalse(button.isHidden())

    def test_metric_color_controls_are_right_aligned_with_metric_hints(self):
        self._switch_to_english()

        self.assertGreater(
            self.window.waveform_controls_layout.indexOf(self.window.metric_colors_label),
            self.window.waveform_controls_layout.indexOf(self.window.add_plot_btn),
        )
        expected_labels = {
            "voltage": "Voltage",
            "strain": "Strain",
            "stress": "Stress",
        }
        for metric, label in expected_labels.items():
            button = self.window.metric_color_buttons[metric]
            self.assertIn(label, button.toolTip())
            self.assertIn(label, button.accessibleName())

    def test_pyqtgraph_context_menus_default_to_chinese(self):
        plot_item = self.window.plot_widgets[0].getPlotItem()

        view_menu_texts = [action.text() for action in plot_item.vb.menu.actions()]
        plot_menu_texts = [action.text() for action in plot_item.ctrlMenu.actions()]

        self.assertIn("显示全部", view_menu_texts)
        self.assertEqual(plot_item.ctrlMenu.title(), "绘图选项")
        self.assertIn("降采样", plot_menu_texts)
        self.assertNotIn("View All", view_menu_texts)

    def test_pyqtgraph_nested_context_menus_default_to_chinese(self):
        plot_item = self.window.plot_widgets[0].getPlotItem()
        transform_action = next(
            action
            for action in plot_item.ctrlMenu.actions()
            if action.property("reducer_original_text") == "Transforms"
        )
        transform_menu = transform_action.menu()

        transform_menu.aboutToShow.emit()
        transform_texts = self._menu_widget_texts(transform_menu)

        self.assertIn("功率谱 (FFT)", transform_texts)
        self.assertIn("减去均值", transform_texts)
        self.assertNotIn("Power Spectrum (FFT)", transform_texts)
        self.assertNotIn("Subtract Mean", transform_texts)

    def test_pyqtgraph_axis_context_menus_default_to_chinese(self):
        plot_item = self.window.plot_widgets[0].getPlotItem()
        x_axis_action = next(
            action
            for action in plot_item.vb.menu.actions()
            if action.property("reducer_original_text") == "X axis"
        )
        x_axis_menu = x_axis_action.menu()

        x_axis_menu.aboutToShow.emit()
        axis_texts = self._menu_widget_texts(x_axis_menu)

        self.assertIn("链接坐标轴：", axis_texts)
        self.assertIn("手动", axis_texts)
        self.assertIn("反转坐标轴", axis_texts)
        self.assertIn("启用鼠标", axis_texts)
        self.assertIn("仅可见数据", axis_texts)
        self.assertIn("仅自动平移", axis_texts)
        self.assertNotIn("Link Axis:", axis_texts)
        self.assertNotIn("Manual", axis_texts)

    def test_pyqtgraph_context_menus_restore_english_when_language_changes(self):
        self._switch_to_english()
        plot_item = self.window.plot_widgets[0].getPlotItem()

        view_menu_texts = [action.text() for action in plot_item.vb.menu.actions()]

        self.assertIn("View All", view_menu_texts)
        self.assertEqual(plot_item.ctrlMenu.title(), "Plot Options")

    def _tree_texts(self, tree):
        texts = []

        def visit(item):
            for column in range(item.columnCount()):
                text = item.text(column)
                if text:
                    texts.append(text)
            for index in range(item.childCount()):
                visit(item.child(index))

        for index in range(tree.topLevelItemCount()):
            visit(tree.topLevelItem(index))
        return texts

    def _menu_widget_texts(self, menu):
        texts = []
        for action in menu.actions():
            widget = action.defaultWidget() if hasattr(action, "defaultWidget") else None
            if widget is None:
                continue
            for child in widget.findChildren(QLabel) + widget.findChildren(QAbstractButton):
                text = child.text()
                if text:
                    texts.append(text)
        return texts

    def test_pyqtgraph_export_dialog_defaults_to_chinese(self):
        scene = self.window.plot_widgets[0].scene()
        scene.contextMenuItem = self.window.plot_widgets[0].getPlotItem()
        scene.showExportDialog()
        dialog = scene.exportDialog
        self.addCleanup(dialog.close)

        format_texts = [
            dialog.ui.formatList.item(index).text()
            for index in range(dialog.ui.formatList.count())
        ]
        option_texts = self._tree_texts(dialog.ui.paramTree)

        self.assertEqual(dialog.windowTitle(), "导出")
        self.assertEqual(dialog.ui.label.text(), "导出对象：")
        self.assertEqual(dialog.ui.label_2.text(), "导出格式")
        self.assertEqual(dialog.ui.label_3.text(), "导出选项")
        self.assertEqual(dialog.ui.copyBtn.text(), "复制")
        self.assertEqual(dialog.ui.exportBtn.text(), "导出")
        self.assertEqual(dialog.ui.closeBtn.text(), "关闭")
        self.assertIn("整个场景", self._tree_texts(dialog.ui.itemTree))
        self.assertIn("原始曲线数据 CSV", format_texts)
        self.assertIn("图片文件 (PNG, TIF, JPG, ...)", format_texts)
        self.assertIn("分隔符", option_texts)
        self.assertIn("精度", option_texts)
        self.assertIn("列模式", option_texts)

    def test_pyqtgraph_export_dialog_restores_english_when_language_changes(self):
        scene = self.window.plot_widgets[0].scene()
        scene.contextMenuItem = self.window.plot_widgets[0].getPlotItem()
        scene.showExportDialog()
        dialog = scene.exportDialog
        self.addCleanup(dialog.close)

        self._switch_to_english()
        format_texts = [
            dialog.ui.formatList.item(index).text()
            for index in range(dialog.ui.formatList.count())
        ]

        self.assertEqual(dialog.windowTitle(), "Export")
        self.assertEqual(dialog.ui.label.text(), "Item to export:")
        self.assertIn("Entire Scene", self._tree_texts(dialog.ui.itemTree))
        self.assertIn("CSV of original plot data", format_texts)

    def test_waveform_plot_defaults_to_voltage_curve(self):
        checkboxes = self.window.plot_metric_checkboxes[0]

        self.assertTrue(checkboxes["voltage"].isChecked())
        self.assertFalse(checkboxes["strain"].isChecked())
        self.assertFalse(checkboxes["stress"].isChecked())

    def test_waveform_plot_can_overlay_multiple_metrics(self):
        self.window.plot_metric_checkboxes[0]["strain"].setChecked(True)
        self.window.plot_metric_checkboxes[0]["stress"].setChecked(True)
        self.window.on_data_updated(0, {
            "voltage": 12.5,
            "strain": 34.0,
            "stress": 56.0,
            "samples": 1,
        })
        self.window.update_plots()

        curves = self.window.plot_metric_curves[0]
        self.assertEqual(list(curves["voltage"].getData()[1]), [12.5])
        self.assertEqual(list(curves["strain"].getData()[1]), [34.0])
        self.assertEqual(list(curves["stress"].getData()[1]), [56.0])
        self.assertTrue(all(curve.isVisible() for curve in curves.values()))

    def test_waveform_curve_color_can_be_customized(self):
        self.window._set_metric_color("voltage", "#abcdef")

        self.assertEqual(self.window.global_metric_colors["voltage"], "#abcdef")
        for curves, colors in zip(
            self.window.plot_metric_curves,
            self.window.plot_metric_colors,
        ):
            self.assertEqual(colors["voltage"], "#abcdef")
            self.assertEqual(curves["voltage"].opts["pen"].color().name(), "#abcdef")
            self.assertNotEqual(curves["strain"].opts["pen"].color().name(), "#abcdef")
        self.assertIn("#abcdef", self.window.metric_color_buttons["voltage"].styleSheet())

    def test_waveform_hover_shows_nearest_curve_coordinates(self):
        self.window.on_data_updated(0, {
            "voltage": 12.5,
            "strain": 34.0,
            "stress": 56.0,
            "samples": 1,
        })
        self.window.update_plots()
        self.window.show()
        self.app.processEvents()

        plot = self.window.plot_widgets[0]
        scene_pos = plot.getPlotItem().vb.mapViewToScene(QPointF(0.0, 12.5))
        self.window._on_plot_mouse_moved(plot, scene_pos)

        hover_items = self.window.plot_hover_items[0]
        self.assertTrue(hover_items["label"].isVisible())
        self.assertIn(": 0", hover_items["label"].toPlainText())
        self.assertIn("12.500 mV", hover_items["label"].toPlainText())

        self.window.eventFilter(plot, QEvent(QEvent.Type.Leave))
        self.assertFalse(hover_items["label"].isVisible())

    def test_waveform_plot_keeps_at_least_one_metric_selected(self):
        checkbox = self.window.plot_metric_checkboxes[0]["voltage"]

        checkbox.setChecked(False)

        self.assertTrue(checkbox.isChecked())

    def test_waveform_plots_can_be_added_up_to_eight_cards(self):
        while len(self.window.plot_panels) < MAX_VISIBLE_PLOTS:
            self.window._add_waveform_plot()

        self.assertEqual(len(self.window.plot_panels), MAX_VISIBLE_PLOTS)
        self.assertEqual(self.window.plot_channels, [0, 1, 2, 3, 4, 5, 6, 7])
        self.assertFalse(self.window.add_plot_btn.isEnabled())
        for plot_index, panel in enumerate(self.window.plot_panels):
            index = self.window.waveform_layout.indexOf(panel)
            row, column, row_span, column_span = self.window.waveform_layout.getItemPosition(index)
            self.assertEqual((row, column, row_span, column_span),
                             (plot_index // 4, plot_index % 4, 1, 1))

        self.window._add_waveform_plot()
        self.assertEqual(len(self.window.plot_panels), MAX_VISIBLE_PLOTS)

    def test_adding_plot_does_not_override_mcu_channel_mask(self):
        self.window.can_bus.send_frame.reset_mock()
        self.window._add_waveform_plot()
        self.window.can_bus.send_frame.assert_not_called()
        self.assertEqual(self.window._displayed_channel_mask(), 0xFF)

    def test_waveform_plot_can_change_bound_channel(self):
        self.window.plot_channel_combos[0].setCurrentIndex(5)
        self.window.on_data_updated(5, {
            "voltage": 12.5,
            "strain": 0.0,
            "stress": 0.0,
            "samples": 1,
        })
        self.window.update_plots()

        self.assertEqual(self.window.plot_channels[0], 5)
        _, y_data = self.window.plot_curves[0].getData()
        self.assertEqual(list(y_data), [12.5])
        self.window.can_bus.send_frame.assert_not_called()

    def test_waveform_plot_can_be_removed(self):
        panel = self.window.plot_panels[-1]

        self.window._remove_waveform_plot(panel)

        self.assertEqual(len(self.window.plot_panels), DEFAULT_VISIBLE_PLOTS - 1)
        self.assertNotIn(panel, self.window.plot_panels)
        self.window.can_bus.send_frame.assert_not_called()

    def test_removing_all_waveform_plots_stops_mcu_sampling(self):
        for panel in list(self.window.plot_panels):
            self.window._remove_waveform_plot(panel)

        self.assertEqual(self.window.plot_panels, [])
        self.assertEqual(self.window._displayed_channel_mask(), 0xFF)
        self.window.can_bus.send_frame.assert_not_called()

    def test_duplicate_plots_subscribe_mcu_channel_once(self):
        self.window._sync_mcu_channel_mask()
        self.window.can_bus.send_frame.reset_mock()

        self.window._add_waveform_plot(channel=0)

        self.assertEqual(self.window._displayed_channel_mask(), 0xFF)
        self.window.can_bus.send_frame.assert_not_called()

    def test_config_snapshot_updates_acquisition_channel_checkboxes(self):
        self.window._apply_config_snapshot(ConfigFrame(
            saved=True,
            zero_valid=False,
            pga_gain=8,
            filter_length=7,
            telemetry_mode=TELEMETRY_MODE_RAW,
            channel_mask=0x0F,
            sample_rate_x10=10000,
            vref_uv=2_498_500,
            sequence=12,
            zero_offsets=list(range(8)),
        ))

        self.assertFalse(hasattr(self.window, "channel_mask_spin"))
        self.assertEqual(self.window.acquisition_channel_mask, 0x0F)
        self.assertEqual(
            [checkbox.isChecked() for checkbox in self.window.acquisition_channel_checkboxes],
            [True, True, True, True, False, False, False, False],
        )
        self.assertEqual(self.window._displayed_channel_mask(), 0x0F)

    def test_acquisition_checkbox_sends_channel_mask_command(self):
        self.window._apply_config_snapshot(ConfigFrame(
            saved=True,
            zero_valid=False,
            pga_gain=8,
            filter_length=7,
            telemetry_mode=TELEMETRY_MODE_RAW,
            channel_mask=0x0F,
            sample_rate_x10=10000,
            vref_uv=2_498_500,
            sequence=12,
            zero_offsets=list(range(8)),
        ))
        self.window.can_bus.send_frame.reset_mock()

        self.window.acquisition_channel_checkboxes[5].setChecked(True)

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_CHANNEL_MASK)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), 0x2F)
        self.assertEqual(self.window.acquisition_channel_mask, 0x2F)

    def test_acquisition_checkboxes_allow_zero_mask(self):
        self.window._set_acquisition_channel_mask(0x01)
        self.window.can_bus.send_frame.reset_mock()

        self.window.acquisition_channel_checkboxes[0].setChecked(False)

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_CHANNEL_MASK)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), 0)
        self.assertEqual(
            [checkbox.isChecked() for checkbox in self.window.acquisition_channel_checkboxes],
            [False] * NUM_CHANNELS,
        )
        self.assertEqual(self.window._displayed_channel_mask(), 0)

    def test_parse_valid_telemetry_updates_channel(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_telemetry_payload(channel=1, voltage_001mv=2345, strain_ue=321, stress_01mpa=45),
        )

        self.window.on_can_frame_received(frame)
        self.window.update_plots()

        self.assertAlmostEqual(self.window.channel_data[1].voltage_mv, 23.45, places=2)
        self.assertEqual(self.window.channel_data[1].strain_ue, 321.0)
        self.assertAlmostEqual(self.window.channel_data[1].stress_mpa, 4.5, places=3)
        self.assertEqual(self.window.channel_data[1].samples, 1)

    def test_parse_batched_telemetry_updates_channels(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_telemetry_batch_payload([
                (0, 1234, 321, 45),
                (1, 5678, -123, -5),
            ]),
        )

        self.window.on_can_frame_received(frame)
        self.window.update_plots()

        self.assertAlmostEqual(self.window.channel_data[0].voltage_mv, 12.34)
        self.assertAlmostEqual(self.window.channel_data[1].voltage_mv, 56.78)
        self.assertEqual(self.window.rx_telemetry_count, 2)

    def test_parse_v2_raw_telemetry_is_converted_on_gui_thread(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_raw_v2_payload([
                (0, 838860),
                (1, -838860),
            ]),
        )

        self.window.on_can_frame_received(frame)
        self.assertEqual(self.window.channel_data[0].samples, 0)
        self.window.update_plots()

        self.assertEqual(self.window.rx_telemetry_count, 2)
        self.assertEqual(self.window.channel_data[0].samples, 1)
        self.assertEqual(self.window.channel_data[1].samples, 1)
        self.assertGreater(self.window.channel_data[0].voltage_mv, 0.0)
        self.assertLess(self.window.channel_data[1].voltage_mv, 0.0)

    def test_bad_telemetry_crc_is_rejected(self):
        payload = bytearray(build_telemetry_payload(channel=0, voltage_001mv=1000, strain_ue=100, stress_01mpa=10))
        payload[-1] ^= 0xAA
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))

        self.window.on_can_frame_received(frame)

        self.assertEqual(self.window.channel_data[0].voltage_mv, 0.0)

    def test_status_frame_updates_status_bar(self):
        self._switch_to_english()
        self.window.pending_commands[7] = "Set Filter Size"
        frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(sequence=7, cmd_type=CAN_CMD_SET_FILTER_SIZE, status=CAN_STATUS_OK, value=32),
        )

        self.window.on_can_frame_received(frame)

        self.assertEqual(self.window.status_bar.currentMessage(), "Set Filter Size acknowledged (value=32)")

    def test_rejected_status_frame_updates_status_bar(self):
        self._switch_to_english()
        self.window.pending_commands[5] = "Set Filter Size"
        frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(
                sequence=5,
                cmd_type=CAN_CMD_SET_FILTER_SIZE,
                status=CAN_STATUS_BAD_VALUE,
                value=99,
                detail=99,
            ),
        )

        self.window.on_can_frame_received(frame)

        self.assertIn("rejected", self.window.status_bar.currentMessage())

    def test_storage_error_status_updates_status_bar(self):
        self._switch_to_english()
        self.window.pending_commands[8] = "Save Zero"
        frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(
                sequence=8,
                cmd_type=CAN_CMD_SAVE_ZERO,
                status=CAN_STATUS_STORAGE_ERROR,
                value=0,
                detail=1,
            ),
        )

        self.window.on_can_frame_received(frame)

        self.assertIn("storage error", self.window.status_bar.currentMessage())

    def test_manual_channel_text_takes_precedence(self):
        self.window.channel_combo.clear()
        self.window.channel_combo.addItem("COM1 - Existing Adapter", "COM1")
        self.window.channel_combo.setEditText("COM9")

        self.assertEqual(self.window._selected_channel(), "COM9")

    def test_language_can_switch_between_english_and_chinese(self):
        self.window.on_data_updated(0, {
            "voltage": 12.5,
            "strain": 34.0,
            "stress": 56.0,
            "samples": 1,
        })
        self.window._show_status(
            "command_acknowledged", command="Set Filter Size", value=32
        )

        self._switch_to_english()
        self.window.language_combo.setCurrentIndex(
            self.window.language_combo.findData("zh")
        )

        self.assertEqual(self.window.windowTitle(), "减速器柔轮监视器")
        self.assertEqual(self.window.connect_btn.text(), "断开连接")
        self.assertEqual(self.window.tabs.tabText(0), "波形")
        self.assertEqual(self.window.data_table.horizontalHeaderItem(0).text(), "通道")
        self.assertEqual(self.window.stats_labels[0][0].text(), "最小值: 12.500 mV")
        self.assertEqual(
            self.window.status_bar.currentMessage(), "设置滤波长度 已确认（值=32）"
        )

        self.window.language_combo.setCurrentIndex(
            self.window.language_combo.findData("en")
        )

        self.assertEqual(self.window.windowTitle(), "Reducer Flexspline Monitor")
        self.assertEqual(self.window.tabs.tabText(0), "Waveforms")
        self.assertEqual(
            self.window.status_bar.currentMessage(),
            "Set Filter Size acknowledged (value=32)",
        )

    def test_zero_datum_command(self):
        self.window.on_zero_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.id, CAN_ID_RX_COMMAND)
        self.assertEqual(sent_frame.data[3], CAN_CMD_ZERO_DATUM)

    def test_target_stress_sets_zero_offset_from_latest_raw_sample(self):
        self.window.zero_offsets[2] = 1000
        self.window._apply_telemetry_values(
            2,
            voltage_mv=1.25,
            strain_ue=118.0,
            stress_mpa=24.8,
            voltage_001mv=125,
            stress_01mpa=248,
            timestamp=None,
            raw_value=32000,
        )
        self.window.zero_offset_channel_combo.setCurrentIndex(2)
        self.window.zero_offset_target_stress_spin.setValue(21.0)

        self.window.on_target_stress_zero_clicked()

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        target_voltage_mv = self.window._stress_mpa_to_voltage_mv(21.0)
        target_raw = round(target_voltage_mv / self.window._raw_to_mv_scale())
        expected_offset = 32000 + 1000 - target_raw
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_ZERO_OFFSET)
        self.assertEqual(sent_frame.data[4], 2)
        self.assertEqual(
            int.from_bytes(sent_frame.data[6:10], "little"),
            expected_offset & 0xFFFFFFFF,
        )
        self.assertIn(str(expected_offset), self.window.zero_offset_result_label.text())

    def test_config_snapshot_updates_adc_controls(self):
        self.window._apply_config_snapshot(ConfigFrame(
            saved=True,
            zero_valid=False,
            pga_gain=8,
            filter_length=7,
            telemetry_mode=TELEMETRY_MODE_RAW,
            channel_mask=0x0F,
            sample_rate_x10=10000,
            vref_uv=2_498_500,
            sequence=12,
            zero_offsets=list(range(8)),
        ))
        self.assertAlmostEqual(self.window.vref_spin.value(), 2.4985)
        self.assertEqual(self.window.pga_combo.currentData(), 8)
        self.assertEqual(self.window.acquisition_channel_mask, 0x0F)
        self.assertIn("需要重新调零", self.window.config_state_label.text())

    def test_redundant_zero_buttons_are_removed(self):
        self.assertFalse(hasattr(self.window, "save_zero_btn"))
        self.assertFalse(hasattr(self.window, "load_zero_btn"))

    def test_clear_zero_command(self):
        self.window.on_clear_zero_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_CLEAR_ZERO)

    def test_calib_command(self):
        self.window.on_calib_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_START_CALIB)

    def test_filter_size_command(self):
        self.window.on_filter_size_changed(32)
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_FILTER_SIZE)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), 32)

    def test_sample_rate_command(self):
        self.window.sample_rate_combo.setCurrentText("500 SPS")
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_SAMPLE_RATE)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), 500)

    def test_telemetry_mode_command(self):
        self.window.telemetry_mode_combo.setCurrentIndex(
            self.window.telemetry_mode_combo.findData(TELEMETRY_MODE_PHYSICAL)
        )
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_TELEMETRY_MODE)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), TELEMETRY_MODE_PHYSICAL)
        self.assertEqual(self.window.telemetry_mode, TELEMETRY_MODE_RAW)

    def test_all_ads1256_sample_rates_are_available(self):
        self.assertEqual(
            SUPPORTED_SAMPLE_RATES,
            [2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
             3750, 7500, 15000, 30000],
        )

    def test_fractional_sample_rate_uses_deci_sps_command_encoding(self):
        self.window.sample_rate_combo.setCurrentText("2.5 SPS")

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_SAMPLE_RATE)
        self.assertEqual(sent_frame.data[4], 1)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), 25)

    def test_high_ads_rate_does_not_auto_decimate_telemetry(self):
        self.window._set_acquisition_channel_mask(0x0F)
        self.assertEqual(self.window._estimated_telemetry_rate(30000), 4374)

        self.window._set_acquisition_channel_mask(0xFF)
        self.window._add_waveform_plot()

        self.assertEqual(self.window._estimated_telemetry_rate(30000), 8748)

    def test_health_frame_updates_system_health_panel(self):
        self._switch_to_english()
        self.window.on_can_frame_received(
            CANFrame(id=CAN_ID_TX_HEALTH, data=build_health_v2_payload())
        )

        self.assertEqual(self.window.sample_rate_sps, 30000)
        self.assertEqual(self.window.telemetry_mode, TELEMETRY_MODE_RAW)
        self.assertIn("new drops 0, total 4", self.window.health_summary_label.text())
        self.assertIn("new overflows 0, total 5", self.window.health_summary_label.text())
        self.assertEqual(
            self.window.health_icon_label.pixmap().toImage()
            .pixelColor(7, 7).name(),
            "#35b66a",
        )

        self.window.on_can_frame_received(
            CANFrame(
                id=CAN_ID_TX_HEALTH,
                data=build_health_v2_payload(tx_drop=6, overflow=6, recovery=7),
            )
        )
        self.assertIn("new drops 2, total 6", self.window.health_summary_label.text())
        self.assertEqual(
            self.window.health_icon_label.pixmap().toImage()
            .pixelColor(7, 7).name(),
            "#ed8b2c",
        )

    def test_health_summary_is_in_status_bar_with_waiting_icon(self):
        self._switch_to_english()
        self.assertIs(self.window.health_summary_label.parent(), self.window.status_bar)
        self.assertFalse(hasattr(self.window, "health_group"))
        self.assertFalse(self.window.health_icon_label.pixmap().isNull())
        self.assertIn("Waiting", self.window.health_summary_label.text())

    def test_healthy_adc_frame_sets_green_status_icon(self):
        self.window.on_can_frame_received(
            CANFrame(
                id=CAN_ID_TX_HEALTH,
                data=build_health_v2_payload(tx_drop=0, overflow=0, recovery=0),
            )
        )

        self.assertEqual(
            self.window.health_icon_label.pixmap().toImage()
            .pixelColor(7, 7).name(),
            "#35b66a",
        )

    def test_high_sample_rate_is_available_over_canable2_usb_cdc(self):
        self.window.sample_rate_combo.setCurrentText("30000 SPS")

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[3], CAN_CMD_SET_SAMPLE_RATE)
        self.assertEqual(int.from_bytes(sent_frame.data[6:10], "little"), 30000)

    def test_connection_options_can_be_locked(self):
        self.window.is_connected = False
        self.window._set_connection_controls_enabled(False)

        self.assertFalse(self.window.interface_combo.isEnabled())
        self.assertFalse(self.window.channel_combo.isEnabled())
        self.assertFalse(hasattr(self.window, "serial_baud_combo"))
        self.assertFalse(hasattr(self.window, "baud_combo"))
        self.assertFalse(hasattr(self.window, "can_baud_value"))
        self.assertFalse(self.window.refresh_btn.isEnabled())
        self.assertTrue(self.window.language_combo.isEnabled())

    def test_can_bitrate_is_fixed_to_match_mcu_firmware(self):
        self.assertEqual(FIXED_CAN_BITRATE, Baudrate.BAUD_500K)
        self.assertFalse(hasattr(self.window, "baud_combo"))
        self.assertFalse(hasattr(self.window, "can_baud_value"))

    def test_slcan_usb_serial_speed_defaults_to_adapter_compatible_value(self):
        self.assertEqual(
            self.window.slcan_speed_combo.currentData(),
            DEFAULT_SLCAN_TTY_BAUDRATE,
        )

    def test_command_fails_when_disconnected(self):
        self.window.is_connected = False
        self.window.can_bus = None
        self.assertFalse(self.window.send_command(CAN_CMD_ZERO_DATUM))

    def test_command_fails_when_all_sequences_are_pending(self):
        self.window.pending_commands = {
            sequence: "Pending" for sequence in range(256)
        }

        self.assertFalse(self.window.send_command(CAN_CMD_ZERO_DATUM))
        self.window.can_bus.send_frame.assert_not_called()

    def test_waveform_buffer_limit(self):
        from reducer_monitor import WAVEFORM_BUFFER_SIZE

        for i in range(WAVEFORM_BUFFER_SIZE + 50):
            self.window.on_data_updated(0, {
                "voltage": float(i),
                "strain": 0.0,
                "stress": 0.0,
                "samples": i + 1,
            })

        self.assertEqual(len(self.window.waveform_buffers[0]), WAVEFORM_BUFFER_SIZE)

    def test_auto_scale_defaults_on_and_can_be_disabled(self):
        self.assertTrue(self.window.auto_scale_checkbox.isChecked())

        self.window.auto_scale_checkbox.setChecked(False)
        self.window.on_data_updated(0, {
            "voltage": 12.0,
            "strain": 0.0,
            "stress": 0.0,
            "samples": 1,
        })
        self.window.update_plots()

        self.assertFalse(self.window.auto_scale_checkbox.isChecked())

    def test_paused_view_freezes_curves_while_buffers_continue(self):
        self.window.on_data_updated(0, {
            "voltage": 1.0,
            "strain": 2.0,
            "stress": 3.0,
            "samples": 1,
        })
        self.window.update_plots()
        self.window._set_view_paused(True)

        self.window.on_data_updated(0, {
            "voltage": 4.0,
            "strain": 5.0,
            "stress": 6.0,
            "samples": 2,
        })
        self.window.update_plots()

        _, y_data = self.window.plot_curves[0].getData()
        self.assertEqual(list(y_data), [1.0])
        self.assertEqual(len(self.window.waveform_buffers[0]), 2)

        self.window._set_view_paused(False)
        self.window.update_plots()

        _, y_data = self.window.plot_curves[0].getData()
        self.assertEqual(list(y_data), [1.0, 4.0])

    def test_paused_visible_range_export_uses_frozen_snapshot(self):
        for sample, value in enumerate((10.0, 20.0, 30.0), start=1):
            self.window.on_data_updated(0, {
                "voltage": value,
                "strain": value + 1.0,
                "stress": value + 2.0,
                "samples": sample,
            })
        self.window.update_plots()
        self.window._set_view_paused(True)
        self.window.on_data_updated(0, {
            "voltage": 99.0,
            "strain": 99.0,
            "stress": 99.0,
            "samples": 4,
        })
        self.window.plot_widgets[0].setXRange(1, 2, padding=0)

        export_file = tempfile.NamedTemporaryFile(delete=False, suffix=".csv")
        export_file.close()
        try:
            self.window._export_paused_selection_to_csv(export_file.name)
            with open(export_file.name, "r", newline="") as handle:
                rows = list(csv.DictReader(handle))
        finally:
            os.unlink(export_file.name)

        self.assertEqual([row["voltage_mv"] for row in rows], ["20.0", "30.0"])
        self.assertEqual([row["samples"] for row in rows], ["2", "3"])

    def test_plot_refresh_rate_is_fixed_at_60_hz(self):
        self.assertFalse(hasattr(self.window, "plot_refresh_combo"))
        self.assertEqual(self.window.update_timer.interval(), 17)

    def test_clear_plots_keeps_current_values(self):
        self.window.on_data_updated(0, {
            "voltage": 12.0,
            "strain": 34.0,
            "stress": 56.0,
            "samples": 1,
        })
        self.window.update_plots()

        self.window._clear_plots()

        self.assertTrue(all(len(buffer) == 0 for buffer in self.window.waveform_buffers))
        self.assertEqual(self.window.data_table.item(0, 1).text(), "12.000")
        for curves in self.window.plot_metric_curves:
            for curve in curves.values():
                x_data, y_data = curve.getData()
                self.assertTrue(x_data is None or len(x_data) == 0)
                self.assertTrue(y_data is None or len(y_data) == 0)

    def test_plot_maximize_toggle(self):
        self.assertIsNone(self.window.maximized_plot_channel)

        self.window._toggle_plot_maximize(2)
        self.assertEqual(self.window.maximized_plot_channel, 2)
        self.assertFalse(self.window.plot_panels[2].isHidden())
        self.assertTrue(self.window.plot_panels[0].isHidden())

        self.window._toggle_plot_maximize(2)
        self.assertIsNone(self.window.maximized_plot_channel)
        self.assertTrue(all(not panel.isHidden() for panel in self.window.plot_panels))
        self.assertEqual(self.window.waveform_layout.count(), DEFAULT_VISIBLE_PLOTS)
        for plot_index, panel in enumerate(self.window.plot_panels):
            index = self.window.waveform_layout.indexOf(panel)
            row, column, row_span, column_span = self.window.waveform_layout.getItemPosition(index)
            self.assertEqual((row, column, row_span, column_span),
                             (plot_index // 2, plot_index % 2, 1, 1))


class TestCsvLogging(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()
        self.temp_file = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".csv", newline="")
        self.temp_file.close()
        self.csv_path = self.temp_file.name

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        if os.path.exists(self.csv_path):
            os.unlink(self.csv_path)

    def test_csv_row_written(self):
        self.window.csv_file = open(self.csv_path, "w", newline="")
        self.window.csv_writer = csv.writer(self.window.csv_file)
        self.window.csv_writer.writerow(["header"])
        self.window.logging_enabled = True

        self.window.on_data_updated(0, {
            "voltage": 123.45,
            "strain": 500.0,
            "stress": 10.5,
            "samples": 1,
        })
        self.window.update_plots()

        self.window.csv_file.close()

        with open(self.csv_path, "r", newline="") as handle:
            rows = list(csv.reader(handle))

        self.assertEqual(len(rows), 2)
        self.assertIn("123.45", rows[1][2])

    def test_csv_import_opens_offline_window_without_changing_live_waveforms(self):
        with open(self.csv_path, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "timestamp", "channel", "voltage_mv",
                "strain_ue", "stress_mpa", "samples",
            ])
            writer.writerow(["2026-05-31T12:00:00", 0, 1.25, 2.0, 3.0, 1])
            writer.writerow(["2026-05-31T12:00:01", 1, -4.5, -5.0, -6.0, 1])
            writer.writerow(["2026-05-31T12:00:02", 0, 7.75, 8.0, 9.0, 2])

        with patch(
            "reducer_monitor.QFileDialog.getOpenFileName",
            return_value=(self.csv_path, "CSV Files (*.csv)"),
        ):
            self.window.import_csv()

        self.assertEqual(len(self.window.offline_waveform_windows), 1)
        offline_window = self.window.offline_waveform_windows[0]
        self.assertIsInstance(offline_window, OfflineWaveformWindow)
        self.assertEqual(offline_window.theme, "dark")
        self.assertEqual(offline_window.waveform_buffers[0], [1.25, 7.75])
        self.assertEqual(offline_window.waveform_buffers[1], [-4.5])
        self.assertTrue(all(len(buffer) == 0 for buffer in self.window.waveform_buffers))

        self.window.language_combo.setCurrentIndex(
            self.window.language_combo.findData("zh")
        )
        self.assertTrue(offline_window.windowTitle().startswith("减速器波形记录"))
        self.assertEqual(offline_window.auto_scale_checkbox.text(), "自动缩放")

        offline_window._toggle_plot_maximize(0)
        self.assertEqual(offline_window.maximized_plot_channel, 0)
        index = offline_window.waveform_layout.indexOf(offline_window.plot_widgets[0])
        row, column, row_span, column_span = offline_window.waveform_layout.getItemPosition(index)
        self.assertEqual((row, column, row_span, column_span), (0, 0, 1, 2))
        self.assertEqual(offline_window.waveform_layout.rowStretch(0), 1)
        self.assertEqual(offline_window.waveform_layout.rowStretch(1), 0)
        offline_window._toggle_plot_maximize(0)
        self.assertIsNone(offline_window.maximized_plot_channel)
        self.assertEqual(offline_window.waveform_layout.count(), 2)
        offline_window.close()

    def test_offline_window_only_creates_plots_for_channels_with_data(self):
        window = OfflineWaveformWindow(
            "sparse.csv",
            [[1.0], [], [2.0], [], [], [], [], []],
            "zh",
        )
        self.assertEqual(window.plot_channels, [0, 2])
        self.assertEqual(len(window.plot_widgets), 2)
        window.close()

    def test_offline_window_uses_four_columns_for_five_or_more_plots(self):
        window = OfflineWaveformWindow(
            "wide.csv",
            [[float(channel)] for channel in range(5)] + [[], [], []],
            "zh",
        )

        index = window.waveform_layout.indexOf(window.plot_widgets[4])
        row, column, row_span, column_span = window.waveform_layout.getItemPosition(index)

        self.assertEqual((row, column, row_span, column_span), (1, 0, 1, 1))
        window.close()


def run_tests():
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return result.wasSuccessful()


if __name__ == "__main__":
    sys.exit(0 if run_tests() else 1)
