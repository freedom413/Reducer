"""
test_gui.py - Tests for the reducer monitor GUI and CAN protocol helpers.
"""

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).parent))

from PyQt6.QtWidgets import QApplication

from can_protocol import (
    CANFrame,
    CAN_FRAME_TYPE_STATUS,
    CAN_FRAME_TYPE_TELEMETRY,
    CAN_ID_RX_COMMAND,
    CAN_ID_TX_STATUS,
    CAN_ID_TX_TELEMETRY,
    CAN_STATUS_BAD_VALUE,
    CAN_STATUS_OK,
    build_command_frame,
    crc8_xor,
    parse_status_frame,
    parse_telemetry_frame,
)
from reducer_monitor import (
    CAN_CMD_CLEAR_ZERO,
    CAN_CMD_LOAD_ZERO,
    CAN_CMD_SAVE_ZERO,
    CAN_CMD_SET_SAMPLE_RATE,
    CAN_CMD_SET_FILTER_SIZE,
    CAN_CMD_START_CALIB,
    CAN_CMD_ZERO_DATUM,
    DEFAULT_ELASTIC_MODULUS_MPA,
    NUM_CHANNELS,
    ChannelData,
    ReducerMonitorWindow,
)


def build_telemetry_payload(channel: int, voltage_01mv: int, strain_ue: int, stress_01mpa: int) -> bytes:
    payload = bytes([
        CAN_FRAME_TYPE_TELEMETRY,
        channel,
        (voltage_01mv >> 8) & 0xFF,
        voltage_01mv & 0xFF,
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


class TestProtocolHelpers(unittest.TestCase):
    def test_crc8_xor_basic(self):
        self.assertEqual(crc8_xor(bytes([1, 2, 3])), 0x00)

    def test_build_command_frame_layout(self):
        frame = build_command_frame(sequence=0x12, cmd_type=CAN_CMD_SET_FILTER_SIZE, param=0x34, value=0x5678)

        self.assertEqual(frame.id, CAN_ID_RX_COMMAND)
        self.assertEqual(frame.data[0], 0xA0)
        self.assertEqual(frame.data[1], 0x12)
        self.assertEqual(frame.data[2], CAN_CMD_SET_FILTER_SIZE)
        self.assertEqual(frame.data[3], 0x34)
        self.assertEqual(frame.data[4], 0x78)
        self.assertEqual(frame.data[5], 0x56)
        self.assertEqual(frame.data[7], crc8_xor(frame.data[:7]))

    def test_parse_telemetry_frame(self):
        payload = build_telemetry_payload(channel=2, voltage_01mv=1234, strain_ue=-456, stress_01mpa=-12)
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=payload)

        parsed = parse_telemetry_frame(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.channel, 2)
        self.assertEqual(parsed.voltage_01mv, 1234)
        self.assertEqual(parsed.strain_ue, -456)
        self.assertEqual(parsed.stress_01mpa, -12)

    def test_parse_status_frame(self):
        payload = build_status_payload(sequence=3, cmd_type=CAN_CMD_ZERO_DATUM, status=CAN_STATUS_OK, value=16, detail=0x55)
        frame = CANFrame(id=CAN_ID_TX_STATUS, data=payload)

        parsed = parse_status_frame(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.sequence, 3)
        self.assertEqual(parsed.cmd_type, CAN_CMD_ZERO_DATUM)
        self.assertEqual(parsed.status, CAN_STATUS_OK)
        self.assertEqual(parsed.value, 16)
        self.assertEqual(parsed.detail, 0x55)

    def test_parse_rejects_bad_crc(self):
        payload = bytearray(build_telemetry_payload(channel=0, voltage_01mv=100, strain_ue=50, stress_01mpa=5))
        payload[-1] ^= 0xFF
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))

        self.assertIsNone(parse_telemetry_frame(frame))


class TestChannelData(unittest.TestCase):
    def test_default_values(self):
        data = ChannelData()
        self.assertEqual(data.voltage, 0.0)
        self.assertEqual(data.strain, 0.0)
        self.assertEqual(data.stress, 0.0)
        self.assertEqual(data.displacement, 0.0)
        self.assertEqual(data.raw_value, 0)


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

    def test_available_channel_count(self):
        self.assertEqual(len(self.window.channel_data), NUM_CHANNELS)

    def test_parse_valid_telemetry_updates_channel(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_telemetry_payload(channel=1, voltage_01mv=2345, strain_ue=321, stress_01mpa=45),
        )

        self.window.on_can_frame_received(frame)

        self.assertAlmostEqual(self.window.channel_data[1].voltage, 234.5, places=1)
        self.assertEqual(self.window.channel_data[1].strain, 321.0)
        expected_stress = 321.0 * DEFAULT_ELASTIC_MODULUS_MPA / 1_000_000.0
        self.assertAlmostEqual(self.window.channel_data[1].stress, expected_stress, places=3)

    def test_bad_telemetry_crc_is_rejected(self):
        payload = bytearray(build_telemetry_payload(channel=0, voltage_01mv=1000, strain_ue=100, stress_01mpa=10))
        payload[-1] ^= 0xAA
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))

        self.window.on_can_frame_received(frame)

        self.assertEqual(self.window.channel_data[0].voltage, 0.0)

    def test_status_frame_updates_status_bar(self):
        self.window.pending_commands[7] = "Set Filter Size"
        frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(sequence=7, cmd_type=CAN_CMD_SET_FILTER_SIZE, status=CAN_STATUS_OK, value=32),
        )

        self.window.on_can_frame_received(frame)

        self.assertEqual(self.window.status_bar.currentMessage(), "Set Filter Size acknowledged")

    def test_rejected_status_frame_updates_status_bar(self):
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

    def test_zero_datum_command(self):
        self.window.on_zero_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.id, CAN_ID_RX_COMMAND)
        self.assertEqual(sent_frame.data[2], CAN_CMD_ZERO_DATUM)

    def test_save_zero_command(self):
        self.window.on_save_zero_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SAVE_ZERO)

    def test_load_zero_command(self):
        self.window.on_load_zero_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_LOAD_ZERO)

    def test_clear_zero_command(self):
        self.window.on_clear_zero_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_CLEAR_ZERO)

    def test_calib_command(self):
        self.window.on_calib_clicked()
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_START_CALIB)

    def test_filter_size_command(self):
        self.window.on_filter_size_changed(32)
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_FILTER_SIZE)
        self.assertEqual(sent_frame.data[4], 32)
        self.assertEqual(sent_frame.data[5], 0)

    def test_sample_rate_command(self):
        self.window.sample_rate_combo.setCurrentText("500 SPS")
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_SAMPLE_RATE)
        self.assertEqual(sent_frame.data[4], 0xF4)
        self.assertEqual(sent_frame.data[5], 0x01)

    def test_command_fails_when_disconnected(self):
        self.window.is_connected = False
        self.window.can_bus = None
        self.assertFalse(self.window.send_command(CAN_CMD_ZERO_DATUM))

    def test_waveform_buffer_limit(self):
        from reducer_monitor import WAVEFORM_BUFFER_SIZE

        for i in range(WAVEFORM_BUFFER_SIZE + 50):
            self.window.on_data_updated(0, {
                "voltage": float(i),
                "strain": 0.0,
                "stress": 0.0,
                "displacement": 0.0,
            })

        self.assertEqual(len(self.window.waveform_buffers[0]), WAVEFORM_BUFFER_SIZE)


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
            "displacement": 2.5,
        })

        self.window.csv_file.close()

        with open(self.csv_path, "r", newline="") as handle:
            rows = list(csv.reader(handle))

        self.assertEqual(len(rows), 2)
        self.assertIn("123.45", rows[1][2])


def run_tests():
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return result.wasSuccessful()


if __name__ == "__main__":
    sys.exit(0 if run_tests() else 1)
