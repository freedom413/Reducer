"""
test_gui.py - Test suite for Reducer Flexspline Monitor GUI

Tests all GUI functionality including:
- Serial connection UI
- Command sending (Zero, Calibrate)
- CAN frame parsing (combined format only)
- CSV logging
- Data display updates
"""

import sys
import os
import unittest
import tempfile
import csv
import struct
from unittest.mock import Mock, MagicMock, patch, call
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer, QEventLoop

# Import the modules under test
from reducer_monitor import (
    ReducerMonitorWindow,
    crc8_xor,
    CAN_FRAME_COMBINED,
    CAN_ID_TX_DATA,
    CAN_ID_RX_CONFIG,
    NUM_CHANNELS,
    ChannelData,
    CAN_CMD_ZERO_DATUM,
    CAN_CMD_SAVE_ZERO,
    CAN_CMD_LOAD_ZERO,
    CAN_CMD_CLEAR_ZERO,
    CAN_CMD_SET_FILTER_SIZE,
    CAN_CMD_START_CALIB,
)
from slcan_protocol import SLCANProtocol


class TestCRCFunctions(unittest.TestCase):
    """Test CRC calculation functions"""

    def test_crc8_xor_basic(self):
        """Test basic CRC-8 XOR calculation"""
        data = bytes([0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        result = crc8_xor(data)
        self.assertIsInstance(result, int)
        self.assertEqual(result, 0x05)  # XOR of [0x05, 0,0,0,0,0,0]

    def test_crc8_xor_combined_frame(self):
        """Test CRC-8 for a typical combined frame"""
        # Simulated frame: type=0x05, ch=0, voltage=1234, strain=100, stress=50
        frame = bytes([0x05, 0x00, 0x04, 0xD2, 0x00, 0x64, 0x32])
        crc = crc8_xor(frame)
        expected = 0x05 ^ 0x00 ^ 0x04 ^ 0xD2 ^ 0x00 ^ 0x64 ^ 0x32
        self.assertEqual(crc, expected)

    def test_crc8_consistency(self):
        """Test that CRC calculation is consistent"""
        data = bytes([0x05, 0x03, 0x12, 0x34, 0x00, 0x64, 0x1E])
        result1 = crc8_xor(data)
        result2 = crc8_xor(data)
        self.assertEqual(result1, result2)


class TestCombinedFrameParsing(unittest.TestCase):
    """Test combined CAN frame parsing"""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()
        self.window.is_connected = True

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()

    def _create_combined_frame(self, channel, voltage_01mv, strain_ue, stress_01mpa):
        """Helper to create valid combined CAN frames"""
        voltage_bytes = voltage_01mv.to_bytes(2, byteorder='big', signed=True)
        strain_bytes = strain_ue.to_bytes(2, byteorder='big', signed=True)
        stress_byte = stress_01mpa.to_bytes(1, byteorder='big', signed=True)

        data = bytes([CAN_FRAME_COMBINED, channel]) + voltage_bytes + strain_bytes + stress_byte
        crc = crc8_xor(data)
        return data + bytes([crc])

    def test_parse_combined_frame_ch0(self):
        """Test parsing combined frame for channel 0"""
        voltage_01mv = 1234  # 123.4 mV
        strain_ue = 100      # 100 µε
        stress_01mpa = 50    # 5.0 MPa

        frame_data = self._create_combined_frame(0, voltage_01mv, strain_ue, stress_01mpa)

        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=frame_data)
        self.window.on_can_frame_received(frame)

        self.assertAlmostEqual(self.window.channel_data[0].voltage, 123.4, places=1)
        self.assertEqual(self.window.channel_data[0].strain, 100.0)
        self.assertAlmostEqual(self.window.channel_data[0].stress, 5.0, places=1)

    def test_parse_combined_frame_all_channels(self):
        """Test parsing combined frames for all 6 channels"""
        for ch in range(NUM_CHANNELS):
            voltage = 1000 + ch * 100  # e.g., 1000 = 100.0 mV
            strain = 50 + ch * 10      # e.g., 50 = 50 µε
            stress = 10 + ch           # e.g., 10 = stress value in 0.1MPa units = 1.0 MPa

            frame_data = self._create_combined_frame(ch, voltage, strain, stress)

            from slcan_protocol import CANFrame
            frame = CANFrame(id=CAN_ID_TX_DATA, data=frame_data)
            self.window.on_can_frame_received(frame)

            self.assertAlmostEqual(
                self.window.channel_data[ch].voltage,
                voltage / 10.0,
                places=1,
                msg=f"Channel {ch} voltage mismatch"
            )
            self.assertEqual(
                self.window.channel_data[ch].strain,
                float(strain),
                msg=f"Channel {ch} strain mismatch"
            )
            # stress in frame is in 0.1MPa units, GUI converts to MPa by dividing by 10
            self.assertAlmostEqual(
                self.window.channel_data[ch].stress,
                stress / 10.0,  # Convert 0.1MPa units to MPa
                places=1,
                msg=f"Channel {ch} stress mismatch"
            )

    def test_negative_values(self):
        """Test handling of negative voltage and stress values"""
        voltage_01mv = -500   # -50.0 mV
        strain_ue = -1000    # -1000 µε
        stress_01mpa = -30   # -3.0 MPa

        frame_data = self._create_combined_frame(0, voltage_01mv, strain_ue, stress_01mpa)

        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=frame_data)
        self.window.on_can_frame_received(frame)

        self.assertAlmostEqual(self.window.channel_data[0].voltage, -50.0, places=1)
        self.assertEqual(self.window.channel_data[0].strain, -1000.0)
        self.assertAlmostEqual(self.window.channel_data[0].stress, -3.0, places=1)

    def test_crc_error_rejected(self):
        """Test that frames with bad CRC are rejected"""
        # Create valid frame then corrupt CRC
        frame_data = self._create_combined_frame(0, 1000, 100, 50)
        frame_data = frame_data[:7] + bytes([0xFF])  # Corrupt CRC

        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=frame_data)
        self.window.on_can_frame_received(frame)

        # Data should not be updated (stays at default 0)
        self.assertEqual(self.window.channel_data[0].voltage, 0.0)
        self.assertEqual(self.window.channel_data[0].strain, 0.0)
        self.assertEqual(self.window.channel_data[0].stress, 0.0)

    def test_invalid_channel_rejected(self):
        """Test that invalid channel numbers are rejected"""
        frame_data = self._create_combined_frame(10, 1000, 100, 50)

        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=frame_data)
        # Should not crash, just log warning
        self.window.on_can_frame_received(frame)

    def test_wrong_can_id_rejected(self):
        """Test that frames with wrong CAN ID are ignored"""
        frame_data = self._create_combined_frame(0, 1000, 100, 50)

        from slcan_protocol import CANFrame
        frame = CANFrame(id=0x999, data=frame_data)  # Wrong ID
        self.window.on_can_frame_received(frame)

        # Data should not be updated
        self.assertEqual(self.window.channel_data[0].voltage, 0.0)

    def test_short_frame_rejected(self):
        """Test that frames shorter than 8 bytes are rejected"""
        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=bytes([0x05, 0, 0, 0, 0, 0]))  # Only 6 bytes
        self.window.on_can_frame_received(frame)

        # Data should not be updated
        self.assertEqual(self.window.channel_data[0].voltage, 0.0)

    def test_unknown_frame_type_rejected(self):
        """Test that unknown frame types are rejected"""
        # Create frame with type 0x01 instead of 0x05
        data = bytes([0x01, 0x00, 0x04, 0xD2, 0x00, 0x64, 0x32])
        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=data)
        self.window.on_can_frame_received(frame)

        # Data should not be updated
        self.assertEqual(self.window.channel_data[0].voltage, 0.0)


class TestCSVLogging(unittest.TestCase):
    """Test CSV logging functionality"""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()
        self.temp_file = tempfile.NamedTemporaryFile(
            mode='w', delete=False, suffix='.csv', newline=''
        )
        self.temp_file.close()
        self.csv_path = self.temp_file.name

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        try:
            os.unlink(self.csv_path)
        except:
            pass

    def test_csv_header_written(self):
        """Test that CSV file has correct header"""
        self.window.csv_file = open(self.csv_path, 'w', newline='')
        self.window.csv_writer = csv.writer(self.window.csv_file)
        self.window.csv_writer.writerow([
            'timestamp', 'channel',
            'voltage_mv', 'strain_ue', 'stress_mpa', 'displacement_derived'
        ])
        self.window.csv_file.close()

        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            self.assertEqual(header[0], 'timestamp')
            self.assertEqual(header[1], 'channel')
            self.assertEqual(header[2], 'voltage_mv')

    def test_csv_data_written(self):
        """Test that data rows are written correctly"""
        self.window.csv_file = open(self.csv_path, 'w', newline='')
        self.window.csv_writer = csv.writer(self.window.csv_file)
        self.window.csv_writer.writerow(['header'])
        self.window.logging_enabled = True

        # Simulate data update
        self.window.on_data_updated(0, {
            'voltage': 123.45,
            'strain': 500.0,
            'stress': 10.5,
            'displacement': 2.5
        })

        self.window.csv_file.close()

        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)
            self.assertEqual(len(rows), 2)  # header + 1 data row
            self.assertIn('123.45', rows[1][2])
            self.assertIn('500.0', rows[1][3])


class TestSLCANProtocol(unittest.TestCase):
    """Test SLCAN ASCII payload handling"""

    def test_send_standard_frame_uses_ascii_hex_payload(self):
        proto = SLCANProtocol()
        proto.serial = MagicMock()
        proto.serial.is_open = True
        proto.serial.read = MagicMock(return_value=b'\r')
        proto.is_open = True

        ok = proto.send_standard_data(0x123, bytes([0x11, 0x22, 0xAB]))

        self.assertTrue(ok)
        proto.serial.write.assert_called_once_with(b't12331122AB\r')

    def test_parse_standard_frame_decodes_ascii_hex_payload(self):
        proto = SLCANProtocol()

        frame = proto._parse_line(b't12331122AB')

        self.assertIsNotNone(frame)
        self.assertEqual(frame.id, 0x123)
        self.assertEqual(frame.data, bytes([0x11, 0x22, 0xAB]))


class TestChannelData(unittest.TestCase):
    """Test ChannelData dataclass"""

    def test_default_values(self):
        """Test default values are all zero"""
        data = ChannelData()
        self.assertEqual(data.voltage, 0.0)
        self.assertEqual(data.strain, 0.0)
        self.assertEqual(data.stress, 0.0)
        self.assertEqual(data.displacement, 0.0)
        self.assertEqual(data.raw_value, 0)

    def test_custom_values(self):
        """Test custom values can be set"""
        data = ChannelData(
            voltage=1.5,
            strain=2.6,
            stress=3.7,
            displacement=4.8,
            raw_value=12345
        )
        self.assertEqual(data.voltage, 1.5)
        self.assertEqual(data.strain, 2.6)


class TestWaveformBuffer(unittest.TestCase):
    """Test waveform buffer management"""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()

    def test_buffer_size_limit(self):
        """Test that waveform buffer doesn't exceed max size when updated via on_data_updated"""
        from reducer_monitor import WAVEFORM_BUFFER_SIZE

        # Simulate data updates via on_data_updated (which enforces limit)
        for i in range(WAVEFORM_BUFFER_SIZE + 100):
            self.window.on_data_updated(0, {
                'voltage': float(i),
                'strain': 0.0,
                'stress': 0.0,
                'displacement': 0.0
            })

        self.assertEqual(
            len(self.window.waveform_buffers[0]),
            WAVEFORM_BUFFER_SIZE
        )

    def test_stats_labels_update_with_voltage_history(self):
        self.window.on_data_updated(0, {
            'voltage': 10.0,
            'strain': 0.0,
            'stress': 0.0,
            'displacement': 0.0
        })
        self.window.on_data_updated(0, {
            'voltage': 20.0,
            'strain': 0.0,
            'stress': 0.0,
            'displacement': 0.0
        })

        min_lbl, max_lbl, avg_lbl = self.window.stats_labels[0]
        self.assertEqual(min_lbl.text(), "Min: 10.000 mV")
        self.assertEqual(max_lbl.text(), "Max: 20.000 mV")
        self.assertEqual(avg_lbl.text(), "Avg: 15.000 mV")


class TestConstants(unittest.TestCase):
    """Test that constants match expected values"""

    def test_frame_type(self):
        """Test frame type constant"""
        self.assertEqual(CAN_FRAME_COMBINED, 0x05)

    def test_can_ids(self):
        """Test CAN ID constants"""
        self.assertEqual(CAN_ID_TX_DATA, 0x101)
        self.assertEqual(CAN_ID_RX_CONFIG, 0x100)

    def test_num_channels(self):
        """Test channel count"""
        self.assertEqual(NUM_CHANNELS, 6)

    def test_command_constants(self):
        """Test all command type constants"""
        self.assertEqual(CAN_CMD_ZERO_DATUM, 0x03)
        self.assertEqual(CAN_CMD_SAVE_ZERO, 0x05)
        self.assertEqual(CAN_CMD_LOAD_ZERO, 0x06)
        self.assertEqual(CAN_CMD_CLEAR_ZERO, 0x07)
        self.assertEqual(CAN_CMD_SET_FILTER_SIZE, 0x02)
        self.assertEqual(CAN_CMD_START_CALIB, 0x04)


class TestDataUpdateSignal(unittest.TestCase):
    """Test data update signal handling"""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()
        self.received_updates = []

        # Connect a test handler
        self.window.data_updated.connect(self._on_data_updated)

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()

    def _on_data_updated(self, channel, data):
        self.received_updates.append((channel, data))

    def test_signal_emitted_on_valid_frame(self):
        """Test that data_updated signal is emitted for valid frame"""
        voltage = 2500  # 250.0 mV
        strain = 150
        stress = 75

        frame_data = self._create_combined_frame(2, voltage, strain, stress)

        from slcan_protocol import CANFrame
        frame = CANFrame(id=CAN_ID_TX_DATA, data=frame_data)
        self.window.on_can_frame_received(frame)

        self.assertEqual(len(self.received_updates), 1)
        ch, data = self.received_updates[0]
        self.assertEqual(ch, 2)
        self.assertAlmostEqual(data['voltage'], 250.0, places=1)
        self.assertEqual(data['strain'], 150.0)
        self.assertAlmostEqual(data['stress'], 7.5, places=1)

    def _create_combined_frame(self, channel, voltage_01mv, strain_ue, stress_01mpa):
        """Helper to create valid combined CAN frames"""
        voltage_bytes = voltage_01mv.to_bytes(2, byteorder='big', signed=True)
        strain_bytes = strain_ue.to_bytes(2, byteorder='big', signed=True)
        stress_byte = stress_01mpa.to_bytes(1, byteorder='big', signed=True)

        data = bytes([CAN_FRAME_COMBINED, channel]) + voltage_bytes + strain_bytes + stress_byte
        crc = crc8_xor(data)
        return data + bytes([crc])


class TestCommandSending(unittest.TestCase):
    """Test command sending functionality"""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = ReducerMonitorWindow()
        self.window.is_connected = True
        # Mock slcan to avoid actual serial operations
        self.window.slcan = MagicMock()
        self.window.slcan.send_frame = MagicMock(return_value=True)

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()

    def test_zero_datum_command(self):
        """Test ZERO_DATUM command is sent correctly"""
        self.window.on_zero_clicked()
        self.window.slcan.send_frame.assert_called()
        call_args = self.window.slcan.send_frame.call_args[0][0]
        self.assertEqual(call_args.id, CAN_ID_RX_CONFIG)
        self.assertEqual(call_args.data[0], CAN_CMD_ZERO_DATUM)

    def test_save_zero_command(self):
        """Test SAVE_ZERO command is sent correctly"""
        self.window.on_save_zero_clicked()
        self.window.slcan.send_frame.assert_called()
        call_args = self.window.slcan.send_frame.call_args[0][0]
        self.assertEqual(call_args.id, CAN_ID_RX_CONFIG)
        self.assertEqual(call_args.data[0], CAN_CMD_SAVE_ZERO)

    def test_load_zero_command(self):
        """Test LOAD_ZERO command is sent correctly"""
        self.window.on_load_zero_clicked()
        self.window.slcan.send_frame.assert_called()
        call_args = self.window.slcan.send_frame.call_args[0][0]
        self.assertEqual(call_args.id, CAN_ID_RX_CONFIG)
        self.assertEqual(call_args.data[0], CAN_CMD_LOAD_ZERO)

    def test_clear_zero_command(self):
        """Test CLEAR_ZERO command is sent correctly"""
        self.window.on_clear_zero_clicked()
        self.window.slcan.send_frame.assert_called()
        call_args = self.window.slcan.send_frame.call_args[0][0]
        self.assertEqual(call_args.id, CAN_ID_RX_CONFIG)
        self.assertEqual(call_args.data[0], CAN_CMD_CLEAR_ZERO)

    def test_calib_command(self):
        """Test START_CALIB command is sent correctly"""
        self.window.on_calib_clicked()
        self.window.slcan.send_frame.assert_called()
        call_args = self.window.slcan.send_frame.call_args[0][0]
        self.assertEqual(call_args.id, CAN_ID_RX_CONFIG)
        self.assertEqual(call_args.data[0], CAN_CMD_START_CALIB)

    def test_filter_size_command(self):
        """Test SET_FILTER_SIZE command is sent correctly"""
        self.window.on_filter_size_changed(32)
        self.window.slcan.send_frame.assert_called()
        call_args = self.window.slcan.send_frame.call_args[0][0]
        self.assertEqual(call_args.id, CAN_ID_RX_CONFIG)
        self.assertEqual(call_args.data[0], CAN_CMD_SET_FILTER_SIZE)
        # value is 32 in little-endian uint32 at bytes 2-5
        self.assertEqual(call_args.data[2], 32)
        self.assertEqual(call_args.data[3], 0)
        self.assertEqual(call_args.data[4], 0)
        self.assertEqual(call_args.data[5], 0)

    def test_command_fails_when_disconnected(self):
        """Test commands fail gracefully when not connected"""
        self.window.is_connected = False
        self.window.slcan = None
        result = self.window.send_command(CAN_CMD_ZERO_DATUM)
        self.assertFalse(result)


def run_tests():
    """Run all tests and return results"""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestCRCFunctions))
    suite.addTests(loader.loadTestsFromTestCase(TestCombinedFrameParsing))
    suite.addTests(loader.loadTestsFromTestCase(TestCSVLogging))
    suite.addTests(loader.loadTestsFromTestCase(TestSLCANProtocol))
    suite.addTests(loader.loadTestsFromTestCase(TestChannelData))
    suite.addTests(loader.loadTestsFromTestCase(TestWaveformBuffer))
    suite.addTests(loader.loadTestsFromTestCase(TestConstants))
    suite.addTests(loader.loadTestsFromTestCase(TestDataUpdateSignal))
    suite.addTests(loader.loadTestsFromTestCase(TestCommandSending))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")

    if result.wasSuccessful():
        print("\nALL TESTS PASSED!")
    else:
        print("\nSOME TESTS FAILED!")
        for fail in result.failures:
            print(f"\nFAILURE: {fail[0]}")
            print(fail[1])
        for err in result.errors:
            print(f"\nERROR: {err[0]}")
            print(err[1])

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
