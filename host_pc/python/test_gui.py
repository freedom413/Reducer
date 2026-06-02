"""
test_gui.py - Tests for the reducer monitor GUI and CAN protocol helpers.
"""

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

from can_protocol import (
    CANFrame,
    CAN_FRAME_TYPE_STATUS,
    CAN_FRAME_TYPE_TELEMETRY,
    CAN_FRAME_TYPE_HEALTH,
    CAN_HEALTH_VERSION,
    CAN_ID_RX_COMMAND,
    CAN_ID_TX_HEALTH,
    CAN_ID_TX_STATUS,
    CAN_ID_TX_TELEMETRY,
    CAN_STATUS_BAD_VALUE,
    CAN_STATUS_OK,
    CAN_STATUS_STORAGE_ERROR,
    CAN_FD_DATA_BITRATE,
    DEFAULT_CAN_SEND_TIMEOUT_S,
    DEFAULT_SLCAN_TTY_BAUDRATE,
    Baudrate,
    PythonCANInterface,
    _PythonCanListener,
    available_interfaces,
    build_command_frame,
    crc8_xor,
    list_can_channels,
    parse_status_frame,
    parse_telemetry_frame,
    parse_health_frame,
)
from reducer_monitor import (
    CAN_CMD_CLEAR_ZERO,
    CAN_CMD_LOAD_ZERO,
    CAN_CMD_SAVE_ZERO,
    CAN_CMD_SET_CHANNEL_MASK,
    CAN_CMD_SET_SAMPLE_RATE,
    CAN_CMD_SET_FILTER_SIZE,
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
        payload = build_telemetry_payload(channel=2, voltage_001mv=1234, strain_ue=-456, stress_01mpa=-12)
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=payload)

        parsed = parse_telemetry_frame(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.channel, 2)
        self.assertEqual(parsed.voltage_001mv, 1234)
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

    def test_parse_health_frame(self):
        parsed = parse_health_frame(
            CANFrame(id=CAN_ID_TX_HEALTH, data=build_health_payload())
        )

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.sample_rate_sps, 30000)
        self.assertEqual(parsed.telemetry_decimation, 3)
        self.assertEqual(parsed.tx_drop_count, 4)
        self.assertEqual(parsed.adc_overflow_count, 5)
        self.assertEqual(parsed.adc_recovery_count, 6)
        self.assertEqual(parsed.active_adc_count, 2)
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
        ):
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
            bus.shutdown()

        serial_writes = [call.args[0] for call in serial_port.write.call_args_list]
        self.assertEqual(serial_writes[:4], [b"C\r", b"S6\r", b"Y2\r", b"O\r"])
        self.assertIn(b"b1008A001020304050607\r", serial_writes)

    def test_default_slcan_tty_baudrate_matches_canable2_cdc_compatibility_value(self):
        self.assertEqual(DEFAULT_SLCAN_TTY_BAUDRATE, 115200)

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

    def test_default_waveform_plot_count(self):
        self.assertEqual(len(self.window.plot_panels), DEFAULT_VISIBLE_PLOTS)
        self.assertEqual(self.window.plot_channels, [0, 1, 2, 3])

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

        self.window._add_waveform_plot()
        self.assertEqual(len(self.window.plot_panels), MAX_VISIBLE_PLOTS)

    def test_adding_plot_subscribes_mcu_channel(self):
        self.window._add_waveform_plot()

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_CHANNEL_MASK)
        self.assertEqual(sent_frame.data[4], 0x1F)

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
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_CHANNEL_MASK)
        self.assertEqual(sent_frame.data[4], 0x2E)

    def test_waveform_plot_can_be_removed(self):
        panel = self.window.plot_panels[-1]

        self.window._remove_waveform_plot(panel)

        self.assertEqual(len(self.window.plot_panels), DEFAULT_VISIBLE_PLOTS - 1)
        self.assertNotIn(panel, self.window.plot_panels)
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_CHANNEL_MASK)
        self.assertEqual(sent_frame.data[4], 0x07)

    def test_removing_all_waveform_plots_stops_mcu_sampling(self):
        for panel in list(self.window.plot_panels):
            self.window._remove_waveform_plot(panel)

        self.assertEqual(self.window.plot_panels, [])
        self.assertEqual(self.window._displayed_channel_mask(), 0)
        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_CHANNEL_MASK)
        self.assertEqual(sent_frame.data[4], 0x00)

    def test_duplicate_plots_subscribe_mcu_channel_once(self):
        self.window._sync_mcu_channel_mask()
        self.window.can_bus.send_frame.reset_mock()

        self.window._add_waveform_plot(channel=0)

        self.assertEqual(self.window._displayed_channel_mask(), 0x0F)
        self.window.can_bus.send_frame.assert_not_called()

    def test_parse_valid_telemetry_updates_channel(self):
        frame = CANFrame(
            id=CAN_ID_TX_TELEMETRY,
            data=build_telemetry_payload(channel=1, voltage_001mv=2345, strain_ue=321, stress_01mpa=45),
        )

        self.window.on_can_frame_received(frame)

        self.assertAlmostEqual(self.window.channel_data[1].voltage_mv, 23.45, places=2)
        self.assertEqual(self.window.channel_data[1].strain_ue, 321.0)
        self.assertAlmostEqual(self.window.channel_data[1].stress_mpa, 67.41, places=3)
        self.assertEqual(self.window.channel_data[1].samples, 1)

    def test_bad_telemetry_crc_is_rejected(self):
        payload = bytearray(build_telemetry_payload(channel=0, voltage_001mv=1000, strain_ue=100, stress_01mpa=10))
        payload[-1] ^= 0xAA
        frame = CANFrame(id=CAN_ID_TX_TELEMETRY, data=bytes(payload))

        self.window.on_can_frame_received(frame)

        self.assertEqual(self.window.channel_data[0].voltage_mv, 0.0)

    def test_status_frame_updates_status_bar(self):
        self.window.pending_commands[7] = "Set Filter Size"
        frame = CANFrame(
            id=CAN_ID_TX_STATUS,
            data=build_status_payload(sequence=7, cmd_type=CAN_CMD_SET_FILTER_SIZE, status=CAN_STATUS_OK, value=32),
        )

        self.window.on_can_frame_received(frame)

        self.assertEqual(self.window.status_bar.currentMessage(), "Set Filter Size acknowledged (value=32)")

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

    def test_storage_error_status_updates_status_bar(self):
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

    def test_all_ads1256_sample_rates_are_available(self):
        self.assertEqual(
            SUPPORTED_SAMPLE_RATES,
            [2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
             3750, 7500, 15000, 30000],
        )

    def test_fractional_sample_rate_uses_deci_sps_command_encoding(self):
        self.window.sample_rate_combo.setCurrentText("2.5 SPS")

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_SAMPLE_RATE)
        self.assertEqual(sent_frame.data[3], 1)
        self.assertEqual(sent_frame.data[4], 25)
        self.assertEqual(sent_frame.data[5], 0)

    def test_high_ads_rate_uses_bounded_telemetry_decimation(self):
        self.assertEqual(self.window._telemetry_decimation(30000), 2)

        self.window._add_waveform_plot()

        self.assertEqual(self.window._telemetry_decimation(30000), 3)

    def test_health_frame_updates_system_health_panel(self):
        self.window.on_can_frame_received(
            CANFrame(id=CAN_ID_TX_HEALTH, data=build_health_payload())
        )

        self.assertEqual(self.window.sample_rate_sps, 30000)
        self.assertIn("TX drops 4", self.window.health_summary_label.text())
        self.assertIn("ADC overflows 5", self.window.health_summary_label.text())

    def test_high_sample_rate_is_available_over_canable2_usb_cdc(self):
        self.window.sample_rate_combo.setCurrentText("30000 SPS")

        sent_frame = self.window.can_bus.send_frame.call_args[0][0]
        self.assertEqual(sent_frame.data[2], CAN_CMD_SET_SAMPLE_RATE)
        self.assertEqual(sent_frame.data[4], 0x30)
        self.assertEqual(sent_frame.data[5], 0x75)

    def test_connection_options_can_be_locked(self):
        self.window.is_connected = False
        self.window._set_connection_controls_enabled(False)

        self.assertFalse(self.window.interface_combo.isEnabled())
        self.assertFalse(self.window.channel_combo.isEnabled())
        self.assertFalse(hasattr(self.window, "serial_baud_combo"))
        self.assertFalse(hasattr(self.window, "baud_combo"))
        self.assertEqual(self.window.can_baud_value.text(), "500K / 2M FD+BRS")
        self.assertFalse(self.window.refresh_btn.isEnabled())
        self.assertTrue(self.window.language_combo.isEnabled())

    def test_can_bitrate_is_fixed_to_match_mcu_firmware(self):
        self.assertEqual(FIXED_CAN_BITRATE, Baudrate.BAUD_500K)
        self.assertFalse(hasattr(self.window, "baud_combo"))
        self.assertEqual(self.window.can_baud_value.text(), "500K / 2M FD+BRS")

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
        offline_window._toggle_plot_maximize(0)
        self.assertIsNone(offline_window.maximized_plot_channel)
        self.assertEqual(offline_window.waveform_layout.count(), NUM_CHANNELS)
        offline_window.close()


def run_tests():
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return result.wasSuccessful()


if __name__ == "__main__":
    sys.exit(0 if run_tests() else 1)
