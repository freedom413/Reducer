"""
reducer_monitor.py - Reducer Flexspline State Monitoring GUI

A PyQt6-based real-time monitoring application for the Reducer Flexspline
State Detection System. Uses python-can to communicate with USB-CAN adapters,
with SLCAN serial adapters as the primary transport.
"""

import csv
import datetime
import logging
import sys
import time
from collections import deque
from typing import Dict, List, Optional
from dataclasses import dataclass
from threading import Lock

# PyQt6 imports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QSpinBox, QStatusBar, QFileDialog,
    QMessageBox, QTabWidget, QTableWidget, QTableWidgetItem,
)
from PyQt6.QtCore import QTimer, pyqtSignal, QThread

# Plotting
import pyqtgraph as pg

from can_protocol import (
    Baudrate,
    CANFrame,
    CAN_ID_TX_STATUS,
    CAN_ID_TX_TELEMETRY,
    CAN_STATUS_BAD_CMD,
    CAN_STATUS_BAD_CRC,
    CAN_STATUS_BAD_TYPE,
    CAN_STATUS_BAD_VALUE,
    CAN_STATUS_OK,
    CAN_STATUS_STORAGE_ERROR,
    DEFAULT_SLCAN_TTY_BAUDRATE,
    PythonCANInterface,
    StatusFrame,
    SUPPORTED_SLCAN_SERIAL_BAUDRATES,
    available_interfaces,
    build_command_frame,
    list_can_channels,
    parse_status_frame,
    parse_telemetry_frame,
)

# Command types (must match embedded firmware)
CAN_CMD_SET_SAMPLE_RATE = 0x01
CAN_CMD_SET_FILTER_SIZE = 0x02
CAN_CMD_ZERO_DATUM = 0x03
CAN_CMD_START_CALIB = 0x04
CAN_CMD_SAVE_ZERO = 0x05
CAN_CMD_LOAD_ZERO = 0x06
CAN_CMD_CLEAR_ZERO = 0x07

# Number of channels
NUM_CHANNELS = 6

# Waveform buffer size
WAVEFORM_BUFFER_SIZE = 1000
COMMAND_ACK_TIMEOUT_S = 2.0

DEFAULT_ELASTIC_MODULUS_MPA = 210000.0
SUPPORTED_SAMPLE_RATES = [5, 10, 15, 25, 30, 50, 60, 100, 500, 1000]


logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class ChannelData:
    """Holds current data for a single channel"""
    voltage: float = 0.0
    strain: float = 0.0
    stress: float = 0.0
    displacement: float = 0.0
    raw_value: int = 0


class CANReceiver(QThread):
    """Background thread for forwarding python-can frames to the UI thread"""

    frame_received = pyqtSignal(object)

    def __init__(self, can_bus: PythonCANInterface):
        super().__init__()
        self.can_bus = can_bus
        self.running = True
        self._callback = None

    def run(self):
        def callback(frame: CANFrame):
            self.frame_received.emit(frame)

        self._callback = callback
        self.can_bus.register_rx_callback(self._callback)

        try:
            while self.running:
                QThread.msleep(100)
        finally:
            if self._callback is not None:
                self.can_bus.unregister_rx_callback(self._callback)
                self._callback = None

    def stop(self):
        self.running = False


class ReducerMonitorWindow(QMainWindow):
    """Main window for the Reducer Flexspline Monitor"""

    # Signal for updating UI from CAN receiver thread
    data_updated = pyqtSignal(int, dict)

    def __init__(self):
        super().__init__()

        # CAN protocol
        self.can_bus: Optional[PythonCANInterface] = None
        self.can_receiver: Optional[CANReceiver] = None
        self.command_sequence = 0
        self.pending_commands: Dict[int, str] = {}
        self.pending_command_deadlines: Dict[int, float] = {}

        # Channel data
        self.channel_data: List[ChannelData] = [ChannelData() for _ in range(NUM_CHANNELS)]
        self.channel_stats: List[Dict[str, Optional[float]]] = []
        self.lock = Lock()

        # Waveform data buffers
        self.waveform_buffers = [deque(maxlen=WAVEFORM_BUFFER_SIZE) for _ in range(NUM_CHANNELS)]
        self.plot_dirty = [False for _ in range(NUM_CHANNELS)]

        # CSV logging
        self.csv_writer: Optional[csv.writer] = None
        self.csv_file: Optional[object] = None
        self.logging_enabled = False

        # UI state
        self.is_connected = False

        # Setup UI
        self.init_ui()
        self._refresh_channels()
        self._update_connection_options()
        self._reset_measurements()

        # Connect signals
        self.data_updated.connect(self.on_data_updated)

        # Update timer for waveform plot
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(50)  # 20 Hz update rate

        self.command_timeout_timer = QTimer()
        self.command_timeout_timer.timeout.connect(self._check_command_timeouts)
        self.command_timeout_timer.start(250)

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Reducer Flexspline Monitor")
        self.setGeometry(100, 100, 1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        # Connection group
        conn_group = self._create_connection_group()
        main_layout.addWidget(conn_group)

        # Command group
        cmd_group = self._create_command_group()
        main_layout.addWidget(cmd_group)

        # Tab widget for waveforms and data
        tabs = QTabWidget()

        # Waveform tab
        waveform_tab = self._create_waveform_tab()
        tabs.addTab(waveform_tab, "Waveforms")

        # Data panel tab
        data_tab = self._create_data_panel_tab()
        tabs.addTab(data_tab, "Data Panel")

        main_layout.addWidget(tabs)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected")

    def _create_connection_group(self) -> QGroupBox:
        """Create the CAN connection configuration group"""
        group = QGroupBox("CAN Connection")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("Interface:"))
        self.interface_combo = QComboBox()
        for interface_name, label in available_interfaces():
            self.interface_combo.addItem(label, interface_name)
        self.interface_combo.currentIndexChanged.connect(self._on_interface_changed)
        layout.addWidget(self.interface_combo)

        layout.addWidget(QLabel("Channel:"))
        self.channel_combo = QComboBox()
        self.channel_combo.setEditable(True)
        layout.addWidget(self.channel_combo)

        self.serial_baud_label = QLabel("Adapter Baud:")
        layout.addWidget(self.serial_baud_label)
        self.serial_baud_combo = QComboBox()
        for serial_baudrate in SUPPORTED_SLCAN_SERIAL_BAUDRATES:
            self.serial_baud_combo.addItem(str(serial_baudrate), serial_baudrate)
        serial_baud_index = self.serial_baud_combo.findData(DEFAULT_SLCAN_TTY_BAUDRATE)
        if serial_baud_index >= 0:
            self.serial_baud_combo.setCurrentIndex(serial_baud_index)
        layout.addWidget(self.serial_baud_combo)

        # CAN bitrate selection
        layout.addWidget(QLabel("CAN Baudrate:"))
        self.baud_combo = QComboBox()
        for br in Baudrate:
            self.baud_combo.addItem(f"{br.bps // 1000}K", br)
        self.baud_combo.setCurrentText("500K")
        layout.addWidget(self.baud_combo)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_btn)

        # Refresh ports button
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_channels)
        layout.addWidget(refresh_btn)

        layout.addStretch()

        # Log button
        self.log_btn = QPushButton("Start Logging")
        self.log_btn.clicked.connect(self.on_log_clicked)
        self.log_btn.setEnabled(False)
        layout.addWidget(self.log_btn)

        slcan_index = self.interface_combo.findData("slcan")
        if slcan_index >= 0:
            self.interface_combo.setCurrentIndex(slcan_index)

        group.setLayout(layout)
        return group

    def _create_waveform_tab(self) -> QWidget:
        """Create the waveform display tab with 6 channel plots"""
        widget = QWidget()
        layout = QGridLayout(widget)

        # Create 6 plot widgets in a 2x3 grid
        self.plot_widgets = []
        self.plot_curves = []

        for i in range(NUM_CHANNELS):
            # Create plot widget
            pw = pg.PlotWidget(title=f"CH{i} Voltage")
            pw.setYRange(-3000, 3000)
            pw.setLabel('left', 'Voltage', units='mV')
            pw.setLabel('bottom', 'Sample')
            pw.showGrid(x=True, y=True, alpha=0.3)

            # Create curve
            curve = pw.plot(pen=pg.mkPen(color=self._get_channel_color(i), width=1.5))
            self.plot_widgets.append(pw)
            self.plot_curves.append(curve)

            row = i // 3
            col = i % 3
            layout.addWidget(pw, row, col)

        return widget

    def _create_data_panel_tab(self) -> QWidget:
        """Create the numerical data display panel"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Table for data display
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(5)
        self.data_table.setHorizontalHeaderLabels(["Channel", "Voltage (mV)", "Strain (µε)", "Stress (MPa)", "Displacement (µm)"])
        self.data_table.setRowCount(NUM_CHANNELS)

        # Set channel numbers and initialize data columns
        for i in range(NUM_CHANNELS):
            self.data_table.setItem(i, 0, QTableWidgetItem(f"CH{i}"))
            self.data_table.setItem(i, 1, QTableWidgetItem("-"))
            self.data_table.setItem(i, 2, QTableWidgetItem("-"))
            self.data_table.setItem(i, 3, QTableWidgetItem("-"))
            self.data_table.setItem(i, 4, QTableWidgetItem("-"))

        # Stretch columns
        header = self.data_table.horizontalHeader()
        header.setStretchLastSection(True)

        layout.addWidget(self.data_table)

        # Statistics section
        stats_group = QGroupBox("Statistics (since connect)")
        stats_layout = QGridLayout()

        self.stats_labels = []
        for ch in range(NUM_CHANNELS):
            ch_label = QLabel(f"CH{ch}:")
            stats_layout.addWidget(ch_label, ch, 0)

            min_lbl = QLabel("Min: -")
            max_lbl = QLabel("Max: -")
            avg_lbl = QLabel("Avg: -")
            self.stats_labels.append((min_lbl, max_lbl, avg_lbl))

            stats_layout.addWidget(min_lbl, ch, 1)
            stats_layout.addWidget(max_lbl, ch, 2)
            stats_layout.addWidget(avg_lbl, ch, 3)

        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)

        return widget

    def _on_interface_changed(self, _index: int):
        self._refresh_channels()
        self._update_connection_options()

    def _refresh_channels(self):
        """Refresh the list of available CAN channels"""
        interface = self.interface_combo.currentData()
        selected_channel = self._selected_channel()
        self.channel_combo.clear()
        for channel, desc in list_can_channels(interface):
            label = channel if not desc or desc == channel else f"{channel} - {desc}"
            self.channel_combo.addItem(label, channel)
        if self.channel_combo.count() == 0:
            fallback_channel = "COM1" if interface == "slcan" else "can0"
            self.channel_combo.addItem(fallback_channel, fallback_channel)

        if selected_channel:
            existing_index = self.channel_combo.findData(selected_channel)
            if existing_index >= 0:
                self.channel_combo.setCurrentIndex(existing_index)
            else:
                self.channel_combo.setEditText(selected_channel)

    def _update_connection_options(self):
        uses_serial_adapter = self.interface_combo.currentData() == "slcan"
        self.serial_baud_label.setEnabled(uses_serial_adapter)
        self.serial_baud_combo.setEnabled(uses_serial_adapter)

    def _selected_channel(self) -> str:
        text = self.channel_combo.currentText().strip()
        data = self.channel_combo.currentData()
        current_index = self.channel_combo.currentIndex()

        if current_index >= 0:
            item_text = self.channel_combo.itemText(current_index).strip()
            item_data = self.channel_combo.itemData(current_index)
            expected_text = str(item_data) if item_data is not None else item_text
            if text and text not in (item_text, expected_text):
                return text

        if data is not None:
            return str(data).strip()
        return text

    def _get_channel_color(self, ch: int) -> str:
        """Get a unique color for each channel"""
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        return colors[ch % len(colors)]

    def _reset_measurements(self):
        with self.lock:
            self.channel_data = [ChannelData() for _ in range(NUM_CHANNELS)]
            self.waveform_buffers = [deque(maxlen=WAVEFORM_BUFFER_SIZE) for _ in range(NUM_CHANNELS)]
            self.plot_dirty = [False for _ in range(NUM_CHANNELS)]
            self.channel_stats = [
                {'min': None, 'max': None, 'sum': 0.0, 'count': 0}
                for _ in range(NUM_CHANNELS)
            ]

        if hasattr(self, 'data_table'):
            for row in range(NUM_CHANNELS):
                for col in range(1, 5):
                    self.data_table.item(row, col).setText("-")

        if hasattr(self, 'stats_labels'):
            for min_lbl, max_lbl, avg_lbl in self.stats_labels:
                min_lbl.setText("Min: -")
                max_lbl.setText("Max: -")
                avg_lbl.setText("Avg: -")

        if hasattr(self, 'plot_curves'):
            for curve in self.plot_curves:
                curve.setData([])

    def on_connect_clicked(self):
        """Handle connect/disconnect button click"""
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()

    def send_command(self, cmd_type: int, param: int = 0, value: int = 0) -> bool:
        """Send command to embedded device"""
        if not self.can_bus or not self.is_connected:
            return False
        frame = build_command_frame(self.command_sequence, cmd_type, param, value)
        command_name = self._command_name(cmd_type)
        sent = self.can_bus.send_frame(frame)
        if sent:
            self.pending_commands[self.command_sequence] = command_name
            self.pending_command_deadlines[self.command_sequence] = time.monotonic() + COMMAND_ACK_TIMEOUT_S
            self.command_sequence = (self.command_sequence + 1) & 0xFF
        return sent

    def on_zero_clicked(self):
        """Handle Zero Sensor button click - saves zero offset to Flash"""
        if self.send_command(CAN_CMD_ZERO_DATUM):
            self.status_bar.showMessage("Zero command sent, waiting for device ACK")
            logger.info("Zero command sent")
        else:
            self.status_bar.showMessage("Failed to send zero command")
            logger.warning("Failed to send zero command")

    def on_calib_clicked(self):
        """Handle Calibrate button click"""
        if self.send_command(CAN_CMD_START_CALIB):
            self.status_bar.showMessage("Calibration command sent, waiting for device ACK")
            logger.info("Calibration command sent")
        else:
            self.status_bar.showMessage("Failed to send calibration command")
            logger.warning("Failed to send calibration command")

    def on_filter_size_changed(self, value):
        """Handle filter size spinbox change"""
        if self.send_command(CAN_CMD_SET_FILTER_SIZE, param=0, value=value):
            logger.info(f"Filter size set to {value}")
        else:
            self.status_bar.showMessage("Failed to send filter size command")
            logger.warning("Failed to send filter size command")

    def on_sample_rate_changed(self, index: int):
        """Handle sample rate combobox change"""
        if index < 0 or not self.is_connected:
            return

        sample_rate = self.sample_rate_combo.currentData()
        if sample_rate is None:
            return

        if self.send_command(CAN_CMD_SET_SAMPLE_RATE, param=0, value=int(sample_rate)):
            logger.info("Sample rate set to %s SPS", sample_rate)
        else:
            self.status_bar.showMessage("Failed to send sample rate command")
            logger.warning("Failed to send sample rate command")

    def on_save_zero_clicked(self):
        """Handle Save Zero button click - saves current offsets to Flash"""
        if self.send_command(CAN_CMD_SAVE_ZERO):
            self.status_bar.showMessage("Save Zero command sent, waiting for device ACK")
            logger.info("Save Zero command sent")
        else:
            self.status_bar.showMessage("Failed to save zero offset")
            logger.warning("Failed to save zero offset")

    def on_load_zero_clicked(self):
        """Handle Load Zero button click - loads offsets from Flash"""
        if self.send_command(CAN_CMD_LOAD_ZERO):
            self.status_bar.showMessage("Load Zero command sent, waiting for device ACK")
            logger.info("Load Zero command sent")
        else:
            self.status_bar.showMessage("Failed to load zero offset")
            logger.warning("Failed to load zero offset")

    def on_clear_zero_clicked(self):
        """Handle Clear Zero button click - clears offsets from Flash"""
        if self.send_command(CAN_CMD_CLEAR_ZERO):
            self.status_bar.showMessage("Clear Zero command sent, waiting for device ACK")
            logger.info("Clear Zero command sent")
        else:
            self.status_bar.showMessage("Failed to clear zero offset")
            logger.warning("Failed to clear zero offset")

    def _create_command_group(self) -> QGroupBox:
        """Create the command control group"""
        group = QGroupBox("Commands")
        layout = QHBoxLayout()

        self.zero_btn = QPushButton("Zero Sensor")
        self.zero_btn.clicked.connect(self.on_zero_clicked)
        self.zero_btn.setEnabled(False)
        layout.addWidget(self.zero_btn)

        self.calib_btn = QPushButton("Calibrate")
        self.calib_btn.clicked.connect(self.on_calib_clicked)
        self.calib_btn.setEnabled(False)
        layout.addWidget(self.calib_btn)

        # Zero offset Flash storage controls
        self.save_zero_btn = QPushButton("Save Zero")
        self.save_zero_btn.clicked.connect(self.on_save_zero_clicked)
        self.save_zero_btn.setEnabled(False)
        layout.addWidget(self.save_zero_btn)

        self.load_zero_btn = QPushButton("Load Zero")
        self.load_zero_btn.clicked.connect(self.on_load_zero_clicked)
        self.load_zero_btn.setEnabled(False)
        layout.addWidget(self.load_zero_btn)

        self.clear_zero_btn = QPushButton("Clear Zero")
        self.clear_zero_btn.clicked.connect(self.on_clear_zero_clicked)
        self.clear_zero_btn.setEnabled(False)
        layout.addWidget(self.clear_zero_btn)

        # Filter size control
        layout.addWidget(QLabel("Sample Rate:"))
        self.sample_rate_combo = QComboBox()
        for sample_rate in SUPPORTED_SAMPLE_RATES:
            self.sample_rate_combo.addItem(f"{sample_rate} SPS", sample_rate)
        self.sample_rate_combo.setCurrentText("100 SPS")
        self.sample_rate_combo.setEnabled(False)
        self.sample_rate_combo.currentIndexChanged.connect(self.on_sample_rate_changed)
        layout.addWidget(self.sample_rate_combo)

        # Filter size control
        layout.addWidget(QLabel("Filter Size:"))
        self.filter_size_spin = QSpinBox()
        self.filter_size_spin.setMinimum(2)
        self.filter_size_spin.setMaximum(64)
        self.filter_size_spin.setValue(16)
        self.filter_size_spin.setEnabled(False)
        self.filter_size_spin.valueChanged.connect(self.on_filter_size_changed)
        layout.addWidget(self.filter_size_spin)

        layout.addStretch()

        group.setLayout(layout)
        return group

    def connect(self):
        """Connect to the CAN adapter"""
        interface = self.interface_combo.currentData()
        channel = self._selected_channel()
        if not channel:
            QMessageBox.warning(self, "Error", "Please select a CAN channel or serial port")
            return

        baudrate = self.baud_combo.currentData()
        tty_baudrate = int(self.serial_baud_combo.currentData() or DEFAULT_SLCAN_TTY_BAUDRATE)

        try:
            self.can_bus = PythonCANInterface()
            if not self.can_bus.connect(
                interface,
                channel,
                baudrate,
                tty_baudrate=tty_baudrate,
            ):
                interface_help = (
                    "Check the COM port name, adapter baudrate, and CAN bitrate."
                    if interface == "slcan"
                    else "If using socketcan, make sure the interface is already up."
                )
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to connect to {interface}:{channel}\n"
                    f"{interface_help}",
                )
                self.can_bus = None
                return

            # Start receiver thread
            self.can_receiver = CANReceiver(self.can_bus)
            self.can_receiver.frame_received.connect(self.on_can_frame_received)
            self.can_receiver.start()
            self._reset_measurements()
            self.pending_commands.clear()
            self.command_sequence = 0

            self.is_connected = True
            self.connect_btn.setText("Disconnect")
            self.log_btn.setEnabled(True)
            self.zero_btn.setEnabled(True)
            self.calib_btn.setEnabled(True)
            self.save_zero_btn.setEnabled(True)
            self.load_zero_btn.setEnabled(True)
            self.clear_zero_btn.setEnabled(True)
            self.sample_rate_combo.setEnabled(True)
            self.filter_size_spin.setEnabled(True)
            if interface == "slcan":
                self.status_bar.showMessage(
                    f"Connected to {channel} via slcan (adapter {tty_baudrate}, CAN {baudrate.bps} bps)"
                )
            else:
                self.status_bar.showMessage(
                    f"Connected to {interface}:{channel} at {baudrate.bps} bps"
                )

            logger.info(
                "Connected to %s:%s (adapter baud %s, CAN bitrate %s)",
                interface,
                channel,
                tty_baudrate if interface == "slcan" else "n/a",
                baudrate.bps,
            )

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Connection failed: {e}")
            logger.error(f"Connection failed: {e}")
            if self.can_bus:
                self.can_bus.disconnect()
                self.can_bus = None

    def disconnect(self):
        """Disconnect from the CAN adapter"""
        if self.logging_enabled:
            self.stop_logging()

        if self.can_receiver:
            self.can_receiver.stop()
            self.can_receiver.wait(1000)
            self.can_receiver = None

        if self.can_bus:
            self.can_bus.disconnect()
            self.can_bus = None

        self.is_connected = False
        self.pending_commands.clear()
        self.pending_command_deadlines.clear()
        self._reset_measurements()
        self.connect_btn.setText("Connect")
        self.log_btn.setEnabled(False)
        self.zero_btn.setEnabled(False)
        self.calib_btn.setEnabled(False)
        self.save_zero_btn.setEnabled(False)
        self.load_zero_btn.setEnabled(False)
        self.clear_zero_btn.setEnabled(False)
        self.sample_rate_combo.setEnabled(False)
        self.filter_size_spin.setEnabled(False)
        self.status_bar.showMessage("Disconnected")
        logger.info("Disconnected")

    def on_can_frame_received(self, frame: CANFrame):
        """Handle received CAN frames from the adapter"""
        status = parse_status_frame(frame)
        if status is not None:
            self._handle_status_frame(status)
            return

        telemetry = parse_telemetry_frame(frame)
        if telemetry is None:
            if frame.id in (CAN_ID_TX_TELEMETRY, CAN_ID_TX_STATUS):
                logger.warning("Rejected malformed protocol frame on CAN ID 0x%03X", frame.id)
            return

        channel = telemetry.channel

        if channel >= NUM_CHANNELS:
            logger.warning(f"Invalid channel: {channel}")
            return

        with self.lock:
            self.channel_data[channel].voltage = telemetry.voltage_01mv / 10.0
            self.channel_data[channel].strain = float(telemetry.strain_ue)
            self.channel_data[channel].stress = (
                self.channel_data[channel].strain * DEFAULT_ELASTIC_MODULUS_MPA / 1_000_000.0
            )
            self.channel_data[channel].displacement = self.channel_data[channel].stress * 0.01

        self.data_updated.emit(channel, {
            'voltage': self.channel_data[channel].voltage,
            'strain': self.channel_data[channel].strain,
            'stress': self.channel_data[channel].stress,
            'displacement': self.channel_data[channel].displacement
        })

    def on_data_updated(self, channel: int, data: dict):
        """Handle data update signal"""
        # Update waveform buffer
        self.waveform_buffers[channel].append(data['voltage'])
        self.plot_dirty[channel] = True

        # Update data table
        if channel < self.data_table.rowCount():
            self.data_table.item(channel, 1).setText(f"{data['voltage']:.3f}")
            self.data_table.item(channel, 2).setText(f"{data['strain']:.2f}")
            self.data_table.item(channel, 3).setText(f"{data['stress']:.4f}")
            self.data_table.item(channel, 4).setText(f"{data['displacement']:.4f}")

        stats = self.channel_stats[channel]
        voltage = data['voltage']
        stats['count'] += 1
        stats['sum'] += voltage
        stats['min'] = voltage if stats['min'] is None else min(stats['min'], voltage)
        stats['max'] = voltage if stats['max'] is None else max(stats['max'], voltage)
        avg = stats['sum'] / stats['count']
        min_lbl, max_lbl, avg_lbl = self.stats_labels[channel]
        min_lbl.setText(f"Min: {stats['min']:.3f} mV")
        max_lbl.setText(f"Max: {stats['max']:.3f} mV")
        avg_lbl.setText(f"Avg: {avg:.3f} mV")

        # Write to CSV if logging
        if self.logging_enabled and self.csv_writer:
            timestamp = datetime.datetime.now().isoformat()
            row = [timestamp, channel,
                   data['voltage'], data['strain'],
                   data['stress'], data['displacement']]
            self.csv_writer.writerow(row)

    def update_plots(self):
        """Update waveform plots (called by timer)"""
        with self.lock:
            for ch in range(NUM_CHANNELS):
                if self.plot_dirty[ch]:
                    self.plot_curves[ch].setData(list(self.waveform_buffers[ch]))
                    self.plot_dirty[ch] = False

    def on_log_clicked(self):
        """Handle logging button click"""
        if self.logging_enabled:
            self.stop_logging()
        else:
            self.start_logging()

    def start_logging(self):
        """Start CSV logging"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Log File",
            f"reducer_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )

        if not filename:
            return

        try:
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp', 'channel',
                'voltage_mv', 'strain_ue', 'stress_mpa', 'displacement_derived'
            ])
            self.logging_enabled = True
            self.log_btn.setText("Stop Logging")
            self.status_bar.showMessage(f"Logging to {filename}")
            logger.info(f"Started logging to {filename}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start logging: {e}")
            logger.error(f"Failed to start logging: {e}")

    def stop_logging(self):
        """Stop CSV logging"""
        if self.csv_file:
            try:
                self.csv_file.close()
            except:
                pass
            self.csv_file = None
        self.csv_writer = None
        self.logging_enabled = False
        self.log_btn.setText("Start Logging")
        self.status_bar.showMessage("Logging stopped")
        logger.info("Stopped logging")

    def closeEvent(self, event):
        """Handle window close event"""
        if self.logging_enabled:
            self.stop_logging()

        if self.is_connected:
            self.disconnect()

        event.accept()

    def _command_name(self, cmd_type: int) -> str:
        command_names = {
            CAN_CMD_SET_SAMPLE_RATE: "Set Sample Rate",
            CAN_CMD_SET_FILTER_SIZE: "Set Filter Size",
            CAN_CMD_ZERO_DATUM: "Zero Sensor",
            CAN_CMD_START_CALIB: "Calibrate",
            CAN_CMD_SAVE_ZERO: "Save Zero",
            CAN_CMD_LOAD_ZERO: "Load Zero",
            CAN_CMD_CLEAR_ZERO: "Clear Zero",
        }
        return command_names.get(cmd_type, f"Command 0x{cmd_type:02X}")

    def _handle_status_frame(self, status: StatusFrame) -> None:
        command_name = self.pending_commands.pop(status.sequence, self._command_name(status.cmd_type))
        self.pending_command_deadlines.pop(status.sequence, None)
        if status.status == CAN_STATUS_OK:
            self.status_bar.showMessage(f"{command_name} acknowledged")
            logger.info("%s acknowledged, value=%u", command_name, status.value)
            return

        reason = {
            CAN_STATUS_BAD_CRC: "CRC mismatch",
            CAN_STATUS_BAD_TYPE: "invalid frame type",
            CAN_STATUS_BAD_CMD: "unsupported command",
            CAN_STATUS_BAD_VALUE: "invalid value",
            CAN_STATUS_STORAGE_ERROR: "storage error",
        }.get(status.status, f"status 0x{status.status:02X}")
        self.status_bar.showMessage(f"{command_name} rejected: {reason}")
        logger.warning(
            "%s rejected: %s (detail=0x%02X, value=%u)",
            command_name,
            reason,
            status.detail,
            status.value,
        )

    def _check_command_timeouts(self) -> None:
        now = time.monotonic()
        expired = [
            sequence for sequence, deadline in self.pending_command_deadlines.items()
            if deadline <= now
        ]
        for sequence in expired:
            self.pending_command_deadlines.pop(sequence, None)
            command_name = self.pending_commands.pop(sequence, f"Command sequence {sequence}")
            self.status_bar.showMessage(f"{command_name} timed out waiting for ACK")
            logger.warning("%s timed out waiting for ACK", command_name)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Set application info
    app.setApplicationName("Reducer Flexspline Monitor")
    app.setOrganizationName("ReducerProject")

    # Create and show window
    window = ReducerMonitorWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
