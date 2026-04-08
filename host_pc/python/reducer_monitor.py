"""
reducer_monitor.py - Reducer Flexspline State Monitoring GUI

A PyQt6-based real-time monitoring application for the Reducer Flexspline
State Detection System. Displays 6-channel ADC data received via CAN (slcan).

Features:
- Real-time multi-channel waveform display using pyqtgraph
- Data panel showing voltage, strain, stress, displacement
- CSV data logging
- Connection status and error handling
"""

import sys
import os
import csv
import datetime
import logging
from typing import Dict, List, Optional
from dataclasses import dataclass
from threading import Lock

# PyQt6 imports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QSpinBox, QDoubleSpinBox, QTextEdit, QStatusBar, QFileDialog,
    QMessageBox, QTabWidget, QTableWidget, QTableWidgetItem,
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont

# Plotting
import pyqtgraph as pg

# Serial/CAN
import serial.tools.list_ports
import struct
from slcan_protocol import SLCANProtocol, CANFrame, Baudrate, list_serial_ports


def crc8_xor(data: bytes) -> int:
    """Calculate CRC-8 XOR checksum (for combined frames)"""
    crc = 0
    for byte in data:
        crc ^= byte
    return crc

# Constants
from slcan_protocol import CAN_ID_TX_DATA, CAN_ID_RX_CONFIG

# Frame type - only combined frame (0x05) is used
CAN_FRAME_COMBINED = 0x05

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
    """Background thread for receiving CAN frames"""

    frame_received = pyqtSignal(CANFrame)

    def __init__(self, slcan: SLCANProtocol):
        super().__init__()
        self.slcan = slcan
        self.running = True
        self._callback = None

    def run(self):
        def callback(frame: CANFrame):
            self.frame_received.emit(frame)

        self._callback = callback
        self.slcan.register_rx_callback(self._callback)

        try:
            while self.running:
                QThread.msleep(100)
        finally:
            if self._callback is not None:
                self.slcan.unregister_rx_callback(self._callback)
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
        self.slcan: Optional[SLCANProtocol] = None
        self.can_receiver: Optional[CANReceiver] = None

        # Channel data
        self.channel_data: List[ChannelData] = [ChannelData() for _ in range(NUM_CHANNELS)]
        self.channel_stats: List[Dict[str, Optional[float]]] = []
        self.lock = Lock()

        # Waveform data buffers
        self.waveform_buffers: List[List[float]] = [[] for _ in range(NUM_CHANNELS)]

        # CSV logging
        self.csv_writer: Optional[csv.writer] = None
        self.csv_file: Optional[object] = None
        self.logging_enabled = False

        # UI state
        self.is_connected = False

        # Setup UI
        self.init_ui()
        self._reset_measurements()

        # Connect signals
        self.data_updated.connect(self.on_data_updated)

        # Update timer for waveform plot
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(50)  # 20 Hz update rate

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
        """Create the serial connection configuration group"""
        group = QGroupBox("Serial Connection")
        layout = QHBoxLayout()

        # Port selection
        layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self._refresh_ports()
        layout.addWidget(self.port_combo)

        # Baudrate selection
        layout.addWidget(QLabel("Baudrate:"))
        self.baud_combo = QComboBox()
        for br in Baudrate:
            self.baud_combo.addItem(f"{br.bps // 1000}K", br)
        self.baud_combo.setCurrentIndex(5)  # Default to 500K
        layout.addWidget(self.baud_combo)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_btn)

        # Refresh ports button
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        layout.addWidget(refresh_btn)

        layout.addStretch()

        # Log button
        self.log_btn = QPushButton("Start Logging")
        self.log_btn.clicked.connect(self.on_log_clicked)
        self.log_btn.setEnabled(False)
        layout.addWidget(self.log_btn)

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

    def _refresh_ports(self):
        """Refresh the list of available COM ports"""
        self.port_combo.clear()
        ports = list_serial_ports()
        for device, desc in ports:
            self.port_combo.addItem(f"{device} - {desc[:40]}", device)

    def _get_channel_color(self, ch: int) -> str:
        """Get a unique color for each channel"""
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        return colors[ch % len(colors)]

    def _reset_measurements(self):
        with self.lock:
            self.channel_data = [ChannelData() for _ in range(NUM_CHANNELS)]
            self.waveform_buffers = [[] for _ in range(NUM_CHANNELS)]
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
        if not self.slcan or not self.is_connected:
            return False
        data = struct.pack('<BB', cmd_type, param)
        data += struct.pack('<I', value)
        frame = CANFrame(id=CAN_ID_RX_CONFIG, data=data, is_extended=False, is_remote=False)
        return self.slcan.send_frame(frame)

    def on_zero_clicked(self):
        """Handle Zero Sensor button click - saves zero offset to Flash"""
        if self.send_command(CAN_CMD_ZERO_DATUM):
            self.status_bar.showMessage("Zero calibration saved to Flash")
            logger.info("Zero calibration saved to Flash")
        else:
            self.status_bar.showMessage("Failed to send zero command")
            logger.warning("Failed to send zero command")

    def on_calib_clicked(self):
        """Handle Calibrate button click"""
        if self.send_command(CAN_CMD_START_CALIB):
            self.status_bar.showMessage("Calibration command sent")
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

    def on_save_zero_clicked(self):
        """Handle Save Zero button click - saves current offsets to Flash"""
        if self.send_command(CAN_CMD_SAVE_ZERO):
            self.status_bar.showMessage("Zero offset saved to Flash")
            logger.info("Zero offset saved to Flash")
        else:
            self.status_bar.showMessage("Failed to save zero offset")
            logger.warning("Failed to save zero offset")

    def on_load_zero_clicked(self):
        """Handle Load Zero button click - loads offsets from Flash"""
        if self.send_command(CAN_CMD_LOAD_ZERO):
            self.status_bar.showMessage("Zero offset loaded from Flash")
            logger.info("Zero offset loaded from Flash")
        else:
            self.status_bar.showMessage("Failed to load zero offset")
            logger.warning("Failed to load zero offset")

    def on_clear_zero_clicked(self):
        """Handle Clear Zero button click - clears offsets from Flash"""
        if self.send_command(CAN_CMD_CLEAR_ZERO):
            self.status_bar.showMessage("Zero offset cleared from Flash")
            logger.info("Zero offset cleared from Flash")
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
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "Error", "Please select a COM port")
            return

        baudrate = self.baud_combo.currentData()

        try:
            self.slcan = SLCANProtocol()
            if not self.slcan.connect(port, baudrate):
                QMessageBox.critical(self, "Error", f"Failed to connect to {port}")
                return

            if not self.slcan.open():
                QMessageBox.critical(self, "Error", "Failed to open CAN channel")
                self.slcan.disconnect()
                self.slcan = None
                return

            # Start receiver thread
            self.can_receiver = CANReceiver(self.slcan)
            self.can_receiver.frame_received.connect(self.on_can_frame_received)
            self.can_receiver.start()
            self._reset_measurements()

            self.is_connected = True
            self.connect_btn.setText("Disconnect")
            self.log_btn.setEnabled(True)
            self.zero_btn.setEnabled(True)
            self.calib_btn.setEnabled(True)
            self.save_zero_btn.setEnabled(True)
            self.load_zero_btn.setEnabled(True)
            self.clear_zero_btn.setEnabled(True)
            self.filter_size_spin.setEnabled(True)
            self.status_bar.showMessage(f"Connected to {port} at {baudrate.bps} bps")

            logger.info(f"Connected to {port}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Connection failed: {e}")
            logger.error(f"Connection failed: {e}")
            if self.slcan:
                self.slcan.disconnect()
                self.slcan = None

    def disconnect(self):
        """Disconnect from the CAN adapter"""
        if self.can_receiver:
            self.can_receiver.stop()
            self.can_receiver.wait(1000)
            self.can_receiver = None

        if self.slcan:
            self.slcan.close()
            self.slcan.disconnect()
            self.slcan = None

        self.is_connected = False
        self._reset_measurements()
        self.connect_btn.setText("Connect")
        self.log_btn.setEnabled(False)
        self.zero_btn.setEnabled(False)
        self.calib_btn.setEnabled(False)
        self.save_zero_btn.setEnabled(False)
        self.load_zero_btn.setEnabled(False)
        self.clear_zero_btn.setEnabled(False)
        self.filter_size_spin.setEnabled(False)
        self.status_bar.showMessage("Disconnected")
        logger.info("Disconnected")

    def on_can_frame_received(self, frame: CANFrame):
        """Handle received CAN frame - only combined frame format (0x05)"""
        if frame.id != CAN_ID_TX_DATA:
            return

        if len(frame.data) < 8:
            logger.warning(f"Short frame received: {len(frame.data)} bytes")
            return

        frame_type = frame.data[0]
        channel = frame.data[1]

        if channel >= NUM_CHANNELS:
            logger.warning(f"Invalid channel: {channel}")
            return

        # Only handle combined frame format (frame_type=0x05)
        if frame_type != CAN_FRAME_COMBINED:
            logger.warning(f"Unknown frame type: 0x{frame_type:02X}")
            return

        # Combined frame format: all data in one frame
        # Byte 0: frame_type (0x05)
        # Byte 1: channel
        # Bytes 2-3: voltage (int16, BE, 0.1 mV)
        # Bytes 4-5: strain (int16, BE, µε)
        # Byte 6: stress (int8, signed, 0.1 MPa)
        # Byte 7: crc8 (XOR checksum of bytes 0-6)

        # Verify CRC-8
        received_crc = frame.data[7]
        calculated_crc = crc8_xor(frame.data[:7])
        if received_crc != calculated_crc:
            logger.warning(f"CRC-8 mismatch: received=0x{received_crc:02X}, calculated=0x{calculated_crc:02X}")
            return

        voltage_01mv = int.from_bytes(frame.data[2:4], byteorder='big', signed=True)
        strain_ue = int.from_bytes(frame.data[4:6], byteorder='big', signed=True)
        stress_01mpa = struct.unpack('>b', bytes([frame.data[6]]))[0]  # signed int8

        with self.lock:
            self.channel_data[channel].voltage = voltage_01mv / 10.0  # Convert 0.1mV to mV
            self.channel_data[channel].strain = float(strain_ue)
            self.channel_data[channel].stress = stress_01mpa / 10.0  # Convert 0.1MPa to MPa
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
        if len(self.waveform_buffers[channel]) > WAVEFORM_BUFFER_SIZE:
            self.waveform_buffers[channel].pop(0)

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
                if self.waveform_buffers[ch]:
                    self.plot_curves[ch].setData(self.waveform_buffers[ch])

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
