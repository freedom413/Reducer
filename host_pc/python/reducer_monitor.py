"""
reducer_monitor.py - Reducer Flexspline State Monitoring GUI

A PyQt6-based real-time monitoring application for the Reducer Flexspline
State Detection System. Uses CANable 2.0 SLCAN FD and python-can adapters.
"""

import csv
import datetime
import importlib.util
import logging
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass
from threading import Lock

import numpy as np

# PyQt6 imports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QSpinBox, QStatusBar, QFileDialog, QCheckBox,
    QMessageBox, QTabWidget, QTableWidget, QTableWidgetItem,
    QColorDialog, QDoubleSpinBox,
)
from PyQt6.QtCore import QEvent, QSignalBlocker, QTimer, pyqtSignal, QThread, Qt
from PyQt6.QtGui import QColor, QIcon, QPainter, QPixmap

# Plotting
import pyqtgraph as pg

OPENGL_DEPENDENCIES_AVAILABLE = importlib.util.find_spec("OpenGL") is not None


def configure_plot_backend(use_opengl: bool) -> bool:
    if use_opengl and not OPENGL_DEPENDENCIES_AVAILABLE:
        pg.setConfigOptions(useOpenGL=False, antialias=False)
        return False
    try:
        pg.setConfigOptions(useOpenGL=bool(use_opengl), antialias=False)
        return bool(use_opengl)
    except Exception:
        pg.setConfigOptions(useOpenGL=False, antialias=False)
        return False


OPENGL_PLOT_AVAILABLE = configure_plot_backend(True)

from can_protocol import (
    Baudrate,
    CANFrame,
    CAN_ID_TX_HEALTH,
    CAN_ID_TX_CONFIG,
    CAN_ID_TX_STATUS,
    CAN_ID_TX_TELEMETRY,
    CAN_STATUS_OK,
    CAN_FD_DATA_BITRATE,
    TELEMETRY_MODE_PHYSICAL,
    TELEMETRY_MODE_RAW,
    COMMAND_NAMES,
    DEFAULT_SLCAN_TTY_BAUDRATE,
    PythonCANInterface,
    STATUS_NAMES,
    StatusFrame,
    available_interfaces,
    build_command_frame,
    list_can_channels,
    parse_health_frame,
    parse_config_frame,
    parse_status_frame,
    parse_telemetry_frames,
)

# Command types (must match embedded firmware)
CAN_CMD_SET_SAMPLE_RATE = 0x01
CAN_CMD_SET_FILTER_SIZE = 0x02
CAN_CMD_ZERO_DATUM = 0x03
CAN_CMD_START_CALIB = 0x04
CAN_CMD_SAVE_ZERO = 0x05
CAN_CMD_LOAD_ZERO = 0x06
CAN_CMD_CLEAR_ZERO = 0x07
CAN_CMD_SET_CHANNEL_MASK = 0x08
CAN_CMD_SET_TELEMETRY_MODE = 0x09
CAN_CMD_GET_CONFIG = 0x0A
CAN_CMD_SET_VREF_UV = 0x0B
CAN_CMD_SET_PGA = 0x0C
CAN_CMD_RESTORE_DEFAULTS = 0x0D

# Number of channels
NUM_CHANNELS = 8
DEFAULT_VISIBLE_PLOTS = 4
MAX_VISIBLE_PLOTS = 8
FIXED_CAN_BITRATE = Baudrate.BAUD_500K
ADC_CHANNEL_MASKS = (0x0F, 0xF0)
ELASTIC_MODULUS_MPA = 210000.0
ADC_REF_VOLTAGE = 2.5
ADC_PGA_GAIN = 16
BRIDGE_EXCITATION_V = 5.0
GAUGE_FACTOR = 2.11
MAX_STRAIN_UE = 20000.0
MV_TO_MICROSTRAIN_SCALE = 1000.0 / (BRIDGE_EXCITATION_V * GAUGE_FACTOR)

# Waveform buffer size
WAVEFORM_BUFFER_SIZE = 5000
PLOT_VISIBLE_SAMPLES = 300
PLOT_MIN_Y_RANGE_MV = 0.05
DEFAULT_PLOT_REFRESH_HZ = 60
COMMAND_ACK_TIMEOUT_S = 2.0
DEFAULT_THEME = "dark"
APP_ICON_PATH = Path(__file__).resolve().parent / "assets" / "reducer_monitor.svg"

THEME_COLORS = {
    "dark": {
        "window_bg": "#1e1e1e",
        "panel_bg": "#252526",
        "panel_alt": "#2d2d30",
        "border": "#3c3c3c",
        "text": "#cccccc",
        "strong_text": "#ffffff",
        "muted_text": "#969696",
        "input_bg": "#1f1f1f",
        "accent": "#0e639c",
        "accent_hover": "#1177bb",
        "disabled_bg": "#3a3d41",
        "disabled_text": "#858585",
        "selection_bg": "#264f78",
        "plot_bg": "#1e1e1e",
        "plot_axis": "#cccccc",
    },
}

HEALTH_ICON_COLORS = {
    "waiting": "#e0a33a",
    "ok": "#35b66a",
    "warning": "#ed8b2c",
    "error": "#e05252",
}

PLOT_METRICS = {
    "voltage": {
        "label_key": "voltage_mv", "axis_key": "voltage",
        "color": "#569cd6", "units": "mV",
    },
    "strain": {
        "label_key": "strain_ue", "axis_key": "strain",
        "color": "#d7ba7d", "units": "ue",
    },
    "stress": {
        "label_key": "stress_mpa", "axis_key": "stress",
        "color": "#f44747", "units": "MPa",
    },
}

SUPPORTED_SAMPLE_RATES = [
    2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000, 3750, 7500,
    15000, 30000,
]
ADS1256_CYCLING_RATES = {
    2.5: 2.5, 5: 5, 10: 10, 15: 15, 25: 25, 30: 30, 50: 50, 60: 59,
    100: 98, 500: 456, 1000: 837, 2000: 1438, 3750: 2165, 7500: 3043,
    15000: 3817, 30000: 4374,
}


class WaveformRingBuffer:
    def __init__(self, capacity: int):
        self.capacity = int(capacity)
        self.values = np.zeros(self.capacity, dtype=np.float64)
        self.start = 0
        self.count = 0

    def append(self, value: float) -> None:
        index = (self.start + self.count) % self.capacity
        if self.count == self.capacity:
            self.values[self.start] = value
            self.start = (self.start + 1) % self.capacity
        else:
            self.values[index] = value
            self.count += 1

    def extend(self, values) -> None:
        array = np.asarray(values, dtype=np.float64)
        if array.size == 0:
            return
        if array.size >= self.capacity:
            self.values[:] = array[-self.capacity:]
            self.start = 0
            self.count = self.capacity
            return
        for value in array:
            self.append(float(value))

    def clear(self) -> None:
        self.start = 0
        self.count = 0

    def to_array(self, last: Optional[int] = None) -> np.ndarray:
        if self.count == 0:
            return np.array([], dtype=np.float64)
        if self.start + self.count <= self.capacity:
            data = self.values[self.start:self.start + self.count]
        else:
            data = np.concatenate((
                self.values[self.start:],
                self.values[:(self.start + self.count) % self.capacity],
            ))
        if last is not None and data.size > last:
            data = data[-last:]
        return data.copy()

    def __len__(self) -> int:
        return self.count

    def __bool__(self) -> bool:
        return self.count > 0

    def __iter__(self):
        return iter(self.to_array().tolist())

    def __getitem__(self, item):
        return self.to_array()[item]


def theme_stylesheet(theme: str) -> str:
    colors = THEME_COLORS[theme]
    return f"""
        QMainWindow, QWidget#centralWidget {{
            background: {colors["window_bg"]};
        }}
        QWidget {{
            color: {colors["text"]};
        }}
        QGroupBox {{
            background: {colors["panel_bg"]};
            border: 1px solid {colors["border"]};
            border-radius: 4px; margin-top: 10px; padding: 8px;
            font-weight: 600;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; left: 12px; padding: 0 4px;
            color: {colors["text"]};
        }}
        QPushButton {{
            background: {colors["accent"]}; color: white; border: 0;
            border-radius: 2px; padding: 6px 12px;
        }}
        QPushButton:disabled {{
            background: {colors["disabled_bg"]};
            color: {colors["disabled_text"]};
        }}
        QPushButton:hover:!disabled {{
            background: {colors["accent_hover"]};
        }}
        QComboBox, QSpinBox {{
            background: {colors["input_bg"]}; color: {colors["text"]};
            border: 1px solid {colors["border"]};
            border-radius: 2px; padding: 4px 7px;
        }}
        QComboBox QAbstractItemView {{
            background: {colors["panel_bg"]}; color: {colors["text"]};
            selection-background-color: {colors["selection_bg"]};
            selection-color: {colors["strong_text"]};
        }}
        QLineEdit {{
            background: {colors["input_bg"]}; color: {colors["text"]};
        }}
        QLabel, QCheckBox {{
            color: {colors["text"]};
        }}
        QTabWidget::pane {{
            border: 1px solid {colors["border"]};
            background: {colors["panel_bg"]};
        }}
        QTabBar::tab {{
            background: {colors["panel_alt"]}; color: {colors["text"]};
            border: 1px solid {colors["border"]}; padding: 7px 16px;
        }}
        QTabBar::tab:selected {{
            background: {colors["panel_bg"]};
            color: {colors["strong_text"]};
        }}
        QTableWidget {{
            background: {colors["panel_bg"]}; color: {colors["text"]};
            gridline-color: {colors["border"]};
        }}
        QHeaderView::section {{
            background: {colors["panel_alt"]}; color: {colors["text"]};
            border: 1px solid {colors["border"]}; padding: 4px;
        }}
        QStatusBar {{
            background: {colors["panel_alt"]}; color: {colors["text"]};
        }}
        QToolTip {{
            background: {colors["panel_bg"]}; color: {colors["text"]};
            border: 1px solid {colors["border"]};
        }}
    """


def format_bitrate(bps: int) -> str:
    if bps % 1000000 == 0:
        return f"{bps // 1000000}M"
    if bps % 1000 == 0:
        return f"{bps // 1000}K"
    return str(bps)


def apply_plot_theme(plot: pg.PlotWidget, theme: str) -> None:
    colors = THEME_COLORS[theme]
    plot.setBackground(colors["plot_bg"])
    for axis_name in ("left", "bottom"):
        axis = plot.getPlotItem().getAxis(axis_name)
        axis.setPen(pg.mkPen(colors["plot_axis"]))
        axis.setTextPen(pg.mkPen(colors["plot_axis"]))


def curve_color_button_stylesheet(color: str) -> str:
    return (
        f"background: {color}; border: 1px solid #6a6a6a; "
        "border-radius: 2px; padding: 0px;"
    )


def create_plot_hover_items(plot: pg.PlotWidget) -> dict[str, object]:
    line = pg.InfiniteLine(
        angle=90,
        movable=False,
        pen=pg.mkPen("#6a6a6a", width=1, style=Qt.PenStyle.DashLine),
    )
    marker = pg.ScatterPlotItem(
        size=9,
        pen=pg.mkPen("#ffffff", width=1),
        brush=pg.mkBrush(PLOT_METRICS["voltage"]["color"]),
    )
    label = pg.TextItem(
        anchor=(0, 1),
        color=THEME_COLORS[DEFAULT_THEME]["text"],
        fill=pg.mkBrush(37, 37, 38, 235),
        border=pg.mkPen(THEME_COLORS[DEFAULT_THEME]["border"]),
    )
    for item in (line, marker, label):
        item.setZValue(100)
        item.hide()
        plot.addItem(item)
    return {"line": line, "marker": marker, "label": label}


def hide_plot_hover_items(items: dict[str, object]) -> None:
    for item in items.values():
        item.hide()


def status_icon_pixmap(state: str) -> QPixmap:
    pixmap = QPixmap(14, 14)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(QColor(HEALTH_ICON_COLORS[state]))
    painter.drawEllipse(2, 2, 10, 10)
    painter.end()
    return pixmap


TRANSLATIONS = {
    "en": {
        "window_title": "Reducer Flexspline Monitor",
        "offline_window_title": "Reducer Waveform Log - {filename}",
        "can_connection": "CAN Connection",
        "interface": "Interface:",
        "channel": "Channel:",
        "can_baudrate": "CAN Baudrate:",
        "connect": "Connect",
        "disconnect": "Disconnect",
        "refresh": "Refresh",
        "start_logging": "Start Logging",
        "stop_logging": "Stop Logging",
        "language": "Language:",
        "waveforms": "Waveforms",
        "data_panel": "Data Panel",
        "auto_scale": "Auto Scale",
        "auto_scale_tooltip": "Automatically fit each plot to recent samples. Uncheck to keep a fixed scale.",
        "opengl_plot": "OpenGL Plot",
        "opengl_plot_tooltip": "Use pyqtgraph OpenGL rendering when PyOpenGL is available; numeric conversion stays on CPU.",
        "opengl_unavailable": "OpenGL plotting is unavailable, using normal rendering",
        "clear_plots": "Clear Plots",
        "clear_plots_tooltip": "Clear waveform history without changing MCU settings or current values",
        "import_csv": "Import CSV",
        "import_csv_tooltip": "Load recorded CSV data into the waveform plots",
        "add_plot": "Add Plot",
        "add_plot_tooltip": "Add a waveform plot and request MCU sampling, up to 8 plots total",
        "remove_plot": "Remove",
        "remove_plot_tooltip": "Remove this plot and stop MCU sampling if no other plot uses its channel",
        "plot_source": "Source:",
        "plot_source_tooltip": "Select the displayed channel and update MCU sampling automatically",
        "plot_metrics": "Curves:",
        "plot_metrics_tooltip": "Select one or more curves to overlay in this plot",
        "plot_title": "CH{channel} Curves",
        "value": "Value",
        "voltage": "Voltage",
        "strain": "Strain",
        "stress": "Stress",
        "sample": "Sample",
        "plot_tooltip": "Double-click to maximize or return to the grid view",
        "table_channel": "Channel",
        "voltage_mv": "Voltage (mV)",
        "strain_ue": "Strain (ue)",
        "stress_mpa": "Stress (MPa)",
        "samples": "Samples",
        "voltage_statistics": "Voltage Statistics (since connect)",
        "min": "Min",
        "max": "Max",
        "avg": "Avg",
        "commands": "Commands",
        "zero_sensor": "Zero Sensor",
        "calibrate": "Calibrate",
        "save_zero": "Save Zero",
        "load_zero": "Load Zero",
        "clear_zero": "Clear Zero",
        "sample_rate": "Sample Rate:",
        "filter_size": "Filter Size:",
        "telemetry_mode": "Telemetry:",
        "telemetry_raw": "Raw High Performance",
        "telemetry_physical": "Physical Debug",
        "disconnected": "Disconnected",
        "waveforms_cleared": "Waveforms cleared",
        "loaded_log": "Loaded {filename}",
        "opened_waveform_log": "Opened waveform log {filename}",
        "import_waveform_log": "Import Waveform Log",
        "save_log_file": "Save Log File",
        "csv_files": "CSV Files (*.csv)",
        "error": "Error",
        "failed_import_csv": "Failed to import CSV: {error}",
        "please_select_channel": "Please select a CAN channel or serial port",
        "slcan_help": "Check the COM port and CANable 2.0 SLCAN FD firmware.",
        "socketcan_help": "If using socketcan, make sure the interface is already up.",
        "failed_connect_target": "Failed to connect to {interface}:{channel}\n{help}",
        "connection_failed": "Connection failed: {error}",
        "connected_slcan": "Connected to {channel} via CANable 2.0 SLCAN FD (USB CDC, CAN FD {can_baudrate}/{data_bitrate} bps, BRS)",
        "connected_generic": "Connected to {interface}:{channel} (CAN FD {can_baudrate}/{data_bitrate} bps, BRS)",
        "connected_device": "Connected to {interface}:{channel}",
        "zero_sent": "Zero command sent, waiting for device ACK",
        "zero_failed": "Failed to send zero command",
        "calibration_sent": "Calibration command sent, waiting for device ACK",
        "calibration_failed": "Failed to send calibration command",
        "filter_size_failed": "Failed to send filter size command",
        "sample_rate_failed": "Failed to send sample rate command",
        "telemetry_mode_failed": "Failed to send telemetry mode command",
        "stream_summary": "ADC {sample_rate} SPS | scan {scan_rate:.0f} fps | telemetry {telemetry_rate:.0f} samples/s | {mode}",
        "health": "System Health",
        "health_waiting": "Waiting for MCU health frame | RX {rx_rate:.0f} samples/s | protocol errors {bad}",
        "health_summary": "MCU {adc_state} | RX {rx_rate:.0f} samples/s | MCU TX {tx_samples} samples/s in {tx_frames} frames/s | ADC {sample_rate} SPS | {mode} | new drops {drop_delta}, total {tx_drop} | new overflows {overflow_delta}, total {overflow} | recoveries {recovery} | protocol errors {bad}",
        "adc_running": "RUN",
        "adc_stopped": "STOP",
        "save_zero_sent": "Save Zero command sent, waiting for device ACK",
        "save_zero_failed": "Failed to save zero offset",
        "load_zero_sent": "Load Zero command sent, waiting for device ACK",
        "load_zero_failed": "Failed to load zero offset",
        "clear_zero_sent": "Clear Zero command sent, waiting for device ACK",
        "clear_zero_failed": "Failed to clear zero offset",
        "channel_mask_sent": "ADC channel selection sent: 0x{mask:02X}",
        "channel_mask_failed": "Failed to update ADC channel selection",
        "logging_to": "Logging to {filename}",
        "logging_stopped": "Logging stopped",
        "failed_start_logging": "Failed to start logging: {error}",
        "command_acknowledged": "{command} acknowledged (value={value})",
        "command_rejected": "{command} rejected: {reason}",
        "command_timeout": "{command} timed out waiting for ACK",
        "command_unknown": "Command 0x{cmd_type:02X}",
        "command_sequence": "Command sequence {sequence}",
    },
    "zh": {
        "channel_mask_sent": "ADC 通道选择已发送：0x{mask:02X}",
        "channel_mask_failed": "ADC 通道选择更新失败",
        "add_plot": "新增图表",
        "add_plot_tooltip": "新增一个波形图表，最多可显示 8 张",
        "remove_plot": "移除",
        "remove_plot_tooltip": "移除当前波形图表",
        "plot_source": "数据源：",
        "plot_source_tooltip": "选择当前图表展示的通道",
        "plot_metrics": "曲线：",
        "plot_metrics_tooltip": "选择当前图表中叠加显示的一种或多种曲线",
        "window_title": "减速器柔轮监视器",
        "offline_window_title": "减速器波形记录 - {filename}",
        "can_connection": "CAN 连接",
        "interface": "接口：",
        "channel": "通道：",
        "can_baudrate": "CAN 波特率：",
        "connect": "连接",
        "disconnect": "断开连接",
        "refresh": "刷新",
        "start_logging": "开始记录",
        "stop_logging": "停止记录",
        "language": "语言：",
        "waveforms": "波形",
        "data_panel": "数据面板",
        "auto_scale": "自动缩放",
        "auto_scale_tooltip": "自动适配各通道近期采样范围。取消勾选后保持固定范围。",
        "clear_plots": "清空曲线",
        "clear_plots_tooltip": "清空波形历史，不改变 MCU 设置和当前数值",
        "import_csv": "导入 CSV",
        "import_csv_tooltip": "将已记录的 CSV 数据加载到波形窗口",
        "plot_title": "CH{channel} 曲线",
        "value": "数值",
        "voltage": "电压",
        "strain": "应变",
        "stress": "应力",
        "sample": "采样点",
        "plot_tooltip": "双击可最大化曲线或返回网格视图",
        "table_channel": "通道",
        "voltage_mv": "电压 (mV)",
        "strain_ue": "应变 (ue)",
        "stress_mpa": "应力 (MPa)",
        "samples": "采样数",
        "voltage_statistics": "电压统计（自连接以来）",
        "min": "最小值",
        "max": "最大值",
        "avg": "平均值",
        "commands": "命令",
        "zero_sensor": "传感器调零",
        "calibrate": "校准",
        "save_zero": "保存零点",
        "load_zero": "加载零点",
        "clear_zero": "清除零点",
        "sample_rate": "采样率：",
        "filter_size": "滤波长度：",
        "telemetry_mode": "遥测：",
        "telemetry_raw": "Raw 高性能",
        "telemetry_physical": "物理量调试",
        "disconnected": "未连接",
        "waveforms_cleared": "波形已清空",
        "loaded_log": "已加载 {filename}",
        "opened_waveform_log": "已打开波形记录 {filename}",
        "import_waveform_log": "导入波形记录",
        "save_log_file": "保存记录文件",
        "csv_files": "CSV 文件 (*.csv)",
        "error": "错误",
        "failed_import_csv": "导入 CSV 失败：{error}",
        "please_select_channel": "请选择 CAN 通道或串口",
        "slcan_help": "请检查串口名称和 CANable 2.0 SLCAN FD 固件。",
        "socketcan_help": "使用 socketcan 时，请确认接口已经启用。",
        "failed_connect_target": "连接 {interface}:{channel} 失败\n{help}",
        "connection_failed": "连接失败：{error}",
        "connected_slcan": "已通过 CANable 2.0 SLCAN FD 连接 {channel}（USB CDC，CAN FD {can_baudrate}/{data_bitrate} bps BRS）",
        "connected_generic": "已连接 {interface}:{channel}，CAN FD {can_baudrate}/{data_bitrate} bps BRS",
        "zero_sent": "调零命令已发送，正在等待设备确认",
        "zero_failed": "调零命令发送失败",
        "calibration_sent": "校准命令已发送，正在等待设备确认",
        "calibration_failed": "校准命令发送失败",
        "filter_size_failed": "滤波长度命令发送失败",
        "sample_rate_failed": "采样率命令发送失败",
        "telemetry_mode_failed": "遥测模式命令发送失败",
        "stream_summary": "ADC {sample_rate} SPS | 扫描 {scan_rate:.0f} 帧/秒 | 遥测 {telemetry_rate:.0f} 样本/秒 | {mode}",
        "health": "系统健康状态",
        "health_waiting": "等待 MCU 健康帧 | 接收 {rx_rate:.0f} 样本/秒 | 协议错误 {bad}",
        "health_summary": "MCU {adc_state} | 接收 {rx_rate:.0f} 样本/秒 | MCU 发送 {tx_samples} 样本/秒、{tx_frames} 帧/秒 | ADC {sample_rate} SPS | {mode} | 新增丢弃 {drop_delta}、累计 {tx_drop} | 新增溢出 {overflow_delta}、累计 {overflow} | 恢复 {recovery} | 协议错误 {bad}",
        "adc_running": "运行",
        "adc_stopped": "停止",
        "save_zero_sent": "保存零点命令已发送，正在等待设备确认",
        "save_zero_failed": "保存零点失败",
        "load_zero_sent": "加载零点命令已发送，正在等待设备确认",
        "load_zero_failed": "加载零点失败",
        "clear_zero_sent": "清除零点命令已发送，正在等待设备确认",
        "clear_zero_failed": "清除零点失败",
        "logging_to": "正在记录到 {filename}",
        "logging_stopped": "记录已停止",
        "failed_start_logging": "开始记录失败：{error}",
        "command_acknowledged": "{command} 已确认（值={value}）",
        "command_rejected": "{command} 被拒绝：{reason}",
        "command_timeout": "{command} 等待确认超时",
        "command_unknown": "命令 0x{cmd_type:02X}",
        "command_sequence": "命令序号 {sequence}",
    },
}

COMMAND_TRANSLATIONS = {
    "Set Sample Rate": "设置采样率",
    "Set Filter Size": "设置滤波长度",
    "Zero Datum": "传感器调零",
    "Calibrate": "校准",
    "Save Zero": "保存零点",
    "Load Zero": "加载零点",
    "Clear Zero": "清除零点",
    "Set Channel Mask": "设置 ADC 通道",
}

STATUS_TRANSLATIONS = {
    "OK": "正常",
    "CRC mismatch": "CRC 校验不一致",
    "invalid frame type": "帧类型无效",
    "unsupported command": "不支持的命令",
    "invalid value": "数值无效",
    "storage error": "存储错误",
}


def translate(language: str, key: str, **kwargs) -> str:
    template = TRANSLATIONS.get(language, TRANSLATIONS["en"]).get(
        key, TRANSLATIONS["en"].get(key, key)
    )
    return template.format(**kwargs)


logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class ChannelData:
    """Holds current data for a single channel"""
    voltage_mv: float = 0.0
    strain_ue: float = 0.0
    stress_mpa: float = 0.0
    voltage_001mv: int = 0
    stress_01mpa: int = 0
    samples: int = 0
    last_timestamp: float = 0.0


class CANReceiver(QThread):
    """Background thread for forwarding python-can frames to the UI thread"""

    frame_received = pyqtSignal(object)

    def __init__(self, can_bus: PythonCANInterface):
        super().__init__()
        self.can_bus = can_bus
        self.running = True
        self._callback = lambda frame: self.frame_received.emit(frame)
        self.can_bus.register_rx_callback(self._callback)

    def run(self):
        while self.running:
            QThread.msleep(100)

    def stop(self):
        self.running = False
        if self._callback is not None:
            self.can_bus.unregister_rx_callback(self._callback)
            self._callback = None


class OfflineWaveformWindow(QMainWindow):
    """Read-only window for inspecting a recorded waveform CSV."""

    def __init__(
        self,
        filename: str,
        waveform_buffers: List[List[float]],
        language: str = "en",
    ):
        super().__init__()
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        self.setWindowIcon(QIcon(str(APP_ICON_PATH)))
        self.filename = filename
        self.waveform_buffers = waveform_buffers
        self.language = language
        self.theme = DEFAULT_THEME
        self.maximized_plot_channel: Optional[int] = None

        self.setGeometry(140, 140, 1200, 800)
        self.setStyleSheet(theme_stylesheet(self.theme))

        central_widget = QWidget()
        central_widget.setObjectName("centralWidget")
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        controls = QHBoxLayout()
        self.auto_scale_checkbox = QCheckBox()
        self.auto_scale_checkbox.setChecked(True)
        self.auto_scale_checkbox.toggled.connect(self._on_auto_scale_toggled)
        controls.addWidget(self.auto_scale_checkbox)
        controls.addWidget(QLabel(os.path.basename(filename)))
        controls.addStretch()
        layout.addLayout(controls)

        plots_widget = QWidget()
        self.waveform_layout = QGridLayout(plots_widget)
        layout.addWidget(plots_widget)

        self.plot_widgets = []
        self.plot_curves = []
        self.plot_hover_items = []
        self.plot_channels = [
            channel for channel, values in enumerate(waveform_buffers) if values
        ]
        for channel in self.plot_channels:
            plot = pg.PlotWidget()
            plot.installEventFilter(self)
            apply_plot_theme(plot, self.theme)
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.disableAutoRange()
            plot.setXRange(0, 100, padding=0.0)
            plot.setYRange(-1.0, 1.0, padding=0.0)
            plot.scene().sigMouseClicked.connect(
                lambda event, selected_channel=channel:
                    self._on_plot_mouse_clicked(selected_channel, event)
            )
            hover_items = create_plot_hover_items(plot)
            plot.scene().sigMouseMoved.connect(
                lambda scene_pos, selected_channel=channel:
                    self._on_plot_mouse_moved(selected_channel, scene_pos)
            )
            curve = plot.plot(
                waveform_buffers[channel],
                pen=pg.mkPen(color=self._get_channel_color(channel), width=1.5),
            )
            self.plot_widgets.append(plot)
            self.plot_curves.append(curve)
            self.plot_hover_items.append(hover_items)

        self._refresh_waveform_layout()
        self._fit_all_plots()
        self._retranslate_ui()

    @classmethod
    def from_csv(cls, filename: str, language: str = "en"):
        waveform_buffers = [[] for _ in range(NUM_CHANNELS)]
        with open(filename, "r", newline="") as handle:
            reader = csv.DictReader(handle)
            required_fields = {"channel", "voltage_mv"}
            if reader.fieldnames is None or not required_fields.issubset(reader.fieldnames):
                raise ValueError("CSV header does not match a Reducer waveform log")

            for row in reader:
                channel = int(row["channel"])
                if 0 <= channel < NUM_CHANNELS:
                    waveform_buffers[channel].append(float(row["voltage_mv"]))

        return cls(filename, waveform_buffers, language)

    def set_language(self, language: str):
        self.language = language
        self._retranslate_ui()

    def _retranslate_ui(self):
        basename = os.path.basename(self.filename)
        self.setWindowTitle(
            translate(self.language, "offline_window_title", filename=basename)
        )
        self.auto_scale_checkbox.setText(translate(self.language, "auto_scale"))
        for channel, plot in zip(self.plot_channels, self.plot_widgets):
            plot.setTitle(
                translate(self.language, "plot_title", channel=channel),
                color=THEME_COLORS[self.theme]["plot_axis"],
            )
            plot.setLabel("left", translate(self.language, "voltage"), units="mV")
            plot.setLabel("bottom", translate(self.language, "sample"))
            plot.setToolTip(translate(self.language, "plot_tooltip"))
        self.statusBar().showMessage(
            translate(self.language, "loaded_log", filename=self.filename)
        )

    @staticmethod
    def _get_channel_color(channel: int) -> str:
        colors = ["#569cd6", "#d7ba7d", "#6a9955", "#c586c0", "#4ec9b0", "#ce9178"]
        return colors[channel % len(colors)]

    def _on_plot_mouse_clicked(self, channel: int, event):
        if event.double():
            self._toggle_plot_maximize(channel)

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.Leave and watched in self.plot_widgets:
            plot_index = self.plot_widgets.index(watched)
            hide_plot_hover_items(self.plot_hover_items[plot_index])
        return super().eventFilter(watched, event)

    def _on_plot_mouse_moved(self, channel: int, scene_pos):
        plot_index = self.plot_channels.index(channel)
        plot = self.plot_widgets[plot_index]
        hover_items = self.plot_hover_items[plot_index]
        if not plot.getPlotItem().sceneBoundingRect().contains(scene_pos):
            hide_plot_hover_items(hover_items)
            return

        mouse = plot.getPlotItem().vb.mapSceneToView(scene_pos)
        sample = int(round(mouse.x()))
        values = self.waveform_buffers[channel]
        if not 0 <= sample < len(values):
            hide_plot_hover_items(hover_items)
            return

        x_range, y_range = plot.viewRange()
        x_tolerance = max(1.0, (x_range[1] - x_range[0]) * 12.0 / max(plot.width(), 1))
        y_tolerance = max(0.05, (y_range[1] - y_range[0]) * 0.08)
        value = values[sample]
        if (
            abs(mouse.x() - sample) > x_tolerance
            or abs(mouse.y() - value) > y_tolerance
        ):
            hide_plot_hover_items(hover_items)
            return

        color = self._get_channel_color(channel)
        hover_items["line"].setPos(sample)
        hover_items["marker"].setData([sample], [value], brush=pg.mkBrush(color))
        hover_items["label"].setText(
            f"{translate(self.language, 'voltage')}\n"
            f"{translate(self.language, 'sample')}: {sample}\n"
            f"{value:.3f} mV"
        )
        hover_items["label"].setAnchor(
            (1, 1) if sample > sum(x_range) / 2.0 else (0, 1)
        )
        hover_items["label"].setPos(sample, value)
        for item in hover_items.values():
            item.show()

    def _toggle_plot_maximize(self, channel: int):
        self.maximized_plot_channel = (
            None if self.maximized_plot_channel == channel else channel
        )
        self._refresh_waveform_layout()

    def _refresh_waveform_layout(self):
        while self.waveform_layout.count():
            self.waveform_layout.takeAt(0)

        for plot in self.plot_widgets:
            plot.hide()

        if self.maximized_plot_channel is not None:
            for row in range(2):
                self.waveform_layout.setRowStretch(row, 0)
            for column in range(3):
                self.waveform_layout.setColumnStretch(column, 0)

            plot = self.plot_widgets[self.plot_channels.index(self.maximized_plot_channel)]
            self.waveform_layout.addWidget(plot, 0, 0)
            plot.show()
            return

        count = len(self.plot_widgets)
        columns = 1 if count <= 2 else 2 if count <= 4 else 3
        rows = max(1, (count + columns - 1) // columns)
        for row in range(NUM_CHANNELS):
            self.waveform_layout.setRowStretch(row, 0)
        for column in range(NUM_CHANNELS):
            self.waveform_layout.setColumnStretch(column, 0)
        for row in range(rows):
            self.waveform_layout.setRowStretch(row, 1)
        for column in range(columns):
            self.waveform_layout.setColumnStretch(column, 1)
        for plot_index, plot in enumerate(self.plot_widgets):
            self.waveform_layout.addWidget(
                plot, plot_index // columns, plot_index % columns
            )
            plot.show()

    def _on_auto_scale_toggled(self, enabled: bool):
        if enabled:
            self._fit_all_plots()

    def _fit_all_plots(self):
        for channel in self.plot_channels:
            values = self.waveform_buffers[channel]
            if values:
                self._fit_plot(channel, values)

    def _fit_plot(self, channel: int, values: List[float]):
        minimum = min(values)
        maximum = max(values)
        center = (minimum + maximum) / 2.0
        y_range = max(maximum - minimum, PLOT_MIN_Y_RANGE_MV)
        y_padding = y_range * 0.1
        last_sample = max(1, len(values) - 1)

        plot = self.plot_widgets[self.plot_channels.index(channel)]
        plot.setXRange(0, last_sample, padding=0.01)
        plot.setYRange(center - (y_range / 2.0) - y_padding,
                       center + (y_range / 2.0) + y_padding,
                       padding=0.0)


class ReducerMonitorWindow(QMainWindow):
    """Main window for the Reducer Flexspline Monitor"""

    # Signal for updating UI from CAN receiver thread
    data_updated = pyqtSignal(int, dict)

    def __init__(self):
        super().__init__()
        self.setWindowIcon(QIcon(str(APP_ICON_PATH)))

        # CAN protocol
        self.can_bus: Optional[PythonCANInterface] = None
        self.can_receiver: Optional[CANReceiver] = None
        self.command_sequence = 0
        self.pending_commands: Dict[int, str] = {}
        self.pending_command_deadlines: Dict[int, float] = {}

        # Channel data
        self.channel_data: List[ChannelData] = [ChannelData() for _ in range(NUM_CHANNELS)]
        self.channel_stats: List[Dict[str, Optional[float]]] = [
            {'min': None, 'max': None, 'sum': 0.0, 'count': 0}
            for _ in range(NUM_CHANNELS)
        ]
        self.lock = Lock()

        # Waveform data buffers
        self.waveform_buffers, self.waveform_metric_buffers = (
            self._create_waveform_buffers()
        )
        self.plot_dirty = [False for _ in range(NUM_CHANNELS)]

        # CSV logging
        self.csv_writer: Optional[csv.writer] = None
        self.csv_file: Optional[object] = None
        self.logging_enabled = False
        self.offline_waveform_windows: List[OfflineWaveformWindow] = []

        # UI state
        self.is_connected = False
        self.rx_telemetry_count = 0
        self.rx_status_count = 0
        self.rx_bad_protocol_count = 0
        self.last_rx_telemetry_count = 0
        self.rx_telemetry_rate_hz = 0.0
        self.latest_health = None
        self.health_tx_drop_delta = 0
        self.health_adc_overflow_delta = 0
        self.health_adc_recovery_delta = 0
        self.telemetry_mode = TELEMETRY_MODE_RAW
        self.vref_uv = 2_500_000
        self.pga_gain = 16
        self.config_sequence = 0
        self.zero_offsets = [0 for _ in range(NUM_CHANNELS)]
        self.zero_valid = False
        self.pending_telemetry_batches = deque(maxlen=2048)
        self.display_drop_count = 0
        self.csv_pending_rows = []
        self.maximized_plot_channel: Optional[int] = None
        self.maximized_plot_panel: Optional[QWidget] = None
        self.last_sent_channel_mask: Optional[int] = None
        self.language = "zh"
        self.theme = DEFAULT_THEME
        self.sample_rate_sps = 100.0
        self._status_message = None

        # Setup UI
        self.init_ui()
        self._refresh_channels()
        self._reset_measurements()

        # Kept for tests and low-rate manual updates; high-rate CAN uses batches.
        self.data_updated.connect(self.on_data_updated)

        # Update timer for waveform plot
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self._set_plot_refresh_rate(DEFAULT_PLOT_REFRESH_HZ)

        self.command_timeout_timer = QTimer()
        self.command_timeout_timer.timeout.connect(self._check_command_timeouts)
        self.command_timeout_timer.start(250)

        self.health_timer = QTimer()
        self.health_timer.timeout.connect(self._refresh_health_panel)
        self.health_timer.start(1000)

    def init_ui(self):
        """Initialize the user interface"""
        self.setGeometry(100, 100, 1200, 800)
        self.setMinimumSize(1100, 720)
        self.setStyleSheet(theme_stylesheet(self.theme))

        central_widget = QWidget()
        central_widget.setObjectName("centralWidget")
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        # Connection group
        self.conn_group = self._create_connection_group()
        main_layout.addWidget(self.conn_group)

        # Command group
        self.cmd_group = self._create_command_group()
        main_layout.addWidget(self.cmd_group)

        # Tab widget for waveforms and data
        self.tabs = QTabWidget()

        # Waveform tab
        waveform_tab = self._create_waveform_tab()
        self.tabs.addTab(waveform_tab, "")

        # Data panel tab
        data_tab = self._create_data_panel_tab()
        self.tabs.addTab(data_tab, "")

        main_layout.addWidget(self.tabs)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.health_icon_label = QLabel()
        self.health_icon_label.setFixedSize(16, 16)
        self.health_summary_label = QLabel()
        self.status_bar.addPermanentWidget(self.health_icon_label)
        self.status_bar.addPermanentWidget(self.health_summary_label)
        self._retranslate_ui()
        self._show_status("disconnected")

    def _create_connection_group(self) -> QGroupBox:
        """Create the CAN connection configuration group"""
        group = QGroupBox()
        layout = QHBoxLayout()

        self.interface_label = QLabel()
        layout.addWidget(self.interface_label)
        self.interface_combo = QComboBox()
        for interface_name, label in available_interfaces():
            self.interface_combo.addItem(label, interface_name)
        self.interface_combo.currentIndexChanged.connect(self._on_interface_changed)
        layout.addWidget(self.interface_combo)

        self.channel_label = QLabel()
        layout.addWidget(self.channel_label)
        self.channel_combo = QComboBox()
        self.channel_combo.setEditable(True)
        layout.addWidget(self.channel_combo)

        self.slcan_speed_label = QLabel("USB 串口:")
        layout.addWidget(self.slcan_speed_label)
        self.slcan_speed_combo = QComboBox()
        for speed in (2_000_000, 921_600, 460_800, 115_200):
            self.slcan_speed_combo.addItem(str(speed), speed)
        layout.addWidget(self.slcan_speed_combo)

        # Connect button
        self.connect_btn = QPushButton()
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_btn)

        # Refresh ports button
        self.refresh_btn = QPushButton()
        self.refresh_btn.clicked.connect(self._refresh_channels)
        layout.addWidget(self.refresh_btn)

        layout.addStretch()

        # Log button
        self.log_btn = QPushButton()
        self.log_btn.clicked.connect(self.on_log_clicked)
        self.log_btn.setEnabled(False)
        layout.addWidget(self.log_btn)

        self.language_label = QLabel()
        layout.addWidget(self.language_label)
        self.language_combo = QComboBox()
        self.language_combo.addItem("English", "en")
        self.language_combo.addItem("中文", "zh")
        language_index = self.language_combo.findData(self.language)
        if language_index >= 0:
            self.language_combo.setCurrentIndex(language_index)
        self.language_combo.currentIndexChanged.connect(self._on_language_changed)
        layout.addWidget(self.language_combo)

        slcan_index = self.interface_combo.findData("slcan")
        if slcan_index >= 0:
            self.interface_combo.setCurrentIndex(slcan_index)

        group.setLayout(layout)
        return group

    def _create_waveform_tab(self) -> QWidget:
        """Create the waveform display tab with configurable channel plots"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        controls = QHBoxLayout()
        self.auto_scale_checkbox = QCheckBox()
        self.auto_scale_checkbox.setChecked(True)
        self.auto_scale_checkbox.toggled.connect(self._on_auto_scale_toggled)
        controls.addWidget(self.auto_scale_checkbox)

        self.opengl_checkbox = QCheckBox()
        self.opengl_checkbox.setChecked(OPENGL_PLOT_AVAILABLE)
        self.opengl_checkbox.toggled.connect(self._on_opengl_toggled)
        controls.addWidget(self.opengl_checkbox)

        self.clear_plots_btn = QPushButton()
        self.clear_plots_btn.clicked.connect(self._clear_plots)
        controls.addWidget(self.clear_plots_btn)

        self.import_csv_btn = QPushButton()
        self.import_csv_btn.clicked.connect(self.import_csv)
        controls.addWidget(self.import_csv_btn)

        self.add_plot_btn = QPushButton()
        self.add_plot_btn.clicked.connect(self._add_waveform_plot)
        controls.addWidget(self.add_plot_btn)

        controls.addStretch()
        layout.addLayout(controls)

        plots_widget = QWidget()
        self.waveform_layout = QGridLayout(plots_widget)
        layout.addWidget(plots_widget)

        # Keep acquisition buffers separate from plot cards so each card can
        # select a source channel without affecting telemetry collection.
        self.plot_panels = []
        self.plot_widgets = []
        self.plot_curves = []
        self.plot_channel_combos = []
        self.plot_source_labels = []
        self.plot_metrics_labels = []
        self.plot_metric_checkboxes = []
        self.plot_metric_color_buttons = []
        self.plot_metric_colors = []
        self.plot_metric_curves = []
        self.plot_hover_items = []
        self.plot_remove_buttons = []
        self.plot_channels = []

        for channel in range(min(DEFAULT_VISIBLE_PLOTS, NUM_CHANNELS)):
            self._add_waveform_plot(channel)

        return widget

    def _create_data_panel_tab(self) -> QWidget:
        """Create the numerical data display panel"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        config_group = QGroupBox("MCU ADC 配置")
        config_layout = QHBoxLayout(config_group)
        config_layout.addWidget(QLabel("Vref (V):"))
        self.vref_spin = QDoubleSpinBox()
        self.vref_spin.setRange(1.0, 5.0)
        self.vref_spin.setDecimals(6)
        self.vref_spin.setSingleStep(0.0001)
        self.vref_spin.setEnabled(False)
        self.vref_spin.editingFinished.connect(self.on_vref_changed)
        config_layout.addWidget(self.vref_spin)
        config_layout.addWidget(QLabel("PGA:"))
        self.pga_combo = QComboBox()
        for gain in (1, 2, 4, 8, 16, 32, 64):
            self.pga_combo.addItem(str(gain), gain)
        self.pga_combo.setEnabled(False)
        self.pga_combo.currentIndexChanged.connect(self.on_pga_changed)
        config_layout.addWidget(self.pga_combo)
        config_layout.addWidget(QLabel("通道掩码:"))
        self.channel_mask_spin = QSpinBox()
        self.channel_mask_spin.setRange(0, 0xFF)
        self.channel_mask_spin.setDisplayIntegerBase(16)
        self.channel_mask_spin.setPrefix("0x")
        self.channel_mask_spin.setEnabled(False)
        self.channel_mask_spin.editingFinished.connect(self.on_channel_mask_changed)
        config_layout.addWidget(self.channel_mask_spin)
        self.restore_defaults_btn = QPushButton("恢复默认配置")
        self.restore_defaults_btn.setEnabled(False)
        self.restore_defaults_btn.clicked.connect(self.on_restore_defaults_clicked)
        config_layout.addWidget(self.restore_defaults_btn)
        self.config_state_label = QLabel("等待 MCU 配置")
        config_layout.addWidget(self.config_state_label)
        config_layout.addStretch()
        layout.addWidget(config_group)

        # Table for data display
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(7)
        self.data_table.setRowCount(NUM_CHANNELS)

        # Set channel numbers and initialize data columns
        for i in range(NUM_CHANNELS):
            self.data_table.setItem(i, 0, QTableWidgetItem(f"CH{i}"))
            self.data_table.setItem(i, 1, QTableWidgetItem("-"))
            self.data_table.setItem(i, 2, QTableWidgetItem("-"))
            self.data_table.setItem(i, 3, QTableWidgetItem("-"))
            self.data_table.setItem(i, 4, QTableWidgetItem("-"))
            self.data_table.setItem(i, 5, QTableWidgetItem("-"))
            self.data_table.setItem(i, 6, QTableWidgetItem("-"))

        # Stretch columns
        header = self.data_table.horizontalHeader()
        header.setStretchLastSection(True)

        layout.addWidget(self.data_table)

        # Statistics section
        self.stats_group = QGroupBox()
        stats_layout = QGridLayout()

        self.stats_labels = []
        for ch in range(NUM_CHANNELS):
            ch_label = QLabel(f"CH{ch}:")
            stats_layout.addWidget(ch_label, ch, 0)

            min_lbl = QLabel()
            max_lbl = QLabel()
            avg_lbl = QLabel()
            self.stats_labels.append((min_lbl, max_lbl, avg_lbl))

            stats_layout.addWidget(min_lbl, ch, 1)
            stats_layout.addWidget(max_lbl, ch, 2)
            stats_layout.addWidget(avg_lbl, ch, 3)

        self.stats_group.setLayout(stats_layout)
        layout.addWidget(self.stats_group)

        return widget

    def _tr(self, key: str, **kwargs) -> str:
        return translate(self.language, key, **kwargs)

    def _on_language_changed(self, _index: int):
        language = self.language_combo.currentData()
        if language is None:
            return

        self.language = str(language)
        self._retranslate_ui()
        for window in self.offline_waveform_windows:
            window.set_language(self.language)

    def _retranslate_ui(self):
        self.setWindowTitle(self._tr("window_title"))
        self.conn_group.setTitle(self._tr("can_connection"))
        self.interface_label.setText(self._tr("interface"))
        self.channel_label.setText(self._tr("channel"))
        self.connect_btn.setText(
            self._tr("disconnect") if self.is_connected else self._tr("connect")
        )
        self.refresh_btn.setText(self._tr("refresh"))
        self.log_btn.setText(
            self._tr("stop_logging") if self.logging_enabled else self._tr("start_logging")
        )
        self.language_label.setText(self._tr("language"))

        self.cmd_group.setTitle(self._tr("commands"))
        self.zero_btn.setText(self._tr("zero_sensor"))
        self.calib_btn.setText(self._tr("calibrate"))
        self.clear_zero_btn.setText(self._tr("clear_zero"))
        self.sample_rate_label.setText(self._tr("sample_rate"))
        self.filter_size_label.setText(self._tr("filter_size"))
        self.telemetry_mode_label.setText(self._tr("telemetry_mode"))
        raw_index = self.telemetry_mode_combo.findData(TELEMETRY_MODE_RAW)
        if raw_index >= 0:
            self.telemetry_mode_combo.setItemText(raw_index, self._tr("telemetry_raw"))
        physical_index = self.telemetry_mode_combo.findData(TELEMETRY_MODE_PHYSICAL)
        if physical_index >= 0:
            self.telemetry_mode_combo.setItemText(
                physical_index, self._tr("telemetry_physical")
            )
        self._refresh_health_panel(update_rx_rate=False)
        self._update_stream_summary()

        self.tabs.setTabText(0, self._tr("waveforms"))
        self.tabs.setTabText(1, self._tr("data_panel"))
        self.auto_scale_checkbox.setText(self._tr("auto_scale"))
        self.auto_scale_checkbox.setToolTip(self._tr("auto_scale_tooltip"))
        self.opengl_checkbox.setText(self._tr("opengl_plot"))
        self.opengl_checkbox.setToolTip(self._tr("opengl_plot_tooltip"))
        self.clear_plots_btn.setText(self._tr("clear_plots"))
        self.clear_plots_btn.setToolTip(self._tr("clear_plots_tooltip"))
        self.import_csv_btn.setText(self._tr("import_csv"))
        self.import_csv_btn.setToolTip(self._tr("import_csv_tooltip"))
        self.add_plot_btn.setText(self._tr("add_plot"))
        self.add_plot_btn.setToolTip(self._tr("add_plot_tooltip"))

        for plot_index in range(len(self.plot_widgets)):
            self._update_plot_presentation(plot_index)

        self.data_table.setHorizontalHeaderLabels([
            self._tr("table_channel"),
            self._tr("voltage_mv"),
            self._tr("strain_ue"),
            self._tr("stress_mpa"),
            "Raw",
            "零点 Raw",
            self._tr("samples"),
        ])
        self.stats_group.setTitle(self._tr("voltage_statistics"))
        self._refresh_statistics_labels()

        if self._status_message is not None:
            key, kwargs = self._status_message
            self.status_bar.showMessage(self._format_message(key, **kwargs))

    def _format_message(self, key: str, **kwargs) -> str:
        values = dict(kwargs)
        if "command" in values:
            values["command"] = self._display_command_name(values["command"])
        if "reason" in values:
            values["reason"] = self._display_status_reason(values["reason"])
        return self._tr(key, **values)

    def _show_status(self, key: str, **kwargs):
        self._status_message = (key, kwargs)
        self.status_bar.showMessage(self._format_message(key, **kwargs))

    def _display_command_name(self, command_name: str) -> str:
        if self.language == "zh":
            return COMMAND_TRANSLATIONS.get(command_name, command_name)
        return command_name

    def _display_status_reason(self, reason: str) -> str:
        if self.language == "zh":
            return STATUS_TRANSLATIONS.get(reason, reason)
        return reason

    def _physical_values_from_raw(self, raw_values) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        raw = np.asarray(raw_values, dtype=np.float64)
        raw_to_mv_scale = (
            (2.0 * (self.vref_uv / 1_000_000.0)) / self.pga_gain
        ) * (1000.0 / 8388608.0)
        voltage = raw * raw_to_mv_scale
        strain = np.clip(
            voltage * MV_TO_MICROSTRAIN_SCALE,
            -MAX_STRAIN_UE,
            MAX_STRAIN_UE,
        )
        stress = strain * ELASTIC_MODULUS_MPA / 1000000.0
        return voltage, strain, stress

    def _refresh_statistics_labels(self):
        for channel in range(NUM_CHANNELS):
            self._update_statistics_label(channel)

    def _update_statistics_label(self, channel: int):
        stats = self.channel_stats[channel]
        min_lbl, max_lbl, avg_lbl = self.stats_labels[channel]
        if not stats["count"]:
            min_lbl.setText(f"{self._tr('min')}: -")
            max_lbl.setText(f"{self._tr('max')}: -")
            avg_lbl.setText(f"{self._tr('avg')}: -")
            return

        avg = stats["sum"] / stats["count"]
        min_lbl.setText(f"{self._tr('min')}: {stats['min']:.3f} mV")
        max_lbl.setText(f"{self._tr('max')}: {stats['max']:.3f} mV")
        avg_lbl.setText(f"{self._tr('avg')}: {avg:.3f} mV")

    def _on_interface_changed(self, _index: int):
        self._refresh_channels()

    def _refresh_channels(self):
        """Refresh the list of available CAN channels"""
        interface = self.interface_combo.currentData()
        selected_channel = self._selected_channel()
        self.channel_combo.clear()
        for channel, desc in list_can_channels(interface):
            label = channel if not desc or desc == channel else f"{channel} - {desc}"
            self.channel_combo.addItem(label, channel)
        if self.channel_combo.count() == 0:
            fallback_channel = {
                "slcan": "COM1",
                "pcan": "PCAN_USBBUS1",
                "ixxat": "0",
                "vector": "0",
            }.get(interface, "can0")
            self.channel_combo.addItem(fallback_channel, fallback_channel)

        if selected_channel:
            existing_index = self.channel_combo.findData(selected_channel)
            if existing_index >= 0:
                self.channel_combo.setCurrentIndex(existing_index)
            else:
                self.channel_combo.setEditText(selected_channel)

    def _set_connection_controls_enabled(self, enabled: bool):
        self.interface_combo.setEnabled(enabled)
        self.channel_combo.setEnabled(enabled)
        self.slcan_speed_combo.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)

    @staticmethod
    def _format_sample_rate(sample_rate: float) -> str:
        return f"{sample_rate:g}"

    def _active_adc_count(self) -> int:
        channel_mask = self._displayed_channel_mask()
        return sum(
            1 for adc_mask in ADC_CHANNEL_MASKS if channel_mask & adc_mask
        )

    @staticmethod
    def _ads1256_cycling_rate(sample_rate: float) -> float:
        return ADS1256_CYCLING_RATES.get(sample_rate, sample_rate)

    def _estimated_telemetry_rate(self, sample_rate: float) -> float:
        return (
            self._ads1256_cycling_rate(sample_rate) * self._active_adc_count()
        )

    def _telemetry_mode_text(self, mode: Optional[int] = None) -> str:
        selected = self.telemetry_mode if mode is None else mode
        return (
            self._tr("telemetry_physical")
            if selected == TELEMETRY_MODE_PHYSICAL
            else self._tr("telemetry_raw")
        )

    def _update_stream_summary(self):
        if not hasattr(self, "stream_summary_label"):
            return
        self.stream_summary_label.setText(
            self._tr(
                "stream_summary",
                sample_rate=self._format_sample_rate(self.sample_rate_sps),
                scan_rate=(
                    self._ads1256_cycling_rate(self.sample_rate_sps) *
                    self._active_adc_count()
                ),
                telemetry_rate=self._estimated_telemetry_rate(self.sample_rate_sps),
                mode=self._telemetry_mode_text(),
            )
        )

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
        colors = ['#569cd6', '#d7ba7d', '#6a9955', '#c586c0', '#4ec9b0', '#ce9178']
        return colors[ch % len(colors)]

    @staticmethod
    def _create_waveform_buffers():
        voltage_buffers = [
            WaveformRingBuffer(WAVEFORM_BUFFER_SIZE) for _ in range(NUM_CHANNELS)
        ]
        return voltage_buffers, {
            "voltage": voltage_buffers,
            "strain": [
                WaveformRingBuffer(WAVEFORM_BUFFER_SIZE) for _ in range(NUM_CHANNELS)
            ],
            "stress": [
                WaveformRingBuffer(WAVEFORM_BUFFER_SIZE) for _ in range(NUM_CHANNELS)
            ],
        }

    def _add_waveform_plot(self, channel: Optional[int] = None):
        if len(self.plot_panels) >= MAX_VISIBLE_PLOTS:
            return

        if channel is None:
            used_channels = set(self.plot_channels)
            channel = next(
                (ch for ch in range(NUM_CHANNELS) if ch not in used_channels),
                0,
            )

        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)

        header = QHBoxLayout()
        source_label = QLabel()
        header.addWidget(source_label)

        channel_combo = QComboBox()
        for source_channel in range(NUM_CHANNELS):
            channel_combo.addItem(f"CH{source_channel}", source_channel)
        channel_combo.setCurrentIndex(channel)
        header.addWidget(channel_combo)

        metrics_label = QLabel()
        header.addWidget(metrics_label)
        metric_checkboxes = {}
        metric_color_buttons = {}
        metric_colors = {
            metric: config["color"] for metric, config in PLOT_METRICS.items()
        }
        for metric in PLOT_METRICS:
            checkbox = QCheckBox()
            checkbox.setChecked(metric == "voltage")
            metric_checkboxes[metric] = checkbox
            header.addWidget(checkbox)
            color_button = QPushButton()
            color_button.setFixedSize(18, 18)
            color_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            metric_color_buttons[metric] = color_button
            header.addWidget(color_button)
        header.addStretch()

        remove_button = QPushButton()
        remove_button.clicked.connect(
            lambda _checked=False, selected_panel=panel:
                self._remove_waveform_plot(selected_panel)
        )
        header.addWidget(remove_button)
        panel_layout.addLayout(header)

        plot = pg.PlotWidget()
        plot.installEventFilter(self)
        apply_plot_theme(plot, self.theme)
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.disableAutoRange()
        plot.setXRange(0, 100, padding=0.0)
        plot.setYRange(-1.0, 1.0, padding=0.0)
        plot.scene().sigMouseClicked.connect(
            lambda event, selected_panel=panel:
                self._on_plot_mouse_clicked(selected_panel, event)
        )
        hover_items = create_plot_hover_items(plot)
        plot.scene().sigMouseMoved.connect(
            lambda scene_pos, selected_plot=plot:
                self._on_plot_mouse_moved(selected_plot, scene_pos)
        )
        metric_curves = {}
        for metric, config in PLOT_METRICS.items():
            curve = plot.plot(
                pen=pg.mkPen(color=metric_colors[metric], width=1.5)
            )
            curve.setVisible(metric == "voltage")
            metric_curves[metric] = curve
        panel_layout.addWidget(plot)

        self.plot_panels.append(panel)
        self.plot_widgets.append(plot)
        self.plot_curves.append(metric_curves["voltage"])
        self.plot_channel_combos.append(channel_combo)
        self.plot_source_labels.append(source_label)
        self.plot_metrics_labels.append(metrics_label)
        self.plot_metric_checkboxes.append(metric_checkboxes)
        self.plot_metric_color_buttons.append(metric_color_buttons)
        self.plot_metric_colors.append(metric_colors)
        self.plot_metric_curves.append(metric_curves)
        self.plot_hover_items.append(hover_items)
        self.plot_remove_buttons.append(remove_button)
        self.plot_channels.append(channel)

        channel_combo.currentIndexChanged.connect(
            lambda _index, selected_plot=plot:
                self._on_plot_channel_changed(selected_plot)
        )
        for metric, checkbox in metric_checkboxes.items():
            checkbox.toggled.connect(
                lambda checked, selected_plot=plot, selected_metric=metric:
                    self._on_plot_metric_toggled(
                        selected_plot, selected_metric, checked
                )
            )
        for metric, button in metric_color_buttons.items():
            button.clicked.connect(
                lambda _checked=False, selected_plot=plot, selected_metric=metric:
                    self._choose_plot_metric_color(selected_plot, selected_metric)
            )

        plot_index = len(self.plot_widgets) - 1
        self._update_plot_presentation(plot_index)
        self._refresh_plot_data(plot_index)
        if self.auto_scale_checkbox.isChecked():
            self._update_plot_range(plot_index)

        self._refresh_waveform_layout()
        self._update_plot_controls()

    def _remove_waveform_plot(self, panel: QWidget):
        if panel not in self.plot_panels:
            return

        plot_index = self.plot_panels.index(panel)
        if self.maximized_plot_panel is panel:
            self.maximized_plot_panel = None
            self.maximized_plot_channel = None

        self.waveform_layout.removeWidget(panel)
        self.plot_panels.pop(plot_index)
        self.plot_widgets.pop(plot_index)
        self.plot_curves.pop(plot_index)
        self.plot_channel_combos.pop(plot_index)
        self.plot_source_labels.pop(plot_index)
        self.plot_metrics_labels.pop(plot_index)
        self.plot_metric_checkboxes.pop(plot_index)
        self.plot_metric_color_buttons.pop(plot_index)
        self.plot_metric_colors.pop(plot_index)
        self.plot_metric_curves.pop(plot_index)
        self.plot_hover_items.pop(plot_index)
        self.plot_remove_buttons.pop(plot_index)
        self.plot_channels.pop(plot_index)
        panel.deleteLater()

        self._refresh_waveform_layout()
        self._update_plot_controls()

    def _on_plot_channel_changed(self, plot: pg.PlotWidget):
        if plot not in self.plot_widgets:
            return

        plot_index = self.plot_widgets.index(plot)
        channel = self.plot_channel_combos[plot_index].currentData()
        if channel is None:
            return

        self.plot_channels[plot_index] = int(channel)
        hide_plot_hover_items(self.plot_hover_items[plot_index])
        self._refresh_plot_data(plot_index)
        if self.auto_scale_checkbox.isChecked():
            self._update_plot_range(plot_index)
        self._update_plot_presentation(plot_index)
        if self.maximized_plot_panel is self.plot_panels[plot_index]:
            self.maximized_plot_channel = int(channel)

    def _on_plot_metric_toggled(
        self, plot: pg.PlotWidget, metric: str, enabled: bool
    ):
        if plot not in self.plot_widgets:
            return

        plot_index = self.plot_widgets.index(plot)
        checkboxes = self.plot_metric_checkboxes[plot_index]
        if not enabled and not any(
            checkbox.isChecked() for checkbox in checkboxes.values()
        ):
            with QSignalBlocker(checkboxes[metric]):
                checkboxes[metric].setChecked(True)
            return

        self.plot_metric_curves[plot_index][metric].setVisible(enabled)
        hide_plot_hover_items(self.plot_hover_items[plot_index])
        self._refresh_plot_data(plot_index)
        self._update_plot_presentation(plot_index)
        if self.auto_scale_checkbox.isChecked():
            self._update_plot_range(plot_index)

    def _selected_plot_metrics(self, plot_index: int) -> List[str]:
        return [
            metric
            for metric, checkbox in self.plot_metric_checkboxes[plot_index].items()
            if checkbox.isChecked()
        ]

    def _choose_plot_metric_color(self, plot: pg.PlotWidget, metric: str):
        if plot not in self.plot_widgets:
            return

        plot_index = self.plot_widgets.index(plot)
        selected = QColorDialog.getColor(
            QColor(self.plot_metric_colors[plot_index][metric]),
            self,
            self._tr(PLOT_METRICS[metric]["label_key"]),
        )
        if selected.isValid():
            self._set_plot_metric_color(plot_index, metric, selected.name())

    def _set_plot_metric_color(self, plot_index: int, metric: str, color: str):
        selected = QColor(color)
        if not selected.isValid():
            return

        normalized = selected.name()
        self.plot_metric_colors[plot_index][metric] = normalized
        self.plot_metric_curves[plot_index][metric].setPen(
            pg.mkPen(normalized, width=1.5)
        )
        self.plot_metric_color_buttons[plot_index][metric].setStyleSheet(
            curve_color_button_stylesheet(normalized)
        )

    def _on_plot_mouse_moved(self, plot: pg.PlotWidget, scene_pos):
        if plot not in self.plot_widgets:
            return

        plot_index = self.plot_widgets.index(plot)
        hover_items = self.plot_hover_items[plot_index]
        if not plot.getPlotItem().sceneBoundingRect().contains(scene_pos):
            hide_plot_hover_items(hover_items)
            return

        mouse = plot.getPlotItem().vb.mapSceneToView(scene_pos)
        sample = int(round(mouse.x()))
        x_range, y_range = plot.viewRange()
        x_tolerance = max(1.0, (x_range[1] - x_range[0]) * 12.0 / max(plot.width(), 1))
        y_tolerance = max(0.05, (y_range[1] - y_range[0]) * 0.08)
        if abs(mouse.x() - sample) > x_tolerance:
            hide_plot_hover_items(hover_items)
            return

        channel = self.plot_channels[plot_index]
        nearest = None
        for metric in self._selected_plot_metrics(plot_index):
            values = self.waveform_metric_buffers[metric][channel]
            total = len(values)
            visible = values.to_array(PLOT_VISIBLE_SAMPLES)
            first_sample = max(0, total - len(visible))
            local_sample = sample - first_sample
            if not 0 <= local_sample < len(visible):
                continue
            value = float(visible[local_sample])
            distance = abs(mouse.y() - value)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, metric, value)

        if nearest is None or nearest[0] > y_tolerance:
            hide_plot_hover_items(hover_items)
            return

        _, metric, value = nearest
        config = PLOT_METRICS[metric]
        color = self.plot_metric_colors[plot_index][metric]
        hover_items["line"].setPos(sample)
        hover_items["marker"].setData([sample], [value], brush=pg.mkBrush(color))
        hover_items["label"].setText(
            f"{self._tr(config['axis_key'])}\n"
            f"{self._tr('sample')}: {sample}\n"
            f"{value:.3f} {config['units']}"
        )
        hover_items["label"].setAnchor(
            (1, 1) if sample > sum(x_range) / 2.0 else (0, 1)
        )
        hover_items["label"].setPos(sample, value)
        for item in hover_items.values():
            item.show()

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.Leave and watched in self.plot_widgets:
            plot_index = self.plot_widgets.index(watched)
            hide_plot_hover_items(self.plot_hover_items[plot_index])
        return super().eventFilter(watched, event)

    def _refresh_plot_data(self, plot_index: int):
        channel = self.plot_channels[plot_index]
        for metric, curve in self.plot_metric_curves[plot_index].items():
            values = self.waveform_metric_buffers[metric][channel].to_array(
                PLOT_VISIBLE_SAMPLES
            )
            if values.size:
                first_sample = max(0, len(self.waveform_metric_buffers[metric][channel]) - values.size)
                curve.setData(np.arange(first_sample, first_sample + values.size), values)
            else:
                curve.setData([], [])

    def _update_plot_presentation(self, plot_index: int):
        channel = self.plot_channels[plot_index]
        plot = self.plot_widgets[plot_index]
        plot.setTitle(
            self._tr("plot_title", channel=channel),
            color=THEME_COLORS[self.theme]["plot_axis"],
        )
        selected_metrics = self._selected_plot_metrics(plot_index)
        if len(selected_metrics) == 1:
            config = PLOT_METRICS[selected_metrics[0]]
            plot.setLabel(
                "left", self._tr(config["axis_key"]), units=config["units"]
            )
        else:
            plot.setLabel("left", self._tr("value"))
        plot.setLabel("bottom", self._tr("sample"))
        plot.setToolTip(self._tr("plot_tooltip"))
        self.plot_source_labels[plot_index].setText(self._tr("plot_source"))
        self.plot_channel_combos[plot_index].setToolTip(
            self._tr("plot_source_tooltip")
        )
        self.plot_metrics_labels[plot_index].setText(self._tr("plot_metrics"))
        for metric, checkbox in self.plot_metric_checkboxes[plot_index].items():
            config = PLOT_METRICS[metric]
            color = self.plot_metric_colors[plot_index][metric]
            checkbox.setText(self._tr(config["label_key"]))
            checkbox.setToolTip(self._tr("plot_metrics_tooltip"))
            checkbox.setStyleSheet(
                f"color: {THEME_COLORS[self.theme]['text']};"
            )
            color_button = self.plot_metric_color_buttons[plot_index][metric]
            color_button.setStyleSheet(curve_color_button_stylesheet(color))
            color_button.setToolTip(self._tr("plot_metrics_tooltip"))
        self.plot_remove_buttons[plot_index].setText(self._tr("remove_plot"))
        self.plot_remove_buttons[plot_index].setToolTip(
            self._tr("remove_plot_tooltip")
        )

    def _update_plot_controls(self):
        self.add_plot_btn.setEnabled(len(self.plot_panels) < MAX_VISIBLE_PLOTS)
        for button in self.plot_remove_buttons:
            button.setEnabled(True)

    def _displayed_channel_mask(self) -> int:
        channel_mask = 0
        for channel in self.plot_channels:
            channel_mask |= 1 << channel
        return channel_mask

    def _sync_mcu_channel_mask(
        self, *, show_status: bool = True, force: bool = False
    ) -> bool:
        self._update_stream_summary()
        if not self.is_connected:
            return True

        channel_mask = self._displayed_channel_mask()
        if not force and channel_mask == self.last_sent_channel_mask:
            return True

        if self.send_command(CAN_CMD_SET_CHANNEL_MASK, value=channel_mask):
            self.last_sent_channel_mask = channel_mask
            if show_status:
                self._show_status("channel_mask_sent", mask=channel_mask)
            return True

        if show_status:
            self._show_status("channel_mask_failed")
        return False

    def _on_plot_mouse_clicked(self, panel: QWidget, event):
        if event.double():
            self._toggle_plot_maximize(panel)

    def _toggle_plot_maximize(self, plot):
        panel = self.plot_panels[plot] if isinstance(plot, int) else plot
        if self.maximized_plot_panel is panel:
            self.maximized_plot_panel = None
            self.maximized_plot_channel = None
        else:
            self.maximized_plot_panel = panel
            plot_index = self.plot_panels.index(panel)
            self.maximized_plot_channel = self.plot_channels[plot_index]
        self._refresh_waveform_layout()

    def _on_auto_scale_toggled(self, enabled: bool):
        if enabled:
            self.plot_dirty = [True for _ in range(NUM_CHANNELS)]

    def _on_opengl_toggled(self, enabled: bool):
        applied = configure_plot_backend(enabled)
        if enabled and not applied:
            with QSignalBlocker(self.opengl_checkbox):
                self.opengl_checkbox.setChecked(False)
            self._show_status("opengl_unavailable")
        for plot_index in range(len(self.plot_widgets)):
            self._refresh_plot_data(plot_index)

    def _set_plot_refresh_rate(self, refresh_hz: int):
        interval_ms = max(1, round(1000 / refresh_hz))
        self.update_timer.start(interval_ms)

    def _clear_plots(self):
        with self.lock:
            self.waveform_buffers, self.waveform_metric_buffers = (
                self._create_waveform_buffers()
            )
            self.plot_dirty = [False for _ in range(NUM_CHANNELS)]
            self.pending_telemetry_batches.clear()
            self.csv_pending_rows.clear()

        for curves in self.plot_metric_curves:
            for curve in curves.values():
                curve.setData([], [])
        for hover_items in self.plot_hover_items:
            hide_plot_hover_items(hover_items)

        self._show_status("waveforms_cleared")

    def import_csv(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, self._tr("import_waveform_log"), "", self._tr("csv_files")
        )
        if filename:
            try:
                window = OfflineWaveformWindow.from_csv(
                    filename, self.language
                )
            except Exception as exc:
                QMessageBox.critical(
                    self, self._tr("error"), self._tr("failed_import_csv", error=exc)
                )
                logger.error("Failed to import CSV %s: %s", filename, exc)
                return

            self.offline_waveform_windows.append(window)
            window.destroyed.connect(
                lambda _=None, opened_window=window:
                    self._discard_offline_waveform_window(opened_window)
            )
            window.show()
            self._show_status("opened_waveform_log", filename=filename)
            logger.info("Opened waveform log %s", filename)

    def _discard_offline_waveform_window(
        self, window: OfflineWaveformWindow
    ) -> None:
        if window in self.offline_waveform_windows:
            self.offline_waveform_windows.remove(window)

    def _refresh_waveform_layout(self):
        while self.waveform_layout.count():
            self.waveform_layout.takeAt(0)

        for panel in self.plot_panels:
            panel.hide()

        if self.maximized_plot_panel is not None:
            for row in range(MAX_VISIBLE_PLOTS):
                self.waveform_layout.setRowStretch(row, 0)
            for column in range(MAX_VISIBLE_PLOTS):
                self.waveform_layout.setColumnStretch(column, 0)

            self.waveform_layout.addWidget(self.maximized_plot_panel, 0, 0, 1, 1)
            self.maximized_plot_panel.show()
            return

        columns = 2 if len(self.plot_panels) <= 4 else 3
        rows = (len(self.plot_panels) + columns - 1) // columns
        for row in range(MAX_VISIBLE_PLOTS):
            self.waveform_layout.setRowStretch(row, 0)
        for column in range(MAX_VISIBLE_PLOTS):
            self.waveform_layout.setColumnStretch(column, 0)
        for row in range(rows):
            self.waveform_layout.setRowStretch(row, 1)
        for column in range(columns):
            self.waveform_layout.setColumnStretch(column, 1)

        for plot_index, panel in enumerate(self.plot_panels):
            self.waveform_layout.addWidget(
                panel, plot_index // columns, plot_index % columns
            )
            panel.show()

    def _reset_measurements(self):
        with self.lock:
            self.channel_data = [ChannelData() for _ in range(NUM_CHANNELS)]
            self.waveform_buffers, self.waveform_metric_buffers = (
                self._create_waveform_buffers()
            )
            self.plot_dirty = [False for _ in range(NUM_CHANNELS)]
            self.pending_table_data = [None for _ in range(NUM_CHANNELS)]
            self.channel_stats = [
                {'min': None, 'max': None, 'sum': 0.0, 'count': 0}
                for _ in range(NUM_CHANNELS)
            ]
            self.rx_telemetry_count = 0
            self.rx_status_count = 0
            self.rx_bad_protocol_count = 0
            self.last_rx_telemetry_count = 0
            self.rx_telemetry_rate_hz = 0.0
            self.latest_health = None
            self.health_tx_drop_delta = 0
            self.health_adc_overflow_delta = 0
            self.health_adc_recovery_delta = 0
            self.pending_telemetry_batches.clear()
            self.csv_pending_rows.clear()

        if hasattr(self, 'data_table'):
            for row in range(NUM_CHANNELS):
                for col in range(1, 7):
                    self.data_table.item(row, col).setText("-")

        if hasattr(self, 'stats_labels'):
            self._refresh_statistics_labels()

        if hasattr(self, 'plot_curves'):
            for curves in self.plot_metric_curves:
                for curve in curves.values():
                    curve.setData([], [])
        self._refresh_health_panel(update_rx_rate=False)

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

        sequence = self.command_sequence
        for _ in range(256):
            if sequence not in self.pending_commands:
                break
            sequence = (sequence + 1) & 0xFF
        else:
            logger.warning("Cannot send command: all command sequences are pending")
            return False

        frame = build_command_frame(sequence, cmd_type, param, value)
        command_name = self._command_name(cmd_type)
        sent = self.can_bus.send_frame(frame)
        if sent:
            self.pending_commands[sequence] = command_name
            self.pending_command_deadlines[sequence] = time.monotonic() + COMMAND_ACK_TIMEOUT_S
            self.command_sequence = (sequence + 1) & 0xFF
        return sent

    def on_zero_clicked(self):
        """Handle Zero Sensor button click - saves zero offset to Flash"""
        if self.send_command(CAN_CMD_ZERO_DATUM):
            self._show_status("zero_sent")
            logger.info("Zero command sent")
        else:
            self._show_status("zero_failed")
            logger.warning("Failed to send zero command")

    def on_calib_clicked(self):
        """Handle Calibrate button click"""
        if self.send_command(CAN_CMD_START_CALIB):
            self._show_status("calibration_sent")
            logger.info("Calibration command sent")
        else:
            self._show_status("calibration_failed")
            logger.warning("Failed to send calibration command")

    def on_filter_size_changed(self, value):
        """Handle filter size spinbox change"""
        if self.send_command(CAN_CMD_SET_FILTER_SIZE, param=0, value=value):
            logger.info(f"Filter size set to {value}")
        else:
            self._show_status("filter_size_failed")
            logger.warning("Failed to send filter size command")

    def on_sample_rate_changed(self, index: int):
        """Handle sample rate combobox change"""
        if index < 0 or not self.is_connected:
            return

        sample_rate = self.sample_rate_combo.currentData()
        if sample_rate is None:
            return

        sample_rate = float(sample_rate)

        if sample_rate.is_integer():
            sample_rate_param = 0
            sample_rate_value = int(sample_rate)
        else:
            sample_rate_param = 1
            sample_rate_value = round(sample_rate * 10)

        if self.send_command(
            CAN_CMD_SET_SAMPLE_RATE,
            param=sample_rate_param,
            value=sample_rate_value,
        ):
            logger.info("Sample rate set to %s SPS", sample_rate)
        else:
            self._show_status("sample_rate_failed")
            logger.warning("Failed to send sample rate command")

    def on_telemetry_mode_changed(self, index: int):
        if index < 0 or not self.is_connected:
            return

        mode = self.telemetry_mode_combo.currentData()
        if mode is None:
            return

        mode = int(mode)
        if self.send_command(CAN_CMD_SET_TELEMETRY_MODE, param=0, value=mode):
            logger.info("Telemetry mode set to %s", mode)
        else:
            self._show_status("telemetry_mode_failed")
            logger.warning("Failed to send telemetry mode command")

    def on_clear_zero_clicked(self):
        """Handle Clear Zero button click - clears offsets from Flash"""
        if self.send_command(CAN_CMD_CLEAR_ZERO):
            self._show_status("clear_zero_sent")
            logger.info("Clear Zero command sent")
        else:
            self._show_status("clear_zero_failed")
            logger.warning("Failed to clear zero offset")

    def on_vref_changed(self):
        if self.is_connected:
            self.send_command(CAN_CMD_SET_VREF_UV, value=round(self.vref_spin.value() * 1_000_000))

    def on_pga_changed(self, index: int):
        if self.is_connected and index >= 0:
            gain = self.pga_combo.currentData()
            if gain is not None:
                self.send_command(CAN_CMD_SET_PGA, value=int(gain))

    def on_channel_mask_changed(self):
        if self.is_connected:
            self.send_command(CAN_CMD_SET_CHANNEL_MASK, value=self.channel_mask_spin.value())

    def on_restore_defaults_clicked(self):
        answer = QMessageBox.question(
            self, "恢复默认配置", "恢复默认配置会清除所有零点，是否继续？"
        )
        if answer == QMessageBox.StandardButton.Yes:
            self.send_command(CAN_CMD_RESTORE_DEFAULTS)

    def _apply_config_snapshot(self, config):
        self.vref_uv = config.vref_uv
        self.pga_gain = config.pga_gain
        self.config_sequence = config.sequence
        self.zero_offsets = list(config.zero_offsets)
        self.zero_valid = config.zero_valid
        self.sample_rate_sps = config.sample_rate_x10 / 10.0
        self.telemetry_mode = config.telemetry_mode
        with QSignalBlocker(self.vref_spin):
            self.vref_spin.setValue(self.vref_uv / 1_000_000.0)
        with QSignalBlocker(self.pga_combo):
            index = self.pga_combo.findData(self.pga_gain)
            if index >= 0:
                self.pga_combo.setCurrentIndex(index)
        with QSignalBlocker(self.channel_mask_spin):
            self.channel_mask_spin.setValue(config.channel_mask)
        with QSignalBlocker(self.filter_size_spin):
            self.filter_size_spin.setValue(config.filter_length)
        with QSignalBlocker(self.sample_rate_combo):
            index = self.sample_rate_combo.findData(self.sample_rate_sps)
            if index >= 0:
                self.sample_rate_combo.setCurrentIndex(index)
        with QSignalBlocker(self.telemetry_mode_combo):
            index = self.telemetry_mode_combo.findData(self.telemetry_mode)
            if index >= 0:
                self.telemetry_mode_combo.setCurrentIndex(index)
        self.data_table.setColumnHidden(4, self.telemetry_mode != TELEMETRY_MODE_RAW)
        self.data_table.setColumnHidden(5, self.telemetry_mode != TELEMETRY_MODE_RAW)
        self.config_state_label.setText(
            ("已保存" if config.saved else "等待保存") +
            (" | 零点有效" if config.zero_valid else " | 需要重新调零")
        )
        self._update_stream_summary()

    def _create_command_group(self) -> QGroupBox:
        """Create the command control group"""
        group = QGroupBox()
        layout = QVBoxLayout()
        controls = QHBoxLayout()

        self.zero_btn = QPushButton()
        self.zero_btn.clicked.connect(self.on_zero_clicked)
        self.zero_btn.setEnabled(False)
        controls.addWidget(self.zero_btn)

        self.calib_btn = QPushButton()
        self.calib_btn.clicked.connect(self.on_calib_clicked)
        self.calib_btn.setEnabled(False)
        controls.addWidget(self.calib_btn)

        self.clear_zero_btn = QPushButton()
        self.clear_zero_btn.clicked.connect(self.on_clear_zero_clicked)
        self.clear_zero_btn.setEnabled(False)
        controls.addWidget(self.clear_zero_btn)

        # Filter size control
        self.sample_rate_label = QLabel()
        controls.addWidget(self.sample_rate_label)
        self.sample_rate_combo = QComboBox()
        for sample_rate in SUPPORTED_SAMPLE_RATES:
            self.sample_rate_combo.addItem(
                f"{self._format_sample_rate(float(sample_rate))} SPS", sample_rate
            )
        self.sample_rate_combo.setCurrentText("100 SPS")
        self.sample_rate_combo.setEnabled(False)
        self.sample_rate_combo.currentIndexChanged.connect(self.on_sample_rate_changed)
        controls.addWidget(self.sample_rate_combo)

        # Filter size control
        self.filter_size_label = QLabel()
        controls.addWidget(self.filter_size_label)
        self.filter_size_spin = QSpinBox()
        self.filter_size_spin.setMinimum(2)
        self.filter_size_spin.setMaximum(64)
        self.filter_size_spin.setValue(16)
        self.filter_size_spin.setEnabled(False)
        self.filter_size_spin.valueChanged.connect(self.on_filter_size_changed)
        controls.addWidget(self.filter_size_spin)

        self.telemetry_mode_label = QLabel()
        controls.addWidget(self.telemetry_mode_label)
        self.telemetry_mode_combo = QComboBox()
        self.telemetry_mode_combo.addItem("", TELEMETRY_MODE_RAW)
        self.telemetry_mode_combo.addItem("", TELEMETRY_MODE_PHYSICAL)
        self.telemetry_mode_combo.setEnabled(False)
        self.telemetry_mode_combo.currentIndexChanged.connect(
            self.on_telemetry_mode_changed
        )
        controls.addWidget(self.telemetry_mode_combo)

        controls.addStretch()
        layout.addLayout(controls)

        self.stream_summary_label = QLabel()
        self.stream_summary_label.setStyleSheet(
            f"color: {THEME_COLORS[self.theme]['muted_text']}; font-weight: 600;"
        )
        layout.addWidget(self.stream_summary_label)

        group.setLayout(layout)
        return group

    def _refresh_health_panel(self, *, update_rx_rate: bool = True):
        if not hasattr(self, "health_summary_label"):
            return

        if update_rx_rate:
            telemetry_delta = self.rx_telemetry_count - self.last_rx_telemetry_count
            self.last_rx_telemetry_count = self.rx_telemetry_count
            self.rx_telemetry_rate_hz = float(max(0, telemetry_delta))

        health = self.latest_health
        if health is None:
            self._set_health_status(
                "waiting",
                self._tr(
                    "health_waiting",
                    rx_rate=self.rx_telemetry_rate_hz,
                    bad=self.rx_bad_protocol_count,
                ),
            )
            return

        counters = (
            self.health_tx_drop_delta,
            self.health_adc_overflow_delta,
            self.health_adc_recovery_delta,
            self.rx_bad_protocol_count,
        )
        state = "ok" if health.adc_running and not any(counters) else "warning"
        if not health.adc_running:
            state = "error"
        self._set_health_status(
            state,
            self._tr(
                "health_summary",
                adc_state=self._tr(
                    "adc_running" if health.adc_running else "adc_stopped"
                ),
                rx_rate=self.rx_telemetry_rate_hz,
                sample_rate=self._format_sample_rate(health.sample_rate_sps),
                mode=self._telemetry_mode_text(health.telemetry_mode),
                tx_samples=health.telemetry_samples_per_second,
                tx_frames=health.telemetry_frames_per_second,
                drop_delta=self.health_tx_drop_delta,
                tx_drop=health.tx_drop_count,
                overflow_delta=self.health_adc_overflow_delta,
                overflow=health.adc_overflow_count,
                recovery=health.adc_recovery_count,
                bad=self.rx_bad_protocol_count,
            ),
        )

    def _set_health_status(self, state: str, text: str):
        self.health_icon_label.setPixmap(status_icon_pixmap(state))
        self.health_icon_label.setToolTip(text)
        self.health_summary_label.setText(text)
        self.health_summary_label.setToolTip(text)

    def connect(self):
        """Connect to the CAN adapter"""
        interface = self.interface_combo.currentData()
        channel = self._selected_channel()
        if not channel:
            QMessageBox.warning(
                self, self._tr("error"), self._tr("please_select_channel")
            )
            return

        baudrate = FIXED_CAN_BITRATE
        tty_baudrate = int(self.slcan_speed_combo.currentData() or DEFAULT_SLCAN_TTY_BAUDRATE)

        try:
            self.can_bus = PythonCANInterface()
            speeds = [tty_baudrate]
            if interface == "slcan":
                speeds.extend(
                    speed for speed in (921_600, 460_800, 115_200)
                    if speed < tty_baudrate
                )
            connected = False
            for speed in speeds:
                if self.can_bus.connect(
                    interface, channel, baudrate, tty_baudrate=speed
                ):
                    tty_baudrate = speed
                    speed_index = self.slcan_speed_combo.findData(speed)
                    if speed_index >= 0:
                        self.slcan_speed_combo.setCurrentIndex(speed_index)
                    connected = True
                    break
            if not connected:
                interface_help = (
                    self._tr("slcan_help")
                    if interface == "slcan"
                    else self._tr("socketcan_help")
                )
                QMessageBox.critical(
                    self,
                    self._tr("error"),
                    self._tr(
                        "failed_connect_target",
                        interface=interface,
                        channel=channel,
                        help=interface_help,
                    ),
                )
                self.can_bus = None
                return

            # Start receiver thread
            self.can_receiver = CANReceiver(self.can_bus)
            self.can_receiver.frame_received.connect(self.on_can_frame_received)
            self.can_receiver.start()
            self._reset_measurements()
            self.pending_commands.clear()
            self.pending_command_deadlines.clear()
            self.command_sequence = 0

            self.is_connected = True
            self._set_connection_controls_enabled(False)
            self.connect_btn.setText(self._tr("disconnect"))
            self.log_btn.setEnabled(True)
            self.zero_btn.setEnabled(True)
            self.calib_btn.setEnabled(True)
            self.clear_zero_btn.setEnabled(True)
            self.sample_rate_combo.setEnabled(True)
            self.filter_size_spin.setEnabled(True)
            self.telemetry_mode_combo.setEnabled(True)
            self.vref_spin.setEnabled(True)
            self.pga_combo.setEnabled(True)
            self.channel_mask_spin.setEnabled(True)
            self.restore_defaults_btn.setEnabled(True)
            self.send_command(CAN_CMD_GET_CONFIG)
            self._show_status("connected_device", interface=interface, channel=channel)

            logger.info(
                "Connected to %s:%s (CAN FD bitrate %s/%s)",
                interface,
                channel,
                baudrate.bps,
                CAN_FD_DATA_BITRATE,
            )

        except Exception as e:
            QMessageBox.critical(
                self, self._tr("error"), self._tr("connection_failed", error=e)
            )
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
        self.last_sent_channel_mask = None
        self._set_connection_controls_enabled(True)
        self.pending_commands.clear()
        self.pending_command_deadlines.clear()
        self._reset_measurements()
        self.connect_btn.setText(self._tr("connect"))
        self.log_btn.setEnabled(False)
        self.zero_btn.setEnabled(False)
        self.calib_btn.setEnabled(False)
        self.clear_zero_btn.setEnabled(False)
        self.sample_rate_combo.setEnabled(False)
        self.filter_size_spin.setEnabled(False)
        self.telemetry_mode_combo.setEnabled(False)
        self.vref_spin.setEnabled(False)
        self.pga_combo.setEnabled(False)
        self.channel_mask_spin.setEnabled(False)
        self.restore_defaults_btn.setEnabled(False)
        self._show_status("disconnected")
        logger.info("Disconnected")

    def on_can_frame_received(self, frame: CANFrame):
        """Handle received CAN frames from the adapter"""
        status = parse_status_frame(frame)
        if status is not None:
            self.rx_status_count += 1
            self._handle_status_frame(status)
            return

        config = parse_config_frame(frame)
        if config is not None:
            self._apply_config_snapshot(config)
            return

        health = parse_health_frame(frame)
        if health is not None:
            previous_health = self.latest_health
            if previous_health is None:
                self.health_tx_drop_delta = 0
                self.health_adc_overflow_delta = 0
                self.health_adc_recovery_delta = 0
            else:
                self.health_tx_drop_delta = max(
                    0, health.tx_drop_count - previous_health.tx_drop_count
                )
                self.health_adc_overflow_delta = max(
                    0,
                    health.adc_overflow_count -
                    previous_health.adc_overflow_count,
                )
                self.health_adc_recovery_delta = max(
                    0,
                    health.adc_recovery_count -
                    previous_health.adc_recovery_count,
                )
            self.latest_health = health
            self.sample_rate_sps = health.sample_rate_sps
            self.telemetry_mode = health.telemetry_mode
            with QSignalBlocker(self.sample_rate_combo):
                sample_rate_index = self.sample_rate_combo.findData(
                    self.sample_rate_sps
                )
                if sample_rate_index >= 0:
                    self.sample_rate_combo.setCurrentIndex(sample_rate_index)
            with QSignalBlocker(self.telemetry_mode_combo):
                mode_index = self.telemetry_mode_combo.findData(self.telemetry_mode)
                if mode_index >= 0:
                    self.telemetry_mode_combo.setCurrentIndex(mode_index)
            self._update_stream_summary()
            self._refresh_health_panel(update_rx_rate=False)
            return

        telemetry_frames = parse_telemetry_frames(frame)
        if telemetry_frames is None:
            if frame.id in (CAN_ID_TX_TELEMETRY, CAN_ID_TX_STATUS,
                            CAN_ID_TX_HEALTH, CAN_ID_TX_CONFIG):
                self.rx_bad_protocol_count += 1
                logger.warning("Rejected malformed protocol frame on CAN ID 0x%03X", frame.id)
            return

        with self.lock:
            if len(self.pending_telemetry_batches) == self.pending_telemetry_batches.maxlen:
                self.display_drop_count += len(self.pending_telemetry_batches[0][0])
            self.pending_telemetry_batches.append((telemetry_frames, frame.timestamp))
            self.rx_telemetry_count += len(telemetry_frames)

    def _ingest_telemetry(self, telemetry, timestamp: Optional[float]):
        channel = telemetry.channel

        if channel >= NUM_CHANNELS:
            self.rx_bad_protocol_count += 1
            logger.warning(f"Invalid channel: {channel}")
            return

        if telemetry.raw_value is not None:
            voltage, strain, stress = self._physical_values_from_raw([telemetry.raw_value])
            voltage_mv = float(voltage[0])
            strain_ue = float(strain[0])
            stress_mpa = float(stress[0])
            voltage_001mv = int(round(voltage_mv * 100.0))
            stress_01mpa = int(round(stress_mpa * 10.0))
            raw_value = telemetry.raw_value
        else:
            voltage_mv = (
                telemetry.voltage_uv / 1000.0
                if telemetry.voltage_uv is not None
                else telemetry.voltage_001mv / 100.0
            )
            voltage_001mv = int(round(voltage_mv * 100.0))
            strain_ue = float(telemetry.strain_ue)
            stress_mpa = (
                telemetry.stress_qmpa * 0.25
                if telemetry.stress_qmpa is not None
                else telemetry.stress_01mpa / 10.0
            )
            stress_01mpa = int(round(stress_mpa * 10.0))
            raw_value = None

        self._apply_telemetry_values(
            channel, voltage_mv, strain_ue, stress_mpa,
            voltage_001mv, stress_01mpa, timestamp, raw_value,
        )

    def _apply_telemetry_values(
        self,
        channel: int,
        voltage_mv: float,
        strain_ue: float,
        stress_mpa: float,
        voltage_001mv: int,
        stress_01mpa: int,
        timestamp: Optional[float],
        raw_value: Optional[int] = None,
    ) -> None:
        data = self.channel_data[channel]
        data.voltage_001mv = voltage_001mv
        data.voltage_mv = voltage_mv
        data.strain_ue = strain_ue
        data.stress_01mpa = stress_01mpa
        data.stress_mpa = stress_mpa
        data.samples += 1
        data.last_timestamp = timestamp if timestamp is not None else time.time()
        payload = {
            'voltage': voltage_mv,
            'strain': strain_ue,
            'stress': stress_mpa,
            'samples': data.samples,
            'raw': raw_value,
        }
        self.on_data_updated(channel, payload)

    def _drain_pending_telemetry(self) -> set[int]:
        dirty_channels = set()
        while self.pending_telemetry_batches:
            telemetry_frames, timestamp = self.pending_telemetry_batches.popleft()
            raw_frames = [
                telemetry for telemetry in telemetry_frames
                if telemetry.raw_value is not None and telemetry.channel < NUM_CHANNELS
            ]
            if raw_frames:
                voltage, strain, stress = self._physical_values_from_raw(
                    [telemetry.raw_value for telemetry in raw_frames]
                )
                for telemetry, voltage_mv, strain_ue, stress_mpa in zip(
                    raw_frames, voltage, strain, stress
                ):
                    self._apply_telemetry_values(
                        telemetry.channel,
                        float(voltage_mv),
                        float(strain_ue),
                        float(stress_mpa),
                        int(round(float(voltage_mv) * 100.0)),
                        int(round(float(stress_mpa) * 10.0)),
                        timestamp,
                        telemetry.raw_value,
                    )
                    dirty_channels.add(telemetry.channel)
            for telemetry in telemetry_frames:
                channel = telemetry.channel
                if channel >= NUM_CHANNELS:
                    self.rx_bad_protocol_count += 1
                    logger.warning("Invalid channel: %s", channel)
                    continue
                if telemetry.raw_value is not None:
                    continue
                self._ingest_telemetry(telemetry, timestamp)
                dirty_channels.add(channel)
        return dirty_channels

    def on_data_updated(self, channel: int, data: dict):
        """Handle data update signal"""
        # Update waveform buffer
        self.waveform_metric_buffers["voltage"][channel].append(data['voltage'])
        self.waveform_metric_buffers["strain"][channel].append(data['strain'])
        self.waveform_metric_buffers["stress"][channel].append(data['stress'])
        self.plot_dirty[channel] = True
        self.pending_table_data[channel] = data

        stats = self.channel_stats[channel]
        voltage = data['voltage']
        stats['count'] += 1
        stats['sum'] += voltage
        stats['min'] = voltage if stats['min'] is None else min(stats['min'], voltage)
        stats['max'] = voltage if stats['max'] is None else max(stats['max'], voltage)
        # Write to CSV if logging
        if self.logging_enabled and self.csv_writer:
            timestamp = datetime.datetime.now().isoformat()
            row = [timestamp, channel,
                   data['voltage'], data['strain'],
                   data['stress'], data.get('raw', ''), self.telemetry_mode,
                   self.config_sequence, data['samples']]
            self.csv_pending_rows.append(row)

    def update_plots(self):
        """Update waveform plots (called by timer)"""
        with self.lock:
            drained_channels = self._drain_pending_telemetry()
            dirty_channels = {
                ch for ch in range(NUM_CHANNELS) if self.plot_dirty[ch]
            }
            dirty_channels.update(drained_channels)
            for plot_index, channel in enumerate(self.plot_channels):
                if channel in dirty_channels:
                    self._refresh_plot_data(plot_index)
                    if self.auto_scale_checkbox.isChecked():
                        self._update_plot_range(plot_index)
            for channel in dirty_channels:
                self.plot_dirty[channel] = False
                self._update_data_table_row(channel)
                self._update_statistics_label(channel)
            if self.csv_writer and self.csv_pending_rows:
                self.csv_writer.writerows(self.csv_pending_rows)
                self.csv_pending_rows.clear()

    def _update_data_table_row(self, channel: int):
        if channel >= self.data_table.rowCount():
            return

        data = self.pending_table_data[channel]
        if data is None:
            return
        self.data_table.item(channel, 1).setText(f"{data['voltage']:.3f}")
        self.data_table.item(channel, 2).setText(f"{data['strain']:.2f}")
        self.data_table.item(channel, 3).setText(f"{data['stress']:.4f}")
        raw = data.get("raw")
        self.data_table.item(channel, 4).setText("-" if raw is None else str(raw))
        self.data_table.item(channel, 5).setText(
            str(self.zero_offsets[channel]) if self.telemetry_mode == TELEMETRY_MODE_RAW else "-"
        )
        self.data_table.item(channel, 6).setText(str(data['samples']))

    def _update_plot_range(self, plot_index: int):
        channel = self.plot_channels[plot_index]
        series = [
            self.waveform_metric_buffers[metric][channel].to_array(PLOT_VISIBLE_SAMPLES)
            for metric in self._selected_plot_metrics(plot_index)
        ]
        series = [values for values in series if values.size]
        if not series:
            return

        sample_count = max(
            len(self.waveform_metric_buffers[metric][channel])
            for metric in self._selected_plot_metrics(plot_index)
        )
        visible_count = min(sample_count, PLOT_VISIBLE_SAMPLES)
        first_sample = max(0, sample_count - visible_count)
        last_sample = max(first_sample + 1, sample_count - 1)
        visible_values = np.concatenate(series)

        minimum = float(np.min(visible_values))
        maximum = float(np.max(visible_values))
        center = (minimum + maximum) / 2.0
        y_range = max(maximum - minimum, PLOT_MIN_Y_RANGE_MV)
        y_padding = y_range * 0.1

        plot = self.plot_widgets[plot_index]
        plot.setXRange(first_sample, last_sample, padding=0.01)
        plot.setYRange(center - (y_range / 2.0) - y_padding,
                       center + (y_range / 2.0) + y_padding,
                       padding=0.0)

    def on_log_clicked(self):
        """Handle logging button click"""
        if self.logging_enabled:
            self.stop_logging()
        else:
            self.start_logging()

    def start_logging(self):
        """Start CSV logging"""
        filename, _ = QFileDialog.getSaveFileName(
            self, self._tr("save_log_file"),
            f"reducer_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            self._tr("csv_files")
        )

        if not filename:
            return

        try:
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp', 'channel',
                'voltage_mv', 'strain_ue', 'stress_mpa', 'raw_value',
                'telemetry_mode', 'config_sequence', 'samples'
            ])
            self.logging_enabled = True
            self.log_btn.setText(self._tr("stop_logging"))
            self._show_status("logging_to", filename=filename)
            logger.info(f"Started logging to {filename}")

        except Exception as e:
            QMessageBox.critical(
                self, self._tr("error"), self._tr("failed_start_logging", error=e)
            )
            logger.error(f"Failed to start logging: {e}")

    def stop_logging(self):
        """Stop CSV logging"""
        if self.csv_writer and self.csv_pending_rows:
            try:
                self.csv_writer.writerows(self.csv_pending_rows)
                self.csv_pending_rows.clear()
            except Exception:
                logger.exception("Failed to flush pending CSV rows")
        if self.csv_file:
            try:
                self.csv_file.close()
            except:
                pass
            self.csv_file = None
        self.csv_writer = None
        self.logging_enabled = False
        self.log_btn.setText(self._tr("start_logging"))
        self._show_status("logging_stopped")
        logger.info("Stopped logging")

    def closeEvent(self, event):
        """Handle window close event"""
        if self.logging_enabled:
            self.stop_logging()

        if self.is_connected:
            self.disconnect()

        event.accept()

    def _command_name(self, cmd_type: int) -> str:
        return COMMAND_NAMES.get(cmd_type, self._tr("command_unknown", cmd_type=cmd_type))

    def _handle_status_frame(self, status: StatusFrame) -> None:
        command_name = self.pending_commands.pop(status.sequence, self._command_name(status.cmd_type))
        self.pending_command_deadlines.pop(status.sequence, None)
        if status.status == CAN_STATUS_OK:
            self._show_status(
                "command_acknowledged", command=command_name, value=status.value
            )
            logger.info("%s acknowledged, value=%u", command_name, status.value)
            return

        if (
            status.cmd_type == CAN_CMD_SET_CHANNEL_MASK and
            self.last_sent_channel_mask == status.value
        ):
            self.last_sent_channel_mask = None

        reason = STATUS_NAMES.get(status.status, f"status 0x{status.status:02X}")
        self._show_status("command_rejected", command=command_name, reason=reason)
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
            command_name = self.pending_commands.pop(
                sequence, self._tr("command_sequence", sequence=sequence)
            )
            self._show_status("command_timeout", command=command_name)
            logger.warning("%s timed out waiting for ACK", command_name)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setWindowIcon(QIcon(str(APP_ICON_PATH)))

    # Set application info
    app.setApplicationName("Reducer Flexspline Monitor")
    app.setOrganizationName("ReducerProject")

    # Create and show window
    window = ReducerMonitorWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
