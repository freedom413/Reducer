"""
slcan_protocol.py - Standard SLCAN (Serial Line CAN) protocol implementation

This module implements the standard SLCAN protocol used by many CAN-to-USB adapters
(e.g., LCAN FD100, FD200, LAWICEL compatible adapters).

SLCAN Commands:
- S0-S8: Set CAN baudrate
  S0=10K, S1=20K, S2=50K, S3=100K, S4=125K, S5=250K, S6=500K, S7=800K, S8=1M
- s: Get current baudrate
- O: Open CAN channel
- C: Close CAN channel
- tIIIL: Transmit standard (11-bit) data frame, 8 bytes max
- rIIIL: Transmit standard remote frame
- TIIIL: Transmit extended (29-bit) data frame (not typically used)
- RIIIL: Transmit extended remote frame
- F: Get hardware status flags
- Z: Enable/Disable timestamps (Z0=off, Z1=on)

When receiving CAN frames, the adapter sends:
- tIIILDDD... for standard data frames (L = DLC, D = data bytes)
- rIIIL for standard remote frames
"""

import serial
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, Callable, List
from enum import Enum

logger = logging.getLogger(__name__)


# CAN ID definitions (must match embedded firmware)
CAN_ID_TX_DATA = 0x101  # Data from device to host
CAN_ID_RX_CONFIG = 0x100  # Config commands from host to device


# SLCAN baudrate codes
class Baudrate(Enum):
    BAUD_10K = ('S0', 10000)
    BAUD_20K = ('S1', 20000)
    BAUD_50K = ('S2', 50000)
    BAUD_100K = ('S3', 100000)
    BAUD_125K = ('S4', 125000)
    BAUD_250K = ('S5', 250000)
    BAUD_500K = ('S6', 500000)
    BAUD_800K = ('S7', 800000)
    BAUD_1M = ('S8', 1000000)

    def __init__(self, cmd: str, bps: int):
        self.cmd = cmd
        self.bps = bps


@dataclass
class CANFrame:
    """Represents a CAN frame (received or to be transmitted)"""
    id: int          # CAN ID (11-bit for standard, 29-bit for extended)
    data: bytes      # Data bytes (0-8 bytes)
    is_extended: bool = False
    is_remote: bool = False
    timestamp: Optional[float] = None


class SLCANProtocol:
    """SLCAN protocol handler with threading support for receiving"""

    def __init__(self, port: str = None, baudrate: Baudrate = Baudrate.BAUD_500K):
        self.serial: Optional[serial.Serial] = None
        self.port = port
        self.baudrate = baudrate
        self.is_open = False
        self.timestamps_enabled = False

        self._rx_thread: Optional[threading.Thread] = None
        self._rx_running = False
        self._rx_callbacks: List[Callable[[CANFrame], None]] = []

    def connect(self, port: str, baudrate: Baudrate = Baudrate.BAUD_500K) -> bool:
        """Connect to SLCAN device"""
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=115200,  # SLCAN always uses 115200
                timeout=1.0,
                write_timeout=1.0
            )
            self.port = port
            self.baudrate = baudrate

            # Set CAN baudrate
            if not self._set_baudrate(baudrate):
                logger.error("Failed to set baudrate")
                self.disconnect()
                return False

            logger.info(f"Connected to {port} at {baudrate.bps} bps")
            return True

        except serial.SerialException as e:
            logger.error(f"Failed to connect to {port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from SLCAN device"""
        self.close()
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
            self.serial = None
        self.port = None
        self.is_open = False
        logger.info("Disconnected")

    def open(self) -> bool:
        """Open CAN channel (send 'O' command)"""
        if not self.serial or not self.serial.is_open:
            logger.error("Serial port not open")
            return False

        try:
            self.serial.write(b'O\r')
            response = self.serial.read(1)
            if response == b'\r':
                self.is_open = True
                self._start_rx_thread()
                logger.info("CAN channel opened")
                return True
            else:
                logger.error(f"Open failed, got: {response}")
                return False
        except serial.SerialException as e:
            logger.error(f"Open failed: {e}")
            return False

    def close(self) -> bool:
        """Close CAN channel (send 'C' command)"""
        self._stop_rx_thread()

        if not self.serial or not self.serial.is_open:
            return True

        try:
            self.serial.write(b'C\r')
            response = self.serial.read(1)
            if response == b'\r':
                self.is_open = False
                logger.info("CAN channel closed")
                return True
            else:
                logger.error(f"Close failed, got: {response}")
                return False
        except serial.SerialException as e:
            logger.error(f"Close failed: {e}")
            return False

    def _set_baudrate(self, baudrate: Baudrate) -> bool:
        """Set CAN baudrate"""
        try:
            cmd = (baudrate.cmd + '\r').encode('ascii')
            self.serial.write(cmd)
            response = self.serial.read(1)
            if response == b'\r':
                logger.info(f"Baudrate set to {baudrate.bps}")
                return True
            else:
                logger.error(f"Baudrate set failed, got: {response}")
                return False
        except serial.SerialException as e:
            logger.error(f"Baudrate set failed: {e}")
            return False

    def send_frame(self, frame: CANFrame) -> bool:
        """Send a CAN frame

        Args:
            frame: CANFrame to send

        Returns:
            True if sent successfully
        """
        if not self.serial or not self.serial.is_open or not self.is_open:
            logger.error("CAN channel not open")
            return False

        try:
            # Build command
            if frame.is_extended:
                cmd_char = 'T' if not frame.is_remote else 'R'
                id_str = f"{frame.id:08X}"  # 8 hex digits for extended
            else:
                cmd_char = 't' if not frame.is_remote else 'r'
                id_str = f"{frame.id:03X}"  # 3 hex digits for standard

            dlc = len(frame.data)
            if dlc > 8:
                logger.error(f"Data too long: {dlc} bytes")
                return False

            cmd = f"{cmd_char}{id_str}{dlc:X}".encode('ascii')

            if not frame.is_remote:
                cmd += frame.data

            cmd += b'\r'

            self.serial.write(cmd)
            response = self.serial.read(1)

            if response == b'\r':
                return True
            else:
                logger.warning(f"Send got: {response}")
                return False

        except serial.SerialException as e:
            logger.error(f"Send failed: {e}")
            return False

    def send_standard_data(self, can_id: int, data: bytes) -> bool:
        """Convenience method to send standard (11-bit) data frame"""
        frame = CANFrame(id=can_id, data=data, is_extended=False, is_remote=False)
        return self.send_frame(frame)

    def register_rx_callback(self, callback: Callable[[CANFrame], None]):
        """Register a callback for received CAN frames"""
        self._rx_callbacks.append(callback)

    def unregister_rx_callback(self, callback: Callable[[CANFrame], None]):
        """Unregister a callback"""
        if callback in self._rx_callbacks:
            self._rx_callbacks.remove(callback)

    def _start_rx_thread(self):
        """Start the receive thread"""
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self._rx_thread.start()

    def _stop_rx_thread(self):
        """Stop the receive thread"""
        self._rx_running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def _rx_worker(self):
        """Worker thread for receiving data from serial port"""
        buffer = bytearray()

        while self._rx_running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)

                    # Process complete frames in buffer
                    while b'\r' in buffer:
                        line_end = buffer.index(b'\r')
                        line = bytes(buffer[:line_end])
                        del buffer[:line_end + 1]

                        if line:
                            frame = self._parse_line(line)
                            if frame:
                                for callback in self._rx_callbacks:
                                    try:
                                        callback(frame)
                                    except Exception as e:
                                        logger.error(f"Callback error: {e}")
                else:
                    time.sleep(0.001)  # Small sleep to avoid busy waiting

            except Exception as e:
                logger.error(f"RX worker error: {e}")
                time.sleep(0.1)

    def _parse_line(self, line: bytes) -> Optional[CANFrame]:
        """Parse a received SLCAN line into a CANFrame

        SLCAN received frame formats:
        - tIIILDDDD... : Standard data frame (11-bit ID)
        - rIIIL : Standard remote frame
        - TIIILDDDD... : Extended data frame (29-bit ID)
        - RIIIL : Extended remote frame
        - f0 : Status - no CAN controller
        - f1 : Status - CAN controller error active
        - f2 : Status - CAN controller error passive
        - f3 : Status - CAN bus off
        """
        try:
            if not line:
                return None

            cmd = chr(line[0])

            if cmd == 't':
                # Standard data frame: tIIILDDDD...
                if len(line) < 5:
                    return None
                can_id = int(line[1:4], 16)
                dlc = int(chr(line[4]), 16)
                if len(line) < 5 + dlc:
                    return None
                data = bytes(line[5:5 + dlc])
                return CANFrame(id=can_id, data=data, is_extended=False,
                               is_remote=False, timestamp=time.time())

            elif cmd == 'r':
                # Standard remote frame: rIIIL
                if len(line) < 5:
                    return None
                can_id = int(line[1:4], 16)
                dlc = int(chr(line[4]), 16)
                return CANFrame(id=can_id, data=bytes(dlc), is_extended=False,
                               is_remote=True, timestamp=time.time())

            elif cmd == 'T':
                # Extended data frame: TIIILDDDD...
                if len(line) < 9:
                    return None
                can_id = int(line[1:9], 16)
                dlc = int(chr(line[9]), 16)
                if len(line) < 10 + dlc:
                    return None
                data = bytes(line[10:10 + dlc])
                return CANFrame(id=can_id, data=data, is_extended=True,
                               is_remote=False, timestamp=time.time())

            elif cmd == 'R':
                # Extended remote frame: RIIIL
                if len(line) < 10:
                    return None
                can_id = int(line[1:9], 16)
                dlc = int(chr(line[9]), 16)
                return CANFrame(id=can_id, data=bytes(dlc), is_extended=True,
                               is_remote=True, timestamp=time.time())

            elif cmd == 'f':
                # Status frame - log and ignore
                status_code = line[1:] if len(line) > 1 else b''
                logger.debug(f"CAN status: {status_code}")
                return None

            else:
                logger.warning(f"Unknown SLCAN command: {cmd}")
                return None

        except (ValueError, IndexError) as e:
            logger.warning(f"Failed to parse SLCAN line '{line}': {e}")
            return None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


# Utility function for quick testing
def list_serial_ports():
    """List all available serial ports"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in ports]


if __name__ == '__main__':
    # Quick test - list ports
    print("Available serial ports:")
    for device, desc in list_serial_ports():
        print(f"  {device}: {desc}")
