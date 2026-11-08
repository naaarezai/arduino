"""
Thread-safe Serial Communication Worker for Robot Arm Digital Twin.
Runs pySerial inside a dedicated PyQt6 QThread so the UI never freezes.
Handles ASCII protocol: <J1,J2,J3,J4,J5,J6>\\n, throttling, auto-disconnect, and exception safety.
"""
import time
from typing import List, Optional
import serial
from serial.tools import list_ports
from src.ui.qt_compat import QMutex, QMutexLocker, QThread, pyqtSignal, pyqtSlot

from src.config import DEFAULT_BAUDRATE, DEFAULT_TIMEOUT, PACKET_FOOTER, PACKET_HEADER, SERIAL_THROTTLE_MS


class SerialWorker(QThread):
    """
    Background worker thread handling serial connection, message transmission,
    and receiving responses without blocking the PyQt6 GUI.
    """
    # Signals
    connected = pyqtSignal(str)        # port name
    disconnected = pyqtSignal(str)     # reason
    data_received = pyqtSignal(str)    # message from Arduino
    packet_sent = pyqtSignal(str)      # sent ASCII message
    error_occurred = pyqtSignal(str)   # error message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial: Optional[serial.Serial] = None
        self._is_running: bool = False
        self._target_port: Optional[str] = None
        self._baudrate: int = DEFAULT_BAUDRATE

        # Angle state buffer protected by mutex
        self._mutex = QMutex()
        self._pending_angles: Optional[List[int]] = None
        self._last_sent_angles: Optional[List[int]] = None
        self._last_send_time: float = 0.0

    @staticmethod
    def list_available_ports() -> List[str]:
        """Lists available serial COM ports on the system."""
        ports = list_ports.comports()
        return [port.device for port in ports]

    @staticmethod
    def list_port_details() -> List[dict]:
        """Returns list of available port metadata with Arduino detection."""
        res = []
        for p in list_ports.comports():
            desc = p.description or ""
            mfg = p.manufacturer or ""
            hwid = p.hwid or ""
            combined = f"{desc} {mfg} {hwid}".lower()
            is_ard = any(x in combined for x in ["arduino", "uno", "2341", "ch340", "cp210", "ftdi"])
            res.append({
                "device": p.device,
                "description": desc,
                "manufacturer": mfg,
                "is_arduino": is_ard
            })
        return res

    def emergency_stop(self) -> None:
        """Immediately halts pending serial transmission and flushes queue."""
        with QMutexLocker(self._mutex):
            self._pending_angles = None

    def connect_serial(self, port: str, baudrate: int = DEFAULT_BAUDRATE) -> None:
        """Requests connection to a specific serial port."""
        with QMutexLocker(self._mutex):
            self._target_port = port
            self._baudrate = baudrate
        if not self.isRunning():
            self._is_running = True
            self.start()

    def disconnect_serial(self, reason: str = "Käyttäjä katkaisi yhteyden") -> None:
        """Requests disconnection from the serial port."""
        with QMutexLocker(self._mutex):
            self._is_running = False
        self.wait(1000)
        self._close_connection(reason)

    def send_angles(self, angles: List[int]) -> None:
        """
        Thread-safe queue of new target angles.
        Will be sent at rate-limited intervals.
        """
        with QMutexLocker(self._mutex):
            self._pending_angles = list(angles)

    def run(self) -> None:
        """Main thread loop for serial I/O."""
        # 1. Attempt connection
        with QMutexLocker(self._mutex):
            port = self._target_port
            baud = self._baudrate

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=DEFAULT_TIMEOUT,
                write_timeout=DEFAULT_TIMEOUT
            )
            # Give Arduino a moment to reset on DTR
            time.sleep(1.5)
            # Flush startup junk
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self.connected.emit(port)
        except (serial.SerialException, OSError) as e:
            self.error_occurred.emit(f"Yhteysvirhe porttiin {port}: {e}")
            self._close_connection("Yhteys epäonnistui")
            return

        # 2. Main loop while connected
        throttle_sec = SERIAL_THROTTLE_MS / 1000.0

        while self._is_running and self._serial and self._serial.is_open:
            try:
                now = time.time()

                # Check if we should send pending angles
                angles_to_send = None
                with QMutexLocker(self._mutex):
                    if self._pending_angles is not None:
                        # Only send if changed and throttle window has passed
                        if (self._pending_angles != self._last_sent_angles and 
                            (now - self._last_send_time) >= throttle_sec):
                            angles_to_send = list(self._pending_angles)
                            self._last_sent_angles = list(angles_to_send)
                            self._last_send_time = now

                if angles_to_send is not None:
                    # Format: <J1,J2,J3,J4,J5,J6>\n
                    payload = f"{PACKET_HEADER}{','.join(str(a) for a in angles_to_send)}{PACKET_FOOTER}"
                    self._serial.write(payload.encode("ascii"))
                    self._serial.flush()
                    self.packet_sent.emit(payload.strip())

                # Read incoming response (non-blocking)
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode("ascii", errors="replace").strip()
                    if line:
                        self.data_received.emit(line)

                # Small sleep to prevent CPU spinning
                time.sleep(0.01)

            except (serial.SerialException, OSError) as e:
                # Cable was unplugged or port failed
                self.error_occurred.emit(f"Sarjaliikenne katkesi (USB irrotettu?): {e}")
                self._close_connection("USB-kaapeli irrotettu tai yhteys katkesi")
                break

    def _close_connection(self, reason: str) -> None:
        """Safely closes serial port and emits disconnected signal."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._last_sent_angles = None
        self.disconnected.emit(reason)
