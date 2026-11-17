"""
Thread-safe Bluetooth Low Energy (BLE) Communication Workers for Robot Arm Digital Twin.
Runs bleak asyncio loops inside dedicated QThreads so the PyQt GUI stays smooth.
Compatible with Arduino Uno R4 WiFi firmware (ArduinoBLE GATT service).
"""
import asyncio
import time
from typing import Dict, List, Optional

from src.config import (
    BLE_CHARACTERISTIC_UUID,
    BLE_DEVICE_NAME,
    BLE_SERVICE_UUID,
    BLE_THROTTLE_MS,
    JOINT_LIMITS,
)
from src.ui.qt_compat import QMutex, QMutexLocker, QThread, pyqtSignal

try:
    from bleak import BleakClient, BleakError, BleakScanner

    BLE_AVAILABLE = True
except ImportError:  # pragma: no cover - optional runtime dependency fallback
    BleakClient = None  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]
    BleakError = Exception  # type: ignore[assignment,misc]
    BLE_AVAILABLE = False


class BleScanWorker(QThread):
    """Background worker that scans nearby BLE devices without blocking the GUI."""

    scan_finished = pyqtSignal(list)  # List of device metadata dicts
    scan_error = pyqtSignal(str)

    def __init__(self, timeout: float = 3.0, parent=None):
        super().__init__(parent)
        self.timeout = float(timeout)

    def run(self) -> None:
        if not BLE_AVAILABLE:
            self.scan_error.emit("Bluetooth-kirjasto 'bleak' ei ole asennettu (pip install bleak).")
            return

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            devices = loop.run_until_complete(self._discover_devices())
            self.scan_finished.emit(devices)
        except Exception as e:
            self.scan_error.emit(f"Bluetooth-haku epäonnistui (onko Bluetooth päällä?): {e}")
        finally:
            loop.close()

    async def _discover_devices(self) -> List[Dict]:
        """Discovers BLE peripherals and flags Arduino-Arm candidates first."""
        discovered = await BleakScanner.discover(timeout=self.timeout, return_adv=True)
        target_svc = BLE_SERVICE_UUID.lower()
        target_name = BLE_DEVICE_NAME.lower()

        results: List[Dict] = []
        for dev, adv in discovered.values():
            name = (adv.local_name or dev.name or "").strip()
            service_uuids = [str(u).lower() for u in (adv.service_uuids or [])]
            is_ard = (
                target_svc in service_uuids
                or target_name in name.lower()
                or "arduino" in name.lower()
            )
            if not name and not is_ard:
                continue

            results.append(
                {
                    "name": name or BLE_DEVICE_NAME,
                    "address": dev.address,
                    "rssi": adv.rssi if adv.rssi is not None else -999,
                    "is_arduino": is_ard,
                }
            )

        results.sort(key=lambda d: (not d["is_arduino"], -d["rssi"]))
        return results


class BleWorker(QThread):
    """
    Background worker thread handling BLE GATT connection, 6-byte servo streaming,
    and disconnect/notification events without freezing the GUI.
    """

    connected = pyqtSignal(str)        # Display label (e.g. "BLE: Arduino-Arm")
    disconnected = pyqtSignal(str)     # Reason message
    data_received = pyqtSignal(str)    # Notification payload from Arduino
    packet_sent = pyqtSignal(str)      # Sent packet representation
    error_occurred = pyqtSignal(str)   # Error message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_running: bool = False
        self._target_address: Optional[str] = None
        self._target_name: str = BLE_DEVICE_NAME

        self._mutex = QMutex()
        self._pending_angles: Optional[List[int]] = None
        self._last_sent_angles: Optional[List[int]] = None
        self._last_send_time: float = 0.0

    @staticmethod
    def clamp_angles(angles: List[int]) -> List[int]:
        """Clamps all 6 angles strictly within JOINT_LIMITS (Servo 6: 20°..180°)."""
        clamped = []
        for idx, raw in enumerate(angles[:6]):
            if idx < len(JOINT_LIMITS):
                min_v, max_v = JOINT_LIMITS[idx]
            else:
                min_v, max_v = 0, 180
            clamped.append(max(min_v, min(max_v, int(round(raw)))))
        while len(clamped) < 6:
            min_v, max_v = JOINT_LIMITS[len(clamped)]
            clamped.append(max(min_v, min(max_v, 90)))
        return clamped

    @classmethod
    def build_angle_packet(cls, angles: List[int]) -> bytes:
        """Packs 6 hardware-clamped joint angles into the 6-byte Arduino BLE frame."""
        return bytes(cls.clamp_angles(angles))

    def emergency_stop(self) -> None:
        """Immediately halts pending BLE angle transmission and clears queue."""
        with QMutexLocker(self._mutex):
            self._pending_angles = None

    def connect_ble(self, address: str, device_name: str = BLE_DEVICE_NAME) -> None:
        """Initiates asynchronous BLE connection to target peripheral address."""
        with QMutexLocker(self._mutex):
            self._target_address = address
            self._target_name = device_name or BLE_DEVICE_NAME
            self._last_sent_angles = None
            self._pending_angles = None
        if not self.isRunning():
            self._is_running = True
            self.start()

    def disconnect_ble(self, reason: str = "Käyttäjä katkaisi Bluetooth-yhteyden") -> None:
        """Stops the BLE streaming loop and waits cleanly for disconnection."""
        with QMutexLocker(self._mutex):
            self._is_running = False
            self._pending_angles = None
        if self.isRunning():
            self.wait(3000)

    def send_angles(self, angles: List[int]) -> None:
        """Thread-safe queue of target servo angles clamped to safe limits."""
        safe_angles = self.clamp_angles(angles)
        with QMutexLocker(self._mutex):
            self._pending_angles = safe_angles

    def run(self) -> None:
        if not BLE_AVAILABLE:
            self.error_occurred.emit("Bluetooth-kirjasto 'bleak' ei ole asennettu.")
            self.disconnected.emit("Bluetooth-kirjasto puuttuu")
            return

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        reason = "Bluetooth-yhteys katkesi"
        try:
            reason = loop.run_until_complete(self._ble_session())
        except Exception as e:
            self.error_occurred.emit(f"BLE-yhteysvirhe: {e}")
            reason = "BLE-yhteysvirhe"
        finally:
            loop.close()
            self._last_sent_angles = None
            self.disconnected.emit(reason)

    def _resolve_target_characteristic(self, client):
        """Finds matching Arduino control GATT characteristic (with writable fallback)."""
        target_svc = BLE_SERVICE_UUID.lower()
        target_chr = BLE_CHARACTERISTIC_UUID.lower()

        for service in client.services:
            if str(service.uuid).lower() == target_svc:
                for char in service.characteristics:
                    if str(char.uuid).lower() == target_chr:
                        return char

        for service in client.services:
            for char in service.characteristics:
                if str(char.uuid).lower() == target_chr:
                    return char

        for service in client.services:
            for char in service.characteristics:
                props = set(char.properties)
                if "write" in props or "write-without-response" in props:
                    return char

        return None

    async def _ble_session(self) -> str:
        """Runs active BLE GATT connection and throttled angle streaming loop."""
        with QMutexLocker(self._mutex):
            address = self._target_address
            dev_name = self._target_name

        if not address:
            self.error_occurred.emit("BLE-laitteen osoite puuttuu.")
            return "Osoite puuttuu"

        disconnect_event = asyncio.Event()
        throttle_sec = BLE_THROTTLE_MS / 1000.0

        def _on_gatt_disconnected(_client) -> None:
            disconnect_event.set()

        try:
            client = BleakClient(
                address,
                disconnected_callback=_on_gatt_disconnected,
                timeout=12.0,
            )
            async with client:
                target_char = self._resolve_target_characteristic(client)
                if target_char is None:
                    self.error_occurred.emit(
                        "Ohjauskarakteristiikkaa ei löytynyt Bluetooth-laitteesta."
                    )
                    return "Ohjauspalvelua ei löytynyt"

                props = set(target_char.properties)
                use_response = "write-without-response" not in props

                if "notify" in props:
                    try:
                        def _on_notify(_sender, data: bytearray) -> None:
                            if len(data) == 6:
                                self.data_received.emit(f"BLE kuittaus: {list(data)}")

                        await client.start_notify(target_char, _on_notify)
                    except Exception:
                        pass

                self.connected.emit(f"BLE: {dev_name}")

                while (
                    self._is_running
                    and client.is_connected
                    and not disconnect_event.is_set()
                ):
                    now = time.time()
                    angles_to_send: Optional[List[int]] = None

                    with QMutexLocker(self._mutex):
                        if self._pending_angles is not None:
                            if (
                                self._pending_angles != self._last_sent_angles
                                and (now - self._last_send_time) >= throttle_sec
                            ):
                                angles_to_send = list(self._pending_angles)
                                self._last_sent_angles = list(angles_to_send)
                                self._last_send_time = now

                    if angles_to_send is not None:
                        payload = bytes(angles_to_send)
                        await client.write_gatt_char(
                            target_char, payload, response=use_response
                        )
                        self.packet_sent.emit(
                            f"BLE [{','.join(str(a) for a in angles_to_send)}]"
                        )

                    await asyncio.sleep(0.01)

                if not self._is_running:
                    return "Käyttäjä katkaisi Bluetooth-yhteyden"
                return "Bluetooth-yhteys katkesi"

        except (BleakError, OSError, asyncio.TimeoutError) as e:
            self.error_occurred.emit(f"Bluetooth-virhe ({dev_name}): {e}")
            return "Bluetooth-yhteys epäonnistui"

