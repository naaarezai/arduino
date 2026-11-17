"""
Connection toolbar for USB Serial (COM) and Bluetooth Low Energy (BLE) selection
plus Online/Offline state management.
"""
from typing import Callable, Dict, List, Optional

from src.communication.ble_worker import BLE_AVAILABLE, BleScanWorker
from src.communication.serial_worker import SerialWorker
from src.config import DEFAULT_BAUDRATE
from src.ui.qt_compat import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)


class ConnectionBar(QFrame):
    """
    Control bar supporting both USB COM port and wireless Bluetooth (BLE) connections
    with Refresh/Scan, Connect/Disconnect, and OFFLINE/ONLINE status badge.
    """

    def __init__(
        self,
        on_connect_callback: Callable[[str, int], None],
        on_disconnect_callback: Callable[[], None],
        on_ble_connect_callback: Optional[Callable[[str, str], None]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.on_connect = on_connect_callback
        self.on_disconnect = on_disconnect_callback
        self.on_ble_connect = on_ble_connect_callback

        self._is_connected: bool = False
        self._is_connecting: bool = False
        self._is_scanning_ble: bool = False
        self._ble_devices: List[Dict] = []
        self._ble_scanned_once: bool = False
        self._scan_worker: Optional[BleScanWorker] = None

        self._init_ui()
        self.refresh_ports()

    def _init_ui(self) -> None:
        self.setStyleSheet("""
            QFrame {
                background: #1e2227;
                border-radius: 6px;
                padding: 4px;
            }
        """)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 6, 10, 6)
        main_layout.setSpacing(6)

        combo_style = """
            QComboBox {
                background: #282c34;
                color: #ffffff;
                border: 1px solid #3e4451;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 11px;
            }
            QComboBox:disabled {
                color: #7f848e;
                background: #21252b;
            }
            QComboBox QAbstractItemView {
                background: #282c34;
                color: #ffffff;
                selection-background-color: #00bcd4;
            }
        """

        # Row 1: Transport selector (USB vs BLE) + Online/Offline badge
        row_top = QHBoxLayout()
        row_top.setSpacing(8)

        title_lbl = QLabel("Yhteystapa:")
        title_lbl.setStyleSheet("font-weight: bold; color: #eceff1; font-size: 11px;")
        row_top.addWidget(title_lbl)

        self.mode_combo = QComboBox()
        self.mode_combo.setStyleSheet(combo_style)
        self.mode_combo.addItem("🔌 USB (Sarjaportti)", "USB")
        self.mode_combo.addItem("ᛒ Bluetooth (BLE)", "BLE")
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        row_top.addWidget(self.mode_combo)

        row_top.addStretch()

        self.status_badge = QLabel("● OFFLINE (Simulaatio)")
        row_top.addWidget(self.status_badge)
        main_layout.addLayout(row_top)

        # Row 2: Port/Device dropdown + Refresh/Scan + Connect button
        row_bottom = QHBoxLayout()
        row_bottom.setSpacing(8)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(140)
        self.port_combo.setStyleSheet(combo_style)
        row_bottom.addWidget(self.port_combo, 1)

        self.btn_refresh = QPushButton("🔄 Päivitä")
        self.btn_refresh.setStyleSheet("""
            QPushButton {
                background: #3e4451;
                color: #eceff1;
                border: none;
                border-radius: 4px;
                padding: 5px 10px;
                font-size: 11px;
                font-weight: 600;
            }
            QPushButton:hover { background: #4b5263; }
            QPushButton:disabled { background: #2c313a; color: #5c6370; }
        """)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        row_bottom.addWidget(self.btn_refresh)

        self.btn_connect = QPushButton("Yhdistä (Online)")
        self.btn_connect.clicked.connect(self._toggle_connection)
        row_bottom.addWidget(self.btn_connect)

        main_layout.addLayout(row_bottom)
        self.set_offline_ui()

    def current_transport(self) -> str:
        """Returns active transport mode ('USB' or 'BLE')."""
        return self.mode_combo.currentData() or "USB"

    def _on_mode_changed(self) -> None:
        """Switches UI between USB COM port discovery and wireless BLE scanning."""
        transport = self.current_transport()
        if transport == "USB":
            self.btn_refresh.setText("🔄 Päivitä")
            self._populate_usb_ports()
            return

        self.btn_refresh.setText("🔍 Etsi (BLE)")
        if not self._ble_scanned_once and BLE_AVAILABLE:
            self.refresh_ports()
        else:
            self._populate_ble_devices()

    def refresh_ports(self) -> None:
        """Refreshes USB COM ports or launches non-blocking BLE scan."""
        if self._is_connected or self._is_connecting:
            return

        if self.current_transport() == "USB":
            self._populate_usb_ports()
        else:
            self._start_ble_scan()

    def _populate_usb_ports(self) -> None:
        self.port_combo.clear()
        details = SerialWorker.list_port_details()
        selected_index = 0
        if details:
            for idx, p in enumerate(details):
                dev = p["device"]
                desc = p["description"]
                if p["is_arduino"]:
                    label = f"⭐ {dev} ({desc})"
                    selected_index = idx
                else:
                    label = f"🔌 {dev} ({desc})" if desc else dev
                self.port_combo.addItem(label, dev)
            self.port_combo.setCurrentIndex(selected_index)
        else:
            self.port_combo.addItem("Ei USB-portteja", "")

    def _populate_ble_devices(self) -> None:
        self.port_combo.clear()
        if not BLE_AVAILABLE:
            self.port_combo.addItem("Asenna 'bleak' (pip install bleak)", None)
            return

        if not self._ble_devices:
            msg = "Ei BLE-laitteita — paina 'Etsi (BLE)'" if self._ble_scanned_once else "Paina 'Etsi (BLE)' etsiäksesi..."
            self.port_combo.addItem(msg, None)
            return

        selected_idx = 0
        for idx, dev in enumerate(self._ble_devices):
            name = dev["name"]
            addr = dev["address"]
            rssi = dev["rssi"]
            prefix = "⭐" if dev.get("is_arduino") else "ᛒ"
            rssi_txt = f" ({rssi} dBm)" if rssi > -900 else ""
            label = f"{prefix} {name}{rssi_txt} [{addr}]"
            self.port_combo.addItem(label, dev)
            if dev.get("is_arduino") and selected_idx == 0:
                selected_idx = idx
        self.port_combo.setCurrentIndex(selected_idx)

    def _start_ble_scan(self) -> None:
        if not BLE_AVAILABLE:
            self._populate_ble_devices()
            return
        if self._is_scanning_ble:
            return

        self._is_scanning_ble = True
        self.port_combo.clear()
        self.port_combo.addItem("Etsitään Bluetooth-laitteita (~3s)...", None)
        self.btn_refresh.setEnabled(False)
        self.btn_refresh.setText("⏳ Etsitään...")
        self.btn_connect.setEnabled(False)

        self._scan_worker = BleScanWorker(timeout=3.0, parent=self)
        self._scan_worker.scan_finished.connect(self._on_ble_scan_finished)
        self._scan_worker.scan_error.connect(self._on_ble_scan_error)
        self._scan_worker.start()

    def _on_ble_scan_finished(self, devices: List[Dict]) -> None:
        self._is_scanning_ble = False
        self._ble_scanned_once = True
        self._ble_devices = devices

        if self.current_transport() == "BLE":
            self._populate_ble_devices()
            self.btn_refresh.setText("🔍 Etsi (BLE)")
        else:
            self.btn_refresh.setText("🔄 Päivitä")

        if not self._is_connected and not self._is_connecting:
            self.btn_refresh.setEnabled(True)
            self.btn_connect.setEnabled(True)

    def _on_ble_scan_error(self, err: str) -> None:
        self._is_scanning_ble = False
        self._ble_scanned_once = True
        if self.current_transport() == "BLE":
            self.port_combo.clear()
            self.port_combo.addItem(err, None)
            self.btn_refresh.setText("🔍 Etsi (BLE)")
        else:
            self.btn_refresh.setText("🔄 Päivitä")

        if not self._is_connected and not self._is_connecting:
            self.btn_refresh.setEnabled(True)
            self.btn_connect.setEnabled(True)

    def stop_scan(self) -> None:
        """Stops any active BLE background scanner cleanly before window exit."""
        if self._scan_worker and self._scan_worker.isRunning():
            self._scan_worker.wait(1500)

    def _toggle_connection(self) -> None:
        if self._is_connecting:
            return

        if self._is_connected:
            self.on_disconnect()
            return

        if self.current_transport() == "USB":
            port = self.port_combo.currentData()
            if not port:
                return
            self.set_connecting_ui(port)
            self.on_connect(port, DEFAULT_BAUDRATE)
        else:
            dev = self.port_combo.currentData()
            if not isinstance(dev, dict) or not dev.get("address"):
                return
            if self.on_ble_connect is not None:
                self.set_connecting_ui(dev.get("name", "Arduino-Arm"))
                self.on_ble_connect(dev["address"], dev.get("name", "Arduino-Arm"))

    def set_connecting_ui(self, target_name: str = "") -> None:
        """Shows transient connecting state while handshaking with USB or BLE."""
        self._is_connecting = True
        self.mode_combo.setEnabled(False)
        self.port_combo.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.btn_connect.setEnabled(False)
        self.btn_connect.setText("Yhdistetään...")

        label = f"● YHDISTETÄÄN ({target_name})..." if target_name else "● YHDISTETÄÄN..."
        self.status_badge.setText(label)
        self.status_badge.setStyleSheet("""
            QLabel {
                background: #3b2d11;
                color: #ffd600;
                font-weight: bold;
                padding: 5px 12px;
                border-radius: 12px;
                border: 1px solid #ffab00;
            }
        """)

    def set_online_ui(self, port_name: str) -> None:
        """Updates UI elements when connected to Arduino (USB or BLE)."""
        self._is_connecting = False
        self._is_connected = True
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("Katkaise yhteys")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background: #e53935;
                color: #ffffff;
                font-weight: bold;
                border: none;
                border-radius: 4px;
                padding: 6px 16px;
                font-size: 11px;
            }
            QPushButton:hover { background: #ef5350; }
        """)
        self.mode_combo.setEnabled(False)
        self.port_combo.setEnabled(False)
        self.btn_refresh.setEnabled(False)

        self.status_badge.setText(f"● ONLINE ({port_name})")
        self.status_badge.setStyleSheet("""
            QLabel {
                background: #1b5e20;
                color: #69f0ae;
                font-weight: bold;
                padding: 5px 12px;
                border-radius: 12px;
                border: 1px solid #4caf50;
            }
        """)

    def set_offline_ui(self, reason: str = "") -> None:
        """Updates UI elements when in offline simulation mode."""
        self._is_connecting = False
        self._is_connected = False
        self.btn_connect.setEnabled(not self._is_scanning_ble)
        self.btn_connect.setText("Yhdistä (Online)")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background: #00897b;
                color: #ffffff;
                font-weight: bold;
                border: none;
                border-radius: 4px;
                padding: 6px 16px;
                font-size: 11px;
            }
            QPushButton:hover { background: #00bfa5; }
            QPushButton:disabled { background: #263238; color: #5c6370; }
        """)
        self.mode_combo.setEnabled(True)
        self.port_combo.setEnabled(True)
        self.btn_refresh.setEnabled(not self._is_scanning_ble)

        self.status_badge.setText("● OFFLINE (Simulaatio)")
        self.status_badge.setStyleSheet("""
            QLabel {
                background: #263238;
                color: #90a4ae;
                font-weight: bold;
                padding: 5px 12px;
                border-radius: 12px;
                border: 1px solid #455a64;
            }
        """)
