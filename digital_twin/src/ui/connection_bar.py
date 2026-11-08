"""
Connection toolbar for COM port selection and Online/Offline state management.
"""
from typing import Callable, List
from src.config import DEFAULT_BAUDRATE
from src.communication.serial_worker import SerialWorker
from src.ui.qt_compat import (
    QWidget, QHBoxLayout, QLabel, QPushButton, QComboBox, QFrame
)


class ConnectionBar(QFrame):
    """
    Top/control bar showing COM port dropdown, Refresh, Connect/Disconnect,
    and a clear OFFLINE/ONLINE status pill.
    """
    def __init__(self, 
                 on_connect_callback: Callable[[str, int], None],
                 on_disconnect_callback: Callable[[], None],
                 parent=None):
        super().__init__(parent)
        self.on_connect = on_connect_callback
        self.on_disconnect = on_disconnect_callback
        self._is_connected: bool = False

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

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(12)

        # Title / Mode Label
        title_lbl = QLabel("🔌 Sarjaliikenne:")
        title_lbl.setStyleSheet("font-weight: bold; color: #eceff1;")
        layout.addWidget(title_lbl)

        # COM port dropdown
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(110)
        self.port_combo.setStyleSheet("""
            QComboBox {
                background: #282c34;
                color: #ffffff;
                border: 1px solid #3e4451;
                border-radius: 4px;
                padding: 4px 8px;
            }
            QComboBox QAbstractItemView {
                background: #282c34;
                color: #ffffff;
                selection-background-color: #00bcd4;
            }
        """)
        layout.addWidget(self.port_combo)

        # Refresh button
        self.btn_refresh = QPushButton("🔄 Päivitä")
        self.btn_refresh.setStyleSheet("""
            QPushButton {
                background: #3e4451;
                color: #eceff1;
                border: none;
                border-radius: 4px;
                padding: 5px 10px;
            }
            QPushButton:hover {
                background: #4b5263;
            }
        """)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(self.btn_refresh)

        # Connect / Disconnect button
        self.btn_connect = QPushButton("Yhdistä (Online)")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background: #00897b;
                color: #ffffff;
                font-weight: bold;
                border: none;
                border-radius: 4px;
                padding: 6px 16px;
            }
            QPushButton:hover {
                background: #00bfa5;
            }
        """)
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        layout.addStretch()

        # Status badge (OFFLINE / ONLINE)
        self.status_badge = QLabel("OFFLINE SIMULAATIO")
        self.set_offline_ui()
        layout.addWidget(self.status_badge)

    def refresh_ports(self) -> None:
        """Refreshes available COM ports in system."""
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = SerialWorker.list_available_ports()
        if ports:
            self.port_combo.addItems(ports)
            if current in ports:
                self.port_combo.setCurrentText(current)
        else:
            self.port_combo.addItem("Ei portteja")

    def _toggle_connection(self) -> None:
        if not self._is_connected:
            port = self.port_combo.currentText()
            if not port or port == "Ei portteja":
                return
            self.on_connect(port, DEFAULT_BAUDRATE)
        else:
            self.on_disconnect()

    def set_online_ui(self, port_name: str) -> None:
        """Updates UI elements when connected to Arduino."""
        self._is_connected = True
        self.btn_connect.setText("Katkaise yhteys")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background: #e53935;
                color: #ffffff;
                font-weight: bold;
                border: none;
                border-radius: 4px;
                padding: 6px 16px;
            }
            QPushButton:hover {
                background: #ef5350;
            }
        """)
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
        self._is_connected = False
        self.btn_connect.setText("Yhdistä (Online)")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background: #00897b;
                color: #ffffff;
                font-weight: bold;
                border: none;
                border-radius: 4px;
                padding: 6px 16px;
            }
            QPushButton:hover {
                background: #00bfa5;
            }
        """)
        self.port_combo.setEnabled(True)
        self.btn_refresh.setEnabled(True)

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

