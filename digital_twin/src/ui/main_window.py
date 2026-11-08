"""
Main Application Window for 6-DOF Robot Arm Digital Twin / OLP System.
Assembles the 3D PyVista viewport on the left and control/sequencer panels on the right.
"""
from typing import List
from src.core.kinematics import RobotKinematics
from src.communication.serial_worker import SerialWorker
from src.simulation.arm_visualizer_3d import ArmVisualizer3D
from src.ui.connection_bar import ConnectionBar
from src.ui.slider_panel import SliderPanel
from src.ui.sequencer_panel import SequencerPanel
from src.ui.qt_compat import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSplitter, QStatusBar, QMessageBox, Qt, QFrame, QPushButton
)


class MainWindow(QMainWindow):
    """
    Primary desktop window hosting 3D Digital Twin simulation,
    manual joint controls, OLP waypoint sequencer, and serial communication.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🦾 6-DOF Robottikäsivarsi — Digital Twin & OLP System")
        self.resize(1300, 850)
        self.setMinimumSize(950, 650)

        # Core logic
        self.kinematics = RobotKinematics()
        self.serial_worker = SerialWorker(self)

        # Connection state
        self.is_online: bool = False

        self._init_ui()
        self._connect_signals()

        # Initial pose computation
        self._on_angles_changed(
            self.slider_panel.get_angles(),
            self.slider_panel.get_physical_angles()
        )

    def _init_ui(self) -> None:
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 8)
        main_layout.setSpacing(8)

        # 1. Top Connection Bar
        self.conn_bar = ConnectionBar(
            on_connect_callback=self._handle_connect,
            on_disconnect_callback=self._handle_disconnect,
            parent=self
        )
        main_layout.addWidget(self.conn_bar)

        # 2. Main Horizontal Splitter (3D View on left, Controls on right)
        splitter = QSplitter(Qt.Horizontal)

        # Left Container: 3D Viewport + HUD info + Camera buttons
        left_container = QFrame()
        left_container.setStyleSheet("background: #181a1f; border-radius: 6px;")
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(4)

        # Camera toolbar
        cam_bar = QHBoxLayout()
        cam_lbl = QLabel("🎥 Kamera:")
        cam_lbl.setStyleSheet("color: #90a4ae; font-size: 11px;")
        cam_bar.addWidget(cam_lbl)

        btn_iso = QPushButton("Koti (3D)")
        btn_iso.clicked.connect(lambda: self.visualizer_3d.reset_camera())
        btn_top = QPushButton("Ylhäältä")
        btn_top.clicked.connect(lambda: self.visualizer_3d.view_top())
        btn_side = QPushButton("Sivulta")
        btn_side.clicked.connect(lambda: self.visualizer_3d.view_side())

        for btn in (btn_iso, btn_top, btn_side):
            btn.setStyleSheet("""
                QPushButton {
                    background: #282c34; color: #eceff1; border: 1px solid #3e4451;
                    border-radius: 3px; padding: 3px 8px; font-size: 11px;
                }
                QPushButton:hover { background: #3e4451; }
            """)
            cam_bar.addWidget(btn)

        cam_bar.addStretch()

        # HUD coordinate label
        self.hud_label = QLabel("X: 0.0 mm | Y: 0.0 mm | Z: 0.0 mm | Pitch: 0.0°")
        self.hud_label.setStyleSheet("""
            QLabel {
                background: #21252b; color: #00e5ff; font-family: monospace;
                font-weight: bold; padding: 4px 10px; border-radius: 4px; border: 1px solid #3e4451;
            }
        """)
        cam_bar.addWidget(self.hud_label)
        left_layout.addLayout(cam_bar)

        # 3D Viewport
        self.visualizer_3d = ArmVisualizer3D(self)
        left_layout.addWidget(self.visualizer_3d, 1)

        splitter.addWidget(left_container)

        # Right Container: Sliders + Sequencer
        right_container = QFrame()
        right_container.setStyleSheet("background: #1e2227; border-radius: 6px;")
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(10, 10, 10, 10)
        right_layout.setSpacing(10)

        # Sliders Card
        self.slider_panel = SliderPanel(
            on_angles_changed_callback=self._on_angles_changed,
            parent=self
        )
        right_layout.addWidget(self.slider_panel)

        # Sequencer Card
        self.sequencer_panel = SequencerPanel(
            get_current_angles_fn=self.slider_panel.get_angles,
            apply_angles_fn=self.slider_panel.set_angles,
            parent=self
        )
        right_layout.addWidget(self.sequencer_panel)

        right_layout.addStretch()
        splitter.addWidget(right_container)

        # Set initial splitter ratio (60% 3D view, 40% controls)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter, 1)

        # 3. Status Bar
        self.status_bar = QStatusBar(self)
        self.status_bar.setStyleSheet("background: #181a1f; color: #abb2bf; font-size: 11px;")
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Järjestelmä valmis. Offline-simulaatio aktiivinen.")

    def _connect_signals(self) -> None:
        self.serial_worker.connected.connect(self._on_serial_connected)
        self.serial_worker.disconnected.connect(self._on_serial_disconnected)
        self.serial_worker.data_received.connect(self._on_serial_data_received)
        self.serial_worker.packet_sent.connect(self._on_serial_packet_sent)
        self.serial_worker.error_occurred.connect(self._on_serial_error)

    def _on_angles_changed(self, sim_angles: List[int], physical_angles: List[int]) -> None:
        """
        Called whenever any joint angle changes (slider, spinbox, sequence playback).
        Updates 3D model, HUD coordinates, and sends to serial if Online.
        """
        # 1. Update 3D kinematics
        pose = self.kinematics.forward_kinematics([float(a) for a in sim_angles])
        self.visualizer_3d.update_pose(pose)

        # 2. Update HUD display
        self.hud_label.setText(
            f"TCP: X: {pose.tcp_x:6.1f} mm | Y: {pose.tcp_y:6.1f} mm | Z: {pose.tcp_z:6.1f} mm | Pitch: {pose.pitch_deg:5.1f}°"
        )

        # 3. If Online, send physical angles to Arduino via QThread
        if self.is_online:
            self.serial_worker.send_angles(physical_angles)

    def _handle_connect(self, port: str, baudrate: int) -> None:
        self.status_bar.showMessage(f"Yhdistetään porttiin {port} ({baudrate} baud)...")
        self.serial_worker.connect_serial(port, baudrate)

    def _handle_disconnect(self) -> None:
        self.serial_worker.disconnect_serial("Käyttäjä sulki yhteyden")

    def _on_serial_connected(self, port: str) -> None:
        self.is_online = True
        self.conn_bar.set_online_ui(port)
        self.status_bar.showMessage(f"Yhteys muodostettu: {port}. Robotti synkronoitu!")
        # Send current physical angles immediately upon connect
        self.serial_worker.send_angles(self.slider_panel.get_physical_angles())

    def _on_serial_disconnected(self, reason: str) -> None:
        self.is_online = False
        self.conn_bar.set_offline_ui(reason)
        self.status_bar.showMessage(f"Offline-tila: {reason}")

    def _on_serial_packet_sent(self, packet: str) -> None:
        self.status_bar.showMessage(f"Lähetetty Arduinolle: {packet}", 1500)

    def _on_serial_data_received(self, line: str) -> None:
        self.status_bar.showMessage(f"Arduino vastasi: {line}", 1500)

    def _on_serial_error(self, err_msg: str) -> None:
        self.status_bar.showMessage(f"Virhe: {err_msg}", 4000)

    def closeEvent(self, event) -> None:
        """Ensures serial thread terminates gracefully when window closes."""
        self.serial_worker.disconnect_serial("Sovellus suljettiin")
        event.accept()

