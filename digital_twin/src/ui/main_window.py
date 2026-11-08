"""
Main Application Window for 6-DOF Robot Arm Digital Twin / OLP System.
Assembles the 3D PyVista viewport on the left and control/sequencer panels on the right.
"""
from typing import List, Optional
from src.core.kinematics import RobotKinematics
from src.communication.serial_worker import SerialWorker
from src.simulation.arm_visualizer_3d import ArmVisualizer3D
from src.ui.connection_bar import ConnectionBar
from src.ui.slider_panel import SliderPanel
from src.ui.xyz_controller_panel import XyzControllerPanel
from src.ui.sequencer_panel import SequencerPanel
from src.ui.qt_compat import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSplitter, QStatusBar, QMessageBox, Qt, QFrame, QPushButton,
    QTabWidget, QTimer
)


class MainWindow(QMainWindow):
    """
    Primary desktop window hosting 3D Digital Twin simulation,
    manual joint controls, Cartesian XYZ IK jogging, OLP waypoint sequencer,
    and Arduino serial communication.
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

        # Live robot trajectory streaming
        self.stream_timer = QTimer(self)
        self.stream_timer.setInterval(35)
        self.stream_timer.timeout.connect(self._on_stream_tick)
        self._stream_trajectory: List[List[int]] = []
        self._stream_index: int = 0

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

        # Right Container: Tabs (Joints / XYZ IK) + Sequencer
        right_container = QFrame()
        right_container.setStyleSheet("background: #1e2227; border-radius: 6px;")
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(10, 10, 10, 10)
        right_layout.setSpacing(10)

        # Tabs for Joint Control vs Cartesian XYZ IK Control
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #37474f; background: #21252b; border-radius: 5px;
            }
            QTabBar::tab {
                background: #263238; color: #b0bec5; padding: 8px 14px; font-weight: bold;
                border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #00bcd4; color: #000000;
            }
            QTabBar::tab:hover:!selected {
                background: #37474f; color: #eceff1;
            }
        """)

        # Tab 1: Joint Sliders
        self.slider_panel = SliderPanel(
            on_angles_changed_callback=self._on_angles_changed,
            parent=self
        )
        self.tabs.addTab(self.slider_panel, "🎮 Nivelet (J1..J6)")

        # Tab 2: Cartesian XYZ Jogging & Welding (IK)
        self.xyz_panel = XyzControllerPanel(
            kinematics=self.kinematics,
            get_current_angles_fn=self.slider_panel.get_angles,
            apply_angles_fn=self.slider_panel.set_angles,
            parent=self
        )
        self.tabs.addTab(self.xyz_panel, "📐 Avaruus XYZ & Hitsaus")

        right_layout.addWidget(self.tabs)

        # Sequencer Card (OLP)
        self.sequencer_panel = SequencerPanel(
            kinematics=self.kinematics,
            get_current_angles_fn=self.slider_panel.get_angles,
            apply_angles_fn=self.slider_panel.set_angles,
            on_trajectory_changed_fn=self._on_trajectory_changed,
            on_execute_robot_fn=self._on_execute_robot,
            parent=self
        )
        right_layout.addWidget(self.sequencer_panel)

        right_layout.addStretch()
        splitter.addWidget(right_container)

        # Set initial splitter ratio (58% 3D view, 42% controls)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter, 1)

        # 3. Status Bar
        self.status_bar = QStatusBar(self)
        self.status_bar.setStyleSheet("background: #181a1f; color: #abb2bf; font-size: 11px;")
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Järjestelmä valmis. Offline-simulaatio ja OLP aktiivinen.")

    def _connect_signals(self) -> None:
        self.serial_worker.connected.connect(self._on_serial_connected)
        self.serial_worker.disconnected.connect(self._on_serial_disconnected)
        self.serial_worker.data_received.connect(self._on_serial_data_received)
        self.serial_worker.packet_sent.connect(self._on_serial_packet_sent)
        self.serial_worker.error_occurred.connect(self._on_serial_error)

    def _on_angles_changed(self, sim_angles: List[int], physical_angles: List[int]) -> None:
        """
        Called whenever any joint angle changes (slider, spinbox, sequence playback, IK jog).
        Updates 3D model, HUD coordinates, and sends to serial if Online.
        """
        # 1. Update 3D kinematics
        pose = self.kinematics.forward_kinematics([float(a) for a in sim_angles])
        self.visualizer_3d.update_pose(pose, sim_angles)

        # 2. Update HUD display
        self.hud_label.setText(
            f"TCP: X: {pose.tcp_x:6.1f} mm | Y: {pose.tcp_y:6.1f} mm | Z: {pose.tcp_z:6.1f} mm | Pitch: {pose.pitch_deg:5.1f}°"
        )

        # 3. If Online, send physical angles to Arduino via QThread
        if self.is_online and not self.stream_timer.isActive():
            self.serial_worker.send_angles(physical_angles)

    def _on_trajectory_changed(self, keyframes) -> None:
        """Refreshes 3D trajectory line tube and waypoint spheres."""
        self.visualizer_3d.update_trajectory_path(keyframes, self.kinematics)

    def _on_execute_robot(self, trajectory: List[List[int]]) -> None:
        """Streams full interpolated path to Arduino if online."""
        if not self.is_online:
            QMessageBox.warning(
                self,
                "Robotti Offline",
                "Robotti ei ole yhdistetty sarjaporttiin!\n\n"
                "Yhdistä Arduino USB-kaapelilla yläpalkin kautta ('Yhdistä') ennen fyysistä ajoa.\n"
                "Voit sillä välin testata liikkeen '▶️ Simuloi ruudulla' -painikkeella."
            )
            return

        if not trajectory:
            return

        self._stream_trajectory = trajectory
        self._stream_index = 0
        self.status_bar.showMessage(f"Ajetaan liikesarjaa robotilla... ({len(trajectory)} kehystä)")
        self.stream_timer.start(35)

    def _on_stream_tick(self) -> None:
        """Timer callback streaming frames to Arduino."""
        if self._stream_index >= len(self._stream_trajectory):
            self.stream_timer.stop()
            self.status_bar.showMessage("✓ Liikesarja ajettu onnistuneesti fyysisellä robotilla!")
            return

        frame = self._stream_trajectory[self._stream_index]
        phys = self.slider_panel.to_physical_angles(frame)
        self.serial_worker.send_angles(phys)

        if self._stream_index % 10 == 0 or self._stream_index == len(self._stream_trajectory) - 1:
            self.status_bar.showMessage(f"Ajo käynnissä: {self._stream_index + 1} / {len(self._stream_trajectory)} kehystä...")

        self._stream_index += 1

    def _handle_connect(self, port: str, baudrate: int) -> None:
        self.status_bar.showMessage(f"Yhdistetään porttiin {port} ({baudrate} baud)...")
        self.serial_worker.connect_serial(port, baudrate)

    def _handle_disconnect(self) -> None:
        if self.stream_timer.isActive():
            self.stream_timer.stop()
        self.serial_worker.disconnect_serial("Käyttäjä sulki yhteyden")

    def _on_serial_connected(self, port: str) -> None:
        self.is_online = True
        self.conn_bar.set_online_ui(port)
        self.status_bar.showMessage(f"Yhteys muodostettu: {port}. Robotti synkronoitu!")
        self.serial_worker.send_angles(self.slider_panel.get_physical_angles())

    def _on_serial_disconnected(self, reason: str) -> None:
        self.is_online = False
        if self.stream_timer.isActive():
            self.stream_timer.stop()
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
        if self.stream_timer.isActive():
            self.stream_timer.stop()
        self.serial_worker.disconnect_serial("Sovellus suljettiin")
        event.accept()

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

