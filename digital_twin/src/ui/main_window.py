"""
Main Application Window for 6-DOF Robot Arm Digital Twin / OLP System.
Implements Visual Components -style architecture:
1. Interactive 3D Jog Gizmo in the 3D Viewport
2. Structured Statement Tree & Statement Toolbar (MainRoutine)
3. Workspace Tabs / Ribbon (Home/Layout, Program OLP, Simulation)
4. Clean Compact Jogging Panel with +/- step buttons & XYZ jogging
5. 3D Trajectory rendering with tool orientation frames and collision check
6. Live Arduino physical robot execution streaming
"""
from typing import List, Optional
from src.core.kinematics import RobotKinematics
from src.core.program import RobotProgram, MotionStatement
from src.communication.serial_worker import SerialWorker
from src.simulation.arm_visualizer_3d import ArmVisualizer3D
from src.ui.connection_bar import ConnectionBar
from src.ui.statement_tree_panel import StatementTreePanel
from src.ui.compact_jog_panel import CompactJogPanel
from src.ui.simulation_control_panel import SimulationControlPanel
from src.ui.slider_panel import SliderPanel
from src.ui.qt_compat import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSplitter, QStatusBar, QMessageBox, Qt, QFrame, QPushButton,
    QTabWidget, QTimer
)


class MainWindow(QMainWindow):
    """
    Primary desktop window hosting 3D Digital Twin simulation,
    interactive 3D Gizmo manipulation, Statement Tree (OLP),
    and Arduino serial streaming.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🦾 Visual Components OLP & Digital Twin — 6-DOF Robottikäsivarsi")
        self.resize(1380, 880)
        self.setMinimumSize(1000, 700)

        # Core engines
        self.kinematics = RobotKinematics()
        self.program = RobotProgram(kinematics=self.kinematics, name="MainRoutine")
        self.serial_worker = SerialWorker(self)

        # Connection & streaming state
        self.is_online: bool = False
        self.stream_timer = QTimer(self)
        self.stream_timer.setInterval(35)
        self.stream_timer.timeout.connect(self._on_robot_stream_tick)
        self._robot_stream_trajectory: List[List[int]] = []
        self._robot_stream_index: int = 0

        self._init_ui()
        self._connect_signals()

        # Seed initial default home pose
        self._on_angles_changed([90, 90, 90, 90, 90, 40], [90, 90, 90, 90, 90, 40])

    def _init_ui(self) -> None:
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 8)
        main_layout.setSpacing(8)

        # 1. Top Ribbon / Workspace Selector Bar (Visual Components style)
        ribbon_bar = QHBoxLayout()
        ribbon_bar.setSpacing(6)

        title_lbl = QLabel("🦾 VISUAL COMPONENTS OLP")
        title_lbl.setStyleSheet("color: #00e5ff; font-weight: bold; font-size: 13px; padding-right: 12px;")
        ribbon_bar.addWidget(title_lbl)

        self.btn_tab_program = QPushButton("📝 Program (OLP)")
        self.btn_tab_sim = QPushButton("🚀 Simulation")
        self.btn_tab_home = QPushButton("🏠 Layout / Yhteys")

        self.workspace_buttons = [self.btn_tab_program, self.btn_tab_sim, self.btn_tab_home]
        for idx, btn in enumerate(self.workspace_buttons):
            btn.setCheckable(True)
            btn.setStyleSheet("""
                QPushButton {
                    background: #21252b; color: #abb2bf; font-weight: bold; font-size: 12px;
                    border: 1px solid #37474f; border-radius: 4px; padding: 6px 14px;
                }
                QPushButton:checked {
                    background: #00bcd4; color: black; border-color: #80deea;
                }
                QPushButton:hover:!checked { background: #2c313a; color: white; }
            """)
            btn.clicked.connect(lambda _, i=idx: self._switch_workspace(i))
            ribbon_bar.addWidget(btn)

        self.btn_tab_program.setChecked(True)
        ribbon_bar.addStretch()

        # Quick Online / Offline Badge in Ribbon
        self.lbl_conn_badge = QLabel("● OFFLINE")
        self.lbl_conn_badge.setStyleSheet("""
            background: #2c313a; color: #ff5252; font-weight: bold; font-size: 11px;
            padding: 4px 10px; border-radius: 4px; border: 1px solid #3e4451;
        """)
        ribbon_bar.addWidget(self.lbl_conn_badge)
        main_layout.addLayout(ribbon_bar)

        # 2. Main Horizontal Splitter (Left: 3D Viewport, Right: Workspace Panel)
        splitter = QSplitter(Qt.Horizontal)

        # Left Container: 3D Viewport + HUD info + Camera buttons
        left_container = QFrame()
        left_container.setStyleSheet("background: #181a1f; border-radius: 6px;")
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(4)

        # Viewport toolbar
        view_bar = QHBoxLayout()
        view_bar.setSpacing(4)

        lbl_cam = QLabel("🎥 Kamera:")
        lbl_cam.setStyleSheet("color: #90a4ae; font-size: 11px; font-weight: bold;")
        view_bar.addWidget(lbl_cam)

        btn_style_cam = """
            QPushButton {
                background: #282c34; color: #eceff1; border: 1px solid #3e4451;
                border-radius: 3px; padding: 3px 8px; font-size: 11px;
            }
            QPushButton:hover { background: #3e4451; }
        """
        btn_iso = QPushButton("Koti (3D)")
        btn_iso.setStyleSheet(btn_style_cam)
        btn_iso.clicked.connect(lambda: self.visualizer_3d.reset_camera())
        view_bar.addWidget(btn_iso)

        btn_top = QPushButton("Ylhäältä")
        btn_top.setStyleSheet(btn_style_cam)
        btn_top.clicked.connect(lambda: self.visualizer_3d.view_top())
        view_bar.addWidget(btn_top)

        btn_side = QPushButton("Sivulta")
        btn_side.setStyleSheet(btn_style_cam)
        btn_side.clicked.connect(lambda: self.visualizer_3d.view_side())
        view_bar.addWidget(btn_side)

        # 3D Gizmo toggle button in 3D viewport header
        self.btn_view_gizmo = QPushButton("🎯 3D Jog Gizmo")
        self.btn_view_gizmo.setCheckable(True)
        self.btn_view_gizmo.setStyleSheet("""
            QPushButton {
                background: #263238; color: #00e5ff; font-weight: bold; font-size: 11px;
                border: 1px solid #00acc1; border-radius: 3px; padding: 3px 8px;
            }
            QPushButton:checked {
                background: #00acc1; color: black; border-color: #80deea;
            }
            QPushButton:hover:!checked { background: #37474f; }
        """)
        self.btn_view_gizmo.toggled.connect(self._on_gizmo_toggled_from_view)
        view_bar.addWidget(self.btn_view_gizmo)

        view_bar.addStretch()

        # HUD coordinate label
        self.hud_label = QLabel("TCP: X: 0.0 mm | Y: 0.0 mm | Z: 0.0 mm | Pitch: 0.0°")
        self.hud_label.setStyleSheet("""
            QLabel {
                background: #21252b; color: #00e5ff; font-family: monospace;
                font-weight: bold; padding: 4px 10px; border-radius: 4px; border: 1px solid #3e4451;
            }
        """)
        view_bar.addWidget(self.hud_label)
        left_layout.addLayout(view_bar)

        # 3D Viewport Widget
        self.visualizer_3d = ArmVisualizer3D(self)
        left_layout.addWidget(self.visualizer_3d, 1)
        splitter.addWidget(left_container)

        # Right Container: Multi-Workspace Stack
        right_container = QFrame()
        right_container.setStyleSheet("background: #1e2227; border-radius: 6px;")
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(8)

        # Stack Pages
        self.workspace_stack = QTabWidget()
        self.workspace_stack.tabBar().setVisible(False)  # Controlled by Ribbon buttons
        self.workspace_stack.setStyleSheet("QTabWidget::pane { border: none; }")

        # Page 0: PROGRAM (OLP) Workspace
        page_program = QWidget()
        p_layout = QVBoxLayout(page_program)
        p_layout.setContentsMargins(0, 0, 0, 0)
        p_layout.setSpacing(8)

        self.statement_tree = StatementTreePanel(
            program=self.program,
            get_current_angles_fn=self._get_current_sim_angles,
            apply_angles_fn=self._apply_angles_to_arm,
            on_program_changed_fn=self._on_program_changed,
            parent=self
        )
        p_layout.addWidget(self.statement_tree, 2)

        self.compact_jog = CompactJogPanel(
            kinematics=self.kinematics,
            get_angles_fn=self._get_current_sim_angles,
            apply_angles_fn=self._apply_angles_to_arm,
            on_toggle_gizmo_fn=self._on_gizmo_toggled_from_jog,
            parent=self
        )
        p_layout.addWidget(self.compact_jog, 1)
        self.workspace_stack.addTab(page_program, "Program")

        # Page 1: SIMULATION Workspace
        page_sim = QWidget()
        s_layout = QVBoxLayout(page_sim)
        s_layout.setContentsMargins(0, 0, 0, 0)
        s_layout.setSpacing(8)

        self.sim_panel = SimulationControlPanel(
            program=self.program,
            get_angles_fn=self._get_current_sim_angles,
            apply_angles_fn=self._apply_angles_to_arm,
            on_frame_tick_fn=self.statement_tree.highlight_statement,
            on_execute_robot_fn=self._on_execute_robot,
            parent=self
        )
        s_layout.addWidget(self.sim_panel)
        s_layout.addStretch()
        self.workspace_stack.addTab(page_sim, "Simulation")

        # Page 2: LAYOUT / HOME Workspace
        page_home = QWidget()
        h_layout = QVBoxLayout(page_home)
        h_layout.setContentsMargins(0, 0, 0, 0)
        h_layout.setSpacing(8)

        # Connection Bar
        self.conn_bar = ConnectionBar(
            on_connect_callback=self._handle_connect,
            on_disconnect_callback=self._handle_disconnect,
            parent=self
        )
        h_layout.addWidget(self.conn_bar)

        # Full Slider Panel for fine calibration
        self.slider_panel = SliderPanel(
            on_angles_changed_callback=self._on_angles_changed,
            parent=self
        )
        h_layout.addWidget(self.slider_panel)
        h_layout.addStretch()
        self.workspace_stack.addTab(page_home, "Home")

        right_layout.addWidget(self.workspace_stack)
        splitter.addWidget(right_container)

        # Set ratio: 58% 3D View, 42% Panel
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter, 1)

        # 3. Status Bar
        self.status_bar = QStatusBar(self)
        self.status_bar.setStyleSheet("background: #181a1f; color: #abb2bf; font-size: 11px;")
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Visual Components OLP Valmis. Käytä Program-näkymää tai raahaa 3D Gizmolla!")

    def _switch_workspace(self, index: int) -> None:
        self.workspace_stack.setCurrentIndex(index)
        for i, btn in enumerate(self.workspace_buttons):
            btn.setChecked(i == index)

    def _connect_signals(self) -> None:
        self.serial_worker.connected.connect(self._on_serial_connected)
        self.serial_worker.disconnected.connect(self._on_serial_disconnected)
        self.serial_worker.data_received.connect(self._on_serial_data_received)
        self.serial_worker.packet_sent.connect(self._on_serial_packet_sent)
        self.serial_worker.error_occurred.connect(self._on_serial_error)

    def _get_current_sim_angles(self) -> List[int]:
        return self.slider_panel.get_angles()

    def _apply_angles_to_arm(self, angles: List[int]) -> None:
        self.slider_panel.set_angles(angles)
        self.compact_jog.sync_angles(angles)

    def _on_angles_changed(self, sim_angles: List[int], physical_angles: List[int]) -> None:
        """Called whenever arm angles change (Gizmo, Jog, Program double click, simulation)."""
        # 1. Update 3D pose
        pose = self.kinematics.forward_kinematics([float(a) for a in sim_angles])
        self.visualizer_3d.update_pose(pose, sim_angles)

        # 2. Update HUD coordinates
        self.hud_label.setText(
            f"TCP: X: {pose.tcp_x:6.1f} mm | Y: {pose.tcp_y:6.1f} mm | Z: {pose.tcp_z:6.1f} mm | Pitch: {pose.pitch_deg:5.1f}°"
        )

        # 3. Sync compact jog panel values
        self.compact_jog.sync_angles(sim_angles)

        # 4. Stream to Arduino if online and not running batch sequence
        if self.is_online and not self.stream_timer.isActive() and not self.sim_panel.timer.isActive():
            self.serial_worker.send_angles(physical_angles)

    def _on_gizmo_toggled_from_view(self, checked: bool) -> None:
        self.compact_jog.btn_gizmo.setChecked(checked)
        self._set_gizmo_state(checked)

    def _on_gizmo_toggled_from_jog(self, checked: bool) -> None:
        self.btn_view_gizmo.setChecked(checked)
        self._set_gizmo_state(checked)

    def _set_gizmo_state(self, active: bool) -> None:
        self.visualizer_3d.set_gizmo_enabled(
            enabled=active,
            on_jog_callback=self._apply_angles_to_arm,
            kinematics=self.kinematics,
            get_angles_fn=self._get_current_sim_angles
        )
        if active:
            self.status_bar.showMessage("🎯 3D Interactive Jog Gizmo aktiivinen: Raahaa kouran palloa hiirellä 3D-ikkunassa!")
        else:
            self.status_bar.showMessage("3D Interactive Gizmo suljettu.")

    def _on_program_changed(self) -> None:
        """Refreshes 3D trajectory path when statements change."""
        self.visualizer_3d.update_trajectory_path(self.program.statements, self.kinematics)

    def _on_execute_robot(self, trajectory: List[List[int]]) -> None:
        """Streams compiled statement path directly to Arduino via SerialWorker."""
        if not self.is_online:
            QMessageBox.warning(
                self,
                "Robotti Offline",
                "Robotti ei ole yhdistetty sarjaporttiin!\n\n"
                "Yhdistä Arduino USB-kaapelilla '🏠 Layout / Yhteys' -näkymästä ennen fyysistä ajoa.\n"
                "Voit sillä välin testata liikkeen '▶️ Toista' -painikkeella näytöllä."
            )
            return

        if not trajectory:
            return

        self._robot_stream_trajectory = trajectory
        self._robot_stream_index = 0
        self.status_bar.showMessage(f"Ajetaan robottia live: {len(trajectory)} kehystä...")
        self.stream_timer.start(35)

    def _on_robot_stream_tick(self) -> None:
        if self._robot_stream_index >= len(self._robot_stream_trajectory):
            self.stream_timer.stop()
            self.status_bar.showMessage("✓ Liikesarja ajettu onnistuneesti fyysisellä robotilla!")
            return

        frame = self._robot_stream_trajectory[self._robot_stream_index]
        phys = self.compact_jog.to_physical_angles(frame)
        self.serial_worker.send_angles(phys)

        if self._robot_stream_index % 10 == 0 or self._robot_stream_index == len(self._robot_stream_trajectory) - 1:
            self.status_bar.showMessage(f"Ajo robotilla: {self._robot_stream_index + 1} / {len(self._robot_stream_trajectory)}...")

        self._robot_stream_index += 1

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
        self.lbl_conn_badge.setText(f"● ONLINE ({port})")
        self.lbl_conn_badge.setStyleSheet("""
            background: #004d40; color: #69f0ae; font-weight: bold; font-size: 11px;
            padding: 4px 10px; border-radius: 4px; border: 1px solid #00bfa5;
        """)
        self.status_bar.showMessage(f"Yhteys muodostettu: {port}. Robotti synkronoitu!")
        self.serial_worker.send_angles(self.compact_jog.get_physical_angles())

    def _on_serial_disconnected(self, reason: str) -> None:
        self.is_online = False
        if self.stream_timer.isActive():
            self.stream_timer.stop()
        self.conn_bar.set_offline_ui(reason)
        self.lbl_conn_badge.setText("● OFFLINE")
        self.lbl_conn_badge.setStyleSheet("""
            background: #2c313a; color: #ff5252; font-weight: bold; font-size: 11px;
            padding: 4px 10px; border-radius: 4px; border: 1px solid #3e4451;
        """)
        self.status_bar.showMessage(f"Offline-tila: {reason}")

    def _on_serial_packet_sent(self, packet: str) -> None:
        self.status_bar.showMessage(f"Lähetetty Arduinolle: {packet.strip()}", 1200)

    def _on_serial_data_received(self, line: str) -> None:
        self.status_bar.showMessage(f"Arduino kuittaus: {line.strip()}", 1200)

    def _on_serial_error(self, err_msg: str) -> None:
        self.status_bar.showMessage(f"Virhe: {err_msg}", 4000)

    def closeEvent(self, event) -> None:
        if self.stream_timer.isActive():
            self.stream_timer.stop()
        self.serial_worker.disconnect_serial("Sovellus suljettiin")
        event.accept()
