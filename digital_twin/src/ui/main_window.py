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
import math
from typing import List, Optional, Tuple
from src.config import JOINT_LIMITS
from src.core.calibration import CalibrationManager
from src.core.kinematics import RobotKinematics
from src.core.program import RobotProgram, MotionStatement
from src.communication.serial_worker import SerialWorker
from src.simulation.arm_visualizer_3d import ArmVisualizer3D
from src.ui.calibration_dialog import CalibrationDialog
from src.ui.connection_bar import ConnectionBar
from src.ui.statement_tree_panel import StatementTreePanel
from src.ui.compact_jog_panel import CompactJogPanel
from src.ui.simulation_control_panel import SimulationControlPanel
from src.ui.slider_panel import SliderPanel
from src.ui.workpiece_dialog import WorkpieceTransformDialog
from src.ui.qt_compat import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSplitter, QStatusBar, QMessageBox, Qt, QFrame, QPushButton,
    QTabWidget, QTimer, QShortcut, QKeySequence, QFileDialog, QMenu
)


class MainWindow(QMainWindow):
    """
    Primary desktop window hosting 3D Digital Twin simulation,
    interactive 3D Gizmo manipulation, Statement Tree (OLP),
    and Arduino serial streaming.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot OLP Studio — 6-DOF Industrial Twin")
        self.resize(1380, 880)
        self.setMinimumSize(1000, 700)

        # Core engines
        self.kinematics = RobotKinematics()
        self.program = RobotProgram(kinematics=self.kinematics, name="MainRoutine")
        self.calibration = CalibrationManager()
        self.serial_worker = SerialWorker(self)

        # Connection & streaming state
        self.is_online: bool = False
        self.stream_timer = QTimer(self)
        self.stream_timer.setInterval(35)
        self.stream_timer.timeout.connect(self._on_robot_stream_tick)
        self._robot_stream_trajectory: List[List[int]] = []
        self._robot_stream_index: int = 0
        self._seam_point_a: Optional[Tuple[float, float, float]] = None

        # Workpiece tracking
        self._active_demo_fixture: Optional[str] = None
        self._active_workpiece_stl: Optional[str] = None

        self._init_ui()
        self.kinematics.set_active_tool(self.visualizer_3d.active_tool)
        self._setup_shortcuts()
        self._connect_signals()

        # Seed initial default home pose
        self._on_angles_changed([90, 90, 90, 90, 90, 40], [90, 90, 90, 90, 90, 40])

    def _setup_shortcuts(self) -> None:
        """Configures industry-standard hotkeys for simulation and program editing."""
        QShortcut(QKeySequence("Space"), self, self._toggle_play_pause)
        QShortcut(QKeySequence("Ctrl+S"), self, self.statement_tree._save_program)
        QShortcut(QKeySequence("Ctrl+O"), self, self.statement_tree._load_program)
        QShortcut(QKeySequence("Ctrl+Shift+S"), self, self._save_project)
        QShortcut(QKeySequence("Ctrl+Shift+O"), self, self._load_project)
        QShortcut(QKeySequence("Delete"), self, self.statement_tree._delete_selected)
        QShortcut(QKeySequence("Ctrl+H"), self, lambda: self._apply_angles_to_arm([90, 90, 90, 90, 90, 40]))
        QShortcut(QKeySequence("Esc"), self, self._emergency_stop)
        QShortcut(QKeySequence("Ctrl+T"), self, self._open_tool_dialog)

    def _toggle_play_pause(self) -> None:
        if self.sim_panel.timer.isActive():
            self.sim_panel.pause_simulation()
        else:
            self.sim_panel.play_simulation()

    def _init_ui(self) -> None:
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 8)
        main_layout.setSpacing(8)

        # 1. Top Ribbon / Workspace Selector Bar (Visual Components style)
        ribbon_bar = QHBoxLayout()
        ribbon_bar.setSpacing(6)

        title_lbl = QLabel("ROBOT OLP STUDIO")
        title_lbl.setStyleSheet("color: #4fc1ff; font-weight: 700; font-size: 13px; letter-spacing: 0.5px; padding-right: 14px;")
        ribbon_bar.addWidget(title_lbl)

        self.btn_tab_program = QPushButton("Program (OLP)")
        self.btn_tab_sim = QPushButton("Simulation")
        self.btn_tab_home = QPushButton("Layout & Connections")

        self.workspace_buttons = [self.btn_tab_program, self.btn_tab_sim, self.btn_tab_home]
        for idx, btn in enumerate(self.workspace_buttons):
            btn.setCheckable(True)
            btn.setStyleSheet("""
                QPushButton {
                    background: #252526; color: #858585; font-weight: 600; font-size: 11px;
                    border: 1px solid #333333; border-radius: 3px; padding: 6px 14px;
                }
                QPushButton:checked {
                    background: #007acc; color: #ffffff; border-color: #0098ff;
                }
                QPushButton:hover:!checked { background: #2d2d30; color: #d4d4d4; }
            """)
            btn.clicked.connect(lambda _, i=idx: self._switch_workspace(i))
            ribbon_bar.addWidget(btn)

        self.btn_tab_program.setChecked(True)
        ribbon_bar.addStretch()

        # Project Save / Open
        self.btn_save_proj = QPushButton("💾 Tallenna projekti")
        self.btn_save_proj.setToolTip("Tallenna koko työtila (.project.json: ohjelma, työkalu, kappale) [Ctrl+Shift+S]")
        self.btn_save_proj.setStyleSheet("""
            QPushButton {
                background: #1e3a5f; color: #60a5fa; font-weight: 600; font-size: 11px;
                border: 1px solid #2563eb; border-radius: 3px; padding: 5px 12px;
            }
            QPushButton:hover { background: #2563eb; color: #ffffff; }
        """)
        self.btn_save_proj.clicked.connect(self._save_project)
        ribbon_bar.addWidget(self.btn_save_proj)

        self.btn_open_proj = QPushButton("📂 Avaa projekti")
        self.btn_open_proj.setToolTip("Avaa aiemmin tallennettu projektitiedosto (.project.json) [Ctrl+Shift+O]")
        self.btn_open_proj.setStyleSheet("""
            QPushButton {
                background: #252526; color: #cccccc; font-weight: 600; font-size: 11px;
                border: 1px solid #3f3f46; border-radius: 3px; padding: 5px 12px;
            }
            QPushButton:hover { background: #2d2d30; color: #ffffff; }
        """)
        self.btn_open_proj.clicked.connect(self._load_project)
        ribbon_bar.addWidget(self.btn_open_proj)

        # E-STOP Emergency Safety Button
        self.btn_estop = QPushButton("⛔ HÄTÄ-SEIS (Esc)")
        self.btn_estop.setToolTip("Pysäytä välittömästi kaikki robotin liikkeet ja sarjaliikenne [Esc]")
        self.btn_estop.setStyleSheet("""
            QPushButton {
                background: #b71c1c; color: #ffffff; font-weight: bold; font-size: 11px;
                border: 1px solid #ff1744; border-radius: 3px; padding: 5px 14px;
            }
            QPushButton:hover { background: #d32f2f; }
            QPushButton:pressed { background: #7f0000; }
        """)
        self.btn_estop.clicked.connect(self._emergency_stop)
        ribbon_bar.addWidget(self.btn_estop)

        # Online / Offline Badge
        self.lbl_conn_badge = QLabel("OFFLINE")
        self.lbl_conn_badge.setStyleSheet("""
            background: #2d2d30; color: #f48771; font-weight: 600; font-size: 10px;
            padding: 3px 10px; border-radius: 3px; border: 1px solid #3f3f46;
        """)
        ribbon_bar.addWidget(self.lbl_conn_badge)
        main_layout.addLayout(ribbon_bar)

        # 2. Main Horizontal Splitter (Left: 3D Viewport, Right: Workspace Panel)
        splitter = QSplitter(Qt.Horizontal)

        # Left Container: 3D Viewport + HUD info + Camera buttons
        left_container = QFrame()
        left_container.setStyleSheet("background: #181818; border-radius: 4px; border: 1px solid #2d2d30;")
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(4)

        # Viewport toolbar
        view_bar = QHBoxLayout()
        view_bar.setSpacing(4)

        lbl_cam = QLabel("Camera:")
        lbl_cam.setStyleSheet("color: #858585; font-size: 11px; font-weight: 600;")
        view_bar.addWidget(lbl_cam)

        btn_style_cam = """
            QPushButton {
                background: #252526; color: #cccccc; border: 1px solid #333333;
                border-radius: 3px; padding: 3px 8px; font-size: 11px; font-weight: 600;
            }
            QPushButton:hover { background: #2d2d30; color: #ffffff; }
        """
        btn_iso = QPushButton("Isometric")
        btn_iso.setStyleSheet(btn_style_cam)
        btn_iso.clicked.connect(lambda: self.visualizer_3d.reset_camera())
        view_bar.addWidget(btn_iso)

        btn_top = QPushButton("Top")
        btn_top.setStyleSheet(btn_style_cam)
        btn_top.clicked.connect(lambda: self.visualizer_3d.view_top())
        view_bar.addWidget(btn_top)

        btn_side = QPushButton("Side")
        btn_side.setStyleSheet(btn_style_cam)
        btn_side.clicked.connect(lambda: self.visualizer_3d.view_side())
        view_bar.addWidget(btn_side)

        btn_part = QPushButton("Import Part")
        btn_part.setStyleSheet(btn_style_cam)
        btn_part.clicked.connect(self._on_import_workpiece)
        view_bar.addWidget(btn_part)

        # Demo fixtures dropdown button
        btn_demo_fixture = QPushButton("📦 Esimerkkikappaleet ▼")
        btn_demo_fixture.setStyleSheet(btn_style_cam)
        menu_demo = QMenu(self)
        menu_demo.setStyleSheet("""
            QMenu { background: #252526; color: #cccccc; border: 1px solid #3f3f46; padding: 4px; font-size: 11px; }
            QMenu::item { padding: 4px 18px; }
            QMenu::item:selected { background: #007acc; color: white; border-radius: 2px; }
        """)
        act_plate = menu_demo.addAction("V-saumalevyt (V-Groove Seam)")
        act_pipe = menu_demo.addAction("Putkiliitos (Pipe T-Joint)")
        act_bracket = menu_demo.addAction("Kulmakiinnike (Corner Bracket)")
        menu_demo.addSeparator()
        act_clear_demo = menu_demo.addAction("Poista kappale (Clear Part)")

        act_plate.triggered.connect(lambda: self._load_demo_fixture("WELD_PLATE"))
        act_pipe.triggered.connect(lambda: self._load_demo_fixture("PIPE_JOINT"))
        act_bracket.triggered.connect(lambda: self._load_demo_fixture("CORNER_BRACKET"))
        act_clear_demo.triggered.connect(self._clear_workpiece)
        btn_demo_fixture.setMenu(menu_demo)
        view_bar.addWidget(btn_demo_fixture)

        btn_pos_part = QPushButton("📐 Asemoi...")
        btn_pos_part.setStyleSheet(btn_style_cam)
        btn_pos_part.setToolTip("Asemoi ja käännä kappale pöydällä (Workpiece Transform)")
        btn_pos_part.clicked.connect(self._open_workpiece_dialog)
        view_bar.addWidget(btn_pos_part)

        btn_tool = QPushButton("Tool & TCP...")
        btn_tool.setStyleSheet(btn_style_cam)
        btn_tool.clicked.connect(self._open_tool_dialog)
        view_bar.addWidget(btn_tool)

        self.btn_view_envelope = QPushButton("Reach Envelope")
        self.btn_view_envelope.setCheckable(True)
        self.btn_view_envelope.setStyleSheet(btn_style_cam)
        self.btn_view_envelope.toggled.connect(lambda chk: self.visualizer_3d.set_reach_envelope_visible(chk))
        view_bar.addWidget(self.btn_view_envelope)

        # 3D Gizmo toggle button in 3D viewport header
        self.btn_view_gizmo = QPushButton("Interactive Gizmo")
        self.btn_view_gizmo.setCheckable(True)
        self.btn_view_gizmo.setStyleSheet("""
            QPushButton {
                background: #252526; color: #4fc1ff; font-weight: 600; font-size: 11px;
                border: 1px solid #007acc; border-radius: 3px; padding: 3px 8px;
            }
            QPushButton:checked {
                background: #007acc; color: #ffffff; border-color: #0098ff;
            }
            QPushButton:hover:!checked { background: #2d2d30; }
        """)
        self.btn_view_gizmo.toggled.connect(self._on_gizmo_toggled_from_view)
        view_bar.addWidget(self.btn_view_gizmo)

        # Snap TCP to surface toggle
        self.btn_snap_surface = QPushButton("🎯 Snap TCP")
        self.btn_snap_surface.setCheckable(True)
        self.btn_snap_surface.setStyleSheet("""
            QPushButton {
                background: #252526; color: #00e5ff; font-weight: 600; font-size: 11px;
                border: 1px solid #00e5ff; border-radius: 3px; padding: 3px 8px;
            }
            QPushButton:checked {
                background: #00838f; color: #ffffff; border-color: #00e5ff;
            }
            QPushButton:hover:!checked { background: #2d2d30; }
        """)
        self.btn_snap_surface.setToolTip("Napsauta kappaleen pintaa 3D-ikkunassa: siirtää robotin polttimen kärjen suoraan kohteeseen")
        self.btn_snap_surface.toggled.connect(self._toggle_surface_snapping)
        view_bar.addWidget(self.btn_snap_surface)

        # 2-Click Seam Generator toggle
        self.btn_2click_seam = QPushButton("⚡ 2-Click Seam")
        self.btn_2click_seam.setCheckable(True)
        self.btn_2click_seam.setStyleSheet("""
            QPushButton {
                background: #252526; color: #d8b4fe; font-weight: 600; font-size: 11px;
                border: 1px solid #9333ea; border-radius: 3px; padding: 3px 8px;
            }
            QPushButton:checked {
                background: #7c3aed; color: #ffffff; border-color: #a855f7;
            }
            QPushButton:hover:!checked { background: #2d2d30; }
        """)
        self.btn_2click_seam.setToolTip("Kahden klikkauksen hitsaussauma: Napsauta sauman alku- ja loppupistettä kappaleen pinnalla")
        self.btn_2click_seam.toggled.connect(self._toggle_2click_seam)
        view_bar.addWidget(self.btn_2click_seam)

        view_bar.addStretch()

        # HUD coordinate label
        self.hud_label = QLabel("TCP: X: 0.0 mm | Y: 0.0 mm | Z: 0.0 mm | Pitch: 0.0°")
        self.hud_label.setStyleSheet("""
            QLabel {
                background: #1e1e1e; color: #4fc1ff; font-family: Consolas, monospace;
                font-weight: 600; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 1px solid #333333;
            }
        """)
        view_bar.addWidget(self.hud_label)
        left_layout.addLayout(view_bar)

        # Joint Angles & Tool Quick Dashboard Strip
        joint_hud_bar = QHBoxLayout()
        joint_hud_bar.setSpacing(4)

        self.joint_labels = []
        for i in range(6):
            lbl_j = QLabel(f"J{i+1}: 90°" if i < 5 else "J6: 40°")
            lbl_j.setStyleSheet("""
                QLabel {
                    background: #252526; color: #69f0ae; font-family: Consolas, monospace;
                    font-size: 11px; font-weight: 600; padding: 2px 7px;
                    border: 1px solid #333333; border-radius: 3px;
                }
            """)
            self.joint_labels.append(lbl_j)
            joint_hud_bar.addWidget(lbl_j)

        self.lbl_tool_badge = QPushButton("🔧 Työkalu: 2-Leukakoura")
        self.lbl_tool_badge.setToolTip("Klikkaa avataksesi työkalun ja TCP-asetusten hallinnan [Ctrl+T]")
        self.lbl_tool_badge.setStyleSheet("""
            QPushButton {
                background: #1b2838; color: #4fc1ff; font-weight: 600; font-size: 11px;
                border: 1px solid #007acc; border-radius: 3px; padding: 2px 8px;
            }
            QPushButton:hover { background: #007acc; color: white; }
        """)
        self.lbl_tool_badge.clicked.connect(self._open_tool_dialog)
        joint_hud_bar.addWidget(self.lbl_tool_badge)

        joint_hud_bar.addStretch()
        left_layout.addLayout(joint_hud_bar)

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
            on_trigger_2click_seam_fn=self._trigger_2click_seam_from_panel,
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
            on_weld_tick_fn=self._on_weld_tick,
            on_clear_weld_fn=self.visualizer_3d.clear_weld_seams,
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

        # Hardware Zero-Offset Calibration Button
        btn_calib = QPushButton("Hardware Zero-Offset & Trim Calibration...")
        btn_calib.setStyleSheet("""
            QPushButton {
                background: #252526; color: #4fc1ff; font-weight: 600; font-size: 11px;
                border: 1px solid #007acc; border-radius: 3px; padding: 7px;
            }
            QPushButton:hover { background: #007acc; color: white; }
        """)
        btn_calib.clicked.connect(self._open_calibration_dialog)
        h_layout.addWidget(btn_calib)

        # Tool & TCP Setup Button
        btn_tool_layout = QPushButton("End-Effector & TCP Setup (Tool Snap & Coordinates)...")
        btn_tool_layout.setStyleSheet("""
            QPushButton {
                background: #252526; color: #4fc1ff; font-weight: 600; font-size: 11px;
                border: 1px solid #007acc; border-radius: 3px; padding: 7px;
            }
            QPushButton:hover { background: #007acc; color: white; }
        """)
        btn_tool_layout.clicked.connect(self._open_tool_dialog)
        h_layout.addWidget(btn_tool_layout)

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
        if index == 1:
            self.sim_panel.update_program_stats()

    def _on_import_workpiece(self) -> None:
        """Opens file dialog and loads a CAD STL model onto the table for seam planning."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Import Workpiece CAD Part (STL)",
            "",
            "STL 3D Files (*.stl);;All Files (*.*)"
        )
        if file_path:
            ok = self.visualizer_3d.load_workpiece(file_path)
            if ok:
                self._active_demo_fixture = None
                self._active_workpiece_stl = file_path
                from pathlib import Path
                self.status_bar.showMessage(f"CAD-kappale ladattu: {Path(file_path).name} (pöytäkoordinaatit X=260, Y=0, Z=0)", 5000)
            else:
                QMessageBox.warning(self, "Virhe", "Kappaleen STL-tiedostoa ei voitu ladata.")

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

    def _on_angles_changed(self, sim_angles: List[int], physical_angles: List[int]) -> None:
        """Called whenever arm angles change (Gizmo, Jog, Program double click, simulation)."""
        # 1. Update 3D pose
        pose = self.kinematics.forward_kinematics([float(a) for a in sim_angles])
        self.visualizer_3d.update_pose(pose, sim_angles)

        # 2. Collision and Singularity checks
        has_coll, coll_msg = self.kinematics.check_collision(pose, self.visualizer_3d.workpiece_bounds)
        self.visualizer_3d.set_collision_highlight(has_coll)
        sing_msg = self.kinematics.check_singularity(sim_angles)

        # 3. Update HUD coordinates and warnings (cached stylesheet to eliminate UI lag)
        hud_state = "coll" if has_coll else ("sing" if sing_msg else "normal")
        if hud_state != getattr(self, "_last_hud_state", ""):
            self._last_hud_state = hud_state
            if has_coll:
                self.hud_label.setStyleSheet("QLabel { background: #3b1111; color: #ff5252; font-family: Consolas, monospace; font-weight: 600; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 1px solid #ff1744; }")
            elif sing_msg:
                self.hud_label.setStyleSheet("QLabel { background: #3b2d11; color: #ffd600; font-family: Consolas, monospace; font-weight: 600; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 1px solid #ffab00; }")
            else:
                self.hud_label.setStyleSheet("QLabel { background: #1e1e1e; color: #4fc1ff; font-family: Consolas, monospace; font-weight: 600; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 1px solid #333333; }")

        hud_text = f"TCP: X: {pose.tcp_x:6.1f} mm | Y: {pose.tcp_y:6.1f} mm | Z: {pose.tcp_z:6.1f} mm | Pitch: {pose.pitch_deg:5.1f}°"
        if has_coll:
            hud_text += f" | TÖRMÄYS: {coll_msg}"
        elif sing_msg:
            hud_text += f" | VAROITUS: {sing_msg}"
        self.hud_label.setText(hud_text)

        # 4. Sync compact jog panel values
        self.compact_jog.sync_angles(sim_angles)

        # 5. Update joint badges with color-coding near limits (cached to eliminate CSS string re-parsing lag)
        if hasattr(self, "joint_labels") and self.joint_labels:
            if not hasattr(self, "_last_joint_states"):
                self._last_joint_states = [""] * 6

            for i, val in enumerate(sim_angles[:6]):
                min_v, max_v = JOINT_LIMITS[i]
                dist_to_limit = min(abs(val - min_v), abs(val - max_v))
                if dist_to_limit <= 1:
                    state_key = "danger"
                    color = "#ff5252"
                    bg = "#3b1111"
                elif dist_to_limit <= 15:
                    state_key = "warning"
                    color = "#ffd600"
                    bg = "#3b2d11"
                else:
                    state_key = "normal"
                    color = "#69f0ae"
                    bg = "#1b2d20"

                tag = f"J{i+1}: {val}°"
                if i == 5:
                    tag += " (Koura)"
                self.joint_labels[i].setText(tag)

                if self._last_joint_states[i] != state_key:
                    self._last_joint_states[i] = state_key
                    self.joint_labels[i].setStyleSheet(
                        f"QLabel {{ background: {bg}; color: {color}; font-family: Consolas, monospace; "
                        f"font-size: 11px; font-weight: 600; padding: 2px 7px; border: 1px solid {color}; border-radius: 3px; }}"
                    )

        # 6. Stream calibrated angles to Arduino if online and not running batch sequence
        if self.is_online and not self.stream_timer.isActive() and not self.sim_panel.timer.isActive():
            phys_calib = self.calibration.to_physical(sim_angles)
            self.serial_worker.send_angles(phys_calib)

    def _on_weld_tick(self, angles: List[int]) -> None:
        """Deposits weld bead segment at current TCP pose during LIN welding motion."""
        pose = self.kinematics.forward_kinematics([float(a) for a in angles])
        self.visualizer_3d.add_weld_point([pose.tcp_x, pose.tcp_y, pose.tcp_z])

    def _open_calibration_dialog(self) -> None:
        """Opens Hardware Zero-Offset calibration window."""
        dlg = CalibrationDialog(self.calibration, on_test_angles_fn=self._apply_angles_to_arm, parent=self)
        dlg.exec_()

    def _on_gizmo_toggled_from_view(self, checked: bool) -> None:
        self.compact_jog.btn_gizmo.setChecked(checked)
        self._set_gizmo_state(checked)

    def _on_gizmo_toggled_from_jog(self, checked: bool) -> None:
        self.btn_view_gizmo.setChecked(checked)
        self._set_gizmo_state(checked)

    def _set_gizmo_state(self, active: bool) -> None:
        if active:
            if hasattr(self, "btn_snap_surface") and self.btn_snap_surface.isChecked():
                self.btn_snap_surface.setChecked(False)
            if hasattr(self, "btn_2click_seam") and self.btn_2click_seam.isChecked():
                self.btn_2click_seam.setChecked(False)

        self.visualizer_3d.set_gizmo_enabled(
            enabled=active,
            on_jog_callback=self._apply_angles_to_arm,
            kinematics=self.kinematics,
            get_angles_fn=self._get_current_sim_angles
        )
        if active:
            self.status_bar.showMessage("3D Interactive Jog Gizmo aktiivinen: Raahaa TCP-palloa hiirellä 3D-ikkunassa.")
        else:
            self.status_bar.showMessage("3D Interactive Gizmo suljettu.")

    def _open_tool_dialog(self) -> None:
        """Opens Tool & TCP setup dialog for mounting tools and configuring TCP offsets."""
        try:
            from src.ui.tool_dialog import ToolManagerDialog
            dlg = ToolManagerDialog(
                tool_manager=self.visualizer_3d.tool_manager,
                on_apply_callback=self._on_tool_applied,
                parent=self
            )
            dlg.exec_()
        except Exception as e:
            QMessageBox.critical(self, "Virhe työkalun avauksessa", f"Työkaludialogin avaus epäonnistui:\n{e}")

    def _on_tool_applied(self, tool) -> None:
        """Applies new tool profile to kinematics engine, 3D visualizer, and HUD."""
        self.kinematics.set_active_tool(tool)
        self.visualizer_3d.set_active_tool(tool)
        if hasattr(self, "lbl_tool_badge"):
            self.lbl_tool_badge.setText(f"🔧 {tool.name}")
        sim_angles = self._get_current_sim_angles()
        self._on_angles_changed(sim_angles, sim_angles)
        self.status_bar.showMessage(
            f"Aktiivinen työkalu: {tool.name} (TCP X={tool.tcp_x:.1f}, Y={tool.tcp_y:.1f}, Z={tool.tcp_z:.1f} mm)",
            5000
        )

    def _emergency_stop(self) -> None:
        """Immediate hardware & simulation safety stop (E-STOP)."""
        self.sim_panel.stop_simulation()
        if self.is_online:
            self.serial_worker.emergency_stop()
        self.status_bar.showMessage("⛔ HÄTÄ-SEIS AKTIIVINEN! Kaikki liike ja sarjaliikenne pysäytetty välittömästi.", 10000)
        self.hud_label.setText("⛔ HÄTÄ-SEIS AKTIIVINEN — KAIKKI LIIKE PYSÄYTETTY")
        self.hud_label.setStyleSheet("QLabel { background: #500000; color: #ffffff; font-family: Consolas, monospace; font-weight: bold; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 2px solid #ff1744; }")

    def _load_demo_fixture(self, fixture_name: str) -> None:
        """Loads built-in CAD workpiece fixture (V-groove plates, pipe joint, corner bracket)."""
        ok = self.visualizer_3d.load_demo_workpiece(fixture_name)
        if ok:
            self._active_demo_fixture = fixture_name
            self._active_workpiece_stl = None
            names = {
                "WELD_PLATE": "V-saumalevyt (V-Groove Seam)",
                "PIPE_JOINT": "Putkiliitos (Pipe T-Joint)",
                "CORNER_BRACKET": "Kulmakiinnike (Corner Bracket)"
            }
            self.status_bar.showMessage(f"Esimerkkikappale ladattu: {names.get(fixture_name, fixture_name)} (Pöytäkoordinaatit X=260)", 5000)

    def _clear_workpiece(self) -> None:
        """Removes CAD workpiece from 3D scene."""
        if hasattr(self, "btn_snap_surface") and self.btn_snap_surface.isChecked():
            self.btn_snap_surface.setChecked(False)
        if hasattr(self, "btn_2click_seam") and self.btn_2click_seam.isChecked():
            self.btn_2click_seam.setChecked(False)
        self.visualizer_3d.clear_workpiece()
        self._active_demo_fixture = None
        self._active_workpiece_stl = None
        self.status_bar.showMessage("Kappale poistettu 3D-työtilasta.", 3000)

    def _save_project(self) -> None:
        """Saves entire workcell state (program, tool TCP, workpiece CAD, poses) to .project.json."""
        from pathlib import Path
        from src.core.project_manager import export_project_data, save_project_to_file

        path, _ = QFileDialog.getSaveFileName(
            self,
            "Tallenna työtilaprojekti",
            f"{self.program.name}.project.json",
            "Robot Project Files (*.project.json *.json);;All Files (*.*)"
        )
        if not path:
            return

        try:
            active_tool = getattr(self.visualizer_3d, "active_tool", None)
            cur_angles = self._get_current_sim_angles()
            wp_pos = getattr(self.visualizer_3d, "workpiece_pos", [260.0, 0.0, 0.0])
            wp_rot = getattr(self.visualizer_3d, "workpiece_rot_z", 0.0)

            data = export_project_data(
                program=self.program,
                active_tool=active_tool,
                demo_fixture=self._active_demo_fixture,
                workpiece_stl=self._active_workpiece_stl,
                workpiece_pos=wp_pos,
                workpiece_rot_z=wp_rot,
                current_angles=cur_angles,
                project_name=Path(path).stem.replace(".project", "")
            )
            save_project_to_file(Path(path), data)
            self.status_bar.showMessage(f"Projekti tallennettu: {Path(path).name}", 5000)
            QMessageBox.information(self, "Tallennettu", f"Koko työtila tallennettu onnistuneesti tiedostoon:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Tallennusvirhe", f"Projektin tallennus epäonnistui:\n{e}")

    def _load_project(self) -> None:
        """Loads complete workcell state from a .project.json file."""
        from pathlib import Path
        from src.core.project_manager import load_project_from_file
        from src.core.program import Statement
        from src.core.tool_data import ToolProfile

        path, _ = QFileDialog.getOpenFileName(
            self,
            "Avaa työtilaprojekti",
            "",
            "Robot Project Files (*.project.json *.json);;All Files (*.*)"
        )
        if not path:
            return

        try:
            data = load_project_from_file(Path(path))

            # 1. Load Program
            prog_data = data.get("program", {})
            self.program.name = prog_data.get("name", "MainRoutine")
            self.program.statements = [Statement.from_dict(item) for item in prog_data.get("statements", [])]
            self.program._target_counter = len(self.program.statements) + 1
            self.statement_tree.refresh_tree()

            # 2. Load Tool
            tool_data = data.get("active_tool")
            if tool_data:
                tool = ToolProfile.from_dict(tool_data)
                self._on_tool_applied(tool)

            # 3. Load Workpiece
            wp_data = data.get("workpiece", {})
            demo_fixture = wp_data.get("demo_fixture")
            stl_path = wp_data.get("stl_path")
            wp_pos = wp_data.get("pos", [260.0, 0.0, 0.0])
            wp_rot = wp_data.get("rot_z", 0.0)

            if demo_fixture:
                self._load_demo_fixture(demo_fixture)
                self.visualizer_3d.set_workpiece_transform(wp_pos, wp_rot)
            elif stl_path and Path(stl_path).exists():
                self.visualizer_3d.load_workpiece(stl_path)
                self._active_workpiece_stl = stl_path
                self._active_demo_fixture = None
                self.visualizer_3d.set_workpiece_transform(wp_pos, wp_rot)
            else:
                self._clear_workpiece()

            # 4. Load Robot Posture
            angles = data.get("current_angles")
            if angles and len(angles) == 6:
                self._apply_angles_to_arm(angles)

            self.status_bar.showMessage(f"Projekti avattu: {Path(path).name}", 5000)
            QMessageBox.information(self, "Projekti avattu", f"Työtila palautettu onnistuneesti tiedostosta:\n{Path(path).name}")
        except Exception as e:
            QMessageBox.critical(self, "Latausvirhe", f"Projektin avaaminen epäonnistui:\n{e}")

    def _open_workpiece_dialog(self) -> None:
        """Opens dialog to position and rotate the CAD workpiece on the workcell table."""
        if self.visualizer_3d.raw_workpiece_mesh is None:
            res = QMessageBox.question(
                self,
                "Ei kappaletta työtilassa",
                "Työtilassa ei ole vielä ladattuna työkappaletta.\n\n"
                "Haluatko ladata esimerkkikappaleen 'V-saumalevyt'?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if res == QMessageBox.Yes:
                self._load_demo_fixture("WELD_PLATE")
            else:
                return

        dlg = WorkpieceTransformDialog(
            initial_pos=self.visualizer_3d.workpiece_pos,
            initial_rot_z=self.visualizer_3d.workpiece_rot_z,
            on_transform_changed_callback=self.visualizer_3d.set_workpiece_transform,
            on_clear_callback=self._clear_workpiece,
            parent=self
        )
        dlg.exec_()

    def _trigger_2click_seam_from_panel(self) -> None:
        """Triggered from Statement Tree toolbar button."""
        self.btn_2click_seam.setChecked(not self.btn_2click_seam.isChecked())

    def _toggle_surface_snapping(self, checked: bool) -> None:
        """Toggles 1-click surface snapping: clicking 3D surface moves robot TCP to touch it."""
        if checked:
            if self.visualizer_3d.raw_workpiece_mesh is None:
                self.btn_snap_surface.setChecked(False)
                res = QMessageBox.question(
                    self,
                    "Ei kappaletta työtilassa",
                    "Pinnan napsautusta varten työtilassa on oltava työkappale.\n\n"
                    "Ladataanko esimerkkikappale 'V-saumalevyt'?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.Yes
                )
                if res == QMessageBox.Yes:
                    self._load_demo_fixture("WELD_PLATE")
                    self.btn_snap_surface.setChecked(True)
                return

            if self.btn_view_gizmo.isChecked():
                self.btn_view_gizmo.setChecked(False)
            if self.btn_2click_seam.isChecked():
                self.btn_2click_seam.setChecked(False)

            self.visualizer_3d.enable_surface_snapping(self._on_surface_snapped)
            self.status_bar.showMessage("🎯 Surface Snapping aktiivinen: Napsauta kappaleen pintaa viedäksesi polttimen kärki suoraan siihen.", 8000)
            self.hud_label.setText("🎯 SNAP TO SURFACE: Napsauta kappaleen pintaa 3D-ikkunassa")
            self.hud_label.setStyleSheet("QLabel { background: #00363a; color: #80deea; font-family: Consolas, monospace; font-weight: bold; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 1px solid #00acc1; }")
        else:
            self.visualizer_3d.disable_surface_snapping()
            self.status_bar.showMessage("Surface Snapping poistettu käytöstä.", 2000)
            sim_angles = self._get_current_sim_angles()
            self._on_angles_changed(sim_angles, sim_angles)

    def _on_surface_snapped(self, x: float, y: float, z: float) -> None:
        """Solves IK and moves the robot arm to touch the clicked 3D surface point."""
        cur_angles = self._get_current_sim_angles()
        cur_pose = self.kinematics.forward_kinematics([float(a) for a in cur_angles])
        sol, actual = self.kinematics.solve_ik_clamped(
            x, y, z,
            current_angles=cur_angles,
            preferred_pitch=cur_pose.pitch_deg
        )
        if sol is not None:
            self._apply_angles_to_arm(sol)
            self.status_bar.showMessage(
                f"🎯 Kärki asetettu pintaan: X={actual[0]:.1f}, Y={actual[1]:.1f}, Z={actual[2]:.1f} mm | Nivelet: {sol}",
                5000
            )
        else:
            self.status_bar.showMessage("⚠️ Piste on robotin ulottuvuusalueen ulkopuolella! Siirrä kappaletta lähemmäs.", 4000)

    def _toggle_2click_seam(self, checked: bool) -> None:
        """Activates 2-click seam picking mode (Point A -> Point B on workpiece)."""
        if checked:
            if self.visualizer_3d.raw_workpiece_mesh is None:
                self.btn_2click_seam.setChecked(False)
                res = QMessageBox.question(
                    self,
                    "Ei kappaletta työtilassa",
                    "Hitsauksen saumageneraattori vaatii työkappaleen.\n\n"
                    "Ladataanko esimerkkikappale 'V-saumalevyt'?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.Yes
                )
                if res == QMessageBox.Yes:
                    self._load_demo_fixture("WELD_PLATE")
                    self.btn_2click_seam.setChecked(True)
                return

            if self.btn_view_gizmo.isChecked():
                self.btn_view_gizmo.setChecked(False)
            if self.btn_snap_surface.isChecked():
                self.btn_snap_surface.setChecked(False)

            self._seam_point_a = None
            self.visualizer_3d.enable_surface_snapping(self._on_seam_point_picked)
            self.status_bar.showMessage("⚡ VAIHE 1/2: Napsauta kappaleen pinnasta SAUMAN ALKUPISTE (Piste A).", 10000)
            self.hud_label.setText("⚡ 2-CLICK SEAM: Napsauta sauman ALKUPISTE (Piste A)")
            self.hud_label.setStyleSheet("QLabel { background: #3b0764; color: #f3e8ff; font-family: Consolas, monospace; font-weight: bold; font-size: 11px; padding: 3px 10px; border-radius: 3px; border: 1px solid #a855f7; }")
        else:
            self._seam_point_a = None
            self.visualizer_3d.disable_surface_snapping()
            self.status_bar.showMessage("2-Click Seam -tila peruutettu.", 2000)
            sim_angles = self._get_current_sim_angles()
            self._on_angles_changed(sim_angles, sim_angles)

    def _on_seam_point_picked(self, x: float, y: float, z: float) -> None:
        """Handles 2-point clicks on workpiece surface to generate full weld sequence."""
        if self._seam_point_a is None:
            self._seam_point_a = (float(x), float(y), float(z))
            self.visualizer_3d.add_temp_marker(self._seam_point_a, color="#00e676", radius=5.0)
            self.status_bar.showMessage(
                f"⚡ VAIHE 2/2: Alku asetettu (X={x:.1f}, Y={y:.1f}, Z={z:.1f})! Napsauta nyt SAUMAN LOPPUPISTE (Piste B).",
                10000
            )
            self.hud_label.setText(f"⚡ PISTE A: ({x:.0f}, {y:.0f}, {z:.0f}) -> Napsauta PISTE B")
        else:
            pt_a = self._seam_point_a
            pt_b = (float(x), float(y), float(z))
            self.visualizer_3d.add_temp_marker(pt_b, color="#ff9800", radius=5.0)

            self.btn_2click_seam.setChecked(False)
            self.visualizer_3d.disable_surface_snapping()
            self._generate_2click_seam_routine(pt_a, pt_b)

    def _generate_2click_seam_routine(self, pt_a: Tuple[float, float, float], pt_b: Tuple[float, float, float]) -> None:
        """Compiles 5-step industrial welding sequence from two 3D surface points."""
        xA, yA, zA = pt_a
        xB, yB, zB = pt_b

        seam_len = math.sqrt((xB - xA)**2 + (yB - yA)**2 + (zB - zA)**2)
        if seam_len < 2.0:
            QMessageBox.warning(self, "Virhe", "Pisteet A ja B ovat liian lähellä toisiaan (sauman pituus < 2 mm).")
            sim_angles = self._get_current_sim_angles()
            self._on_angles_changed(sim_angles, sim_angles)
            return

        cur_angles = self._get_current_sim_angles()
        cur_pose = self.kinematics.forward_kinematics([float(a) for a in cur_angles])
        pref_pitch = cur_pose.pitch_deg
        app_h = 25.0

        sol_app, _ = self.kinematics.solve_ik_clamped(xA, yA, zA + app_h, current_angles=cur_angles, preferred_pitch=pref_pitch)
        sol_start, _ = self.kinematics.solve_ik_clamped(xA, yA, zA, current_angles=sol_app or cur_angles, preferred_pitch=pref_pitch)
        sol_end, _ = self.kinematics.solve_ik_clamped(xB, yB, zB, current_angles=sol_start or cur_angles, preferred_pitch=pref_pitch)
        sol_retract, _ = self.kinematics.solve_ik_clamped(xB, yB, zB + app_h, current_angles=sol_end or cur_angles, preferred_pitch=pref_pitch)

        if not all([sol_app, sol_start, sol_end, sol_retract]):
            QMessageBox.warning(
                self,
                "Ulottuvuusraja",
                f"Laskettu hitsaussauma ({seam_len:.1f} mm) ylittää osittain robotin ulottuvuusalueen!\n\n"
                "Käytä '📐 Asemoi...' -toimintoa siirtääksesi työkappaleen paremmalle etäisyydelle robotista."
            )
            sim_angles = self._get_current_sim_angles()
            self._on_angles_changed(sim_angles, sim_angles)
            return

        self.statement_tree._push_undo()

        self.program.add_motion(sol_app, motion_type="PTP", speed_percent=100, comment=f"Lähestyminen saumaan (+{int(app_h)}mm)")
        self.program.add_motion(sol_start, motion_type="LIN", speed_percent=50, comment="Kosketus / Valokaari syttyy (ARC ON)")
        self.program.add_motion(sol_end, motion_type="LIN", speed_percent=25, comment=f"Hitsaussauma LIN ({seam_len:.1f} mm)")
        self.program.add_delay(duration_ms=500, comment="Kraatterin täyttö / Valokaari sammuu (ARC OFF)")
        self.program.add_motion(sol_retract, motion_type="LIN", speed_percent=80, comment=f"Nosto sauman päältä (+{int(app_h)}mm)")

        self.statement_tree.refresh_tree()
        self._on_program_changed()
        self._apply_angles_to_arm(sol_app)

        self.status_bar.showMessage(
            f"✅ 5-vaiheinen hitsausohjelma generoitu onnistuneesti! Sauman pituus: {seam_len:.1f} mm.",
            6000
        )

    def _on_program_changed(self) -> None:
        """Refreshes 3D trajectory path when statements change."""
        if hasattr(self, "visualizer_3d"):
            self.visualizer_3d.update_trajectory_path(self.program.statements, self.kinematics)
        if hasattr(self, "sim_panel"):
            self.sim_panel.update_program_stats()

    def _on_execute_robot(self, trajectory: List[List[int]]) -> None:
        """Streams compiled statement path directly to Arduino via SerialWorker."""
        if not self.is_online:
            QMessageBox.warning(
                self,
                "Robotti Offline",
                "Robotti ei ole yhdistetty sarjaporttiin!\n\n"
                "Yhdistä Arduino USB-kaapelilla 'Layout & Connections' -näkymästä ennen fyysistä ajoa.\n"
                "Voit sillä välin testata liikkeen 'Run Simulation' -painikkeella näytöllä."
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
            self.status_bar.showMessage("Liikesarja ajettu onnistuneesti fyysisellä robotilla.")
            return

        frame = self._robot_stream_trajectory[self._robot_stream_index]
        phys = self.calibration.to_physical(frame)
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
