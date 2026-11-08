"""
Simulation Control Panel (Visual Components style).
Provides transport controls (Play, Pause, Stop), speed multipliers (0.5x, 1x, 2x, 4x),
timeline progress, and Arduino execution.
"""
from typing import Callable, List, Optional
from src.core.program import RobotProgram, TrajectoryFrame
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QProgressBar, QSlider, QGroupBox, QTimer, Qt
)


class SimulationControlPanel(QGroupBox):
    """
    Simulation Transport & Execution panel inspired by Visual Components Simulation tab.
    """
    def __init__(self,
                 program: RobotProgram,
                 get_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 on_frame_tick_fn: Optional[Callable[[int], None]] = None,
                 on_execute_robot_fn: Optional[Callable[[List[List[int]]], None]] = None,
                 parent=None):
        super().__init__("Simulation Execution & Analysis", parent)
        self.program = program
        self.get_angles = get_angles_fn
        self.apply_angles = apply_angles_fn
        self.on_frame_tick = on_frame_tick_fn
        self.on_execute_robot = on_execute_robot_fn

        self.speed_multiplier: float = 1.0
        self.is_paused: bool = False
        self._trajectory: List[TrajectoryFrame] = []
        self._current_frame_idx: int = 0

        # Animation timer (35 ms base tick)
        self.timer = QTimer(self)
        self.timer.setInterval(35)
        self.timer.timeout.connect(self._on_tick)

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 1. Main Transport Controls
        row_play = QHBoxLayout()
        row_play.setSpacing(6)

        btn_style_play = """
            QPushButton {
                background: #1e5a22; color: #ffffff; font-weight: 600; font-size: 12px;
                border: 1px solid #2e7d32; border-radius: 3px; padding: 7px 14px;
            }
            QPushButton:hover { background: #2e7d32; }
            QPushButton:pressed { background: #143d17; }
        """
        btn_style_pause = """
            QPushButton {
                background: #6e4b00; color: #ffffff; font-weight: 600; font-size: 12px;
                border: 1px solid #996800; border-radius: 3px; padding: 7px 14px;
            }
            QPushButton:hover { background: #996800; }
        """
        btn_style_stop = """
            QPushButton {
                background: #5a1e1e; color: #ffffff; font-weight: 600; font-size: 12px;
                border: 1px solid #7d2e2e; border-radius: 3px; padding: 7px 14px;
            }
            QPushButton:hover { background: #7d2e2e; }
        """

        self.btn_play = QPushButton("Run Simulation")
        self.btn_play.setStyleSheet(btn_style_play)
        self.btn_play.clicked.connect(self.play_simulation)
        row_play.addWidget(self.btn_play)

        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setStyleSheet(btn_style_pause)
        self.btn_pause.clicked.connect(self.pause_simulation)
        row_play.addWidget(self.btn_pause)

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet(btn_style_stop)
        self.btn_stop.clicked.connect(self.stop_simulation)
        row_play.addWidget(self.btn_stop)

        layout.addLayout(row_play)

        # 2. Speed Multipliers Row
        row_speed = QHBoxLayout()
        row_speed.setSpacing(4)
        lbl_speed = QLabel("Simulation Speed:")
        lbl_speed.setStyleSheet("color: #858585; font-size: 11px; font-weight: 600;")
        row_speed.addWidget(lbl_speed)

        self.speed_buttons: List[QPushButton] = []
        for s in (0.5, 1.0, 2.0, 4.0):
            btn = QPushButton(f"{s:g}x")
            btn.setCheckable(True)
            if s == 1.0:
                btn.setChecked(True)
            btn.setStyleSheet("""
                QPushButton {
                    background: #252526; color: #cccccc; border: 1px solid #3f3f46;
                    border-radius: 3px; padding: 3px 8px; font-size: 11px; font-weight: 600;
                }
                QPushButton:checked {
                    background: #007acc; color: white; border-color: #0098ff;
                }
                QPushButton:hover:!checked { background: #2d2d30; color: #ffffff; }
            """)
            btn.clicked.connect(lambda _, val=s: self._set_speed(val))
            self.speed_buttons.append(btn)
            row_speed.addWidget(btn)

        row_speed.addStretch()
        layout.addLayout(row_speed)

        # 3. Timeline / Progress Bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                background: #181818; border: 1px solid #333333; border-radius: 2px; height: 10px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #007acc, stop:1 #4fc1ff);
                border-radius: 1px;
            }
        """)
        layout.addWidget(self.progress_bar)

        self.lbl_status = QLabel("Ready for simulation (0 statements).")
        self.lbl_status.setStyleSheet("color: #858585; font-size: 11px;")
        layout.addWidget(self.lbl_status)

        # 4. Big "Execute on Robot (Arduino)" Button
        self.btn_execute_robot = QPushButton("Execute Routine on Arduino Hardware")
        self.btn_execute_robot.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #a04e00, stop:1 #c56200);
                color: #ffffff; font-weight: 600; font-size: 12px;
                border: 1px solid #e67e22; border-radius: 4px; padding: 9px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #b85a00, stop:1 #d96d00);
            }
            QPushButton:pressed { background: #783a00; }
        """)
        self.btn_execute_robot.clicked.connect(self._execute_on_robot)
        layout.addWidget(self.btn_execute_robot)

    def _set_speed(self, multiplier: float) -> None:
        self.speed_multiplier = multiplier
        for btn in self.speed_buttons:
            btn.setChecked(btn.text() == f"{multiplier:g}x")
        # Adjust timer interval
        base_interval = 35
        self.timer.setInterval(max(10, int(base_interval / multiplier)))

    def update_program_stats(self) -> None:
        """Calculates estimated cycle time and displays summary."""
        if not self.program.statements:
            self.lbl_status.setText("Valmis simulaatioon (0 käskyä).")
            return

        current_angles = self.get_angles()
        traj = self.program.generate_full_trajectory(current_angles, speed_multiplier=1.0)
        est_seconds = len(traj) * 0.035
        self.lbl_status.setText(f"Arvioitu sykliaika: {est_seconds:.1f} s | {len(self.program.statements)} käskyä | {len(traj)} interpolaatioaskelta")

    def play_simulation(self) -> None:
        if not self.program.statements:
            self.lbl_status.setText("MainRoutine on tyhjä — lisää käskyjä Program-näkymässä.")
            return

        if self.is_paused:
            self.is_paused = False
            self.timer.start()
            self.lbl_status.setText("Simulaatio jatkuu...")
            return

        current_angles = self.get_angles()
        self._trajectory = self.program.generate_full_trajectory(current_angles, speed_multiplier=1.0)
        if not self._trajectory:
            return

        self._current_frame_idx = 0
        self.is_paused = False
        self.timer.start()
        est_seconds = len(self._trajectory) * 0.035 / self.speed_multiplier
        self.lbl_status.setText(f"Simuloidaan: {len(self._trajectory)} kehystä (~{est_seconds:.1f} s, {self.speed_multiplier:g}x)...")

    def pause_simulation(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
            self.is_paused = True
            self.lbl_status.setText("Simulaatio keskeytetty (Pause).")

    def stop_simulation(self) -> None:
        self.timer.stop()
        self.is_paused = False
        self._current_frame_idx = 0
        self.progress_bar.setValue(0)
        self.lbl_status.setText("Simulaatio pysäytetty (Stop).")
        if self.on_frame_tick:
            self.on_frame_tick(-1)

    def _on_tick(self) -> None:
        if self._current_frame_idx >= len(self._trajectory):
            self.stop_simulation()
            self.lbl_status.setText("Simulaatiosykli valmis.")
            return

        frame = self._trajectory[self._current_frame_idx]
        self.apply_angles(frame.angles)

        # Update progress and statement highlight
        prog_pct = int((self._current_frame_idx + 1) * 100 / len(self._trajectory))
        self.progress_bar.setValue(prog_pct)
        welding_tag = " [SEAM ACTIVE]" if frame.is_welding else ""
        self.lbl_status.setText(f"Ajo: {frame.statement_label}{welding_tag} ({prog_pct}%)")

        if self.on_frame_tick:
            self.on_frame_tick(frame.statement_idx)

        self._current_frame_idx += 1

    def _execute_on_robot(self) -> None:
        if not self.program.statements:
            self.lbl_status.setText("Käskyjä ei ole määritetty — opeta liikeradat ensin.")
            return

        current_angles = self.get_angles()
        traj_frames = self.program.generate_full_trajectory(current_angles, speed_multiplier=1.0)
        angle_list = [f.angles for f in traj_frames]

        if self.on_execute_robot:
            self.on_execute_robot(angle_list)
        # Also play simulation visually!
        self.play_simulation()

