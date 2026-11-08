"""
Simulation Control Panel (Visual Components style).
Provides transport controls (Play, Pause, Stop), speed multipliers (0.5x, 1x, 2x, 4x),
timeline progress, and Arduino execution.
"""
from typing import Callable, List, Optional
import numpy as np
from src.core.program import RobotProgram, TrajectoryFrame
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QProgressBar, QSlider, QGroupBox, QTimer, Qt, QCheckBox
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
                 on_weld_tick_fn: Optional[Callable[[List[int]], None]] = None,
                 on_clear_weld_fn: Optional[Callable[[], None]] = None,
                 parent=None):
        super().__init__("Simulation Execution & Analysis", parent)
        self.program = program
        self.get_angles = get_angles_fn
        self.apply_angles = apply_angles_fn
        self.on_frame_tick = on_frame_tick_fn
        self.on_execute_robot = on_execute_robot_fn
        self.on_weld_tick = on_weld_tick_fn
        self.on_clear_weld = on_clear_weld_fn

        self.speed_multiplier: float = 1.0
        self.is_paused: bool = False
        self.cycle_count: int = 0
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

        # 3. Interactive Timeline Scrubber & Progress Bar
        time_hdr = QHBoxLayout()
        lbl_timeline = QLabel("Aikajana (Scrubber):")
        lbl_timeline.setStyleSheet("color: #858585; font-size: 11px; font-weight: 600;")
        time_hdr.addWidget(lbl_timeline)
        self.lbl_time_val = QLabel("0.0 s / 0.0 s")
        self.lbl_time_val.setStyleSheet("color: #4fc1ff; font-family: Consolas, monospace; font-size: 11px;")
        time_hdr.addStretch()
        time_hdr.addWidget(self.lbl_time_val)
        layout.addLayout(time_hdr)

        self.timeline_slider = QSlider(Qt.Horizontal)
        self.timeline_slider.setRange(0, 100)
        self.timeline_slider.setValue(0)
        self.timeline_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px; background: #181818; border: 1px solid #333333; border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #007acc; border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #4fc1ff; border: 1px solid #ffffff; width: 14px;
                margin-top: -4px; margin-bottom: -4px; border-radius: 7px;
            }
            QSlider::handle:horizontal:hover { background: #ffffff; }
        """)
        self.timeline_slider.sliderMoved.connect(self._on_slider_scrub)
        layout.addWidget(self.timeline_slider)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                background: #181818; border: 1px solid #333333; border-radius: 2px; height: 6px;
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

        # 4. Production Loop, Dry Run & Weld Trace Controls
        row_prod = QHBoxLayout()
        row_prod.setSpacing(6)
        self.chk_loop = QCheckBox("Loop (Jatkuva)")
        self.chk_loop.setStyleSheet("color: #d4d4d4; font-size: 11px; font-weight: 600;")
        row_prod.addWidget(self.chk_loop)

        self.chk_dry_run = QCheckBox("🛡 Turva-ajo (Dry Run)")
        self.chk_dry_run.setToolTip("Aja 25% nopeudella ilman hitsauspolttimen/kouran laukaisua")
        self.chk_dry_run.setStyleSheet("color: #69f0ae; font-size: 11px; font-weight: 600;")
        self.chk_dry_run.toggled.connect(self._on_dry_run_toggled)
        row_prod.addWidget(self.chk_dry_run)

        self.lbl_cycles = QLabel("Cycles: 0")
        self.lbl_cycles.setStyleSheet("color: #4fc1ff; font-family: Consolas, monospace; font-size: 11px; font-weight: bold;")
        row_prod.addWidget(self.lbl_cycles)

        row_prod.addStretch()

        self.btn_clear_weld = QPushButton("Tyhjennä saumat")
        self.btn_clear_weld.setStyleSheet("""
            QPushButton {
                background: #252526; color: #ff9100; font-size: 11px; font-weight: 600;
                border: 1px solid #ff9100; border-radius: 3px; padding: 3px 8px;
            }
            QPushButton:hover { background: #3f2a15; }
        """)
        self.btn_clear_weld.clicked.connect(self._clear_weld_trace)
        row_prod.addWidget(self.btn_clear_weld)
        layout.addLayout(row_prod)

        # 5. Big "Execute on Robot (Arduino)" Button
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
        """Calculates estimated cycle time and displays summary with welding seam length."""
        if not self.program.statements:
            self.lbl_status.setText("Valmis simulaatioon (0 käskyä).")
            self.lbl_time_val.setText("0.0 s / 0.0 s")
            return

        current_angles = self.get_angles()
        traj = self.program.generate_full_trajectory(current_angles, speed_multiplier=1.0)
        est_seconds = len(traj) * 0.035

        # Calculate total weld seam distance (mm)
        weld_len_mm = 0.0
        prev_pt = None
        for stmt in self.program.statements:
            if hasattr(stmt, "angles"):
                pose = self.program.kinematics.forward_kinematics([float(a) for a in stmt.angles])
                cur_pt = np.array([pose.tcp_x, pose.tcp_y, pose.tcp_z], dtype=np.float64)
                if hasattr(stmt, "statement_type") and stmt.statement_type == "LIN" and prev_pt is not None:
                    weld_len_mm += float(np.linalg.norm(cur_pt - prev_pt))
                prev_pt = cur_pt

        self.lbl_time_val.setText(f"0.0 s / {est_seconds:.1f} s")
        self.lbl_status.setText(
            f"⏱ Sykliaika: {est_seconds:.1f} s | ⚡ Saumapituus: {weld_len_mm:.1f} mm | {len(self.program.statements)} käskyä ({len(traj)} askelta)"
        )

    def _on_slider_scrub(self, value: int) -> None:
        """Allows scrubbing through simulation trajectory frame by frame with slider."""
        if not self._trajectory:
            current_angles = self.get_angles()
            self._trajectory = self.program.generate_full_trajectory(current_angles, speed_multiplier=1.0)
        if not self._trajectory:
            return

        idx = int((value / 100.0) * (len(self._trajectory) - 1))
        self._current_frame_idx = idx
        frame = self._trajectory[idx]
        self.apply_angles(frame.angles)
        if self.on_frame_tick:
            self.on_frame_tick(frame.statement_idx)
        cur_sec = idx * 0.035
        tot_sec = len(self._trajectory) * 0.035
        self.lbl_time_val.setText(f"{cur_sec:.1f} s / {tot_sec:.1f} s")
        self.progress_bar.setValue(value)
        self.lbl_status.setText(f"Scrub: {frame.statement_label} ({value}%)")

    def _on_dry_run_toggled(self, checked: bool) -> None:
        """Toggles 25% safety velocity for dry-run verification."""
        if checked:
            self._set_speed(0.25)
            self.lbl_status.setText("🛡 Turva-ajo aktivoitu: 25% nopeus.")
        else:
            self._set_speed(1.0)
            self.lbl_status.setText("Normaali ajo (1.0x).")

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

    def _clear_weld_trace(self) -> None:
        if self.on_clear_weld:
            self.on_clear_weld()

    def _on_tick(self) -> None:
        if self._current_frame_idx >= len(self._trajectory):
            self.cycle_count += 1
            self.lbl_cycles.setText(f"Cycles: {self.cycle_count}")
            if self.chk_loop.isChecked():
                self._current_frame_idx = 0
                self.progress_bar.setValue(0)
                return
            self.stop_simulation()
            self.lbl_status.setText(f"Simulaatiosykli valmis. (Sykli {self.cycle_count})")
            return

        frame = self._trajectory[self._current_frame_idx]
        self.apply_angles(frame.angles)

        if frame.is_welding and self.on_weld_tick and not self.chk_dry_run.isChecked():
            self.on_weld_tick(frame.angles)

        # Update progress and statement highlight
        prog_pct = int((self._current_frame_idx + 1) * 100 / len(self._trajectory))
        self.progress_bar.setValue(prog_pct)
        self.timeline_slider.setValue(prog_pct)
        cur_sec = (self._current_frame_idx + 1) * 0.035
        tot_sec = len(self._trajectory) * 0.035
        self.lbl_time_val.setText(f"{cur_sec:.1f} s / {tot_sec:.1f} s")
        welding_tag = " [SEAM ACTIVE]" if frame.is_welding and not self.chk_dry_run.isChecked() else ""
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

