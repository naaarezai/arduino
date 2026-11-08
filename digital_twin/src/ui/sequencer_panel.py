"""
Waypoint Sequencer / OLP Panel for teaching, playing, and executing robot motion sequences.
Supports both PTP and LINEAR straight-line motions, trajectory path visualization, and Arduino execution.
"""
from typing import Callable, List, Optional
from src.core.kinematics import RobotKinematics
from src.core.sequencer import Keyframe, Sequencer
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QGroupBox,
    QFileDialog, QMessageBox, QTimer, QRadioButton, QButtonGroup,
    Qt, QColor
)


class SequencerPanel(QGroupBox):
    """
    Panel for teaching robot trajectories (PTP & LINEAR waypoints),
    interpolating them in 3D simulation, visualizing the 3D trajectory path,
    and executing them live on the physical Arduino robot.
    """
    def __init__(self,
                 kinematics: RobotKinematics,
                 get_current_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 on_trajectory_changed_fn: Optional[Callable[[List[Keyframe]], None]] = None,
                 on_execute_robot_fn: Optional[Callable[[List[List[int]]], None]] = None,
                 parent=None):
        super().__init__("Liikesarjojen Opetus (OLP / Sequencer)", parent)
        self.kinematics = kinematics
        self.get_current_angles = get_current_angles_fn
        self.apply_angles = apply_angles_fn
        self.on_trajectory_changed = on_trajectory_changed_fn
        self.on_execute_robot = on_execute_robot_fn

        self.sequencer = Sequencer(kinematics=self.kinematics)

        # Simulation playback timer
        self.anim_timer = QTimer(self)
        self.anim_timer.setInterval(30)  # ~33 FPS
        self.anim_timer.timeout.connect(self._on_anim_tick)

        self._full_trajectory: List[List[int]] = []
        self._traj_frame_index: int = 0

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Mode Selector: PTP vs. LINEAR
        mode_box = QHBoxLayout()
        mode_lbl = QLabel("Uuden pisteen tyyppi:")
        mode_lbl.setStyleSheet("color: #cfd8dc; font-weight: bold; font-size: 11px;")
        mode_box.addWidget(mode_lbl)

        self.radio_ptp = QRadioButton("● PTP (Nopea siirto)")
        self.radio_ptp.setChecked(True)
        self.radio_ptp.setStyleSheet("color: #00bcd4; font-weight: bold; font-size: 11px;")
        self.radio_lin = QRadioButton("━ LINEAR (Suora hitsaussauma)")
        self.radio_lin.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 11px;")

        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.radio_ptp)
        self.mode_group.addButton(self.radio_lin)

        mode_box.addWidget(self.radio_ptp)
        mode_box.addWidget(self.radio_lin)
        mode_box.addStretch()
        layout.addLayout(mode_box)

        # Keyframe Table
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels([
            "#", "Tyyppi", "Nimi", "Kulmat (J1..J6)", "Kesto"
        ])
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.table.setStyleSheet("""
            QTableWidget {
                background: #21252b; color: #eceff1; gridline-color: #2c313a;
                border: 1px solid #3e4451; border-radius: 4px;
            }
            QHeaderView::section {
                background: #282c34; color: #abb2bf; padding: 4px;
                border: 1px solid #3e4451; font-weight: bold;
            }
        """)
        self.table.setMaximumHeight(150)
        layout.addWidget(self.table)

        # Row 1: Add & Delete
        row1 = QHBoxLayout()
        self.btn_add = QPushButton("➕ Tallenna asento")
        self.btn_add.setStyleSheet("background: #0288d1; color: white; padding: 6px; font-weight: bold;")
        self.btn_add.clicked.connect(self._add_current_pose)
        row1.addWidget(self.btn_add)

        self.btn_delete = QPushButton("🗑️ Poista")
        self.btn_delete.setStyleSheet("background: #455a64; color: white; padding: 6px;")
        self.btn_delete.clicked.connect(self._delete_selected)
        row1.addWidget(self.btn_delete)

        self.btn_clear = QPushButton("Tyhjennä")
        self.btn_clear.setStyleSheet("background: #37474f; color: #cfd8dc; padding: 6px;")
        self.btn_clear.clicked.connect(self._clear_all)
        row1.addWidget(self.btn_clear)
        layout.addLayout(row1)

        # Row 2: Playback and Execution
        row2 = QHBoxLayout()
        self.btn_play_sim = QPushButton("▶️ Simuloi ruudulla")
        self.btn_play_sim.setStyleSheet("""
            QPushButton {
                background: #2e7d32; color: white; font-weight: bold; padding: 7px; border-radius: 4px;
            }
            QPushButton:hover { background: #388e3c; }
        """)
        self.btn_play_sim.clicked.connect(self.play_simulation)
        row2.addWidget(self.btn_play_sim)

        self.btn_execute = QPushButton("🦾 Aja robotilla (Arduino)")
        self.btn_execute.setStyleSheet("""
            QPushButton {
                background: #e65100; color: white; font-weight: bold; padding: 7px; border-radius: 4px;
            }
            QPushButton:hover { background: #f57c00; }
        """)
        self.btn_execute.clicked.connect(self.execute_on_robot)
        row2.addWidget(self.btn_execute)

        self.btn_stop = QPushButton("⏹️ Pysäytä")
        self.btn_stop.setStyleSheet("background: #c62828; color: white; font-weight: bold; padding: 7px; border-radius: 4px;")
        self.btn_stop.clicked.connect(self.stop_playback)
        row2.addWidget(self.btn_stop)
        layout.addLayout(row2)

        # Row 3: Save / Load
        row3 = QHBoxLayout()
        self.btn_save = QPushButton("💾 Tallenna sarja (.json)")
        self.btn_save.setStyleSheet("background: #263238; color: #b0bec5; padding: 5px;")
        self.btn_save.clicked.connect(self._save_file)
        row3.addWidget(self.btn_save)

        self.btn_load = QPushButton("📂 Avaa sarja")
        self.btn_load.setStyleSheet("background: #263238; color: #b0bec5; padding: 5px;")
        self.btn_load.clicked.connect(self._load_file)
        row3.addWidget(self.btn_load)
        layout.addLayout(row3)

    def _refresh_table(self) -> None:
        self.table.setRowCount(0)
        for i, kf in enumerate(self.sequencer.keyframes):
            row = self.table.rowCount()
            self.table.insertRow(row)

            item_idx = QTableWidgetItem(str(i + 1))
            item_type = QTableWidgetItem(kf.motion_type)
            item_type.setForeground(QColor("#00bcd4") if kf.motion_type == "PTP" else QColor("#ff9800"))

            item_name = QTableWidgetItem(kf.name)
            item_angles = QTableWidgetItem(str(kf.angles))
            item_dur = QTableWidgetItem(f"{kf.duration_ms} ms")

            self.table.setItem(row, 0, item_idx)
            self.table.setItem(row, 1, item_type)
            self.table.setItem(row, 2, item_name)
            self.table.setItem(row, 3, item_angles)
            self.table.setItem(row, 4, item_dur)

        if self.on_trajectory_changed:
            self.on_trajectory_changed(self.sequencer.keyframes)

    def _add_current_pose(self) -> None:
        angles = self.get_current_angles()
        m_type = "LIN" if self.radio_lin.isChecked() else "PTP"
        self.sequencer.add_keyframe(angles, motion_type=m_type)
        self._refresh_table()

    def _delete_selected(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.sequencer.remove_keyframe(row)
            self._refresh_table()

    def _clear_all(self) -> None:
        self.sequencer.clear()
        self._refresh_table()

    def play_simulation(self) -> None:
        if not self.sequencer.keyframes:
            QMessageBox.information(self, "Huomio", "Lisää ensin vähintään yksi asento liikesarjaan!")
            return

        current_angles = self.get_current_angles()
        self._full_trajectory = self.sequencer.generate_full_trajectory(current_angles)
        self._traj_frame_index = 0
        self.anim_timer.start()

    def execute_on_robot(self) -> None:
        """Triggers live execution of the planned path on physical Arduino."""
        if not self.sequencer.keyframes:
            QMessageBox.information(self, "Huomio", "Lisää ensin liikesarja ennen ajamista!")
            return

        current_angles = self.get_current_angles()
        traj = self.sequencer.generate_full_trajectory(current_angles)
        if self.on_execute_robot:
            self.on_execute_robot(traj)
        # Also play simultaneously on screen!
        self.play_simulation()

    def stop_playback(self) -> None:
        self.anim_timer.stop()

    def _on_anim_tick(self) -> None:
        if self._traj_frame_index >= len(self._full_trajectory):
            self.stop_playback()
            return

        frame_angles = self._full_trajectory[self._traj_frame_index]
        self.apply_angles(frame_angles)
        self._traj_frame_index += 1

    def _save_file(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Tallenna liikesarja", "", "JSON-tiedostot (*.json)")
        if path:
            from pathlib import Path
            self.sequencer.save_to_json(Path(path))

    def _load_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Avaa liikesarja", "", "JSON-tiedostot (*.json)")
        if path:
            from pathlib import Path
            self.sequencer.load_from_json(Path(path))
            self._refresh_table()
