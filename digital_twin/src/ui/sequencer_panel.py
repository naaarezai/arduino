"""
Waypoint Sequencer / OLP Panel for teaching, playing, and executing robot motion sequences.
"""
from typing import Callable, List
from src.core.sequencer import Keyframe, Sequencer
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QGroupBox,
    QFileDialog, QMessageBox, QTimer
)


class SequencerPanel(QGroupBox):
    """
    Panel for teaching robot trajectories (keyframe poses),
    interpolating them smoothly in 3D simulation or on physical hardware.
    """
    def __init__(self,
                 get_current_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 parent=None):
        super().__init__("Liikesarjojen Opetus (OLP / Sequencer)", parent)
        self.get_current_angles = get_current_angles_fn
        self.apply_angles = apply_angles_fn

        self.sequencer = Sequencer()

        # Playback animation timer
        self.anim_timer = QTimer(self)
        self.anim_timer.setInterval(25)  # 40 FPS playback
        self.anim_timer.timeout.connect(self._on_anim_tick)

        # Playback state
        self._playback_step: int = 0
        self._playback_progress: float = 0.0
        self._current_start_angles: List[int] = []
        self._current_target_angles: List[int] = []
        self._is_paused: bool = False
        self._pause_remaining_ms: float = 0.0

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Keyframe Table
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels([
            "#", "Nimi", "Kulmat (J1..J6)", "Kesto", "Viive", "", "", ""
        ])
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.table.setStyleSheet("""
            QTableWidget {
                background: #21252b;
                color: #eceff1;
                gridline-color: #2c313a;
                border: 1px solid #3e4451;
                border-radius: 4px;
            }
            QHeaderView::section {
                background: #282c34;
                color: #abb2bf;
                padding: 4px;
                border: 1px solid #3e4451;
                font-weight: bold;
            }
        """)
        self.table.setMaximumHeight(160)
        layout.addWidget(self.table)

        # Top buttons row: Record & Delete
        btn_row1 = QHBoxLayout()
        self.btn_add = QPushButton("➕ Tallenna nykyinen asento")
        self.btn_add.setStyleSheet("background: #0288d1; color: white; padding: 6px; font-weight: bold;")
        self.btn_add.clicked.connect(self._add_current_pose)
        btn_row1.addWidget(self.btn_add)

        self.btn_delete = QPushButton("🗑️ Poista valittu")
        self.btn_delete.setStyleSheet("background: #455a64; color: white; padding: 6px;")
        self.btn_delete.clicked.connect(self._delete_selected)
        btn_row1.addWidget(self.btn_delete)

        self.btn_clear = QPushButton("Tyhjennä")
        self.btn_clear.setStyleSheet("background: #37474f; color: #cfd8dc; padding: 6px;")
        self.btn_clear.clicked.connect(self._clear_all)
        btn_row1.addWidget(self.btn_clear)
        layout.addLayout(btn_row1)

        # Execution buttons row
        btn_row2 = QHBoxLayout()
        self.btn_play_sim = QPushButton("▶️ Simuloi ruudulla")
        self.btn_play_sim.setStyleSheet("background: #43a047; color: white; font-weight: bold; padding: 7px;")
        self.btn_play_sim.clicked.connect(self.play_simulation)
        btn_row2.addWidget(self.btn_play_sim)

        self.btn_stop = QPushButton("⏹️ Pysäytä")
        self.btn_stop.setStyleSheet("background: #d32f2f; color: white; font-weight: bold; padding: 7px;")
        self.btn_stop.clicked.connect(self.stop_playback)
        btn_row2.addWidget(self.btn_stop)
        layout.addLayout(btn_row2)

        # File load/save row
        btn_row3 = QHBoxLayout()
        self.btn_save = QPushButton("💾 Tallenna sarja (.json)")
        self.btn_save.setStyleSheet("background: #263238; color: #b0bec5; padding: 5px;")
        self.btn_save.clicked.connect(self._save_file)
        btn_row3.addWidget(self.btn_save)

        self.btn_load = QPushButton("📂 Avaa sarja")
        self.btn_load.setStyleSheet("background: #263238; color: #b0bec5; padding: 5px;")
        self.btn_load.clicked.connect(self._load_file)
        btn_row3.addWidget(self.btn_load)
        layout.addLayout(btn_row3)

    def _refresh_table(self) -> None:
        self.table.setRowCount(0)
        for i, kf in enumerate(self.sequencer.keyframes):
            row = self.table.rowCount()
            self.table.insertRow(row)

            item_idx = QTableWidgetItem(str(i + 1))
            item_name = QTableWidgetItem(kf.name)
            item_angles = QTableWidgetItem(str(kf.angles))
            item_dur = QTableWidgetItem(f"{kf.duration_ms} ms")
            item_pause = QTableWidgetItem(f"{kf.pause_ms} ms")

            self.table.setItem(row, 0, item_idx)
            self.table.setItem(row, 1, item_name)
            self.table.setItem(row, 2, item_angles)
            self.table.setItem(row, 3, item_dur)
            self.table.setItem(row, 4, item_pause)

    def _add_current_pose(self) -> None:
        angles = self.get_current_angles()
        self.sequencer.add_keyframe(angles)
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

        self._playback_step = 0
        self._playback_progress = 0.0
        self._is_paused = False
        self._current_start_angles = list(self.get_current_angles())
        self._current_target_angles = list(self.sequencer.keyframes[0].angles)
        self.anim_timer.start()

    def stop_playback(self) -> None:
        self.anim_timer.stop()

    def _on_anim_tick(self) -> None:
        if self._playback_step >= len(self.sequencer.keyframes):
            self.stop_playback()
            return

        current_kf = self.sequencer.keyframes[self._playback_step]
        step_dt_ms = self.anim_timer.interval()

        if self._is_paused:
            self._pause_remaining_ms -= step_dt_ms
            if self._pause_remaining_ms <= 0:
                self._is_paused = False
                self._playback_step += 1
                if self._playback_step < len(self.sequencer.keyframes):
                    self._current_start_angles = list(current_kf.angles)
                    self._current_target_angles = list(self.sequencer.keyframes[self._playback_step].angles)
                    self._playback_progress = 0.0
                else:
                    self.stop_playback()
            return

        # Motion interpolation
        duration_ms = max(200, current_kf.duration_ms)
        self._playback_progress += step_dt_ms / duration_ms

        if self._playback_progress >= 1.0:
            # Reached target pose
            self._playback_progress = 1.0
            self.apply_angles(self._current_target_angles)
            self._is_paused = True
            self._pause_remaining_ms = float(current_kf.pause_ms)
        else:
            interp = Sequencer.interpolate_angles(
                self._current_start_angles,
                self._current_target_angles,
                self._playback_progress
            )
            self.apply_angles(interp)

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
