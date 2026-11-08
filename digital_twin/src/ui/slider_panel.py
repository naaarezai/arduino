"""
Joint Sliders and Controls Panel for 6-DOF Robot Arm Digital Twin.
Provides fine-tune sliders, numerical spinboxes, preset buttons, and joint direction inversions.
"""
from typing import List, Tuple
from src.config import DEFAULT_HOME_ANGLES, JOINT_LIMITS, JOINT_NAMES
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QSlider, QSpinBox, QPushButton, QGroupBox, QCheckBox, Qt
)


class SliderPanel(QGroupBox):
    """
    Control card with 6 sliders, spinboxes, and presets for J1–J6.
    Emits angles_changed(list[int]) whenever an angle is adjusted.
    """
    def __init__(self, on_angles_changed_callback, parent=None):
        super().__init__("Nivelten Ohjaus (Joint Control)", parent)
        self.on_angles_changed = on_angles_changed_callback

        self.sliders: List[QSlider] = []
        self.spinboxes: List[QSpinBox] = []
        self.invert_checkboxes: List[QCheckBox] = []
        self._block_signals: bool = False

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        grid = QGridLayout()
        grid.setSpacing(8)

        for i in range(6):
            name = JOINT_NAMES[i]
            min_val, max_val = JOINT_LIMITS[i]
            default_val = DEFAULT_HOME_ANGLES[i]

            # Label
            lbl = QLabel(name)
            lbl.setStyleSheet("font-weight: bold; color: #cfd8dc;")

            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue(default_val)
            slider.setStyleSheet("""
                QSlider::groove:horizontal {
                    height: 8px;
                    background: #263238;
                    border-radius: 4px;
                }
                QSlider::sub-page:horizontal {
                    background: #00bcd4;
                    border-radius: 4px;
                }
                QSlider::handle:horizontal {
                    background: #ffffff;
                    width: 18px;
                    margin-top: -5px;
                    margin-bottom: -5px;
                    border-radius: 9px;
                }
                QSlider::handle:horizontal:hover {
                    background: #80deea;
                }
            """)

            # SpinBox
            spin = QSpinBox()
            spin.setRange(min_val, max_val)
            spin.setValue(default_val)
            spin.setSuffix("°")
            spin.setStyleSheet("""
                QSpinBox {
                    background: #263238;
                    color: #ffffff;
                    border: 1px solid #37474f;
                    border-radius: 4px;
                    padding: 3px 6px;
                    font-weight: bold;
                    min-width: 55px;
                }
            """)

            # Invert checkbox
            chk_inv = QCheckBox("Inv")
            chk_inv.setToolTip(f"Käännä servon {i+1} pyörimissuunta (180 - kulma)")
            chk_inv.setStyleSheet("color: #90a4ae; font-size: 11px;")
            if i == 1:  # Shoulder is inverted by default to match arm mechanics
                chk_inv.setChecked(True)

            # Store references
            self.sliders.append(slider)
            self.spinboxes.append(spin)
            self.invert_checkboxes.append(chk_inv)

            # Connect events
            slider.valueChanged.connect(lambda val, idx=i: self._on_slider_changed(idx, val))
            spin.valueChanged.connect(lambda val, idx=i: self._on_spin_changed(idx, val))
            chk_inv.stateChanged.connect(lambda _, idx=i: self._emit_angles())

            # Layout row
            grid.addWidget(lbl, i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(spin, i, 2)
            grid.addWidget(chk_inv, i, 3)

        layout.addLayout(grid)

        # Preset buttons row
        presets_box = QGroupBox("Pika-asennot (Presets)")
        presets_layout = QHBoxLayout(presets_box)
        presets_layout.setContentsMargins(4, 8, 4, 8)

        presets = [
            ("🏠 Koti (Home)", [90, 90, 90, 90, 90, 40]),
            ("⚡ Valmis (Ready)", [90, 60, 120, 90, 90, 40]),
            ("🎯 Kurotus", [90, 45, 45, 90, 90, 100]),
            ("📦 Poiminta", [90, 110, 130, 40, 90, 128]),
        ]

        for title, angles in presets:
            btn = QPushButton(title)
            btn.setStyleSheet("""
                QPushButton {
                    background: #37474f;
                    color: #eceff1;
                    border: none;
                    border-radius: 4px;
                    padding: 6px 10px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background: #455a64;
                    color: #00e5ff;
                }
                QPushButton:pressed {
                    background: #263238;
                }
            """)
            btn.clicked.connect(lambda _, a=angles: self.set_angles(a))
            presets_layout.addWidget(btn)

        layout.addWidget(presets_box)

    def _on_slider_changed(self, index: int, value: int) -> None:
        if self._block_signals:
            return
        self._block_signals = True
        self.spinboxes[index].setValue(value)
        self._block_signals = False
        self._emit_angles()

    def _on_spin_changed(self, index: int, value: int) -> None:
        if self._block_signals:
            return
        self._block_signals = True
        self.sliders[index].setValue(value)
        self._block_signals = False
        self._emit_angles()

    def _emit_angles(self) -> None:
        if self.on_angles_changed:
            self.on_angles_changed(self.get_angles(), self.get_physical_angles())

    def get_angles(self) -> List[int]:
        """Returns standard simulation joint angles J1..J6."""
        return [s.value() for s in self.sliders]

    def get_physical_angles(self) -> List[int]:
        """
        Returns angles mapped for the physical hardware (inversions applied).
        If inverted: physical_angle = 180 - angle.
        """
        angles = self.get_angles()
        phys = []
        for i, val in enumerate(angles):
            if self.invert_checkboxes[i].isChecked():
                phys.append(180 - val)
            else:
                phys.append(val)
        return phys

    def set_angles(self, angles: List[int]) -> None:
        """Sets all sliders and spinboxes simultaneously."""
        self._block_signals = True
        for i in range(min(6, len(angles))):
            min_v, max_v = JOINT_LIMITS[i]
            val = max(min_v, min(max_v, int(angles[i])))
            self.sliders[i].setValue(val)
            self.spinboxes[i].setValue(val)
        self._block_signals = False
        self._emit_angles()
