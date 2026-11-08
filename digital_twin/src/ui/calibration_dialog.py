"""
Zero-Offset & Hardware Calibration Dialog for 6-DOF Robot Arm.
Allows fine-tuning mechanical servo alignments without disassembling 3D-printed joints.
"""
from typing import Callable, List, Optional
from src.config import JOINT_NAMES, JOINT_LIMITS
from src.core.calibration import CalibrationManager
from src.ui.qt_compat import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QSpinBox, QCheckBox, QMessageBox, Qt
)


class CalibrationDialog(QDialog):
    """
    Dialog for tuning per-joint zero-trim offsets and direction inversions.
    """
    def __init__(self,
                 calibration: CalibrationManager,
                 on_test_angles_fn: Optional[Callable[[List[int]], None]] = None,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle("Hardware Zero-Offset & Inversion Calibration")
        self.resize(480, 360)
        self.setStyleSheet("""
            QDialog { background: #1e1e1e; color: #d4d4d4; }
            QLabel { color: #cccccc; font-size: 11px; font-weight: 600; }
            QSpinBox {
                background: #252526; color: #ffffff; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 3px 6px; font-weight: 600; font-size: 11px; min-width: 60px;
            }
            QCheckBox { color: #858585; font-size: 11px; }
            QCheckBox:hover { color: #ffffff; }
        """)
        self.calibration = calibration
        self.on_test_angles = on_test_angles_fn
        self.offset_spinboxes: List[QSpinBox] = []
        self.invert_checkboxes: List[QCheckBox] = []

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        title = QLabel("Servo Mechanical Zero-Offset Trim")
        title.setStyleSheet("font-weight: bold; font-size: 13px; color: #4fc1ff;")
        layout.addWidget(title)

        desc = QLabel(
            "Hienosäädä servojen mekaanisia heittoja (±15°). Arvot tallentuvat 'calibration.json'-tiedostoon "
            "eikä servojen muovirattaita tarvitse irrottaa ja kiinnittää uudelleen."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #858585; font-size: 11px;")
        layout.addWidget(desc)

        grid = QGridLayout()
        grid.setSpacing(8)

        grid.addWidget(QLabel("Nivel (Joint)"), 0, 0)
        grid.addWidget(QLabel("Offset (Trim °)"), 0, 1)
        grid.addWidget(QLabel("Invertoi"), 0, 2)

        for i in range(6):
            lbl = QLabel(JOINT_NAMES[i])
            grid.addWidget(lbl, i + 1, 0)

            spin = QSpinBox()
            spin.setRange(-30, 30)
            spin.setValue(self.calibration.get_offset(i))
            spin.setSuffix("°")
            spin.valueChanged.connect(lambda val, idx=i: self.calibration.set_offset(idx, val))
            self.offset_spinboxes.append(spin)
            grid.addWidget(spin, i + 1, 1)

            chk = QCheckBox("Käänteinen")
            chk.setChecked(self.calibration.is_inverted(i))
            chk.toggled.connect(lambda val, idx=i: self.calibration.set_inverted(idx, val))
            self.invert_checkboxes.append(chk)
            grid.addWidget(chk, i + 1, 2)

        layout.addLayout(grid)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_reset = QPushButton("Reset Defaults")
        btn_reset.setStyleSheet("background: #2d2d30; color: #cccccc; padding: 6px 12px; border-radius: 3px;")
        btn_reset.clicked.connect(self._reset_defaults)
        btn_layout.addWidget(btn_reset)

        btn_layout.addStretch()

        btn_save = QPushButton("Save Calibration")
        btn_save.setStyleSheet("background: #007acc; color: white; font-weight: bold; padding: 6px 16px; border-radius: 3px;")
        btn_save.clicked.connect(self._save_calibration)
        btn_layout.addWidget(btn_save)

        btn_close = QPushButton("Sulje")
        btn_close.setStyleSheet("background: #2d2d30; color: #cccccc; padding: 6px 12px; border-radius: 3px;")
        btn_close.clicked.connect(self.accept)
        btn_layout.addWidget(btn_close)

        layout.addLayout(btn_layout)

    def _reset_defaults(self) -> None:
        for i in range(6):
            self.offset_spinboxes[i].setValue(0)
            self.invert_checkboxes[i].setChecked(False)
            self.calibration.set_offset(i, 0)
            self.calibration.set_inverted(i, False)

    def _save_calibration(self) -> None:
        if self.calibration.save():
            QMessageBox.information(self, "Tallennettu", "Kalibrointiasetukset tallennettu tiedostoon 'calibration.json'.")
        else:
            QMessageBox.warning(self, "Virhe", "Kalibrointitiedoston tallennus epäonnistui.")
