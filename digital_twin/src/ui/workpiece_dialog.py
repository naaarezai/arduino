"""
Workpiece Layout & Positioning Dialog for 6-DOF Robot Arm Digital Twin.
Allows moving, elevating, and rotating the CAD workpiece (STL or demo fixture)
anywhere on the workcell table with live 3D viewport updates.
"""
from typing import Callable, Optional
from src.ui.qt_compat import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QSlider, QSpinBox, QPushButton, QGroupBox, Qt
)


class WorkpieceTransformDialog(QDialog):
    """
    Workpiece layout manager dialog enabling precise Cartesian positioning
    and rotation of the imported workpiece on the robot workcell table.
    """
    def __init__(self,
                 initial_pos: list,
                 initial_rot_z: float,
                 on_transform_changed_callback: Callable[[float, float, float, float], None],
                 on_clear_callback: Optional[Callable[[], None]] = None,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle("Asemoi työkappale (Workpiece Layout)")
        self.resize(380, 360)
        self.setStyleSheet("""
            QDialog { background: #1e1e1e; color: #d4d4d4; }
            QLabel { color: #cccccc; font-size: 11px; font-weight: 600; }
            QGroupBox {
                border: 1px solid #333333; border-radius: 4px;
                margin-top: 8px; font-weight: bold; font-size: 11px; color: #4fc1ff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QSpinBox {
                background: #252526; color: #ffffff; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 4px; font-size: 11px; font-family: Consolas, monospace;
            }
            QSlider::groove:horizontal {
                height: 5px; background: #2d2d30; border-radius: 2px;
            }
            QSlider::sub-page:horizontal { background: #007acc; border-radius: 2px; }
            QSlider::handle:horizontal {
                background: #4fc1ff; width: 14px; margin-top: -5px; margin-bottom: -5px; border-radius: 7px;
            }
            QPushButton {
                background: #252526; color: #cccccc; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 5px 10px; font-size: 11px; font-weight: 600;
            }
            QPushButton:hover { background: #2d2d30; color: #ffffff; }
        """)

        self.on_transform_changed = on_transform_changed_callback
        self.on_clear = on_clear_callback
        self.cur_x = float(initial_pos[0]) if len(initial_pos) > 0 else 260.0
        self.cur_y = float(initial_pos[1]) if len(initial_pos) > 1 else 0.0
        self.cur_z = float(initial_pos[2]) if len(initial_pos) > 2 else 0.0
        self.cur_yaw = float(initial_rot_z)

        self._block_signals = False
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        header = QLabel("📐 Työkappaleen sijainti ja kääntökulma")
        header.setStyleSheet("color: #4fc1ff; font-size: 12px; font-weight: bold;")
        layout.addWidget(header)

        # 1. Transform Grid
        grp_pos = QGroupBox("Pöytäkoordinaatit (mm) & Kulma (°)")
        grid = QGridLayout(grp_pos)
        grid.setSpacing(8)

        # X Position
        grid.addWidget(QLabel("X (Eteen/Taakse):"), 0, 0)
        self.slider_x = QSlider(Qt.Horizontal)
        self.slider_x.setRange(120, 480)
        self.slider_x.setValue(int(self.cur_x))
        grid.addWidget(self.slider_x, 0, 1)

        self.spin_x = QSpinBox()
        self.spin_x.setRange(120, 480)
        self.spin_x.setValue(int(self.cur_x))
        grid.addWidget(self.spin_x, 0, 2)

        # Y Position
        grid.addWidget(QLabel("Y (Vasen/Oikea):"), 1, 0)
        self.slider_y = QSlider(Qt.Horizontal)
        self.slider_y.setRange(-250, 250)
        self.slider_y.setValue(int(self.cur_y))
        grid.addWidget(self.slider_y, 1, 1)

        self.spin_y = QSpinBox()
        self.spin_y.setRange(-250, 250)
        self.spin_y.setValue(int(self.cur_y))
        grid.addWidget(self.spin_y, 1, 2)

        # Z Position
        grid.addWidget(QLabel("Z (Korkeus pöydästä):"), 2, 0)
        self.slider_z = QSlider(Qt.Horizontal)
        self.slider_z.setRange(0, 150)
        self.slider_z.setValue(int(self.cur_z))
        grid.addWidget(self.slider_z, 2, 1)

        self.spin_z = QSpinBox()
        self.spin_z.setRange(0, 150)
        self.spin_z.setValue(int(self.cur_z))
        grid.addWidget(self.spin_z, 2, 2)

        # Yaw Rotation
        grid.addWidget(QLabel("Kääntö (Yaw Z):"), 3, 0)
        self.slider_yaw = QSlider(Qt.Horizontal)
        self.slider_yaw.setRange(0, 360)
        self.slider_yaw.setValue(int(self.cur_yaw))
        grid.addWidget(self.slider_yaw, 3, 1)

        self.spin_yaw = QSpinBox()
        self.spin_yaw.setRange(0, 360)
        self.spin_yaw.setValue(int(self.cur_yaw))
        grid.addWidget(self.spin_yaw, 3, 2)

        layout.addWidget(grp_pos)

        # Sync signals
        self.slider_x.valueChanged.connect(self._on_x_slider)
        self.spin_x.valueChanged.connect(self._on_x_spin)
        self.slider_y.valueChanged.connect(self._on_y_slider)
        self.spin_y.valueChanged.connect(self._on_y_spin)
        self.slider_z.valueChanged.connect(self._on_z_slider)
        self.spin_z.valueChanged.connect(self._on_z_spin)
        self.slider_yaw.valueChanged.connect(self._on_yaw_slider)
        self.spin_yaw.valueChanged.connect(self._on_yaw_spin)

        # 2. Quick Presets
        grp_preset = QGroupBox("Pika-asemat (Presets)")
        row_pre = QHBoxLayout(grp_preset)
        row_pre.setSpacing(4)

        btn_center = QPushButton("Keskitä (X=260)")
        btn_center.clicked.connect(lambda: self._set_values(260, 0, self.cur_z, 0))
        row_pre.addWidget(btn_center)

        btn_left = QPushButton("Vasen (Y=-80)")
        btn_left.clicked.connect(lambda: self._set_values(self.cur_x, -80, self.cur_z, self.cur_yaw))
        row_pre.addWidget(btn_left)

        btn_right = QPushButton("Oikea (Y=+80)")
        btn_right.clicked.connect(lambda: self._set_values(self.cur_x, 80, self.cur_z, self.cur_yaw))
        row_pre.addWidget(btn_right)

        btn_rot90 = QPushButton("Käännä +90°")
        btn_rot90.clicked.connect(lambda: self._set_values(self.cur_x, self.cur_y, self.cur_z, (self.cur_yaw + 90) % 360))
        row_pre.addWidget(btn_rot90)
        layout.addWidget(grp_preset)

        layout.addStretch()

        # 3. Actions Row
        row_act = QHBoxLayout()
        if self.on_clear:
            btn_rem = QPushButton("Poista kappale")
            btn_rem.setStyleSheet("background: #5a1e1e; color: #ffcccc; border-color: #7d2e2e;")
            btn_rem.clicked.connect(self._remove_workpiece)
            row_act.addWidget(btn_rem)

        row_act.addStretch()

        btn_close = QPushButton("Valmis / Sulje")
        btn_close.setStyleSheet("background: #007acc; color: white; font-weight: bold; padding: 6px 16px;")
        btn_close.clicked.connect(self.accept)
        row_act.addWidget(btn_close)
        layout.addLayout(row_act)

    def _set_values(self, x: float, y: float, z: float, yaw: float) -> None:
        self._block_signals = True
        self.cur_x = x
        self.cur_y = y
        self.cur_z = z
        self.cur_yaw = yaw
        self.slider_x.setValue(int(x))
        self.spin_x.setValue(int(x))
        self.slider_y.setValue(int(y))
        self.spin_y.setValue(int(y))
        self.slider_z.setValue(int(z))
        self.spin_z.setValue(int(z))
        self.slider_yaw.setValue(int(yaw))
        self.spin_yaw.setValue(int(yaw))
        self._block_signals = False
        self._emit_change()

    def _on_x_slider(self, val: int) -> None:
        if self._block_signals: return
        self.cur_x = float(val)
        self.spin_x.setValue(val)
        self._emit_change()

    def _on_x_spin(self, val: int) -> None:
        if self._block_signals: return
        self.cur_x = float(val)
        self.slider_x.setValue(val)
        self._emit_change()

    def _on_y_slider(self, val: int) -> None:
        if self._block_signals: return
        self.cur_y = float(val)
        self.spin_y.setValue(val)
        self._emit_change()

    def _on_y_spin(self, val: int) -> None:
        if self._block_signals: return
        self.cur_y = float(val)
        self.slider_y.setValue(val)
        self._emit_change()

    def _on_z_slider(self, val: int) -> None:
        if self._block_signals: return
        self.cur_z = float(val)
        self.spin_z.setValue(val)
        self._emit_change()

    def _on_z_spin(self, val: int) -> None:
        if self._block_signals: return
        self.cur_z = float(val)
        self.slider_z.setValue(val)
        self._emit_change()

    def _on_yaw_slider(self, val: int) -> None:
        if self._block_signals: return
        self.cur_yaw = float(val)
        self.spin_yaw.setValue(val)
        self._emit_change()

    def _on_yaw_spin(self, val: int) -> None:
        if self._block_signals: return
        self.cur_yaw = float(val)
        self.slider_yaw.setValue(val)
        self._emit_change()

    def _emit_change(self) -> None:
        if self.on_transform_changed:
            self.on_transform_changed(self.cur_x, self.cur_y, self.cur_z, self.cur_yaw)

    def _remove_workpiece(self) -> None:
        if self.on_clear:
            self.on_clear()
        self.accept()
