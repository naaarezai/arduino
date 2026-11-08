"""
Compact Jog Panel for Robot Joint and Cartesian 3D Jogging (Visual Components style).
Replaces huge sliders with clean step buttons (+/-), direct numerical entry,
and Cartesian XYZ IK jogging with 3D Gizmo toggle.
"""
from typing import Callable, List, Optional
from src.config import JOINT_LIMITS, JOINT_NAMES
from src.core.kinematics import RobotKinematics
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QSpinBox, QGroupBox, QTabWidget, QCheckBox,
    Qt, QColor
)


class CompactJogPanel(QGroupBox):
    """
    Clean, space-efficient Jogging panel providing Joint (+/-) and Cartesian XYZ jogging
    plus a toggle for 3D Viewport Interactive Drag Gizmo.
    """
    def __init__(self,
                 kinematics: RobotKinematics,
                 get_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 on_toggle_gizmo_fn: Optional[Callable[[bool], None]] = None,
                 parent=None):
        super().__init__("Nivel- ja Avaruusajo (Jog Panel)", parent)
        self.kinematics = kinematics
        self.get_angles = get_angles_fn
        self.apply_angles = apply_angles_fn
        self.on_toggle_gizmo = on_toggle_gizmo_fn

        self.spinboxes: List[QSpinBox] = []
        self.invert_checkboxes: List[QCheckBox] = []
        self._block_signals = False

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # Tab widget: Joints vs Cartesian XYZ
        self.jog_tabs = QTabWidget()
        self.jog_tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #37474f; background: #21252b; border-radius: 4px; }
            QTabBar::tab {
                background: #263238; color: #abb2bf; padding: 4px 10px; font-size: 11px; font-weight: bold;
                border-top-left-radius: 3px; border-top-right-radius: 3px;
            }
            QTabBar::tab:selected { background: #00bcd4; color: black; }
        """)

        # Tab 1: Joint Jogging (J1..J6)
        tab_joints = QWidget()
        j_layout = QVBoxLayout(tab_joints)
        j_layout.setContentsMargins(4, 4, 4, 4)
        j_layout.setSpacing(3)

        btn_step_style = """
            QPushButton {
                background: #37474f; color: #eceff1; border: 1px solid #455a64;
                border-radius: 3px; padding: 2px 6px; font-weight: bold; font-size: 11px;
            }
            QPushButton:hover { background: #455a64; border-color: #00bcd4; }
            QPushButton:pressed { background: #263238; }
        """

        short_names = ["J1 Base", "J2 Shld", "J3 Elbw", "J4 Ptch", "J5 Roll", "J6 Grip"]
        for i in range(6):
            row = QHBoxLayout()
            row.setSpacing(3)

            lbl = QLabel(short_names[i])
            lbl.setStyleSheet("color: #cfd8dc; font-size: 11px; font-weight: bold; min-width: 50px;")
            row.addWidget(lbl)

            btn_m5 = QPushButton("-5°")
            btn_m5.setStyleSheet(btn_step_style)
            btn_m5.clicked.connect(lambda _, idx=i: self._step_joint(idx, -5))
            row.addWidget(btn_m5)

            btn_m1 = QPushButton("-1°")
            btn_m1.setStyleSheet(btn_step_style)
            btn_m1.clicked.connect(lambda _, idx=i: self._step_joint(idx, -1))
            row.addWidget(btn_m1)

            spin = QSpinBox()
            min_v, max_v = JOINT_LIMITS[i]
            spin.setRange(min_v, max_v)
            spin.setValue(90 if i < 5 else 40)
            spin.setStyleSheet("""
                QSpinBox {
                    background: #263238; color: white; border: 1px solid #37474f;
                    border-radius: 3px; padding: 2px 4px; font-weight: bold; min-width: 45px;
                }
            """)
            spin.valueChanged.connect(lambda val, idx=i: self._on_spin_changed(idx, val))
            self.spinboxes.append(spin)
            row.addWidget(spin)

            btn_p1 = QPushButton("+1°")
            btn_p1.setStyleSheet(btn_step_style)
            btn_p1.clicked.connect(lambda _, idx=i: self._step_joint(idx, 1))
            row.addWidget(btn_p1)

            btn_p5 = QPushButton("+5°")
            btn_p5.setStyleSheet(btn_step_style)
            btn_p5.clicked.connect(lambda _, idx=i: self._step_joint(idx, 5))
            row.addWidget(btn_p5)

            chk_inv = QCheckBox("Inv")
            chk_inv.setStyleSheet("color: #78909c; font-size: 10px;")
            if i == 1:
                chk_inv.setChecked(True)
            self.invert_checkboxes.append(chk_inv)
            row.addWidget(chk_inv)

            j_layout.addLayout(row)

        self.jog_tabs.addTab(tab_joints, "Nivelet (J1..J6)")

        # Tab 2: Cartesian XYZ Jogging
        tab_xyz = QWidget()
        x_layout = QVBoxLayout(tab_xyz)
        x_layout.setContentsMargins(4, 4, 4, 4)
        x_layout.setSpacing(4)

        axes_info = [
            ("X (Eteen)", lambda: self._step_cartesian(dx=-10), lambda: self._step_cartesian(dx=10)),
            ("Y (Sivu)",  lambda: self._step_cartesian(dy=-10), lambda: self._step_cartesian(dy=10)),
            ("Z (Korkeus)", lambda: self._step_cartesian(dz=-10), lambda: self._step_cartesian(dz=10)),
            ("Pitch (Kulma)", lambda: self._step_cartesian(dp=-5), lambda: self._step_cartesian(dp=5)),
        ]

        for name, fn_minus, fn_plus in axes_info:
            row = QHBoxLayout()
            row.setSpacing(4)
            lbl = QLabel(name)
            lbl.setStyleSheet("color: #cfd8dc; font-size: 11px; font-weight: bold; min-width: 75px;")
            row.addWidget(lbl)

            btn_m = QPushButton("◀ -10 mm" if "Pitch" not in name else "◀ -5°")
            btn_m.setStyleSheet(btn_step_style)
            btn_m.clicked.connect(fn_minus)
            row.addWidget(btn_m)

            btn_p = QPushButton("+10 mm ▶" if "Pitch" not in name else "+5° ▶")
            btn_p.setStyleSheet(btn_step_style)
            btn_p.clicked.connect(fn_plus)
            row.addWidget(btn_p)

            x_layout.addLayout(row)

        # 3D Gizmo Toggle
        self.btn_gizmo = QPushButton("🎯 3D Interactive Jog Gizmo (Päällä)")
        self.btn_gizmo.setCheckable(True)
        self.btn_gizmo.setChecked(False)
        self.btn_gizmo.setStyleSheet("""
            QPushButton {
                background: #263238; color: #00e5ff; font-weight: bold; font-size: 11px;
                border: 1px solid #00acc1; border-radius: 4px; padding: 6px;
            }
            QPushButton:checked {
                background: #00acc1; color: black; border-color: #80deea;
            }
            QPushButton:hover:!checked { background: #37474f; }
        """)
        self.btn_gizmo.toggled.connect(self._on_gizmo_toggled)
        x_layout.addWidget(self.btn_gizmo)

        self.jog_tabs.addTab(tab_xyz, "Avaruus XYZ (IK)")
        layout.addWidget(self.jog_tabs)

        # Presets Row
        presets_layout = QHBoxLayout()
        presets_layout.setSpacing(4)
        for label, preset in [("Koti", [90, 90, 90, 90, 90, 40]),
                             ("Valmis", [90, 110, 130, 40, 90, 40]),
                             ("Kurotus", [90, 135, 90, 45, 90, 40])]:
            btn = QPushButton(label)
            btn.setStyleSheet("""
                QPushButton {
                    background: #282c34; color: #abb2bf; border: 1px solid #3e4451;
                    border-radius: 3px; padding: 3px 6px; font-size: 11px;
                }
                QPushButton:hover { background: #3e4451; color: white; }
            """)
            btn.clicked.connect(lambda _, p=preset: self.apply_angles(p))
            presets_layout.addWidget(btn)
        layout.addLayout(presets_layout)

    def _step_joint(self, joint_idx: int, step_deg: int) -> None:
        angles = self.get_angles()
        min_v, max_v = JOINT_LIMITS[joint_idx]
        new_val = max(min_v, min(max_v, angles[joint_idx] + step_deg))
        angles[joint_idx] = new_val
        self.apply_angles(angles)

    def _step_cartesian(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dp: float = 0.0) -> None:
        angles = self.get_angles()
        pose = self.kinematics.forward_kinematics([float(a) for a in angles])
        target_x = pose.tcp_x + dx
        target_y = pose.tcp_y + dy
        target_z = max(10.0, pose.tcp_z + dz)
        target_pitch = max(-90.0, min(90.0, pose.pitch_deg + dp))
        roll = float(angles[4])
        gripper = float(angles[5])

        sol = self.kinematics.inverse_kinematics(
            target_x, target_y, target_z,
            pitch_deg=target_pitch, roll_deg=roll, gripper_deg=gripper,
            auto_adjust_pitch=True
        )
        if sol is not None:
            self.apply_angles(sol)

    def _on_spin_changed(self, index: int, value: int) -> None:
        if self._block_signals:
            return
        angles = self.get_angles()
        angles[index] = value
        self.apply_angles(angles)

    def _on_gizmo_toggled(self, checked: bool) -> None:
        if self.on_toggle_gizmo:
            self.on_toggle_gizmo(checked)
        self.btn_gizmo.setText("🎯 3D Jog Gizmo (Päällä)" if checked else "🎯 3D Jog Gizmo (Pois)")

    def sync_angles(self, angles: List[int]) -> None:
        """Updates spinbox values without triggering feedback loops."""
        self._block_signals = True
        for i in range(min(6, len(angles))):
            self.spinboxes[i].setValue(angles[i])
        self._block_signals = False

    def to_physical_angles(self, angles: List[int]) -> List[int]:
        phys = []
        for i, val in enumerate(angles):
            if i < len(self.invert_checkboxes) and self.invert_checkboxes[i].isChecked():
                phys.append(180 - val)
            else:
                phys.append(val)
        return phys

    def get_physical_angles(self) -> List[int]:
        return self.to_physical_angles(self.get_angles())
