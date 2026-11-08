"""
Cartesian XYZ Controller Panel for 6-DOF Robot Arm Digital Twin.
Allows intuitive 3D space jogging (X, Y, Z, Pitch) with Inverse Kinematics (IK),
step size selection (1, 5, 10, 25 mm), and PTP / LINEAR motion mode toggles.
"""
from typing import Callable, List, Optional
from src.core.kinematics import RobotKinematics
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QSpinBox, QDoubleSpinBox, QGroupBox, QRadioButton,
    QButtonGroup, Qt
)


class XyzControllerPanel(QGroupBox):
    """
    Control card for Cartesian 3D jogging (Forward/Back, Left/Right, Up/Down, Pitch)
    using analytical Inverse Kinematics (IK).
    """
    def __init__(self,
                 kinematics: RobotKinematics,
                 get_current_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 parent=None):
        super().__init__("Avaruusohjaus XYZ & Hitsausliikkeet (IK)", parent)
        self.kinematics = kinematics
        self.get_current_angles = get_current_angles_fn
        self.apply_angles = apply_angles_fn

        self.current_step_mm: float = 10.0
        self.step_buttons: List[QPushButton] = []

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # 1. Step Size Selector Row
        step_box = QHBoxLayout()
        step_lbl = QLabel("Askelkoko:")
        step_lbl.setStyleSheet("color: #90a4ae; font-size: 11px; font-weight: bold;")
        step_box.addWidget(step_lbl)

        sizes = [1.0, 5.0, 10.0, 25.0]
        for s in sizes:
            btn = QPushButton(f"{int(s)} mm")
            btn.setCheckable(True)
            if s == 10.0:
                btn.setChecked(True)
            btn.setStyleSheet("""
                QPushButton {
                    background: #263238; color: #eceff1; border: 1px solid #37474f;
                    border-radius: 4px; padding: 4px 8px; font-size: 11px; font-weight: bold;
                }
                QPushButton:checked {
                    background: #00bcd4; color: #000000; border-color: #80deea;
                }
                QPushButton:hover:!checked { background: #37474f; }
            """)
            btn.clicked.connect(lambda _, val=s: self._set_step_size(val))
            self.step_buttons.append(btn)
            step_box.addWidget(btn)

        layout.addLayout(step_box)

        # 2. 3D Jogging Grid (X, Y, Z, Pitch)
        grid = QGridLayout()
        grid.setSpacing(6)

        btn_style = """
            QPushButton {
                background: #37474f; color: #ffffff; font-weight: bold;
                border: 1px solid #455a64; border-radius: 4px; padding: 6px 10px;
                font-size: 12px;
            }
            QPushButton:hover { background: #455a64; border-color: #00bcd4; }
            QPushButton:pressed { background: #263238; }
        """

        # X Jog (Eteen / Taakse)
        lbl_x = QLabel("X (Eteen / Taakse):")
        lbl_x.setStyleSheet("color: #cfd8dc; font-weight: bold;")
        btn_x_minus = QPushButton("◀ Taakse (-X)")
        btn_x_plus = QPushButton("Eteen (+X) ▶")
        btn_x_minus.setStyleSheet(btn_style)
        btn_x_plus.setStyleSheet(btn_style)
        btn_x_minus.clicked.connect(lambda: self._jog_cartesian(dx=-self.current_step_mm))
        btn_x_plus.clicked.connect(lambda: self._jog_cartesian(dx=self.current_step_mm))
        grid.addWidget(lbl_x, 0, 0)
        grid.addWidget(btn_x_minus, 0, 1)
        grid.addWidget(btn_x_plus, 0, 2)

        # Y Jog (Vasen / Oikea)
        lbl_y = QLabel("Y (Vasen / Oikea):")
        lbl_y.setStyleSheet("color: #cfd8dc; font-weight: bold;")
        btn_y_minus = QPushButton("◀ Oikealle (-Y)")
        btn_y_plus = QPushButton("Vasemmalle (+Y) ▶")
        btn_y_minus.setStyleSheet(btn_style)
        btn_y_plus.setStyleSheet(btn_style)
        btn_y_minus.clicked.connect(lambda: self._jog_cartesian(dy=-self.current_step_mm))
        btn_y_plus.clicked.connect(lambda: self._jog_cartesian(dy=self.current_step_mm))
        grid.addWidget(lbl_y, 1, 0)
        grid.addWidget(btn_y_minus, 1, 1)
        grid.addWidget(btn_y_plus, 1, 2)

        # Z Jog (Korkeus pöydästä)
        lbl_z = QLabel("Z (Korkeus):")
        lbl_z.setStyleSheet("color: #cfd8dc; font-weight: bold;")
        btn_z_minus = QPushButton("▼ Alas (-Z)")
        btn_z_plus = QPushButton("Ylös (+Z) ▲")
        btn_z_minus.setStyleSheet(btn_style)
        btn_z_plus.setStyleSheet(btn_style)
        btn_z_minus.clicked.connect(lambda: self._jog_cartesian(dz=-self.current_step_mm))
        btn_z_plus.clicked.connect(lambda: self._jog_cartesian(dz=self.current_step_mm))
        grid.addWidget(lbl_z, 2, 0)
        grid.addWidget(btn_z_minus, 2, 1)
        grid.addWidget(btn_z_plus, 2, 2)

        # Pitch Jog (Kouran / hitsauspään kulma)
        lbl_pitch = QLabel("Pitch (Kouran kulma):")
        lbl_pitch.setStyleSheet("color: #cfd8dc; font-weight: bold;")
        btn_p_minus = QPushButton("⤹ Alas (Pitch -)")
        btn_p_plus = QPushButton("Ylös (Pitch +) ⤸")
        btn_p_minus.setStyleSheet(btn_style)
        btn_p_plus.setStyleSheet(btn_style)
        btn_p_minus.clicked.connect(lambda: self._jog_cartesian(dp=-5.0))
        btn_p_plus.clicked.connect(lambda: self._jog_cartesian(dp=5.0))
        grid.addWidget(lbl_pitch, 3, 0)
        grid.addWidget(btn_p_minus, 3, 1)
        grid.addWidget(btn_p_plus, 3, 2)

        layout.addLayout(grid)

        # Status / Feedback line
        self.lbl_ik_status = QLabel("IK Valmis: Sijainti saavutettavissa.")
        self.lbl_ik_status.setStyleSheet("color: #69f0ae; font-size: 11px; font-weight: bold;")
        layout.addWidget(self.lbl_ik_status)

    def _set_step_size(self, size_mm: float) -> None:
        self.current_step_mm = size_mm
        for btn in self.step_buttons:
            btn.setChecked(btn.text() == f"{int(size_mm)} mm")

    def _jog_cartesian(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dp: float = 0.0) -> None:
        """Solves IK for incremental move and updates arm."""
        angles = self.get_current_angles()
        pose = self.kinematics.forward_kinematics([float(a) for a in angles])

        target_x = pose.tcp_x + dx
        target_y = pose.tcp_y + dy
        target_z = max(10.0, pose.tcp_z + dz)  # Floor protection (Z >= 10mm)
        target_pitch = max(-90.0, min(90.0, pose.pitch_deg + dp))

        roll = float(angles[4]) if len(angles) > 4 else 90.0
        gripper = float(angles[5]) if len(angles) > 5 else 40.0

        sol = self.kinematics.inverse_kinematics(
            target_x, target_y, target_z,
            pitch_deg=target_pitch,
            roll_deg=roll,
            gripper_deg=gripper,
            auto_adjust_pitch=True
        )

        if sol is not None:
            self.apply_angles(sol)
            self.lbl_ik_status.setText(f"✓ IK OK: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f} mm")
            self.lbl_ik_status.setStyleSheet("color: #69f0ae; font-size: 11px; font-weight: bold;")
        else:
            self.lbl_ik_status.setText("⚠️ Piste työalueen ulkopuolella tai servoraja vastassa!")
            self.lbl_ik_status.setStyleSheet("color: #ff5252; font-size: 11px; font-weight: bold;")
