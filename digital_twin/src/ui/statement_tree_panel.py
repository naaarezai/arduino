"""
Statement Tree Panel for Robot Offline Programming (Visual Components style).
Displays the robot program as a structured tree (MainRoutine) with dedicated icons,
badges, statement toolbar (Add PTP, Add LIN, Gripper, Delay, Teach), and reordering.
"""
from typing import Callable, List, Optional
import copy
from src.core.kinematics import RobotKinematics
from src.core.program import (
    RobotProgram, Statement, MotionStatement, GripperStatement, DelayStatement, ToolActionStatement
)
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTreeWidget, QTreeWidgetItem, QHeaderView, QGroupBox,
    QFileDialog, QMessageBox, Qt, QColor, QComboBox, QSpinBox,
    QDialog, QGridLayout, QMenu, QAction, QShortcut, QKeySequence, QInputDialog,
    QTabWidget, QFrame
)
from src.ui.code_preview_panel import CodePreviewPanel


class WeldWizardDialog(QDialog):
    """
    Industrial Weld Seam Wizard dialog.
    Generates a 5-step industrial welding sequence:
    1. PTP Approach (30 mm above start)
    2. LIN Touchdown (seam start)
    3. LIN Weld (linear seam at controlled speed)
    4. DELAY Crater fill dwell
    5. LIN Retract (30 mm above seam end)
    """
    def __init__(self, kinematics: RobotKinematics, current_angles: List[int], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Industrial Weld Seam Wizard")
        self.resize(380, 280)
        self.setStyleSheet("""
            QDialog { background: #1e1e1e; color: #d4d4d4; }
            QLabel { color: #cccccc; font-size: 11px; font-weight: 600; }
            QSpinBox, QComboBox {
                background: #252526; color: #ffffff; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 4px; font-size: 11px;
            }
        """)
        self.kinematics = kinematics
        self.current_angles = current_angles
        self.result_statements = []

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        title = QLabel("Automatic Weld & Contour Path Generator")
        title.setStyleSheet("font-weight: bold; font-size: 12px; color: #a855f7;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(8)

        grid.addWidget(QLabel("Seam Length (mm):"), 0, 0)
        self.spin_len = QSpinBox()
        self.spin_len.setRange(10, 250)
        self.spin_len.setValue(60)
        grid.addWidget(self.spin_len, 0, 1)

        grid.addWidget(QLabel("Direction:"), 1, 0)
        self.combo_dir = QComboBox()
        self.combo_dir.addItems([
            "+X (Eteen / Forward)",
            "-X (Taakse / Backward)",
            "+Y (Vasen / Left)",
            "-Y (Oikea / Right)",
            "+Z (Pystysauma ylös / Up)",
            "-Z (Pystysauma alas / Down)"
        ])
        grid.addWidget(self.combo_dir, 1, 1)

        grid.addWidget(QLabel("Approach Height (mm):"), 2, 0)
        self.spin_approach = QSpinBox()
        self.spin_approach.setRange(10, 100)
        self.spin_approach.setValue(25)
        grid.addWidget(self.spin_approach, 2, 1)

        grid.addWidget(QLabel("Weld Speed (%):"), 3, 0)
        self.spin_speed = QSpinBox()
        self.spin_speed.setRange(10, 100)
        self.spin_speed.setValue(40)
        grid.addWidget(self.spin_speed, 3, 1)

        grid.addWidget(QLabel("Weave / Pendlaus:"), 4, 0)
        self.combo_weave = QComboBox()
        self.combo_weave.addItems(["Ei pendlausta (Suora LIN)", "Siksak (Zigzag)", "Sini (Sine wave)"])
        grid.addWidget(self.combo_weave, 4, 1)

        grid.addWidget(QLabel("Leveys / Amplitude (mm):"), 5, 0)
        self.spin_amp = QSpinBox()
        self.spin_amp.setRange(1, 15)
        self.spin_amp.setValue(3)
        grid.addWidget(self.spin_amp, 5, 1)

        grid.addWidget(QLabel("Aallonpituus / Pitch (mm):"), 6, 0)
        self.spin_pitch = QSpinBox()
        self.spin_pitch.setRange(2, 20)
        self.spin_pitch.setValue(6)
        grid.addWidget(self.spin_pitch, 6, 1)

        grid.addWidget(QLabel("Jätökulma / Drag Angle (°):"), 7, 0)
        self.spin_drag = QSpinBox()
        self.spin_drag.setRange(-25, 25)
        self.spin_drag.setValue(10)
        grid.addWidget(self.spin_drag, 7, 1)

        layout.addLayout(grid)

        # Action Buttons
        btn_box = QHBoxLayout()
        btn_cancel = QPushButton("Cancel")
        btn_cancel.setStyleSheet("background: #2d2d30; color: #cccccc; padding: 6px 12px; border-radius: 3px;")
        btn_cancel.clicked.connect(self.reject)
        btn_box.addWidget(btn_cancel)

        btn_gen = QPushButton("Generate Routine")
        btn_gen.setStyleSheet("background: #7c3aed; color: white; font-weight: bold; padding: 6px 14px; border-radius: 3px;")
        btn_gen.clicked.connect(self._generate)
        btn_box.addWidget(btn_gen)
        layout.addLayout(btn_box)

    def _generate(self):
        p_start = self.kinematics.forward_kinematics([float(a) for a in self.current_angles])
        length = float(self.spin_len.value())
        app_h = float(self.spin_approach.value())
        speed = self.spin_speed.value()
        direction = self.combo_dir.currentIndex()

        dx, dy, dz = 0.0, 0.0, 0.0
        if direction == 0:   dx = length
        elif direction == 1: dx = -length
        elif direction == 2: dy = length
        elif direction == 3: dy = -length
        elif direction == 4: dz = length
        elif direction == 5: dz = -length

        w_idx = self.combo_weave.currentIndex()
        weave_type = "NONE" if w_idx == 0 else ("ZIGZAG" if w_idx == 1 else "SINE")
        amp = float(self.spin_amp.value())
        pitch = float(self.spin_pitch.value())
        drag = float(self.spin_drag.value())

        p_end = (p_start.tcp_x + dx, p_start.tcp_y + dy, p_start.tcp_z + dz)
        from src.core.weld_planner import generate_industrial_weld_sequence
        stmts, err = generate_industrial_weld_sequence(
            (p_start.tcp_x, p_start.tcp_y, p_start.tcp_z),
            p_end,
            kinematics=self.kinematics,
            current_angles=self.current_angles,
            approach_h=app_h,
            speed_percent=speed,
            weave_type=weave_type,
            amplitude=amp,
            pitch=pitch,
            drag_angle_deg=drag
        )
        if err or not stmts:
            QMessageBox.warning(self, "Ulottuvuusraja", f"Laskettu hitsausrata ylittää robotin ulottuvuusalueen:\n{err}\nKokeile pienempää saumapituutta tai toista suuntaa.")
            return

        self.result_statements = []
        for s in stmts:
            if isinstance(s, DelayStatement):
                self.result_statements.append(("DELAY", s.duration_ms, 0, s.comment))
            elif isinstance(s, MotionStatement):
                self.result_statements.append((s.statement_type, s.angles, s.speed_percent, s.comment))
        self.accept()


class MultiPassDialog(QDialog):
    """
    Multi-Pass Weld Generator dialog (Root, Fill, Cap passes).
    """
    def __init__(self, kinematics: RobotKinematics, current_angles: List[int], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Monipalkohitsaus (Multi-Pass Generator)")
        self.resize(380, 320)
        self.setStyleSheet("""
            QDialog { background: #1e1e1e; color: #d4d4d4; }
            QLabel { color: #cccccc; font-size: 11px; font-weight: 600; }
            QSpinBox, QComboBox {
                background: #252526; color: #ffffff; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 4px; font-size: 11px;
            }
        """)
        self.kinematics = kinematics
        self.current_angles = current_angles
        self.result_statements = []

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        title = QLabel("🧬 Monipalkohitsauksen generaattori (Multi-Pass)")
        title.setStyleSheet("font-weight: bold; font-size: 12px; color: #f43f5e;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(8)

        grid.addWidget(QLabel("Sauman pituus (mm):"), 0, 0)
        self.spin_len = QSpinBox()
        self.spin_len.setRange(10, 250)
        self.spin_len.setValue(60)
        grid.addWidget(self.spin_len, 0, 1)

        grid.addWidget(QLabel("Suunta:"), 1, 0)
        self.combo_dir = QComboBox()
        self.combo_dir.addItems([
            "+X (Eteen / Forward)",
            "-X (Taakse / Backward)",
            "+Y (Vasen / Left)",
            "-Y (Oikea / Right)",
            "+Z (Pystysauma ylös / Up)",
            "-Z (Pystysauma alas / Down)"
        ])
        grid.addWidget(self.combo_dir, 1, 1)

        grid.addWidget(QLabel("Palkojen määrä:"), 2, 0)
        self.combo_passes = QComboBox()
        self.combo_passes.addItems(["3 Palkoa (Juuripalko, Täyttö, Pinta)", "2 Palkoa (Juuri, Pinta)"])
        grid.addWidget(self.combo_passes, 2, 1)

        grid.addWidget(QLabel("Sivusiirtymä / Offset (mm):"), 3, 0)
        self.spin_lat = QSpinBox()
        self.spin_lat.setRange(1, 10)
        self.spin_lat.setValue(3)
        grid.addWidget(self.spin_lat, 3, 1)

        grid.addWidget(QLabel("Korkeussiirtymä / Z-Step (mm):"), 4, 0)
        self.spin_vert = QSpinBox()
        self.spin_vert.setRange(1, 8)
        self.spin_vert.setValue(2)
        grid.addWidget(self.spin_vert, 4, 1)

        layout.addLayout(grid)

        btn_box = QHBoxLayout()
        btn_cancel = QPushButton("Peruuta")
        btn_cancel.setStyleSheet("background: #2d2d30; color: #cccccc; padding: 6px 12px; border-radius: 3px;")
        btn_cancel.clicked.connect(self.reject)
        btn_box.addWidget(btn_cancel)

        btn_gen = QPushButton("Generoi Monipalko")
        btn_gen.setStyleSheet("background: #be185d; color: white; font-weight: bold; padding: 6px 14px; border-radius: 3px;")
        btn_gen.clicked.connect(self._generate)
        btn_box.addWidget(btn_gen)
        layout.addLayout(btn_box)

    def _generate(self):
        p_start = self.kinematics.forward_kinematics([float(a) for a in self.current_angles])
        length = float(self.spin_len.value())
        direction = self.combo_dir.currentIndex()

        dx, dy, dz = 0.0, 0.0, 0.0
        if direction == 0:   dx = length
        elif direction == 1: dx = -length
        elif direction == 2: dy = length
        elif direction == 3: dy = -length
        elif direction == 4: dz = length
        elif direction == 5: dz = -length

        num_p = 3 if self.combo_passes.currentIndex() == 0 else 2
        p_end = (p_start.tcp_x + dx, p_start.tcp_y + dy, p_start.tcp_z + dz)

        from src.core.weld_planner import generate_multipass_routine
        stmts, err = generate_multipass_routine(
            (p_start.tcp_x, p_start.tcp_y, p_start.tcp_z),
            p_end,
            kinematics=self.kinematics,
            current_angles=self.current_angles,
            num_passes=num_p,
            lateral_offset_mm=float(self.spin_lat.value()),
            vertical_offset_mm=float(self.spin_vert.value())
        )
        if err or not stmts:
            QMessageBox.warning(self, "Ulottuvuusraja", f"Monipalkorata ylittää robotin ulottuvuusalueen:\n{err}")
            return

        self.result_statements = []
        for s in stmts:
            if isinstance(s, DelayStatement):
                self.result_statements.append(("DELAY", s.duration_ms, 0, s.comment))
            elif isinstance(s, MotionStatement):
                self.result_statements.append((s.statement_type, s.angles, s.speed_percent, s.comment))
        self.accept()


class PickAndPlaceDialog(QDialog):
    """
    Automated Pick & Place routine generator for Gripper and Vacuum tools.
    Generates approach -> descend -> grip/vacuum -> retract -> transfer -> place -> release -> retract.
    """
    def __init__(self, kinematics: RobotKinematics, current_angles: List[int], tool_type: str = "GRIPPER", parent=None):
        super().__init__(parent)
        self.setWindowTitle("Pick & Place -apuri (Gripper)" if tool_type == "GRIPPER" else "Pick & Place -apuri (Vacuum)")
        self.resize(380, 320)
        self.setStyleSheet("""
            QDialog { background: #1e1e1e; color: #d4d4d4; }
            QLabel { color: #cccccc; font-size: 11px; font-weight: 600; }
            QSpinBox, QComboBox {
                background: #252526; color: #ffffff; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 4px; font-size: 11px;
            }
        """)
        self.kinematics = kinematics
        self.current_angles = current_angles
        self.tool_type = tool_type
        self.result_statements = []

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        title = QLabel(f"📦 Pick & Place -apuri ({tool_type})")
        title.setStyleSheet("font-weight: bold; font-size: 12px; color: #38bdf8;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(8)

        grid.addWidget(QLabel("Nostokorkeus / Approach Z (mm):"), 0, 0)
        self.spin_app_z = QSpinBox()
        self.spin_app_z.setRange(10, 100)
        self.spin_app_z.setValue(30)
        grid.addWidget(self.spin_app_z, 0, 1)

        grid.addWidget(QLabel("Siirtosuunta:"), 1, 0)
        self.combo_dir = QComboBox()
        self.combo_dir.addItems([
            "+Y (Vasen / Left)",
            "-Y (Oikea / Right)",
            "+X (Eteen / Forward)",
            "-X (Taakse / Backward)"
        ])
        grid.addWidget(self.combo_dir, 1, 1)

        grid.addWidget(QLabel("Siirtomatka / Distance (mm):"), 2, 0)
        self.spin_dist = QSpinBox()
        self.spin_dist.setRange(20, 300)
        self.spin_dist.setValue(100)
        grid.addWidget(self.spin_dist, 2, 1)

        grid.addWidget(QLabel("Siirtonopeus (%):"), 3, 0)
        self.spin_speed = QSpinBox()
        self.spin_speed.setRange(10, 100)
        self.spin_speed.setValue(50)
        grid.addWidget(self.spin_speed, 3, 1)

        layout.addLayout(grid)

        btn_box = QHBoxLayout()
        btn_cancel = QPushButton("Peruuta")
        btn_cancel.setStyleSheet("background: #2d2d30; color: #cccccc; padding: 6px 12px; border-radius: 3px;")
        btn_cancel.clicked.connect(self.reject)
        btn_box.addWidget(btn_cancel)

        btn_gen = QPushButton("Generoi Pick & Place")
        btn_gen.setStyleSheet("background: #0284c7; color: white; font-weight: bold; padding: 6px 14px; border-radius: 3px;")
        btn_gen.clicked.connect(self._generate)
        btn_box.addWidget(btn_gen)
        layout.addLayout(btn_box)

    def _generate(self):
        p_pick = self.kinematics.forward_kinematics([float(a) for a in self.current_angles])
        app_h = float(self.spin_app_z.value())
        dist = float(self.spin_dist.value())
        d_idx = self.combo_dir.currentIndex()

        dx, dy = 0.0, 0.0
        if d_idx == 0: dy = dist
        elif d_idx == 1: dy = -dist
        elif d_idx == 2: dx = dist
        elif d_idx == 3: dx = -dist

        # Compute Pick and Place waypoints
        # 1. Approach to Pick (+Z)
        sol_app_pick, _ = self.kinematics.solve_ik_clamped(p_pick.tcp_x, p_pick.tcp_y, p_pick.tcp_z + app_h, current_angles=self.current_angles)
        # 2. Pick touchdown (current pose)
        sol_pick = list(self.current_angles)
        # 3. Retract from Pick (+Z)
        sol_ret_pick, _ = self.kinematics.solve_ik_clamped(p_pick.tcp_x, p_pick.tcp_y, p_pick.tcp_z + app_h, current_angles=sol_pick)
        # 4. Approach to Place (Pick + (dx, dy, app_h))
        sol_app_place, _ = self.kinematics.solve_ik_clamped(p_pick.tcp_x + dx, p_pick.tcp_y + dy, p_pick.tcp_z + app_h, current_angles=sol_ret_pick or sol_pick)
        # 5. Place touchdown (Pick + (dx, dy, 0))
        sol_place, _ = self.kinematics.solve_ik_clamped(p_pick.tcp_x + dx, p_pick.tcp_y + dy, p_pick.tcp_z, current_angles=sol_app_place or sol_pick)
        # 6. Retract from Place
        sol_ret_place = sol_app_place

        if not all([sol_app_pick, sol_pick, sol_ret_pick, sol_app_place, sol_place]):
            QMessageBox.warning(self, "Ulottuvuusraja", "Pick & Place -siirron kohde ylittää robotin ulottuvuusalueen!\nKokeile pienempää siirtomatkaa.")
            return

        spd = self.spin_speed.value()
        self.result_statements = []
        # Step 1: PTP Approach Pick
        self.result_statements.append(("PTP", sol_app_pick, 100, f"Lähestyminen poimintaan (+{int(app_h)}mm)"))
        # Step 2: LIN Down to Pick
        self.result_statements.append(("LIN", sol_pick, 40, "Laskeutuminen kappaleeseen (Pick)"))
        # Step 3: Grip / Vacuum ON
        if self.tool_type == "GRIPPER":
            self.result_statements.append(("GRIPPER", "CLOSE", 128, "Tartu kappaleeseen (Koura kiinni)"))
        else:
            self.result_statements.append(("TOOL_ACTION", "VACUUM", "ON", "Imu PÄÄLLE (Tyhjiö aktivoitu)"))
        # Step 4: Dwell
        self.result_statements.append(("DELAY", 500, 0, "Odotus / Ote vakiintuu (500 ms)"))
        # Step 5: LIN Retract Up
        self.result_statements.append(("LIN", sol_ret_pick, 50, f"Nosto poimintapisteestä (+{int(app_h)}mm)"))
        # Step 6: PTP Transfer to Place
        self.result_statements.append(("PTP", sol_app_place, spd, f"Siirto asennuspisteen ylle (+{int(dist)}mm)"))
        # Step 7: LIN Down to Place
        self.result_statements.append(("LIN", sol_place, 40, "Lasku asennuspintaan (Place)"))
        # Step 8: Release / Vacuum OFF
        if self.tool_type == "GRIPPER":
            self.result_statements.append(("GRIPPER", "OPEN", 40, "Päästä kappale (Koura auki)"))
        else:
            self.result_statements.append(("TOOL_ACTION", "VACUUM", "OFF", "Imu POIS (Kappaleen vapautus)"))
        # Step 9: Dwell
        self.result_statements.append(("DELAY", 300, 0, "Odotus / Vapautus (300 ms)"))
        # Step 10: LIN Retract Up
        self.result_statements.append(("LIN", sol_ret_place, 80, "Nosto asennuspisteestä turvakorkeuteen"))

        self.accept()


class StatementTreePanel(QGroupBox):
    """
    Statement Tree view mimicking the Visual Components Robot Programming / OLP interface.
    """
    def __init__(self,
                 program: RobotProgram,
                 get_current_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 on_program_changed_fn: Optional[Callable[[], None]] = None,
                 on_trigger_2click_seam_fn: Optional[Callable[[], None]] = None,
                 parent=None):
        super().__init__("Program Statement Tree (OLP)", parent)
        self.program = program
        self.get_current_angles = get_current_angles_fn
        self.apply_angles = apply_angles_fn
        self.on_program_changed = on_program_changed_fn
        self.on_trigger_2click_seam = on_trigger_2click_seam_fn
        self._undo_stack: List[list] = []
        self._redo_stack: List[list] = []

        self._init_ui()
        self.refresh_tree()

        # Keyboard shortcuts
        QShortcut(QKeySequence("Ctrl+Z"), self, self._undo)
        QShortcut(QKeySequence("Ctrl+Y"), self, self._redo)
        QShortcut(QKeySequence("Delete"), self, self._delete_selected)
        QShortcut(QKeySequence("Ctrl+D"), self, self._duplicate_selected)

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 1. Statement Quick-Add Toolbar (Visual Components Ribbon style)
        tb_layout = QHBoxLayout()
        tb_layout.setSpacing(4)

        btn_style_ptp = """
            QPushButton {
                background: #007acc; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #0098ff; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #0098ff; }
            QPushButton:pressed { background: #005a9e; }
        """
        btn_style_lin = """
            QPushButton {
                background: #c56200; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #e67e22; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #e67e22; }
            QPushButton:pressed { background: #a04e00; }
        """
        btn_style_act = """
            QPushButton {
                background: #2d2d30; color: #cccccc; font-weight: 600; font-size: 11px;
                border: 1px solid #3f3f46; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #3e3e42; color: #ffffff; border-color: #007acc; }
            QPushButton:pressed { background: #1e1e1e; }
        """

        # A. General Motion Group (Always visible on left)
        self.btn_add_ptp = QPushButton("+ PTP Motion")
        self.btn_add_ptp.setStyleSheet(btn_style_ptp)
        self.btn_add_ptp.setToolTip("Lisää nykyinen asento PTP-siirtona")
        self.btn_add_ptp.clicked.connect(self._add_ptp)
        tb_layout.addWidget(self.btn_add_ptp)

        self.btn_add_lin = QPushButton("+ Linear Motion")
        self.btn_add_lin.setStyleSheet(btn_style_lin)
        self.btn_add_lin.setToolTip("Lisää suora Cartesian-siirto")
        self.btn_add_lin.clicked.connect(self._add_lin)
        tb_layout.addWidget(self.btn_add_lin)

        self.btn_add_delay = QPushButton("+ Dwell")
        self.btn_add_delay.setStyleSheet(btn_style_act)
        self.btn_add_delay.setToolTip("Lisää pito/viive (Wait time)")
        self.btn_add_delay.clicked.connect(self._add_delay)
        tb_layout.addWidget(self.btn_add_delay)

        # Separator line
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setFrameShadow(QFrame.Sunken)
        sep.setStyleSheet("color: #3f3f46; margin: 0 4px;")
        tb_layout.addWidget(sep)

        # B. Tool-Specific Dynamic Containers:
        # 1. Welding Tools Container (Active when Tool == WELDING_TORCH)
        self.weld_tools_box = QWidget()
        weld_l = QHBoxLayout(self.weld_tools_box)
        weld_l.setContentsMargins(0, 0, 0, 0)
        weld_l.setSpacing(4)

        self.btn_weld_wiz = QPushButton("🔥 Weld Wizard")
        self.btn_weld_wiz.setStyleSheet("""
            QPushButton {
                background: #5b21b6; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #7c3aed; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #7c3aed; }
        """)
        self.btn_weld_wiz.setToolTip("Generoi automaattinen hitsausrutiini (Lähesty -> Sytytä -> Sauma -> Kraatteri -> Nosta)")
        self.btn_weld_wiz.clicked.connect(self._open_weld_wizard)
        weld_l.addWidget(self.btn_weld_wiz)

        self.btn_2click_seam = QPushButton("⚡ 2-Click Seam")
        self.btn_2click_seam.setStyleSheet("""
            QPushButton {
                background: #3b0764; color: #e9d5ff; font-weight: 600; font-size: 11px;
                border: 1px solid #9333ea; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #581c87; color: white; }
        """)
        self.btn_2click_seam.setToolTip("Kahden klikkauksen hitsaussauma: klikkaa 3D-kappaleelta sauman alku ja loppu")
        if self.on_trigger_2click_seam:
            self.btn_2click_seam.clicked.connect(self.on_trigger_2click_seam)
        weld_l.addWidget(self.btn_2click_seam)

        self.btn_multipass = QPushButton("🧬 Multi-Pass")
        self.btn_multipass.setStyleSheet("""
            QPushButton {
                background: #831843; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #be185d; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #be185d; }
        """)
        self.btn_multipass.setToolTip("Generoi monipalkohitsaus (Root, Fill, Cap passes)")
        self.btn_multipass.clicked.connect(self._open_multipass)
        weld_l.addWidget(self.btn_multipass)

        self.btn_reverse = QPushButton("🔄 Käännä sauma")
        self.btn_reverse.setStyleSheet("""
            QPushButton {
                background: #1e3a8a; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #2563eb; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #2563eb; }
        """)
        self.btn_reverse.setToolTip("Käännä hitsaussauman suunta päinvastaiseksi (alku ↔ loppu)")
        self.btn_reverse.clicked.connect(self._reverse_program)
        weld_l.addWidget(self.btn_reverse)
        tb_layout.addWidget(self.weld_tools_box)

        # 2. Gripper Tools Container (Active when Tool == GRIPPER)
        self.gripper_tools_box = QWidget()
        grip_l = QHBoxLayout(self.gripper_tools_box)
        grip_l.setContentsMargins(0, 0, 0, 0)
        grip_l.setSpacing(4)

        self.btn_grip_close = QPushButton("🤏 Tartu (Close)")
        self.btn_grip_close.setStyleSheet("""
            QPushButton {
                background: #065f46; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #059669; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #059669; }
        """)
        self.btn_grip_close.setToolTip("Sulje koura (Ote kappaleesta, J6=128°)")
        self.btn_grip_close.clicked.connect(lambda: self._add_gripper_fixed("CLOSE", 128))
        grip_l.addWidget(self.btn_grip_close)

        self.btn_grip_open = QPushButton("🖐️ Päästä (Open)")
        self.btn_grip_open.setStyleSheet("""
            QPushButton {
                background: #1e293b; color: #38bdf8; font-weight: 600; font-size: 11px;
                border: 1px solid #0284c7; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #0284c7; color: white; }
        """)
        self.btn_grip_open.setToolTip("Avaa koura (Päästä kappale, J6=40°)")
        self.btn_grip_open.clicked.connect(lambda: self._add_gripper_fixed("OPEN", 40))
        grip_l.addWidget(self.btn_grip_open)

        self.btn_grip_custom = QPushButton("📐 Leukaväli...")
        self.btn_grip_custom.setStyleSheet(btn_style_act)
        self.btn_grip_custom.setToolTip("Aseta mukautettu leukakulma")
        self.btn_grip_custom.clicked.connect(self._add_gripper_action)
        grip_l.addWidget(self.btn_grip_custom)

        self.btn_pick_place = QPushButton("📦 Pick & Place -apuri")
        self.btn_pick_place.setStyleSheet("""
            QPushButton {
                background: #0284c7; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #38bdf8; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #0369a1; }
        """)
        self.btn_pick_place.setToolTip("Generoi automaattinen poiminta- ja siirtorutiini (Pick & Place)")
        self.btn_pick_place.clicked.connect(self._open_pick_and_place)
        grip_l.addWidget(self.btn_pick_place)
        tb_layout.addWidget(self.gripper_tools_box)

        # 3. Vacuum Tools Container (Active when Tool == VACUUM)
        self.vacuum_tools_box = QWidget()
        vac_l = QHBoxLayout(self.vacuum_tools_box)
        vac_l.setContentsMargins(0, 0, 0, 0)
        vac_l.setSpacing(4)

        self.btn_vac_on = QPushButton("💨 Imu PÄÄLLE")
        self.btn_vac_on.setStyleSheet("""
            QPushButton {
                background: #6b21a8; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #9333ea; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #9333ea; }
        """)
        self.btn_vac_on.clicked.connect(lambda: self._add_tool_action("VACUUM", "ON"))
        vac_l.addWidget(self.btn_vac_on)

        self.btn_vac_off = QPushButton("⚪ Imu POIS")
        self.btn_vac_off.setStyleSheet(btn_style_act)
        self.btn_vac_off.clicked.connect(lambda: self._add_tool_action("VACUUM", "OFF"))
        vac_l.addWidget(self.btn_vac_off)

        self.btn_vac_pnp = QPushButton("📦 Pick & Place -apuri")
        self.btn_vac_pnp.setStyleSheet("""
            QPushButton {
                background: #6b21a8; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #9333ea; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #9333ea; }
        """)
        self.btn_vac_pnp.clicked.connect(self._open_pick_and_place)
        vac_l.addWidget(self.btn_vac_pnp)
        tb_layout.addWidget(self.vacuum_tools_box)

        # 4. Dispenser Tools Container (Active when Tool == DISPENSER)
        self.dispenser_tools_box = QWidget()
        disp_l = QHBoxLayout(self.dispenser_tools_box)
        disp_l.setContentsMargins(0, 0, 0, 0)
        disp_l.setSpacing(4)

        self.btn_disp_on = QPushButton("🧪 Pursotus PÄÄLLE")
        self.btn_disp_on.setStyleSheet("""
            QPushButton {
                background: #047857; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #10b981; border-radius: 3px; padding: 5px 9px;
            }
            QPushButton:hover { background: #10b981; }
        """)
        self.btn_disp_on.clicked.connect(lambda: self._add_tool_action("DISPENSER", "ON"))
        disp_l.addWidget(self.btn_disp_on)

        self.btn_disp_off = QPushButton("⚪ Pursotus POIS")
        self.btn_disp_off.setStyleSheet(btn_style_act)
        self.btn_disp_off.clicked.connect(lambda: self._add_tool_action("DISPENSER", "OFF"))
        disp_l.addWidget(self.btn_disp_off)
        tb_layout.addWidget(self.dispenser_tools_box)

        tb_layout.addStretch()
        layout.addLayout(tb_layout)

        # Set initial visibility based on active tool
        self.set_active_tool(getattr(self.program.kinematics, "active_tool", None))

        # 2. Main Tabbed Container (Tree + Live Code Preview)
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #333333; background: #181818; }
            QTabBar::tab {
                background: #252526; color: #858585; padding: 5px 12px;
                border: 1px solid #2d2d30; border-bottom: none; font-size: 11px; font-weight: 600;
            }
            QTabBar::tab:selected { background: #181818; color: #ffffff; border-top: 2px solid #007acc; }
        """)

        # Tab 0: Tree + Action Toolbar Container
        tree_tab = QWidget()
        tree_tab_layout = QVBoxLayout(tree_tab)
        tree_tab_layout.setContentsMargins(0, 4, 0, 0)
        tree_tab_layout.setSpacing(6)

        # Main Statement Tree
        self.tree = QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["Statement", "Coordinates & Joints", "Speed / Duration"])
        header = self.tree.header()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.tree.setStyleSheet("""
            QTreeWidget {
                background: #181818; color: #d4d4d4; border: 1px solid #333333;
                border-radius: 3px; font-size: 11px;
            }
            QTreeWidget::item {
                padding: 4px;
            }
            QTreeWidget::item:selected {
                background: #094771; color: #ffffff; font-weight: 600;
            }
            QHeaderView::section {
                background: #252526; color: #858585; font-weight: 600;
                border: 1px solid #2d2d30; padding: 4px; font-size: 11px;
            }
        """)
        self.tree.itemDoubleClicked.connect(self._on_item_double_clicked)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._show_context_menu)
        tree_tab_layout.addWidget(self.tree, 1)

        # 3. Action Toolbar (Teach, Up, Down, Delete, Undo/Redo, Save/Load)
        act_layout = QHBoxLayout()
        act_layout.setSpacing(4)

        self.btn_teach = QPushButton("Teach Pose")
        self.btn_teach.setStyleSheet("""
            QPushButton {
                background: #007acc; color: white; font-weight: 600; font-size: 11px;
                border-radius: 3px; padding: 4px 10px; border: 1px solid #0098ff;
            }
            QPushButton:hover { background: #0098ff; }
        """)
        self.btn_teach.setToolTip("Päivitä valitun käskyn kohteeksi robotin nykyinen asento [Ctrl+T]")
        self.btn_teach.clicked.connect(self._teach_selected)
        act_layout.addWidget(self.btn_teach)

        self.btn_dup = QPushButton("📋 Duplicate")
        self.btn_dup.setToolTip("Monista valittu käsky [Ctrl+D]")
        self.btn_dup.setStyleSheet(btn_style_act)
        self.btn_dup.clicked.connect(self._duplicate_selected)
        act_layout.addWidget(self.btn_dup)

        self.btn_up = QPushButton("▲ Up")
        self.btn_up.setToolTip("Siirrä käsky ylemmäs")
        self.btn_up.setStyleSheet(btn_style_act)
        self.btn_up.clicked.connect(self._move_up)
        act_layout.addWidget(self.btn_up)

        self.btn_down = QPushButton("▼ Down")
        self.btn_down.setToolTip("Siirrä käsky alemmas")
        self.btn_down.setStyleSheet(btn_style_act)
        self.btn_down.clicked.connect(self._move_down)
        act_layout.addWidget(self.btn_down)

        self.btn_undo = QPushButton("↩ Undo")
        self.btn_undo.setToolTip("Kumoa edellinen toiminto [Ctrl+Z]")
        self.btn_undo.setStyleSheet(btn_style_act)
        self.btn_undo.clicked.connect(self._undo)
        act_layout.addWidget(self.btn_undo)

        self.btn_redo = QPushButton("↪ Redo")
        self.btn_redo.setToolTip("Tee uudelleen peruttu toiminto [Ctrl+Y]")
        self.btn_redo.setStyleSheet(btn_style_act)
        self.btn_redo.clicked.connect(self._redo)
        act_layout.addWidget(self.btn_redo)

        self.btn_delete = QPushButton("Delete")
        self.btn_delete.setToolTip("Poista valittu käsky [Del]")
        self.btn_delete.setStyleSheet("background: #6e1c1c; color: #ffcccc; border: 1px solid #852a2a; border-radius: 3px; padding: 4px 8px;")
        self.btn_delete.clicked.connect(self._delete_selected)
        act_layout.addWidget(self.btn_delete)

        self.btn_clear = QPushButton("Clear")
        self.btn_clear.setStyleSheet(btn_style_act)
        self.btn_clear.clicked.connect(self._clear_all)
        act_layout.addWidget(self.btn_clear)

        act_layout.addStretch()

        self.btn_save = QPushButton("Save JSON")
        self.btn_save.setStyleSheet(btn_style_act)
        self.btn_save.clicked.connect(self._save_program)
        act_layout.addWidget(self.btn_save)

        self.btn_load = QPushButton("Open JSON")
        self.btn_load.setStyleSheet(btn_style_act)
        self.btn_load.clicked.connect(self._load_program)
        act_layout.addWidget(self.btn_load)

        self.btn_export = QPushButton("Export Code")
        self.btn_export.setStyleSheet("""
            QPushButton {
                background: #007acc; color: white; font-weight: 600; font-size: 11px;
                border: 1px solid #0098ff; border-radius: 3px; padding: 4px 9px;
            }
            QPushButton:hover { background: #0098ff; }
        """)
        self.btn_export.clicked.connect(self._export_code)
        act_layout.addWidget(self.btn_export)

        tree_tab_layout.addLayout(act_layout)

        # Tab 1: Live Code Preview
        self.code_panel = CodePreviewPanel(self.program)

        self.tab_widget.addTab(tree_tab, "📋 Ohjelmapuu (Tree)")
        self.tab_widget.addTab(self.code_panel, "💻 Koodiesikatselu (Live Code)")
        self.tab_widget.currentChanged.connect(lambda idx: self.code_panel.refresh_code() if idx == 1 else None)

        layout.addWidget(self.tab_widget, 1)

    def refresh_tree(self) -> None:
        """Rebuilds the Statement Tree representing MainRoutine."""
        self.tree.clear()

        # Root Item: MainRoutine
        root = QTreeWidgetItem(self.tree)
        root.setText(0, f"Routine: {self.program.name} ({len(self.program.statements)} statements)")
        root.setForeground(0, QColor("#e0e0e0"))
        root.setExpanded(True)

        for i, stmt in enumerate(self.program.statements):
            item = QTreeWidgetItem(root)
            item.setData(0, Qt.UserRole, i)

            num_str = f"{i+1:02d}"

            if isinstance(stmt, MotionStatement):
                is_lin = (stmt.statement_type == "LIN")
                active_tool = getattr(self.program.kinematics, "active_tool", None)
                is_torch = bool(active_tool and getattr(active_tool, "tool_type", "") == "WELDING_TORCH")
                is_weld = is_torch and (getattr(stmt, "is_welding", False) or any(w in stmt.comment.lower() for w in ["weld", "hitsaus", "sauma", "arc on", "pendlaus"]))

                if is_weld:
                    tag = "WELD"
                    color = QColor("#ff9800")
                elif is_lin:
                    tag = "LIN"
                    color = QColor("#00bcd4")
                else:
                    tag = "PTP"
                    color = QColor("#4fc1ff")

                item.setText(0, f"{num_str}  [{tag}]  {stmt.target_id}")
                item.setForeground(0, color)

                pose = self.program.kinematics.forward_kinematics([float(a) for a in stmt.angles])
                item.setText(1, f"TCP: [{pose.tcp_x:.1f}, {pose.tcp_y:.1f}, {pose.tcp_z:.1f}] | J:{stmt.angles}")
                item.setText(2, f"{stmt.speed_percent}% ({stmt.duration_ms} ms)")

            elif isinstance(stmt, GripperStatement):
                item.setText(0, f"{num_str}  [GRIPPER]  {stmt.action}")
                item.setForeground(0, QColor("#10b981"))
                item.setText(1, f"Jaw angle: {stmt.target_angle}° | {stmt.comment}")
                item.setText(2, f"{stmt.duration_ms} ms")

            elif isinstance(stmt, ToolActionStatement):
                item.setText(0, f"{num_str}  [{stmt.tool_type}]  {stmt.action}")
                item.setForeground(0, QColor("#e040fb"))
                item.setText(1, stmt.comment)
                item.setText(2, f"{stmt.duration_ms} ms")

            elif isinstance(stmt, DelayStatement):
                item.setText(0, f"{num_str}  [WAIT]  {stmt.duration_ms} ms")
                item.setForeground(0, QColor("#dcdcaa"))
                item.setText(1, stmt.comment)
                item.setText(2, f"{stmt.duration_ms} ms")

        if hasattr(self, "code_panel"):
            self.code_panel.refresh_code()

        if self.on_program_changed:
            self.on_program_changed()

    def highlight_statement(self, index: int) -> None:
        """Visually marks current running statement during simulation."""
        if getattr(self, "_last_highlighted_index", -1) == index:
            return
        self._last_highlighted_index = index

        root = self.tree.topLevelItem(0)
        if not root:
            return
        for i in range(root.childCount()):
            child = root.child(i)
            if i == index:
                child.setBackground(0, QColor("#004d40"))
                child.setBackground(1, QColor("#004d40"))
                child.setBackground(2, QColor("#004d40"))
                self.tree.scrollToItem(child)
            else:
                child.setBackground(0, QColor("transparent"))
                child.setBackground(1, QColor("transparent"))
                child.setBackground(2, QColor("transparent"))

    def _get_selected_index(self) -> Optional[int]:
        items = self.tree.selectedItems()
        if not items:
            return None
        idx = items[0].data(0, Qt.UserRole)
        return idx if idx is not None else None

    def _push_undo(self) -> None:
        """Saves program state snapshot for Undo/Redo."""
        self._undo_stack.append([copy.deepcopy(s) for s in self.program.statements])
        if len(self._undo_stack) > 40:
            self._undo_stack.pop(0)
        self._redo_stack.clear()

    def _undo(self) -> None:
        """Reverts to previous program state [Ctrl+Z]."""
        if not self._undo_stack:
            return
        self._redo_stack.append([copy.deepcopy(s) for s in self.program.statements])
        self.program.statements = self._undo_stack.pop()
        self.refresh_tree()

    def _redo(self) -> None:
        """Restores undone program state [Ctrl+Y]."""
        if not self._redo_stack:
            return
        self._undo_stack.append([copy.deepcopy(s) for s in self.program.statements])
        self.program.statements = self._redo_stack.pop()
        self.refresh_tree()

    def _add_ptp(self) -> None:
        self._push_undo()
        angles = self.get_current_angles()
        self.program.add_motion(angles, motion_type="PTP")
        self.refresh_tree()

    def _add_lin(self) -> None:
        self._push_undo()
        angles = self.get_current_angles()
        self.program.add_motion(angles, motion_type="LIN")
        self.refresh_tree()

    def _add_gripper_action(self) -> None:
        self._push_undo()
        angles = self.get_current_angles()
        cur_j6 = angles[5] if len(angles) >= 6 else 40
        action = "OPEN" if cur_j6 > 84 else "CLOSE"
        target_angle = 40 if action == "OPEN" else 128
        self.program.add_gripper(action=action, target_angle=target_angle)
        self.refresh_tree()

    def _add_delay(self) -> None:
        self._push_undo()
        self.program.add_delay(duration_ms=500)
        self.refresh_tree()

    def _duplicate_selected(self) -> None:
        """Duplicates the currently selected statement [Ctrl+D]."""
        idx = self._get_selected_index()
        if idx is not None and 0 <= idx < len(self.program.statements):
            self._push_undo()
            new_stmt = copy.deepcopy(self.program.statements[idx])
            if hasattr(new_stmt, "target_id"):
                new_stmt.target_id += "_kopio"
            self.program.statements.insert(idx + 1, new_stmt)
            self.refresh_tree()
            self._select_statement(idx + 1)

    def _teach_selected(self) -> None:
        idx = self._get_selected_index()
        if idx is not None:
            self._push_undo()
            angles = self.get_current_angles()
            self.program.update_statement_pose(idx, angles)
            self.refresh_tree()
        else:
            QMessageBox.information(self, "Huomio", "Valitse ensin listalta käsky, jonka asennon haluat päivittää!")

    def _move_up(self) -> None:
        idx = self._get_selected_index()
        if idx is not None and self.program.move_statement_up(idx):
            self._push_undo()
            self.refresh_tree()
            self._select_statement(idx - 1)

    def _move_down(self) -> None:
        idx = self._get_selected_index()
        if idx is not None and self.program.move_statement_down(idx):
            self._push_undo()
            self.refresh_tree()
            self._select_statement(idx + 1)

    def _delete_selected(self) -> None:
        idx = self._get_selected_index()
        if idx is not None:
            self._push_undo()
            if self.program.remove_statement(idx):
                self.refresh_tree()

    def _clear_all(self) -> None:
        if self.program.statements:
            self._push_undo()
            self.program.clear()
            self.refresh_tree()

    def _select_statement(self, index: int) -> None:
        root = self.tree.topLevelItem(0)
        if root and 0 <= index < root.childCount():
            item = root.child(index)
            self.tree.setCurrentItem(item)

    def _move_robot_to_index(self, idx: int) -> None:
        """Drives robot arm to waypoint pose of specified statement."""
        if 0 <= idx < len(self.program.statements):
            stmt = self.program.statements[idx]
            if isinstance(stmt, MotionStatement):
                self.apply_angles(stmt.angles)
            elif isinstance(stmt, GripperStatement):
                angles = self.get_current_angles()
                angles[5] = stmt.target_angle
                self.apply_angles(angles)

    def _edit_statement(self, idx: int) -> None:
        """Inline dialog for editing speed, duration, or comment."""
        if 0 <= idx < len(self.program.statements):
            stmt = self.program.statements[idx]
            self._push_undo()
            if isinstance(stmt, MotionStatement):
                val, ok = QInputDialog.getInt(self, "Muokkaa nopeutta", f"Nopeus (%) käskylle {stmt.target_id}:", stmt.speed_percent, 1, 100)
                if ok:
                    stmt.speed_percent = val
                    self.refresh_tree()
            elif isinstance(stmt, DelayStatement):
                val, ok = QInputDialog.getInt(self, "Muokkaa viivettä", "Pitoaika / viive (ms):", stmt.duration_ms, 50, 10000, 50)
                if ok:
                    stmt.duration_ms = val
                    self.refresh_tree()
            elif isinstance(stmt, GripperStatement):
                val, ok = QInputDialog.getInt(self, "Muokkaa kourakulmaa", "Kouran asento (40°=auki, 128°=kiinni):", stmt.target_angle, 40, 128)
                if ok:
                    stmt.target_angle = val
                    self.refresh_tree()

    def _show_context_menu(self, pos) -> None:
        """Right-click popup menu for statement operations."""
        item = self.tree.itemAt(pos)
        if not item:
            return
        idx = item.data(0, Qt.UserRole)
        if idx is None or idx < 0 or idx >= len(self.program.statements):
            return

        stmt = self.program.statements[idx]
        menu = QMenu(self)
        menu.setStyleSheet("""
            QMenu {
                background: #252526; color: #cccccc; border: 1px solid #3f3f46;
                padding: 4px; font-size: 11px;
            }
            QMenu::item { padding: 5px 20px; }
            QMenu::item:selected { background: #007acc; color: white; border-radius: 2px; }
        """)

        act_move = menu.addAction("▶ Aja robotti tähän pisteeseen (Move to Pose)")
        act_teach = menu.addAction("🎯 Opeta nykyinen asento tähän (Teach Pose)")

        act_toggle = None
        if isinstance(stmt, MotionStatement):
            next_t = "PTP" if stmt.statement_type == "LIN" else "LIN"
            act_toggle = menu.addAction(f"🔄 Vaihda tyypiksi {next_t}")

        act_edit = menu.addAction("✏ Muokkaa arvoja (Nopeus / Kesto / Kulma)...")
        act_dup = menu.addAction("📋 Monista käsky (Duplicate) [Ctrl+D]")
        menu.addSeparator()
        act_up = menu.addAction("▲ Siirrä ylemmäs (Move Up)")
        act_down = menu.addAction("▼ Siirrä alemmas (Move Down)")
        menu.addSeparator()
        act_del = menu.addAction("🗑 Poista käsky (Delete) [Del]")

        action = menu.exec_(self.tree.viewport().mapToGlobal(pos))
        if action == act_move:
            self._move_robot_to_index(idx)
        elif action == act_teach:
            self._teach_selected()
        elif action == act_toggle and act_toggle:
            self._push_undo()
            stmt.statement_type = "PTP" if stmt.statement_type == "LIN" else "LIN"
            self.refresh_tree()
        elif action == act_edit:
            self._edit_statement(idx)
        elif action == act_dup:
            self._duplicate_selected()
        elif action == act_up:
            self._move_up()
        elif action == act_down:
            self._move_down()
        elif action == act_del:
            self._delete_selected()

    def _on_item_double_clicked(self, item: QTreeWidgetItem, column: int) -> None:
        idx = item.data(0, Qt.UserRole)
        if idx is not None and 0 <= idx < len(self.program.statements):
            if column == 0:
                self._move_robot_to_index(idx)
            else:
                self._edit_statement(idx)

    def _open_weld_wizard(self) -> None:
        """Opens interactive wizard to create Approach -> Touchdown -> Weld -> Retract sequence."""
        angles = self.get_current_angles()
        dlg = WeldWizardDialog(self.program.kinematics, angles, self)
        if dlg.exec_():
            self._push_undo()
            for st_type, data, speed, comment in dlg.result_statements:
                if st_type == "DELAY":
                    self.program.add_delay(duration_ms=data, comment=comment)
                else:
                    self.program.add_motion(data, motion_type=st_type, speed_percent=speed, comment=comment)
            self.refresh_tree()

    def _open_multipass(self) -> None:
        """Opens Multi-Pass Generator dialog (Root, Fill, Cap passes)."""
        angles = self.get_current_angles()
        dlg = MultiPassDialog(self.program.kinematics, angles, self)
        if dlg.exec_():
            self._push_undo()
            for st_type, data, speed, comment in dlg.result_statements:
                if st_type == "DELAY":
                    self.program.add_delay(duration_ms=data, comment=comment)
                else:
                    self.program.add_motion(data, motion_type=st_type, speed_percent=speed, comment=comment)
            self.refresh_tree()

    def _reverse_program(self) -> None:
        """Reverses the order and trajectory direction of welding statements in the program."""
        if not self.program.statements:
            QMessageBox.information(self, "Käännä sauma", "Ohjelmassa ei ole käskyjä käännettäväksi.")
            return
        from src.core.weld_planner import reverse_seam_statements
        self._push_undo()
        self.program.statements = reverse_seam_statements(self.program.statements)
        self.refresh_tree()
        QMessageBox.information(self, "Sauma käännetty", "Hitsausradan suunta on käännetty päinvastaiseksi (alku ↔ loppu).")

    def set_active_tool(self, tool) -> None:
        """Adapts the OLP statement toolbar to the active tool context."""
        tool_type = getattr(tool, "tool_type", "GRIPPER") if tool else "GRIPPER"

        if hasattr(self, "weld_tools_box"):
            self.weld_tools_box.setVisible(tool_type == "WELDING_TORCH")
        if hasattr(self, "gripper_tools_box"):
            self.gripper_tools_box.setVisible(tool_type == "GRIPPER")
        if hasattr(self, "vacuum_tools_box"):
            self.vacuum_tools_box.setVisible(tool_type == "VACUUM")
        if hasattr(self, "dispenser_tools_box"):
            self.dispenser_tools_box.setVisible(tool_type == "DISPENSER")

    def _add_gripper_fixed(self, action: str, angle: int) -> None:
        """Instantly adds a standard Grip or Release statement and applies to 3D arm."""
        self._push_undo()
        self.program.add_gripper(action=action, target_angle=angle)
        cur_angles = list(self.get_current_angles())
        if len(cur_angles) >= 6:
            cur_angles[5] = angle
            self.apply_angles(cur_angles)
        self.refresh_tree()

    def _add_tool_action(self, tool_type: str, action: str) -> None:
        """Adds a tool ON/OFF statement (e.g. Vacuum, Dispenser)."""
        self._push_undo()
        self.program.add_tool_action(tool_type=tool_type, action=action)
        self.refresh_tree()

    def _open_pick_and_place(self) -> None:
        """Opens interactive wizard to create Pick & Place sequence for Gripper or Vacuum."""
        angles = self.get_current_angles()
        active_tool = getattr(self.program.kinematics, "active_tool", None)
        tool_type = getattr(active_tool, "tool_type", "GRIPPER") if active_tool else "GRIPPER"
        dlg = PickAndPlaceDialog(self.program.kinematics, angles, tool_type=tool_type, parent=self)
        if dlg.exec_():
            self._push_undo()
            for item in dlg.result_statements:
                st_type = item[0]
                if st_type == "DELAY":
                    self.program.add_delay(duration_ms=item[1], comment=item[3])
                elif st_type == "GRIPPER":
                    self.program.add_gripper(action=item[1], target_angle=item[2], comment=item[3])
                elif st_type == "TOOL_ACTION":
                    self.program.add_tool_action(tool_type=item[1], action=item[2], comment=item[3])
                else:
                    self.program.add_motion(item[1], motion_type=st_type, speed_percent=item[2], comment=item[3], is_welding=False)
            self.refresh_tree()

    def _save_program(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Tallenna OLP-ohjelma", "", "JSON-ohjelmat (*.json)")
        if path:
            from pathlib import Path
            self.program.save_to_json(Path(path))

    def _load_program(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Avaa OLP-ohjelma", "", "JSON-ohjelmat (*.json)")
        if path:
            from pathlib import Path
            self.program.load_from_json(Path(path))
            self.refresh_tree()

    def _export_code(self) -> None:
        """Exports taught routine to standard industrial formats (G-Code, ABB RAPID, Arduino)."""
        if not self.program.statements:
            QMessageBox.information(self, "Export", "Lisää ensin käskyjä ohjelmalistaan ennen vientiä!")
            return

        from src.core.post_processor import RobotPostProcessor
        filters = "G-Code File (*.nc *.gcode);;Arduino Autonomous Sketch (*.ino);;ABB RAPID Module (*.mod)"
        path, selected_filter = QFileDialog.getSaveFileName(self, "Export Industrial Robot Code", f"{self.program.name}", filters)
        if not path:
            return

        if ".ino" in selected_filter or path.endswith(".ino"):
            code = RobotPostProcessor.to_arduino_sketch(self.program)
        elif ".mod" in selected_filter or path.endswith(".mod"):
            code = RobotPostProcessor.to_abb_rapid(self.program, self.program.kinematics)
        else:
            code = RobotPostProcessor.to_gcode(self.program, self.program.kinematics)

        with open(path, "w", encoding="utf-8") as f:
            f.write(code)

        QMessageBox.information(self, "Export Valmis", f"Ohjelmakoodi viety onnistuneesti tiedostoon:\n{path}")

