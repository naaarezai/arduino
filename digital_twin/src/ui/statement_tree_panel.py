"""
Statement Tree Panel for Robot Offline Programming (Visual Components style).
Displays the robot program as a structured tree (MainRoutine) with dedicated icons,
badges, statement toolbar (Add PTP, Add LIN, Gripper, Delay, Teach), and reordering.
"""
from typing import Callable, List, Optional
from src.core.kinematics import RobotKinematics
from src.core.program import (
    RobotProgram, Statement, MotionStatement, GripperStatement, DelayStatement
)
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTreeWidget, QTreeWidgetItem, QHeaderView, QGroupBox,
    QFileDialog, QMessageBox, Qt, QColor, QComboBox, QSpinBox
)


class StatementTreePanel(QGroupBox):
    """
    Statement Tree view mimicking the Visual Components Robot Programming / OLP interface.
    """
    def __init__(self,
                 program: RobotProgram,
                 get_current_angles_fn: Callable[[], List[int]],
                 apply_angles_fn: Callable[[List[int]], None],
                 on_program_changed_fn: Optional[Callable[[], None]] = None,
                 parent=None):
        super().__init__("Ohjelmalista (Statement Tree / OLP)", parent)
        self.program = program
        self.get_current_angles = get_current_angles_fn
        self.apply_angles = apply_angles_fn
        self.on_program_changed = on_program_changed_fn

        self._init_ui()
        self.refresh_tree()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 1. Statement Quick-Add Toolbar (Visual Components Ribbon style)
        tb_layout = QHBoxLayout()
        tb_layout.setSpacing(4)

        btn_style_ptp = """
            QPushButton {
                background: #00838f; color: white; font-weight: bold; font-size: 11px;
                border: 1px solid #00acc1; border-radius: 4px; padding: 5px 8px;
            }
            QPushButton:hover { background: #00acc1; }
        """
        btn_style_lin = """
            QPushButton {
                background: #e65100; color: white; font-weight: bold; font-size: 11px;
                border: 1px solid #fb8c00; border-radius: 4px; padding: 5px 8px;
            }
            QPushButton:hover { background: #fb8c00; }
        """
        btn_style_act = """
            QPushButton {
                background: #37474f; color: #eceff1; font-weight: bold; font-size: 11px;
                border: 1px solid #455a64; border-radius: 4px; padding: 5px 8px;
            }
            QPushButton:hover { background: #455a64; border-color: #00bcd4; }
        """

        self.btn_add_ptp = QPushButton("➕ PTP (Piste)")
        self.btn_add_ptp.setStyleSheet(btn_style_ptp)
        self.btn_add_ptp.setToolTip("Lisää nykyinen asento PTP-siirtona")
        self.btn_add_ptp.clicked.connect(self._add_ptp)
        tb_layout.addWidget(self.btn_add_ptp)

        self.btn_add_lin = QPushButton("━ LIN (Hitsaus)")
        self.btn_add_lin.setStyleSheet(btn_style_lin)
        self.btn_add_lin.setToolTip("Lisää suora Cartesian-hitsaussauma")
        self.btn_add_lin.clicked.connect(self._add_lin)
        tb_layout.addWidget(self.btn_add_lin)

        self.btn_add_grip = QPushButton("✊ Koura")
        self.btn_add_grip.setStyleSheet(btn_style_act)
        self.btn_add_grip.setToolTip("Lisää kouran avaus- tai sulkukäsky")
        self.btn_add_grip.clicked.connect(self._add_gripper_action)
        tb_layout.addWidget(self.btn_add_grip)

        self.btn_add_delay = QPushButton("⏳ Viive")
        self.btn_add_delay.setStyleSheet(btn_style_act)
        self.btn_add_delay.setToolTip("Lisää pito/viive hitsauspisteeseen")
        self.btn_add_delay.clicked.connect(self._add_delay)
        tb_layout.addWidget(self.btn_add_delay)

        tb_layout.addStretch()
        layout.addLayout(tb_layout)

        # 2. Main Statement Tree
        self.tree = QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["Käsky / Liike", "Sijainti / Tiedot", "Nopeus / Aika"])
        header = self.tree.header()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.tree.setStyleSheet("""
            QTreeWidget {
                background: #181a1f; color: #eceff1; border: 1px solid #37474f;
                border-radius: 4px; font-size: 11px;
            }
            QTreeWidget::item {
                padding: 4px;
            }
            QTreeWidget::item:selected {
                background: #2c3840; color: #00e5ff; font-weight: bold;
            }
            QHeaderView::section {
                background: #21252b; color: #90a4ae; font-weight: bold;
                border: 1px solid #282c34; padding: 4px; font-size: 11px;
            }
        """)
        self.tree.itemDoubleClicked.connect(self._on_item_double_clicked)
        layout.addWidget(self.tree, 1)

        # 3. Action Toolbar (Teach, Up, Down, Delete, Save/Load)
        act_layout = QHBoxLayout()
        act_layout.setSpacing(4)

        self.btn_teach = QPushButton("🎯 Teach (Päivitä asento)")
        self.btn_teach.setStyleSheet("""
            QPushButton {
                background: #00897b; color: white; font-weight: bold; font-size: 11px;
                border-radius: 3px; padding: 4px 8px;
            }
            QPushButton:hover { background: #00bfa5; }
        """)
        self.btn_teach.setToolTip("Päivitä valitun käskyn kohteeksi robotin nykyinen asento")
        self.btn_teach.clicked.connect(self._teach_selected)
        act_layout.addWidget(self.btn_teach)

        self.btn_up = QPushButton("⬆️")
        self.btn_up.setToolTip("Siirrä ylöspäin")
        self.btn_up.setStyleSheet(btn_style_act)
        self.btn_up.clicked.connect(self._move_up)
        act_layout.addWidget(self.btn_up)

        self.btn_down = QPushButton("⬇️")
        self.btn_down.setToolTip("Siirrä alaspäin")
        self.btn_down.setStyleSheet(btn_style_act)
        self.btn_down.clicked.connect(self._move_down)
        act_layout.addWidget(self.btn_down)

        self.btn_delete = QPushButton("🗑️")
        self.btn_delete.setToolTip("Poista valittu käsky")
        self.btn_delete.setStyleSheet("background: #5a2020; color: white; border-radius: 3px; padding: 4px 8px;")
        self.btn_delete.clicked.connect(self._delete_selected)
        act_layout.addWidget(self.btn_delete)

        self.btn_clear = QPushButton("🧹 Tyhjennä")
        self.btn_clear.setStyleSheet(btn_style_act)
        self.btn_clear.clicked.connect(self._clear_all)
        act_layout.addWidget(self.btn_clear)

        act_layout.addStretch()

        self.btn_save = QPushButton("💾 Tallenna")
        self.btn_save.setStyleSheet(btn_style_act)
        self.btn_save.clicked.connect(self._save_program)
        act_layout.addWidget(self.btn_save)

        self.btn_load = QPushButton("📂 Avaa")
        self.btn_load.setStyleSheet(btn_style_act)
        self.btn_load.clicked.connect(self._load_program)
        act_layout.addWidget(self.btn_load)

        layout.addLayout(act_layout)

    def refresh_tree(self) -> None:
        """Rebuilds the Statement Tree representing MainRoutine."""
        self.tree.clear()

        # Root Item: MainRoutine
        root = QTreeWidgetItem(self.tree)
        root.setText(0, f"📂 {self.program.name} ({len(self.program.statements)} käskyä)")
        root.setForeground(0, QColor("#e0e0e0"))
        root.setExpanded(True)

        for i, stmt in enumerate(self.program.statements):
            item = QTreeWidgetItem(root)
            item.setData(0, Qt.UserRole, i)

            if isinstance(stmt, MotionStatement):
                is_lin = (stmt.statement_type == "LIN")
                icon = "━" if is_lin else "●"
                item.setText(0, f"{i+1}: {icon} {stmt.statement_type} {stmt.target_id}")
                color = QColor("#ff9800") if is_lin else QColor("#00e5ff")
                item.setForeground(0, color)

                # Pose details
                pose = self.program.kinematics.forward_kinematics([float(a) for a in stmt.angles])
                item.setText(1, f"XYZ: [{pose.tcp_x:.0f}, {pose.tcp_y:.0f}, {pose.tcp_z:.0f}] mm | J:{stmt.angles}")
                item.setText(2, f"{stmt.speed_percent}% ({stmt.duration_ms} ms)")

            elif isinstance(stmt, GripperStatement):
                item.setText(0, f"{i+1}: ✊ Koura {stmt.action}")
                item.setForeground(0, QColor("#ba68c8"))  # Purple
                item.setText(1, f"Kulma: {stmt.target_angle}° | {stmt.comment}")
                item.setText(2, f"{stmt.duration_ms} ms")

            elif isinstance(stmt, DelayStatement):
                item.setText(0, f"{i+1}: ⏳ Viive / Odotus")
                item.setForeground(0, QColor("#ffd54f"))  # Yellow
                item.setText(1, stmt.comment)
                item.setText(2, f"{stmt.duration_ms} ms")

        if self.on_program_changed:
            self.on_program_changed()

    def highlight_statement(self, index: int) -> None:
        """Visually marks current running statement during simulation."""
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

    def _add_ptp(self) -> None:
        angles = self.get_current_angles()
        self.program.add_motion(angles, motion_type="PTP")
        self.refresh_tree()

    def _add_lin(self) -> None:
        angles = self.get_current_angles()
        self.program.add_motion(angles, motion_type="LIN")
        self.refresh_tree()

    def _add_gripper_action(self) -> None:
        angles = self.get_current_angles()
        cur_j6 = angles[5] if len(angles) >= 6 else 40
        action = "OPEN" if cur_j6 > 84 else "CLOSE"
        target_angle = 40 if action == "OPEN" else 128
        self.program.add_gripper(action=action, target_angle=target_angle)
        self.refresh_tree()

    def _add_delay(self) -> None:
        self.program.add_delay(duration_ms=500)
        self.refresh_tree()

    def _teach_selected(self) -> None:
        idx = self._get_selected_index()
        if idx is not None:
            angles = self.get_current_angles()
            self.program.update_statement_pose(idx, angles)
            self.refresh_tree()
        else:
            QMessageBox.information(self, "Huomio", "Valitse ensin listalta käsky, jonka asennon haluat päivittää!")

    def _move_up(self) -> None:
        idx = self._get_selected_index()
        if idx is not None and self.program.move_statement_up(idx):
            self.refresh_tree()
            self._select_statement(idx - 1)

    def _move_down(self) -> None:
        idx = self._get_selected_index()
        if idx is not None and self.program.move_statement_down(idx):
            self.refresh_tree()
            self._select_statement(idx + 1)

    def _delete_selected(self) -> None:
        idx = self._get_selected_index()
        if idx is not None and self.program.remove_statement(idx):
            self.refresh_tree()

    def _clear_all(self) -> None:
        if self.program.statements:
            self.program.clear()
            self.refresh_tree()

    def _select_statement(self, index: int) -> None:
        root = self.tree.topLevelItem(0)
        if root and 0 <= index < root.childCount():
            item = root.child(index)
            self.tree.setCurrentItem(item)

    def _on_item_double_clicked(self, item: QTreeWidgetItem, column: int) -> None:
        idx = item.data(0, Qt.UserRole)
        if idx is not None and 0 <= idx < len(self.program.statements):
            stmt = self.program.statements[idx]
            if isinstance(stmt, MotionStatement):
                # Move 3D robot directly to this waypoint pose!
                self.apply_angles(stmt.angles)
            elif isinstance(stmt, GripperStatement):
                angles = self.get_current_angles()
                angles[5] = stmt.target_angle
                self.apply_angles(angles)

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
