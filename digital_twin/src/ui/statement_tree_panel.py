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
        super().__init__("Program Statement Tree (OLP)", parent)
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

        self.btn_add_ptp = QPushButton("+ PTP Motion")
        self.btn_add_ptp.setStyleSheet(btn_style_ptp)
        self.btn_add_ptp.setToolTip("Lisää nykyinen asento PTP-siirtona")
        self.btn_add_ptp.clicked.connect(self._add_ptp)
        tb_layout.addWidget(self.btn_add_ptp)

        self.btn_add_lin = QPushButton("+ Linear Motion")
        self.btn_add_lin.setStyleSheet(btn_style_lin)
        self.btn_add_lin.setToolTip("Lisää suora Cartesian-hitsaussauma")
        self.btn_add_lin.clicked.connect(self._add_lin)
        tb_layout.addWidget(self.btn_add_lin)

        self.btn_add_grip = QPushButton("+ Gripper")
        self.btn_add_grip.setStyleSheet(btn_style_act)
        self.btn_add_grip.setToolTip("Lisää kouran avaus- tai sulkukäsky")
        self.btn_add_grip.clicked.connect(self._add_gripper_action)
        tb_layout.addWidget(self.btn_add_grip)

        self.btn_add_delay = QPushButton("+ Dwell")
        self.btn_add_delay.setStyleSheet(btn_style_act)
        self.btn_add_delay.setToolTip("Lisää pito/viive hitsauspisteeseen")
        self.btn_add_delay.clicked.connect(self._add_delay)
        tb_layout.addWidget(self.btn_add_delay)

        tb_layout.addStretch()
        layout.addLayout(tb_layout)

        # 2. Main Statement Tree
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
        layout.addWidget(self.tree, 1)

        # 3. Action Toolbar (Teach, Up, Down, Delete, Save/Load)
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
        self.btn_teach.setToolTip("Päivitä valitun käskyn kohteeksi robotin nykyinen asento")
        self.btn_teach.clicked.connect(self._teach_selected)
        act_layout.addWidget(self.btn_teach)

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

        self.btn_delete = QPushButton("Delete")
        self.btn_delete.setToolTip("Poista valittu käsky")
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

        layout.addLayout(act_layout)

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
                tag = "LIN" if is_lin else "PTP"
                item.setText(0, f"{num_str}  [{tag}]  {stmt.target_id}")
                color = QColor("#ff9d3b") if is_lin else QColor("#4fc1ff")
                item.setForeground(0, color)

                pose = self.program.kinematics.forward_kinematics([float(a) for a in stmt.angles])
                item.setText(1, f"TCP: [{pose.tcp_x:.1f}, {pose.tcp_y:.1f}, {pose.tcp_z:.1f}] | J:{stmt.angles}")
                item.setText(2, f"{stmt.speed_percent}% ({stmt.duration_ms} ms)")

            elif isinstance(stmt, GripperStatement):
                item.setText(0, f"{num_str}  [GRIPPER]  {stmt.action}")
                item.setForeground(0, QColor("#c586c0"))
                item.setText(1, f"Jaw angle: {stmt.target_angle}° | {stmt.comment}")
                item.setText(2, f"{stmt.duration_ms} ms")

            elif isinstance(stmt, DelayStatement):
                item.setText(0, f"{num_str}  [WAIT]  {stmt.duration_ms} ms")
                item.setForeground(0, QColor("#dcdcaa"))
                item.setText(1, stmt.comment)
                item.setText(2, f"{stmt.duration_ms} ms")

        if self.on_program_changed:
            self.on_program_changed()

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

