"""
Tool & TCP Configuration Dialog (Visual Components / OLP style).
Allows user to snap end-effectors to the robot mounting flange,
configure Tool Center Point (TCP) coordinates (X, Y, Z, Pitch, Roll),
choose presets (Gripper, Welding Torch, Dispenser, Vacuum), or import custom STL tools.
"""
from pathlib import Path
from typing import Callable, Optional
import math
import numpy as np

from src.core.tool_data import ToolProfile, ToolManager, TOOL_PRESETS
from src.ui.qt_compat import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QDoubleSpinBox, QLineEdit, QFileDialog, QGroupBox,
    QFrame, Qt
)


class ToolManagerDialog(QDialog):
    """
    Industrial Tool & TCP Setup Dialog.
    Enables mounting/snapping tools to the robot flange and calibrating the Tool Center Point (TCP).
    """
    def __init__(self,
                 tool_manager: ToolManager,
                 on_apply_callback: Optional[Callable[[ToolProfile], None]] = None,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle("Työkalun ja TCP:n Hallinta (Tool & TCP Setup)")
        self.setMinimumWidth(520)
        self.tool_manager = tool_manager
        self.on_apply_callback = on_apply_callback
        self.current_tool = ToolProfile.from_dict(tool_manager.active_tool.to_dict())

        self.setStyleSheet("""
            QDialog { background: #1e1e1e; color: #d4d4d4; }
            QLabel { color: #cccccc; font-size: 11px; }
            QGroupBox {
                border: 1px solid #333333; border-radius: 4px;
                margin-top: 10px; padding: 12px; font-weight: 600; color: #4fc1ff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QComboBox, QLineEdit, QDoubleSpinBox {
                background: #252526; color: #ffffff; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 4px 8px; font-size: 11px;
            }
            QComboBox:focus, QLineEdit:focus, QDoubleSpinBox:focus {
                border-color: #007acc;
            }
            QPushButton {
                background: #252526; color: #cccccc; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 6px 12px; font-size: 11px; font-weight: 600;
            }
            QPushButton:hover { background: #2d2d30; color: #ffffff; border-color: #007acc; }
        """)

        self._init_ui()
        self._load_from_profile(self.current_tool)

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        # Header description
        lbl_info = QLabel(
            "Valitse valmis teollisuustyökalu (Hitsauspoltin, Koura, Annostelija) "
            "tai tuo oma STL-malli. 'Snap to Flange' kohdistaa työkalun robotin kiinnityslaippaan, "
            "ja TCP-koordinaatit ohjaavat tarkkaa työskentelypistettä."
        )
        lbl_info.setWordWrap(True)
        lbl_info.setStyleSheet("color: #9cdcfe; font-size: 11px; padding: 4px; background: #252526; border-radius: 3px;")
        layout.addWidget(lbl_info)

        # 1. Preset Selector Group
        grp_preset = QGroupBox("1. Työkalun tyyppi ja esiasetus")
        l_preset = QVBoxLayout(grp_preset)

        h_pre = QHBoxLayout()
        h_pre.addWidget(QLabel("Valitse työkalu:"))
        self.cmb_preset = QComboBox()
        self.cmb_preset.addItem("2-Leukakoura (Standard Gripper)", "GRIPPER")
        self.cmb_preset.addItem("Hitsauspoltin (MIG/MAG Torch 45°)", "WELDING_TORCH")
        self.cmb_preset.addItem("Annostelusuutin / Liimakynä (Dispenser)", "DISPENSER")
        self.cmb_preset.addItem("Imukuppitarttuja (Vacuum Gripper)", "VACUUM")
        self.cmb_preset.addItem("Oma STL-työkalu (Custom STL)", "CUSTOM_STL")
        self.cmb_preset.currentIndexChanged.connect(self._on_preset_changed)
        h_pre.addWidget(self.cmb_preset, 1)
        l_preset.addLayout(h_pre)

        # Custom STL file row
        self.frame_custom = QFrame()
        h_cust = QHBoxLayout(self.frame_custom)
        h_cust.setContentsMargins(0, 4, 0, 0)
        self.txt_stl_path = QLineEdit()
        self.txt_stl_path.setPlaceholderText("Valitse STL-tiedosto...")
        self.txt_stl_path.textChanged.connect(self._on_custom_path_changed)
        h_cust.addWidget(self.txt_stl_path, 1)

        btn_browse = QPushButton("Selaa...")
        btn_browse.clicked.connect(self._browse_stl)
        h_cust.addWidget(btn_browse)

        self.btn_snap = QPushButton("Snap to Flange")
        self.btn_snap.setStyleSheet("color: #4fc1ff; border-color: #007acc;")
        self.btn_snap.clicked.connect(self._snap_stl_to_flange)
        h_cust.addWidget(self.btn_snap)

        l_preset.addWidget(self.frame_custom)
        layout.addWidget(grp_preset)

        # 2. TCP Coordinates Group (Tool Center Point)
        grp_tcp = QGroupBox("2. Työkalun kärkikoordinaatit (Tool Center Point - TCP Offset)")
        l_tcp = QVBoxLayout(grp_tcp)

        lbl_tcp_desc = QLabel("Koordinaatit ranteen kiinnityslaipasta (Tool Flange Z=0) työkalun kärkeen:")
        lbl_tcp_desc.setStyleSheet("color: #858585; font-size: 10px;")
        l_tcp.addWidget(lbl_tcp_desc)

        # X, Y, Z
        h_xyz = QHBoxLayout()
        h_xyz.addWidget(QLabel("X (mm):"))
        self.spn_x = QDoubleSpinBox()
        self.spn_x.setRange(-400.0, 400.0)
        self.spn_x.setSingleStep(1.0)
        self.spn_x.valueChanged.connect(self._update_preview)
        h_xyz.addWidget(self.spn_x)

        h_xyz.addWidget(QLabel("Y (mm):"))
        self.spn_y = QDoubleSpinBox()
        self.spn_y.setRange(-400.0, 400.0)
        self.spn_y.setSingleStep(1.0)
        self.spn_y.valueChanged.connect(self._update_preview)
        h_xyz.addWidget(self.spn_y)

        h_xyz.addWidget(QLabel("Z (mm):"))
        self.spn_z = QDoubleSpinBox()
        self.spn_z.setRange(0.0, 600.0)
        self.spn_z.setSingleStep(1.0)
        self.spn_z.valueChanged.connect(self._update_preview)
        h_xyz.addWidget(self.spn_z)
        l_tcp.addLayout(h_xyz)

        # Pitch, Roll
        h_rot = QHBoxLayout()
        h_rot.addWidget(QLabel("Kallistus (Pitch °):"))
        self.spn_pitch = QDoubleSpinBox()
        self.spn_pitch.setRange(-180.0, 180.0)
        self.spn_pitch.setSingleStep(5.0)
        self.spn_pitch.valueChanged.connect(self._update_preview)
        h_rot.addWidget(self.spn_pitch)

        h_rot.addWidget(QLabel("Kierto (Roll °):"))
        self.spn_roll = QDoubleSpinBox()
        self.spn_roll.setRange(-180.0, 180.0)
        self.spn_roll.setSingleStep(5.0)
        self.spn_roll.valueChanged.connect(self._update_preview)
        h_rot.addWidget(self.spn_roll)

        btn_autodetect = QPushButton("Etsi kärki (Autodetect Tip)")
        btn_autodetect.clicked.connect(self._autodetect_tip)
        h_rot.addWidget(btn_autodetect)
        l_tcp.addLayout(h_rot)

        layout.addWidget(grp_tcp)

        # 3. Summary Card
        self.lbl_summary = QLabel()
        self.lbl_summary.setStyleSheet("""
            background: #25282e; color: #4fc1ff; font-family: monospace; font-size: 11px;
            padding: 8px 12px; border-radius: 4px; border: 1px solid #3f3f46;
        """)
        layout.addWidget(self.lbl_summary)

        # Action Buttons
        h_btn = QHBoxLayout()
        h_btn.addStretch()

        btn_cancel = QPushButton("Peruuta")
        btn_cancel.clicked.connect(self.reject)
        h_btn.addWidget(btn_cancel)

        btn_apply = QPushButton("Tallenna ja Ota Käyttöön")
        btn_apply.setStyleSheet("""
            QPushButton {
                background: #007acc; color: white; font-weight: 700; border: none;
                border-radius: 3px; padding: 7px 18px; font-size: 11px;
            }
            QPushButton:hover { background: #0098ff; }
        """)
        btn_apply.clicked.connect(self._save_and_apply)
        h_btn.addWidget(btn_apply)

        layout.addLayout(h_btn)

    def _load_from_profile(self, tool: ToolProfile):
        idx = self.cmb_preset.findData(tool.tool_type)
        if idx >= 0:
            self.cmb_preset.setCurrentIndex(idx)
        else:
            self.cmb_preset.setCurrentIndex(self.cmb_preset.findData("CUSTOM_STL"))

        if tool.mesh_path:
            self.txt_stl_path.setText(tool.mesh_path)

        self.spn_x.setValue(tool.tcp_x)
        self.spn_y.setValue(tool.tcp_y)
        self.spn_z.setValue(tool.tcp_z)
        self.spn_pitch.setValue(tool.tcp_pitch_deg)
        self.spn_roll.setValue(tool.tcp_roll_deg)
        self._update_preview()

    def _on_preset_changed(self, idx):
        ttype = self.cmb_preset.currentData()
        self.frame_custom.setVisible(ttype == "CUSTOM_STL")

        if ttype in TOOL_PRESETS:
            preset = TOOL_PRESETS[ttype]
            self.spn_x.setValue(preset.tcp_x)
            self.spn_y.setValue(preset.tcp_y)
            self.spn_z.setValue(preset.tcp_z)
            self.spn_pitch.setValue(preset.tcp_pitch_deg)
            self.spn_roll.setValue(preset.tcp_roll_deg)
            self.current_tool.color = preset.color
            self.current_tool.name = preset.name
            self.current_tool.tool_type = preset.tool_type
            self.current_tool.mesh_path = None
        else:
            self.current_tool.tool_type = "CUSTOM_STL"
            self.current_tool.name = "Oma STL-työkalu"
            self.current_tool.color = "#00e5ff"

        self._update_preview()

    def _browse_stl(self):
        f, _ = QFileDialog.getOpenFileName(
            self, "Valitse työkalun STL-malli", "", "3D STL Files (*.stl)"
        )
        if f:
            self.txt_stl_path.setText(f)
            self._snap_stl_to_flange()

    def _on_custom_path_changed(self, txt):
        self.current_tool.mesh_path = txt.strip()
        self._update_preview()

    def _snap_stl_to_flange(self):
        """Analyzes imported STL mesh, snaps its bottom flush to Z=0 flange, and sets default TCP."""
        p = self.txt_stl_path.text().strip()
        if not p or not Path(p).is_file():
            return

        try:
            import pyvista as pv
            m = pv.read(p)
            dz = float(m.bounds[5] - m.bounds[4])
            # Set default TCP to tip of mesh
            self.spn_z.setValue(round(dz, 1))
            self.spn_x.setValue(0.0)
            self.spn_y.setValue(0.0)
            self.current_tool.name = Path(p).stem
            self._update_preview()
        except Exception:
            pass

    def _autodetect_tip(self):
        """Finds the highest vertex along Z in the custom tool mesh."""
        p = self.txt_stl_path.text().strip()
        if p and Path(p).is_file():
            try:
                import pyvista as pv
                m = pv.read(p)
                pts = m.points
                highest_idx = np.argmax(pts[:, 2])
                top_pt = pts[highest_idx]
                z_len = top_pt[2] - m.bounds[4]
                self.spn_z.setValue(round(z_len, 1))
                self.spn_x.setValue(round(top_pt[0] - m.center[0], 1))
                self.spn_y.setValue(round(top_pt[1] - m.center[1], 1))
                self._update_preview()
            except Exception:
                pass

    def _update_preview(self):
        x = self.spn_x.value()
        y = self.spn_y.value()
        z = self.spn_z.value()
        pitch = self.spn_pitch.value()
        roll = self.spn_roll.value()
        reach = math.sqrt(x*x + y*y + z*z)

        ttype = self.cmb_preset.currentData()
        name = self.cmb_preset.currentText()

        self.lbl_summary.setText(
            f"Aktiivinen työkalu: {name}\n"
            f"TCP Offset: X={x:+.1f} mm | Y={y:+.1f} mm | Z={z:+.1f} mm | Pitch={pitch:+.1f}° | Roll={roll:+.1f}°\n"
            f"Työkalun pituus: {reach:.1f} mm kiinnityslaipasta"
        )

    def _save_and_apply(self):
        self.current_tool.tool_type = self.cmb_preset.currentData()
        self.current_tool.name = self.cmb_preset.currentText()
        self.current_tool.tcp_x = self.spn_x.value()
        self.current_tool.tcp_y = self.spn_y.value()
        self.current_tool.tcp_z = self.spn_z.value()
        self.current_tool.tcp_pitch_deg = self.spn_pitch.value()
        self.current_tool.tcp_roll_deg = self.spn_roll.value()
        if self.current_tool.tool_type == "CUSTOM_STL":
            self.current_tool.mesh_path = self.txt_stl_path.text().strip()

        self.tool_manager.save_tool(self.current_tool)
        if self.on_apply_callback:
            self.on_apply_callback(self.current_tool)
        self.accept()
