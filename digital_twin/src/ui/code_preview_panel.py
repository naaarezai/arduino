"""
Live Robot Code Preview Panel for 6-DOF Robot Arm Digital Twin.
Displays real-time compiled robot code:
1. Arduino Autonomous C++ Sketch (.ino) - ready to upload via Arduino IDE
2. Industrial G-Code (.nc) - ISO 6983 standard for CNC / laser / welding
3. ABB RAPID (.mod) - industrial robot standard
Automatically updates whenever statements are added, moved, or edited.
"""
from pathlib import Path
from typing import Optional

from src.core.program import RobotProgram
from src.core.calibration import CalibrationManager
from src.core.post_processor import RobotPostProcessor
from src.ui.qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QPlainTextEdit, QFileDialog, QApplication, QTimer
)


class CodePreviewPanel(QWidget):
    """
    Live real-time code viewer displaying post-processed robot routines.
    """
    def __init__(self,
                 program: RobotProgram,
                 calibration: Optional[CalibrationManager] = None,
                 parent=None):
        super().__init__(parent)
        self.program = program
        self.calibration = calibration

        self._init_ui()
        self.refresh_code()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # Header toolbar
        tb_layout = QHBoxLayout()
        tb_layout.setSpacing(6)

        lbl = QLabel("Koodi:")
        lbl.setStyleSheet("color: #4fc1ff; font-weight: bold; font-size: 11px;")
        tb_layout.addWidget(lbl)

        self.combo_lang = QComboBox()
        self.combo_lang.addItems([
            "Arduino C++ (.ino)",
            "G-Code (.nc)",
            "ABB RAPID (.mod)"
        ])
        self.combo_lang.setStyleSheet("""
            QComboBox {
                background: #252526; color: #d4d4d4; border: 1px solid #3f3f46;
                border-radius: 3px; padding: 3px 8px; font-size: 11px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView { background: #1e1e1e; color: #ffffff; selection-background-color: #007acc; }
        """)
        self.combo_lang.currentIndexChanged.connect(self.refresh_code)
        tb_layout.addWidget(self.combo_lang)

        tb_layout.addStretch()

        self.btn_copy = QPushButton("📋 Kopioi")
        self.btn_copy.setStyleSheet("""
            QPushButton {
                background: #007acc; color: white; font-weight: 600; font-size: 11px;
                border-radius: 3px; padding: 4px 10px; border: 1px solid #0098ff;
            }
            QPushButton:hover { background: #0098ff; }
        """)
        self.btn_copy.clicked.connect(self._copy_to_clipboard)
        tb_layout.addWidget(self.btn_copy)

        self.btn_save = QPushButton("💾 Tallenna...")
        self.btn_save.setStyleSheet("""
            QPushButton {
                background: #252526; color: #cccccc; font-weight: 600; font-size: 11px;
                border-radius: 3px; padding: 4px 10px; border: 1px solid #3f3f46;
            }
            QPushButton:hover { background: #2d2d30; color: white; }
        """)
        self.btn_save.clicked.connect(self._save_code_file)
        tb_layout.addWidget(self.btn_save)

        layout.addLayout(tb_layout)

        # Code Editor View
        self.editor = QPlainTextEdit()
        self.editor.setReadOnly(True)
        self.editor.setStyleSheet("""
            QPlainTextEdit {
                background: #181818; color: #9cdcfe; font-family: Consolas, 'Courier New', monospace;
                font-size: 11px; border: 1px solid #2d2d30; border-radius: 3px;
                line-height: 1.4;
            }
        """)
        layout.addWidget(self.editor, 1)

    def refresh_code(self) -> None:
        """Regenerates code based on currently selected language."""
        if not self.program:
            self.editor.setPlainText("// Ei aktiivista ohjelmaa.")
            return

        idx = self.combo_lang.currentIndex()
        if idx == 0:  # Arduino C++
            code = RobotPostProcessor.to_arduino_sketch(self.program, self.calibration)
        elif idx == 1:  # G-Code
            code = RobotPostProcessor.to_gcode(self.program, getattr(self.program, "kinematics", None))
        else:  # ABB RAPID
            code = RobotPostProcessor.to_abb_rapid(self.program, getattr(self.program, "kinematics", None))

        self.editor.setPlainText(code)

    def _copy_to_clipboard(self) -> None:
        """Copies editor content to OS clipboard."""
        text = self.editor.toPlainText()
        if text:
            clipboard = QApplication.clipboard()
            clipboard.setText(text)
            orig_text = self.btn_copy.text()
            self.btn_copy.setText("✅ Kopioitu!")
            self.btn_copy.setStyleSheet("background: #107c41; color: white; font-weight: bold; padding: 4px 10px;")
            QTimer.singleShot(1500, lambda: self._reset_copy_btn(orig_text))

    def _reset_copy_btn(self, orig_text: str) -> None:
        self.btn_copy.setText(orig_text)
        self.btn_copy.setStyleSheet("""
            QPushButton {
                background: #007acc; color: white; font-weight: 600; font-size: 11px;
                border-radius: 3px; padding: 4px 10px; border: 1px solid #0098ff;
            }
            QPushButton:hover { background: #0098ff; }
        """)

    def _save_code_file(self) -> None:
        """Saves current code view to selected file."""
        idx = self.combo_lang.currentIndex()
        if idx == 0:
            flt = "Arduino Sketch (*.ino);;C++ File (*.cpp);;All Files (*.*)"
            def_ext = "ino"
        elif idx == 1:
            flt = "G-Code File (*.nc *.gcode);;All Files (*.*)"
            def_ext = "nc"
        else:
            flt = "ABB RAPID (*.mod);;All Files (*.*)"
            def_ext = "mod"

        path, _ = QFileDialog.getSaveFileName(self, "Tallenna koodi", f"{self.program.name}.{def_ext}", flt)
        if path:
            try:
                Path(path).write_text(self.editor.toPlainText(), encoding="utf-8")
            except Exception:
                pass
