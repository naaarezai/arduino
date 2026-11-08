"""
Application Entry Point for 6-DOF Robot Arm Digital Twin & OLP System.
"""
import sys
import os
from pathlib import Path

# Ensure package root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Ensure Qt compatibility
if "QT_API" not in os.environ:
    os.environ["QT_API"] = "pyqt5"

from src.ui.qt_compat import QApplication, QPalette, QColor, Qt
from src.ui.main_window import MainWindow


def setup_dark_theme(app: QApplication) -> None:
    """Applies a modern dark engineering theme to the Qt application."""
    app.setStyle("Fusion")
    palette = QPalette()

    # Base colors
    palette.setColor(QPalette.Window, QColor("#1e2227"))
    palette.setColor(QPalette.WindowText, QColor("#abb2bf"))
    palette.setColor(QPalette.Base, QColor("#181a1f"))
    palette.setColor(QPalette.AlternateBase, QColor("#21252b"))
    palette.setColor(QPalette.ToolTipBase, QColor("#282c34"))
    palette.setColor(QPalette.ToolTipText, QColor("#ffffff"))
    palette.setColor(QPalette.Text, QColor("#e06c75"))
    palette.setColor(QPalette.Button, QColor("#282c34"))
    palette.setColor(QPalette.ButtonText, QColor("#abb2bf"))
    palette.setColor(QPalette.BrightText, QColor("#ff4081"))
    palette.setColor(QPalette.Highlight, QColor("#00bcd4"))
    palette.setColor(QPalette.HighlightedText, QColor("#ffffff"))

    app.setPalette(palette)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("RobotArmDigitalTwin")
    setup_dark_theme(app)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

