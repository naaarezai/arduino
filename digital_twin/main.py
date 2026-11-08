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
    """Applies a clean, modern CAD/OLP dark engineering theme to the Qt application."""
    app.setStyle("Fusion")
    palette = QPalette()

    # Refined neutral dark palette (VS Code / Visual Components standard)
    palette.setColor(QPalette.Window, QColor("#1e1e1e"))
    palette.setColor(QPalette.WindowText, QColor("#e0e0e0"))
    palette.setColor(QPalette.Base, QColor("#181818"))
    palette.setColor(QPalette.AlternateBase, QColor("#252526"))
    palette.setColor(QPalette.ToolTipBase, QColor("#2d2d30"))
    palette.setColor(QPalette.ToolTipText, QColor("#ffffff"))
    palette.setColor(QPalette.Text, QColor("#e0e0e0"))
    palette.setColor(QPalette.Button, QColor("#2d2d30"))
    palette.setColor(QPalette.ButtonText, QColor("#e0e0e0"))
    palette.setColor(QPalette.BrightText, QColor("#ffffff"))
    palette.setColor(QPalette.Highlight, QColor("#007acc"))
    palette.setColor(QPalette.HighlightedText, QColor("#ffffff"))

    app.setPalette(palette)

    # Clean typography and subtle border styling across all widgets
    app.setStyleSheet("""
        QWidget {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            font-size: 11px;
            color: #d4d4d4;
        }
        QToolTip {
            background-color: #252526;
            color: #ffffff;
            border: 1px solid #3f3f46;
            padding: 4px 8px;
            border-radius: 3px;
        }
    """)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("RobotArmDigitalTwin")
    setup_dark_theme(app)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

