"""
Qt Compatibility layer ensuring transparent compatibility between PyQt6 and PyQt5
across different Windows environments.
"""
import os

# Set fallback default if not specified
if "QT_API" not in os.environ:
    os.environ["QT_API"] = "pyqt5"

try:
    import PyQt5.QtCore
    from PyQt5.QtCore import (
        QMutex, QMutexLocker, QThread, pyqtSignal, pyqtSlot, QTimer, Qt, QSize
    )
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
        QLabel, QSlider, QSpinBox, QPushButton, QComboBox, QGroupBox, 
        QTableWidget, QTableWidgetItem, QHeaderView, QSplitter, QStatusBar,
        QMessageBox, QFileDialog, QFrame, QCheckBox, QTabWidget,
        QRadioButton, QButtonGroup, QDoubleSpinBox,
        QTreeWidget, QTreeWidgetItem, QProgressBar, QToolBar, QAction
    )
    from PyQt5.QtGui import QColor, QFont, QIcon, QPalette
except Exception as e:
    # Fallback to PyQt6
    from PyQt6.QtCore import (
        QMutex, QMutexLocker, QThread, pyqtSignal, pyqtSlot, QTimer, Qt, QSize
    )
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
        QLabel, QSlider, QSpinBox, QPushButton, QComboBox, QGroupBox, 
        QTableWidget, QTableWidgetItem, QHeaderView, QSplitter, QStatusBar,
        QMessageBox, QFileDialog, QFrame, QCheckBox, QTabWidget,
        QRadioButton, QButtonGroup, QDoubleSpinBox,
        QTreeWidget, QTreeWidgetItem, QProgressBar, QToolBar
    )
    from PyQt6.QtGui import QColor, QFont, QIcon, QPalette, QAction

