"""
Central configuration for Robot Arm Digital Twin / OLP System.
Defines robot kinematics parameters, communication settings, servo limits, and asset paths.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

# Base directories
BASE_DIR = Path(__file__).resolve().parent.parent
ASSETS_DIR = BASE_DIR / "assets"
MODELS_DIR = ASSETS_DIR / "models"

# Serial Communication Settings
DEFAULT_BAUDRATE: int = 115200
DEFAULT_TIMEOUT: float = 0.5  # seconds
SERIAL_THROTTLE_MS: int = 35  # ~28 Hz update rate to prevent serial buffer overflow
PACKET_HEADER: str = "<"
PACKET_FOOTER: str = ">\n"

# Bluetooth Low Energy (BLE) Communication Settings (Matches Arduino Uno R4 WiFi firmware)
BLE_DEVICE_NAME: str = "Arduino-Arm"
BLE_SERVICE_UUID: str = "19B10000-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_UUID: str = "19B10001-E8F2-537E-4F6C-D104768A1214"
BLE_THROTTLE_MS: int = 45  # ~22 Hz update rate matching BLE mobile control

# Kinematic Link Dimensions (in millimeters) - Calibrated to Inventor CAD models
LINK_0_HEIGHT: float = 140.0  # Base height to Shoulder joint (J2)
LINK_1_LENGTH: float = 199.5  # Shoulder to Elbow (J3) - arm1.stl pivot-to-pivot distance
LINK_2_LENGTH: float = 199.5  # Elbow to Wrist Pitch (J4) - arm1.stl pivot-to-pivot distance
LINK_3_LENGTH: float = 149.3  # Wrist Pitch to Gripper tip (J6) (50.0mm ranne flange + 99.3mm Tarttuja tip)

# Joint Names and Limits
JOINT_NAMES = [
    "J1 - Jalusta (Base)",
    "J2 - Olkapää (Shoulder)",
    "J3 - Kyynärpää (Elbow)",
    "J4 - Ranne Pysty (Pitch)",
    "J5 - Ranne Kierto (Roll)",
    "J6 - Koura (Gripper)",
]

# Physical angles limits (min, max) in degrees
# Note: Servo 6 is constrained to 20° - 180° (below 20° gears disengage mechanically)
JOINT_LIMITS: List[Tuple[int, int]] = [
    (0, 180),   # J1
    (0, 180),   # J2
    (0, 180),   # J3
    (0, 180),   # J4
    (0, 180),   # J5
    (20, 180),  # J6 (Gripper safety boundary)
]

# Default Home Angles
DEFAULT_HOME_ANGLES: List[int] = [90, 90, 90, 90, 90, 20]

# STL model filenames mapping
STL_FILES = {
    "base": "base.stl",
    "link1": "link1.stl",
    "link2": "link2.stl",
    "link3": "link3.stl",
    "link4": "link4.stl",
    "link5": "link5.stl",
    "gripper": "gripper.stl",
}

