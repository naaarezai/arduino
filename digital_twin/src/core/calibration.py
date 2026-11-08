"""
Hardware Calibration and Zero-Offset Manager for 6-DOF Robot Arm.
Handles per-joint mechanical trim offsets and direction inversions,
persisting calibration profiles to calibration.json.
"""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import List

from src.config import BASE_DIR, JOINT_LIMITS


@dataclass
class CalibrationProfile:
    """Stores mechanical zero-trim offsets and inversions for all 6 servos."""
    name: str = "Default_Hardware"
    offsets: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    inversions: List[bool] = field(default_factory=lambda: [False, False, False, False, False, False])

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "CalibrationProfile":
        return cls(
            name=data.get("name", "Default_Hardware"),
            offsets=list(data.get("offsets", [0, 0, 0, 0, 0, 0])),
            inversions=list(data.get("inversions", [False, False, False, False, False, False]))
        )


class CalibrationManager:
    """
    Manages loading, saving, and transforming joint angles between
    mathematical simulation angles and physical servo angles.
    """
    def __init__(self, file_path: Path = BASE_DIR / "calibration.json"):
        self.file_path = file_path
        self.profile = CalibrationProfile()
        self.load()

    def load(self) -> bool:
        """Loads calibration from file if exists, otherwise initializes defaults."""
        if self.file_path.is_file():
            try:
                with open(self.file_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.profile = CalibrationProfile.from_dict(data)
                return True
            except Exception:
                pass
        self.profile = CalibrationProfile()
        return False

    def save(self) -> bool:
        """Saves current calibration profile to disk."""
        try:
            self.file_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.file_path, "w", encoding="utf-8") as f:
                json.dump(self.profile.to_dict(), f, indent=2)
            return True
        except Exception:
            return False

    def get_offset(self, joint_idx: int) -> int:
        if 0 <= joint_idx < len(self.profile.offsets):
            return self.profile.offsets[joint_idx]
        return 0

    def set_offset(self, joint_idx: int, value: int) -> None:
        if 0 <= joint_idx < len(self.profile.offsets):
            # Clamp offset to safe range -30° to +30°
            self.profile.offsets[joint_idx] = max(-30, min(30, int(value)))

    def is_inverted(self, joint_idx: int) -> bool:
        if 0 <= joint_idx < len(self.profile.inversions):
            return self.profile.inversions[joint_idx]
        return False

    def set_inverted(self, joint_idx: int, inverted: bool) -> None:
        if 0 <= joint_idx < len(self.profile.inversions):
            self.profile.inversions[joint_idx] = bool(inverted)

    def to_physical(self, sim_angles: List[int]) -> List[int]:
        """
        Converts simulation joint angles to physical servo commands,
        applying inversion and mechanical zero-trim offsets, clamped to servo limits.
        """
        phys = []
        for i, a in enumerate(sim_angles):
            min_v, max_v = JOINT_LIMITS[i]
            val = a
            if self.is_inverted(i):
                val = 180 - val
            val += self.get_offset(i)
            # Enforce hardware servo boundaries (Servo 6: 40° - 128°)
            val = max(min_v, min(max_v, int(round(val))))
            phys.append(val)
        return phys

    def to_simulation(self, phys_angles: List[int]) -> List[int]:
        """Converts physical servo angles back to simulation angles."""
        sim = []
        for i, a in enumerate(phys_angles):
            min_v, max_v = JOINT_LIMITS[i]
            val = a - self.get_offset(i)
            if self.is_inverted(i):
                val = 180 - val
            val = max(min_v, min(max_v, int(round(val))))
            sim.append(val)
        return sim
