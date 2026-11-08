"""
Waypoint Sequencer and Offline Programming (OLP) Engine.
Supports both PTP (Point-to-Point) and LINEAR (Cartesian straight line) motion types,
step durations, and trajectory path generation for welding, gluing, and pick-and-place.
"""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Callable, List, Optional
import numpy as np

from src.core.kinematics import RobotKinematics
from src.core.program import (
    Statement, MotionStatement, GripperStatement, DelayStatement,
    TrajectoryFrame, RobotProgram
)


@dataclass
class Keyframe:
    name: str
    angles: List[int]        # J1..J6
    motion_type: str = "PTP" # "PTP" or "LIN" (Linear straight line)
    duration_ms: int = 1500  # Interpolation duration in ms
    pause_ms: int = 400      # Wait time at this pose before next step (e.g. welding delay)


class Sequencer:
    """Manages an ordered list of robot waypoints with PTP/LIN motion types."""
    def __init__(self, kinematics: Optional[RobotKinematics] = None):
        self.kinematics = kinematics or RobotKinematics()
        self.keyframes: List[Keyframe] = []
        self._current_step_index: int = 0
        self._is_playing: bool = False

    def add_keyframe(self,
                     angles: List[int],
                     name: str = "",
                     motion_type: str = "PTP",
                     duration_ms: int = 1500,
                     pause_ms: int = 400) -> Keyframe:
        step_num = len(self.keyframes) + 1
        if not name:
            type_label = "LIN (Suora)" if motion_type == "LIN" else "PTP (Piste)"
            name = f"Askel {step_num} [{type_label}]"
        kf = Keyframe(
            name=name,
            angles=list(angles),
            motion_type=motion_type,
            duration_ms=duration_ms,
            pause_ms=pause_ms
        )
        self.keyframes.append(kf)
        return kf

    def remove_keyframe(self, index: int) -> None:
        if 0 <= index < len(self.keyframes):
            self.keyframes.pop(index)

    def clear(self) -> None:
        self.keyframes.clear()
        self._current_step_index = 0

    def save_to_json(self, file_path: Path) -> None:
        """Saves current sequence to a JSON file."""
        data = [asdict(kf) for kf in self.keyframes]
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def load_from_json(self, file_path: Path) -> None:
        """Loads sequence from a JSON file."""
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.keyframes = [Keyframe(**item) for item in data]
        self._current_step_index = 0

    def generate_full_trajectory(self, current_arm_angles: List[int]) -> List[List[int]]:
        """
        Builds the complete interpolated list of joint angle frames for all steps in the sequence.
        Interweaves PTP S-curve and LINEAR straight-line steps.
        """
        if not self.keyframes:
            return []

        full_path: List[List[int]] = []
        prev_angles = list(current_arm_angles)

        for kf in self.keyframes:
            if kf.motion_type == "LIN":
                # Linear path in Cartesian 3D space
                segment = self.kinematics.generate_linear_path(prev_angles, kf.angles, step_size_mm=6.0)
                full_path.extend(segment)
            else:
                # PTP S-curve path
                num_ptp_steps = max(5, int(kf.duration_ms / 35.0))
                for s in range(1, num_ptp_steps + 1):
                    prog = s / float(num_ptp_steps)
                    full_path.append(self.interpolate_ptp(prev_angles, kf.angles, prog))

            # Add pause frames (robot holds pose)
            pause_frames = max(1, int(kf.pause_ms / 35.0))
            for _ in range(pause_frames):
                full_path.append(list(kf.angles))

            prev_angles = list(kf.angles)

        return full_path

    @staticmethod
    def interpolate_ptp(start_angles: List[int], end_angles: List[int], progress: float) -> List[int]:
        """Cosine ease-in-out S-curve interpolation."""
        t = (1.0 - np.cos(progress * np.pi)) / 2.0
        return [
            int(round(s + (e - s) * t))
            for s, e in zip(start_angles, end_angles)
        ]
