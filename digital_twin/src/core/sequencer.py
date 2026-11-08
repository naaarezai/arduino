"""
Waypoint Sequencer and Offline Programming (OLP) Engine.
Manages keyframe recording, step durations, sequence playback interpolation,
and saving/loading sequences to JSON.
"""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Callable, List, Optional
import numpy as np


@dataclass
class Keyframe:
    name: str
    angles: List[int]       # J1..J6
    duration_ms: int = 1500 # Interpolation time to reach this pose in ms
    pause_ms: int = 500     # Wait time at this pose before next step


class Sequencer:
    """Manages an ordered list of robot poses (waypoints) and generates interpolated steps."""
    def __init__(self):
        self.keyframes: List[Keyframe] = []
        self._current_step_index: int = 0
        self._is_playing: bool = False
        self._loop: bool = False

    def add_keyframe(self, angles: List[int], name: str = "", duration_ms: int = 1500, pause_ms: int = 500) -> Keyframe:
        step_num = len(self.keyframes) + 1
        if not name:
            name = f"Asento {step_num}"
        kf = Keyframe(name=name, angles=list(angles), duration_ms=duration_ms, pause_ms=pause_ms)
        self.keyframes.append(kf)
        return kf

    def remove_keyframe(self, index: int) -> None:
        if 0 <= index < len(self.keyframes):
            self.keyframes.pop(index)

    def move_keyframe(self, from_idx: int, to_idx: int) -> None:
        if 0 <= from_idx < len(self.keyframes) and 0 <= to_idx < len(self.keyframes):
            kf = self.keyframes.pop(from_idx)
            self.keyframes.insert(to_idx, kf)

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

    @staticmethod
    def interpolate_angles(start_angles: List[int], end_angles: List[int], progress: float) -> List[int]:
        """
        Cosine ease-in-out S-curve interpolation between start and end angles.
        progress: 0.0 to 1.0
        """
        # Cosine ease-in-out formula
        t = (1.0 - np.cos(progress * np.pi)) / 2.0
        return [
            int(round(s + (e - s) * t))
            for s, e in zip(start_angles, end_angles)
        ]

