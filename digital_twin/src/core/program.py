"""
Robot Program & Statement Model (Visual Components / OLP style).
Represents robot routines as structured Statements (PTP, LIN, Set Gripper, Delay)
within a program tree (MainRoutine).
"""
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

from src.core.kinematics import RobotKinematics


@dataclass
class Statement:
    """Base class for all robot program statements."""
    statement_type: str
    comment: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Statement":
        st_type = data.get("statement_type", "PTP")
        if st_type in ("PTP", "LIN"):
            return MotionStatement(
                statement_type=st_type,
                target_id=data.get("target_id", "P1"),
                angles=list(data.get("angles", [90, 90, 90, 90, 90, 40])),
                speed_percent=data.get("speed_percent", 100),
                duration_ms=data.get("duration_ms", 1500),
                comment=data.get("comment", "")
            )
        elif st_type == "GRIPPER":
            return GripperStatement(
                action=data.get("action", "CLOSE"),
                target_angle=data.get("target_angle", 128),
                duration_ms=data.get("duration_ms", 500),
                comment=data.get("comment", "")
            )
        elif st_type == "DELAY":
            return DelayStatement(
                duration_ms=data.get("duration_ms", 500),
                comment=data.get("comment", "")
            )
        return MotionStatement("PTP", "P1", [90, 90, 90, 90, 90, 40])


@dataclass
class MotionStatement(Statement):
    """PTP or LINEAR robot motion statement."""
    statement_type: str = "PTP"  # "PTP" or "LIN"
    target_id: str = "P1"
    angles: List[int] = field(default_factory=lambda: [90, 90, 90, 90, 90, 40])
    speed_percent: int = 100
    duration_ms: int = 1500
    comment: str = ""

    def __post_init__(self):
        if not self.comment:
            if self.statement_type == "LIN":
                self.comment = f"Hitsaussauma / Suora ({self.target_id})"
            else:
                self.comment = f"Siirto pisteeseen {self.target_id}"


@dataclass
class GripperStatement(Statement):
    """Gripper actuation statement (Open, Close, Set Angle)."""
    statement_type: str = "GRIPPER"
    action: str = "CLOSE"         # "CLOSE" (128°), "OPEN" (40°), or "CUSTOM"
    target_angle: int = 128
    duration_ms: int = 500
    comment: str = ""

    def __post_init__(self):
        if not self.comment:
            self.comment = "Koura kiinni (Ote)" if self.action == "CLOSE" else "Koura auki (Päästä)"


@dataclass
class DelayStatement(Statement):
    """Dwell / delay statement (e.g. welding spot time or wait)."""
    statement_type: str = "DELAY"
    duration_ms: int = 500
    comment: str = "Hitsausaika / Odotus"


@dataclass
class TrajectoryFrame:
    """A single interpolated time-step frame for simulation and Arduino execution."""
    angles: List[int]          # [j1, j2, j3, j4, j5, j6]
    statement_idx: int        # Index of statement in tree
    statement_label: str      # Human readable label
    is_welding: bool = False  # True during LIN motions


class RobotProgram:
    """
    Manages the robot statement routine (MainRoutine),
    generates smooth trajectories (PTP S-curve & LINEAR straight paths),
    and handles JSON project serialization.
    """
    def __init__(self, kinematics: Optional[RobotKinematics] = None, name: str = "MainRoutine"):
        self.name = name
        self.kinematics = kinematics or RobotKinematics()
        self.statements: List[Statement] = []
        self._target_counter: int = 1

    def add_motion(self,
                   angles: List[int],
                   motion_type: str = "PTP",
                   target_id: Optional[str] = None,
                   speed_percent: int = 100,
                   duration_ms: int = 1500,
                   comment: str = "") -> MotionStatement:
        if not target_id:
            target_id = f"P{self._target_counter}"
            self._target_counter += 1

        stmt = MotionStatement(
            statement_type=motion_type,
            target_id=target_id,
            angles=list(angles),
            speed_percent=speed_percent,
            duration_ms=duration_ms,
            comment=comment
        )
        self.statements.append(stmt)
        return stmt

    def add_gripper(self, action: str = "CLOSE", target_angle: Optional[int] = None, comment: str = "") -> GripperStatement:
        if target_angle is None:
            target_angle = 128 if action == "CLOSE" else 40
        stmt = GripperStatement(
            action=action,
            target_angle=target_angle,
            duration_ms=500,
            comment=comment
        )
        self.statements.append(stmt)
        return stmt

    def add_delay(self, duration_ms: int = 500, comment: str = "") -> DelayStatement:
        stmt = DelayStatement(duration_ms=duration_ms, comment=comment or f"Odotus {duration_ms} ms")
        self.statements.append(stmt)
        return stmt

    def remove_statement(self, index: int) -> bool:
        if 0 <= index < len(self.statements):
            self.statements.pop(index)
            return True
        return False

    def move_statement_up(self, index: int) -> bool:
        if 1 <= index < len(self.statements):
            self.statements[index - 1], self.statements[index] = self.statements[index], self.statements[index - 1]
            return True
        return False

    def move_statement_down(self, index: int) -> bool:
        if 0 <= index < len(self.statements) - 1:
            self.statements[index + 1], self.statements[index] = self.statements[index], self.statements[index + 1]
            return True
        return False

    def update_statement_pose(self, index: int, new_angles: List[int]) -> bool:
        if 0 <= index < len(self.statements):
            stmt = self.statements[index]
            if isinstance(stmt, MotionStatement):
                stmt.angles = list(new_angles)
                return True
        return False

    def clear(self) -> None:
        self.statements.clear()
        self._target_counter = 1

    def generate_full_trajectory(self,
                                 current_angles: List[int],
                                 speed_multiplier: float = 1.0) -> List[TrajectoryFrame]:
        """
        Compiles the entire statement routine into a seamless sequence of 35ms frames.
        Interweaves PTP S-curve paths, LINEAR Cartesian lines, gripper actions, and pauses.
        """
        if not self.statements:
            return []

        frames: List[TrajectoryFrame] = []
        curr = list(current_angles)
        step_dt_ms = 35.0 / max(0.2, min(5.0, speed_multiplier))

        for idx, stmt in enumerate(self.statements):
            if isinstance(stmt, MotionStatement):
                target = list(stmt.angles)
                is_lin = (stmt.statement_type == "LIN")
                label = f"{stmt.statement_type} {stmt.target_id}"

                if is_lin:
                    # Cartesian straight line
                    segment = self.kinematics.generate_linear_path(curr, target, step_size_mm=6.0)
                    for frame_angles in segment:
                        frames.append(TrajectoryFrame(
                            angles=frame_angles,
                            statement_idx=idx,
                            statement_label=label,
                            is_welding=True
                        ))
                else:
                    # Joint S-curve interpolation
                    eff_duration = max(200, int(stmt.duration_ms / speed_multiplier))
                    steps = max(5, int(eff_duration / step_dt_ms))
                    for s in range(1, steps + 1):
                        prog = s / float(steps)
                        t_curve = (1.0 - math.cos(prog * math.pi)) / 2.0
                        interp_angles = [
                            int(round(c + (tg - c) * t_curve))
                            for c, tg in zip(curr, target)
                        ]
                        frames.append(TrajectoryFrame(
                            angles=interp_angles,
                            statement_idx=idx,
                            statement_label=label,
                            is_welding=False
                        ))
                curr = target

            elif isinstance(stmt, GripperStatement):
                label = f"Koura: {stmt.action}"
                start_j6 = curr[5] if len(curr) >= 6 else 40
                end_j6 = stmt.target_angle
                eff_duration = max(200, int(stmt.duration_ms / speed_multiplier))
                steps = max(4, int(eff_duration / step_dt_ms))

                for s in range(1, steps + 1):
                    prog = s / float(steps)
                    cur_j6 = int(round(start_j6 + (end_j6 - start_j6) * prog))
                    step_angles = list(curr)
                    step_angles[5] = cur_j6
                    frames.append(TrajectoryFrame(
                        angles=step_angles,
                        statement_idx=idx,
                        statement_label=label,
                        is_welding=False
                    ))
                curr[5] = end_j6

            elif isinstance(stmt, DelayStatement):
                label = f"Odotus {stmt.duration_ms} ms"
                eff_duration = max(100, int(stmt.duration_ms / speed_multiplier))
                pause_steps = max(2, int(eff_duration / step_dt_ms))
                for _ in range(pause_steps):
                    frames.append(TrajectoryFrame(
                        angles=list(curr),
                        statement_idx=idx,
                        statement_label=label,
                        is_welding=False
                    ))

        return frames

    def save_to_json(self, file_path: Path) -> None:
        data = {
            "name": self.name,
            "statements": [s.to_dict() for s in self.statements]
        }
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def load_from_json(self, file_path: Path) -> None:
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.name = data.get("name", "MainRoutine")
        self.statements = [Statement.from_dict(item) for item in data.get("statements", [])]
        self._target_counter = len(self.statements) + 1
