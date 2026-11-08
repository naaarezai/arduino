"""
Project Workspace Manager for 6-DOF Robot Arm Digital Twin.
Saves and loads complete workcell state (.project.json):
- Robot program (statements, targets, speeds, delays, gripper actions)
- Active tool & TCP configuration profile (offsets, type, angles)
- Workpiece CAD / demo fixture name and 3D transform (pos, yaw)
- Current robot joint angles
"""
from dataclasses import asdict
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from src.core.program import RobotProgram, Statement
from src.core.tool_data import ToolProfile


def export_project_data(
    program: RobotProgram,
    active_tool: Optional[ToolProfile] = None,
    demo_fixture: Optional[str] = None,
    workpiece_stl: Optional[str] = None,
    workpiece_pos: Optional[List[float]] = None,
    workpiece_rot_z: float = 0.0,
    current_angles: Optional[List[int]] = None,
    project_name: str = "RobotProject"
) -> Dict[str, Any]:
    """
    Serializes complete workcell state into a project dictionary.
    """
    return {
        "version": "1.0",
        "project_name": project_name,
        "program": {
            "name": program.name,
            "statements": [s.to_dict() for s in program.statements]
        },
        "active_tool": active_tool.to_dict() if active_tool else None,
        "workpiece": {
            "demo_fixture": demo_fixture,
            "stl_path": workpiece_stl,
            "pos": list(workpiece_pos) if workpiece_pos else [260.0, 0.0, 0.0],
            "rot_z": float(workpiece_rot_z)
        },
        "current_angles": list(current_angles) if current_angles else [90, 90, 90, 90, 90, 40]
    }


def save_project_to_file(file_path: Path, project_data: Dict[str, Any]) -> None:
    """
    Writes project dictionary to a .project.json file.
    """
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(project_data, f, indent=2, ensure_ascii=False)


def load_project_from_file(file_path: Path) -> Dict[str, Any]:
    """
    Reads and validates project dictionary from a .project.json file.
    """
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Projektitiedoston juuren on oltava JSON-objekti.")
    if "program" not in data:
        raise ValueError("Virheellinen projektitiedosto: 'program'-kenttä puuttuu.")
    return data
