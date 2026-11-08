"""
Tool and Tool Center Point (TCP) Data Model for 6-DOF Robot Arm Digital Twin.
Defines end-effector profiles (Gripper, Welding Torch, Dispenser, Custom STL),
flange snap transformations, and persistent JSON configuration.
"""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class ToolProfile:
    """Configuration profile for a robot end-of-arm tool (EOAT)."""
    name: str = "2-Leukakoura (Standard Gripper)"
    tool_type: str = "GRIPPER"  # "GRIPPER", "WELDING_TORCH", "DISPENSER", "VACUUM", "CUSTOM_STL"
    tcp_x: float = 0.0          # Tool Center Point X offset from flange (mm)
    tcp_y: float = 0.0          # Tool Center Point Y offset from flange (mm)
    tcp_z: float = 99.3         # Tool Center Point Z offset from flange (mm)
    tcp_pitch_deg: float = 0.0  # Tool tilt / inclination angle (deg)
    tcp_roll_deg: float = 0.0   # Tool rotation around axis (deg)
    weight_kg: float = 0.25     # Tool weight for payload calculations
    mesh_path: Optional[str] = None  # Path to custom STL file if tool_type == "CUSTOM_STL"
    color: str = "#00e5ff"      # Tool accent color

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> "ToolProfile":
        return cls(
            name=data.get("name", "2-Leukakoura (Standard Gripper)"),
            tool_type=data.get("tool_type", "GRIPPER"),
            tcp_x=float(data.get("tcp_x", 0.0)),
            tcp_y=float(data.get("tcp_y", 0.0)),
            tcp_z=float(data.get("tcp_z", 99.3)),
            tcp_pitch_deg=float(data.get("tcp_pitch_deg", 0.0)),
            tcp_roll_deg=float(data.get("tcp_roll_deg", 0.0)),
            weight_kg=float(data.get("weight_kg", 0.25)),
            mesh_path=data.get("mesh_path"),
            color=data.get("color", "#00e5ff")
        )


# Industrial standard tool presets
TOOL_PRESETS: Dict[str, ToolProfile] = {
    "GRIPPER": ToolProfile(
        name="2-Leukakoura (Standard Gripper)",
        tool_type="GRIPPER",
        tcp_x=0.0,
        tcp_y=0.0,
        tcp_z=99.3,
        tcp_pitch_deg=0.0,
        tcp_roll_deg=0.0,
        weight_kg=0.22,
        color="#00e5ff"
    ),
    "WELDING_TORCH": ToolProfile(
        name="Hitsauspoltin (MIG/MAG Torch 45°)",
        tool_type="WELDING_TORCH",
        tcp_x=35.0,
        tcp_y=0.0,
        tcp_z=135.0,
        tcp_pitch_deg=45.0,
        tcp_roll_deg=0.0,
        weight_kg=0.35,
        color="#ff9100"
    ),
    "DISPENSER": ToolProfile(
        name="Annostelusuutin / Liimakynä (Dispenser)",
        tool_type="DISPENSER",
        tcp_x=0.0,
        tcp_y=0.0,
        tcp_z=115.0,
        tcp_pitch_deg=0.0,
        tcp_roll_deg=0.0,
        weight_kg=0.18,
        color="#00e676"
    ),
    "VACUUM": ToolProfile(
        name="Imukuppitarttuja (Vacuum Gripper)",
        tool_type="VACUUM",
        tcp_x=0.0,
        tcp_y=0.0,
        tcp_z=65.0,
        tcp_pitch_deg=0.0,
        tcp_roll_deg=0.0,
        weight_kg=0.12,
        color="#e040fb"
    ),
}


class ToolManager:
    """Manages active tool profile, persistence to JSON, and preset switching."""
    CONFIG_FILE = Path(__file__).resolve().parent.parent.parent / "active_tool.json"

    def __init__(self):
        self.active_tool: ToolProfile = self.load_tool()

    def load_tool(self) -> ToolProfile:
        if self.CONFIG_FILE.is_file():
            try:
                with open(self.CONFIG_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    return ToolProfile.from_dict(data)
            except Exception:
                pass
        return ToolProfile()

    def save_tool(self, tool: ToolProfile) -> None:
        self.active_tool = tool
        try:
            with open(self.CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(tool.to_dict(), f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def set_preset(self, preset_key: str) -> ToolProfile:
        if preset_key in TOOL_PRESETS:
            tool = ToolProfile.from_dict(TOOL_PRESETS[preset_key].to_dict())
            self.save_tool(tool)
            return tool
        return self.active_tool
