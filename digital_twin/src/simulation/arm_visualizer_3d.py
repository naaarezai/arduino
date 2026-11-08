"""
3D Simulation Viewport for 6-DOF Robot Arm Digital Twin.
Integrates PyVistaQt QtInteractor into PyQt, rendering calibrated CAD STL models
with animated opening/closing gripper fingers and hardware-accelerated 60 FPS posing.
"""
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import math
import time
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor

from src.config import (
    LINK_0_HEIGHT, LINK_1_LENGTH, LINK_2_LENGTH, LINK_3_LENGTH,
    MODELS_DIR, STL_FILES
)
from src.core.kinematics import PoseData, translation_matrix
from src.core.tool_data import ToolProfile, ToolManager
from src.ui.qt_compat import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel


class ArmVisualizer3D(QWidget):
    """
    Interactive 3D Viewport displaying the robot arm digital twin.
    Supports free mouse rotation, zoom, panning, and real-time kinematic posing.
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # PyVista QtInteractor
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)

        # Style plotter environment
        self.plotter.set_background("#16181c", top="#242830")  # Professional dark studio gradient
        self.plotter.show_axes()
        self.plotter.enable_anti_aliasing('fxaa')

        # Industrial studio lighting setup
        light_key = pv.Light(position=(900, -900, 1200), focal_point=(0, 0, 200), intensity=0.85, color="white")
        light_fill = pv.Light(position=(-700, 700, 800), focal_point=(0, 0, 200), intensity=0.4, color="#b0bec5")
        self.plotter.add_light(light_key)
        self.plotter.add_light(light_fill)

        # Actors storage
        self.actors: List[pv.Actor] = []
        self.gripper_base_actor: Optional[pv.Actor] = None
        self.gripper_left_actor: Optional[pv.Actor] = None
        self.gripper_right_actor: Optional[pv.Actor] = None
        self.welding_torch_actor: Optional[pv.Actor] = None
        self.dispenser_actor: Optional[pv.Actor] = None
        self.vacuum_actor: Optional[pv.Actor] = None
        self.custom_tool_actor: Optional[pv.Actor] = None
        self.active_tool_actor: Optional[pv.Actor] = None
        self.tool_manager = ToolManager()
        self.active_tool: ToolProfile = self.tool_manager.active_tool
        self.tcp_actor: Optional[pv.Actor] = None
        self.tcp_triad_actors: List[pv.Actor] = []
        self.workpiece_actor: Optional[pv.Actor] = None
        self.workpiece_bounds: Optional[tuple] = None
        self.raw_workpiece_mesh: Optional[pv.PolyData] = None
        self.workpiece_pos: List[float] = [260.0, 0.0, 0.0]
        self.workpiece_rot_z: float = 0.0
        self.is_snapping_active: bool = False
        self.temp_marker_actors: List[pv.Actor] = []
        self.weld_bead_actors: List[pv.Actor] = []
        self.weld_points: List[List[float]] = []
        self.envelope_actor: Optional[pv.Actor] = None
        self.trajectory_actors: List[pv.Actor] = []
        self.gizmo_widget = None
        self._is_dragging_gizmo: bool = False
        self._last_drag_time: float = 0.0
        self._current_tcp = [0.0, 0.0, 688.3]

        # Add floor grid plane
        self._add_floor()

        # Build robot links and gripper
        self._init_robot_actors()

        # Initial camera view
        self.reset_camera()

    def _add_floor(self) -> None:
        """Adds industrial CAD floor platform, concentric distance rings, and coordinate cross."""
        floor = pv.Disc(center=(0, 0, -2), inner=0, outer=550, normal=(0, 0, 1), r_res=60)
        self.plotter.add_mesh(floor, color="#1c2025", opacity=0.92, show_edges=False)

        # Concentric distance rings (100, 200, 300, 400, 500 mm)
        for r in range(100, 550, 100):
            ring = pv.Disc(center=(0, 0, -1.5), inner=r - 0.7, outer=r + 0.7, normal=(0, 0, 1), r_res=60)
            self.plotter.add_mesh(ring, color="#2e3842", opacity=0.55)

        # Reach boundary ring (575 mm)
        reach_ring = pv.Disc(center=(0, 0, -1.0), inner=574, outer=576, normal=(0, 0, 1), r_res=80)
        self.plotter.add_mesh(reach_ring, color="#00bcd4", opacity=0.4)

        # Coordinate axes cross on the floor
        axis_x = pv.Line((-500, 0, -1), (500, 0, -1)).tube(radius=0.6)
        axis_y = pv.Line((0, -500, -1), (0, 500, -1)).tube(radius=0.6)
        self.plotter.add_mesh(axis_x, color="#ff5252", opacity=0.4)
        self.plotter.add_mesh(axis_y, color="#69f0ae", opacity=0.4)

    def _init_robot_actors(self) -> None:
        """Loads STL models with exact kinematic alignment, normal smoothing, and industrial PBR materials."""
        MODELS_DIR.mkdir(parents=True, exist_ok=True)

        # Coordinate alignment matrix: maps STL axes to kinematic joint frame
        # STL: X (pivot axis) -> Y_kin, Y (link length) -> Z_kin, Z (link normal) -> X_kin
        mat_align = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # 1. Base (fixed cylinder base on the floor)
        base_f = MODELS_DIR / "base.stl"
        if base_f.is_file():
            base_m = pv.read(str(base_f))
            base_m = base_m.translate((-base_m.center[0], -base_m.center[1], -base_m.bounds[4]), inplace=False)
            base_m = base_m.compute_normals(auto_orient_normals=True)
        else:
            base_m = self._make_procedural_base()
        self.actors.append(self.plotter.add_mesh(base_m, color="#25282e", specular=0.3, specular_power=15, smooth_shading=True))

        # 2. Link 1 (Turntable top plate sitting at Z = 100 on top of base)
        l1_f = MODELS_DIR / "link1.stl"
        if l1_f.is_file():
            l1_m = pv.read(str(l1_f))
            l1_m = l1_m.translate((-l1_m.center[0], -l1_m.center[1], 100.0 - l1_m.bounds[4]), inplace=False)
            l1_m = l1_m.compute_normals(auto_orient_normals=True)
        else:
            l1_m = self._make_procedural_link1()
        self.actors.append(self.plotter.add_mesh(l1_m, color="#374151", specular=0.4, specular_power=20, smooth_shading=True))

        # 3. Link 2 (Upper Arm - arm1.stl): Industrial high-vis cyan
        l2_f = MODELS_DIR / "link2.stl"
        if l2_f.is_file():
            l2_m = pv.read(str(l2_f))
            # Bottom pivot hole is at Y = -15.0; translate by +15.0 to origin
            l2_m = l2_m.translate((0.0, 15.0, 0.0), inplace=False)
            l2_m = l2_m.transform(mat_align, inplace=False)
            l2_m = l2_m.compute_normals(auto_orient_normals=True)
        else:
            l2_m = self._make_procedural_link2()
        self.actors.append(self.plotter.add_mesh(l2_m, color="#0284c7", specular=0.45, specular_power=25, smooth_shading=True))

        # 4. Link 3 (Forearm - arm1.stl): Precision silver-white
        l3_f = MODELS_DIR / "link3.stl"
        if not l3_f.is_file() and l2_f.is_file():
            l3_f = l2_f
        if l3_f.is_file():
            l3_m = pv.read(str(l3_f))
            # Bottom pivot hole is at Y = -15.0; translate by +15.0 to origin
            l3_m = l3_m.translate((0.0, 15.0, 0.0), inplace=False)
            l3_m = l3_m.transform(mat_align, inplace=False)
            l3_m = l3_m.compute_normals(auto_orient_normals=True)
        else:
            l3_m = self._make_procedural_link3()
        self.actors.append(self.plotter.add_mesh(l3_m, color="#e2e8f0", specular=0.45, specular_power=20, smooth_shading=True))

        # 5. Link 4 (Ranne - ranne.stl): Industrial orange safety accent
        l4_f = MODELS_DIR / "link4.stl"
        if l4_f.is_file():
            l4_m = pv.read(str(l4_f))
            # Pivot hole in ranne.stl is already at (0, 0, 0)
            l4_m = l4_m.transform(mat_align, inplace=False)
            l4_m = l4_m.compute_normals(auto_orient_normals=True)
        else:
            l4_m = self._make_procedural_link4()
        self.actors.append(self.plotter.add_mesh(l4_m, color="#ea580c", specular=0.4, specular_power=20, smooth_shading=True))

        # 6. Gripper Assembly (Tarttuja.stl - contains mount flange, servo, and animated claws)
        grip_f = MODELS_DIR / "gripper.stl"
        if grip_f.is_file():
            grip_m = pv.read(str(grip_f))
            # Center X = 16.5, mount face Y = 42.3, Center Z = 39.11
            grip_m = grip_m.translate((-16.5, -42.3, -39.11), inplace=False)
            grip_m = grip_m.rotate_x(-90, point=(0, 0, 0), inplace=False)
            grip_m = grip_m.compute_normals(auto_orient_normals=True)

            bodies = grip_m.split_bodies()
            base_poly = pv.PolyData()
            left_poly = pv.PolyData()
            right_poly = pv.PolyData()

            for b in bodies:
                cx = b.center[0]
                cz = b.center[2]
                if cx < -12.0 and cz > 25.0:
                    left_poly = left_poly + b
                elif cx > 12.0 and cz > 25.0:
                    right_poly = right_poly + b
                else:
                    base_poly = base_poly + b

            self.gripper_base_actor = self.plotter.add_mesh(base_poly, color="#334155", specular=0.4, smooth_shading=True)
            self.gripper_left_actor = self.plotter.add_mesh(left_poly, color="#94a3b8", specular=0.6, specular_power=30, smooth_shading=True)
            self.gripper_right_actor = self.plotter.add_mesh(right_poly, color="#94a3b8", specular=0.6, specular_power=30, smooth_shading=True)
        else:
            grip_m = self._make_procedural_gripper()
            self.gripper_base_actor = self.plotter.add_mesh(grip_m, color="#94a3b8", smooth_shading=True)

        # TCP Coordinate Indicator Sphere
        tcp_sphere = pv.Sphere(radius=6, center=(0, 0, 0))
        self.tcp_actor = self.plotter.add_mesh(
            tcp_sphere,
            color=self.active_tool.color,
            ambient=0.9,
            render_points_as_spheres=True
        )

        # 3D Tool Orientation Triad (Red=X, Green=Y, Blue=Z axes at TCP)
        self.tcp_triad_actors = []
        ax_len = 22.0
        r_tube = 1.1
        mx = pv.Line((0, 0, 0), (ax_len, 0, 0)).tube(radius=r_tube)
        my = pv.Line((0, 0, 0), (0, ax_len, 0)).tube(radius=r_tube)
        mz = pv.Line((0, 0, 0), (0, 0, ax_len)).tube(radius=r_tube)
        self.tcp_triad_actors.append(self.plotter.add_mesh(mx, color="#ff1744", ambient=0.9))
        self.tcp_triad_actors.append(self.plotter.add_mesh(my, color="#00e676", ambient=0.9))
        self.tcp_triad_actors.append(self.plotter.add_mesh(mz, color="#2979ff", ambient=0.9))

        # Initialize active tool if not default gripper
        if self.active_tool.tool_type != "GRIPPER":
            self.set_active_tool(self.active_tool)

    # ---------------- Procedural Fallbacks ----------------
    def _make_procedural_base(self) -> pv.PolyData:
        plate = pv.Cylinder(center=(0, 0, 10), direction=(0, 0, 1), radius=90, height=20, resolution=50)
        ring = pv.Cylinder(center=(0, 0, 25), direction=(0, 0, 1), radius=60, height=10, resolution=50)
        return plate + ring

    def _make_procedural_link1(self) -> pv.PolyData:
        turret = pv.Cylinder(center=(0, 0, 35), direction=(0, 0, 1), radius=50, height=50, resolution=50)
        pylon_left = pv.Cube(center=(-25, 0, 85), x_length=16, y_length=45, z_length=90)
        pylon_right = pv.Cube(center=(25, 0, 85), x_length=16, y_length=45, z_length=90)
        return turret + pylon_left + pylon_right

    def _make_procedural_link2(self) -> pv.PolyData:
        joint_bearing = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=30, height=45, resolution=40)
        beam = pv.Cube(center=(0, 0, self.l1 / 2.0), x_length=30, y_length=35, z_length=self.l1)
        elbow_bearing = pv.Cylinder(center=(0, 0, self.l1), direction=(0, 1, 0), radius=26, height=45, resolution=40)
        return joint_bearing + beam + elbow_bearing

    def _make_procedural_link3(self) -> pv.PolyData:
        joint_bearing = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=24, height=40, resolution=40)
        beam = pv.Cube(center=(0, 0, self.l2 / 2.0), x_length=24, y_length=28, z_length=self.l2)
        wrist_bearing = pv.Cylinder(center=(0, 0, self.l2), direction=(0, 1, 0), radius=20, height=36, resolution=40)
        return joint_bearing + beam + wrist_bearing

    def _make_procedural_link4(self) -> pv.PolyData:
        knuckle = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=18, height=32, resolution=30)
        shaft = pv.Cylinder(center=(0, 0, 22), direction=(0, 0, 1), radius=15, height=45, resolution=30)
        return knuckle + shaft

    def _make_procedural_link5(self) -> pv.PolyData:
        collar = pv.Cylinder(center=(0, 0, 15), direction=(0, 0, 1), radius=22, height=30, resolution=30)
        flange = pv.Cylinder(center=(0, 0, 32), direction=(0, 0, 1), radius=28, height=6, resolution=30)
        return collar + flange

    def _make_procedural_gripper(self) -> pv.PolyData:
        palm = pv.Cube(center=(0, 0, 15), x_length=55, y_length=25, z_length=30)
        finger_l = pv.Cube(center=(-18, 0, 55), x_length=8, y_length=16, z_length=50)
        finger_r = pv.Cube(center=(18, 0, 55), x_length=8, y_length=16, z_length=50)
        return palm + finger_l + finger_r

    def _make_welding_torch(self) -> pv.PolyData:
        c1 = pv.Cylinder(center=(0, 0, 10), direction=(0, 0, 1), radius=18, height=20, resolution=30)
        pts = np.array([
            [0, 0, 20],
            [0, 0, 50],
            [8, 0, 80],
            [25, 0, 110],
            [35, 0, 125]
        ], dtype=np.float64)
        spline = pv.Spline(pts, 30)
        neck = spline.tube(radius=6.0, n_sides=20)
        nozzle = pv.Cylinder(center=(32, 0, 120), direction=(10, 0, 15), radius=8.0, height=22, resolution=24)
        wire = pv.Line((32, 0, 120), (35, 0, 135)).tube(radius=0.9, n_sides=12)
        return c1 + neck + nozzle + wire

    def _make_dispenser(self) -> pv.PolyData:
        collar = pv.Cylinder(center=(0, 0, 12), direction=(0, 0, 1), radius=16, height=24, resolution=30)
        barrel = pv.Cylinder(center=(0, 0, 45), direction=(0, 0, 1), radius=10, height=45, resolution=30)
        taper = pv.Cone(center=(0, 0, 72), direction=(0, 0, 1), radius=10, height=12, resolution=30)
        needle = pv.Cylinder(center=(0, 0, 95), direction=(0, 0, 1), radius=1.2, height=40, resolution=20)
        return collar + barrel + taper + needle

    def _make_vacuum(self) -> pv.PolyData:
        mount = pv.Cylinder(center=(0, 0, 15), direction=(0, 0, 1), radius=14, height=30, resolution=30)
        bellows1 = pv.Cylinder(center=(0, 0, 36), direction=(0, 0, 1), radius=18, height=12, resolution=30)
        bellows2 = pv.Cylinder(center=(0, 0, 48), direction=(0, 0, 1), radius=22, height=12, resolution=30)
        lip = pv.Disc(center=(0, 0, 65), inner=0, outer=24, normal=(0, 0, 1), r_res=30)
        return mount + bellows1 + bellows2 + lip

    def set_active_tool(self, tool: ToolProfile) -> None:
        """
        Dynamically snaps the specified tool to the robot tool flange,
        updating the 3D viewport representation and TCP marker.
        """
        self.active_tool = tool
        ttype = tool.tool_type

        # Hide all tool actors first
        if self.gripper_base_actor: self.gripper_base_actor.SetVisibility(False)
        if self.gripper_left_actor: self.gripper_left_actor.SetVisibility(False)
        if self.gripper_right_actor: self.gripper_right_actor.SetVisibility(False)
        if self.welding_torch_actor: self.welding_torch_actor.SetVisibility(False)
        if self.dispenser_actor: self.dispenser_actor.SetVisibility(False)
        if self.vacuum_actor: self.vacuum_actor.SetVisibility(False)
        if self.custom_tool_actor: self.custom_tool_actor.SetVisibility(False)

        if ttype == "GRIPPER":
            if self.gripper_base_actor: self.gripper_base_actor.SetVisibility(True)
            if self.gripper_left_actor: self.gripper_left_actor.SetVisibility(True)
            if self.gripper_right_actor: self.gripper_right_actor.SetVisibility(True)
            self.active_tool_actor = self.gripper_base_actor
        elif ttype == "WELDING_TORCH":
            if not self.welding_torch_actor:
                mesh = self._make_welding_torch()
                self.welding_torch_actor = self.plotter.add_mesh(
                    mesh, color="#ff9100", specular=0.45, specular_power=25, smooth_shading=True
                )
            self.welding_torch_actor.SetVisibility(True)
            self.active_tool_actor = self.welding_torch_actor
        elif ttype == "DISPENSER":
            if not self.dispenser_actor:
                mesh = self._make_dispenser()
                self.dispenser_actor = self.plotter.add_mesh(
                    mesh, color="#00e676", specular=0.45, specular_power=25, smooth_shading=True
                )
            self.dispenser_actor.SetVisibility(True)
            self.active_tool_actor = self.dispenser_actor
        elif ttype == "VACUUM":
            if not self.vacuum_actor:
                mesh = self._make_vacuum()
                self.vacuum_actor = self.plotter.add_mesh(
                    mesh, color="#e040fb", specular=0.45, specular_power=25, smooth_shading=True
                )
            self.vacuum_actor.SetVisibility(True)
            self.active_tool_actor = self.vacuum_actor
        elif ttype == "CUSTOM_STL":
            if self.custom_tool_actor:
                try:
                    self.plotter.remove_actor(self.custom_tool_actor)
                except Exception:
                    pass
                self.custom_tool_actor = None
            if tool.mesh_path and Path(tool.mesh_path).is_file():
                mesh = pv.read(tool.mesh_path)
                mesh = mesh.translate((-mesh.center[0], -mesh.center[1], -mesh.bounds[4]), inplace=False)
                mesh = mesh.compute_normals(auto_orient_normals=True)
                self.custom_tool_actor = self.plotter.add_mesh(
                    mesh, color=tool.color, specular=0.45, specular_power=25, smooth_shading=True
                )
                self.custom_tool_actor.SetVisibility(True)
                self.active_tool_actor = self.custom_tool_actor

        if self.tcp_actor:
            try:
                self.tcp_actor.GetProperty().SetColor(pv.Color(tool.color).float_rgb)
            except Exception:
                pass

        self.plotter.render()

    @property
    def l0(self) -> float:
        return LINK_0_HEIGHT

    @property
    def l1(self) -> float:
        return LINK_1_LENGTH

    @property
    def l2(self) -> float:
        return LINK_2_LENGTH

    @property
    def l3(self) -> float:
        return LINK_3_LENGTH

    def update_pose(self, pose: PoseData, joint_angles: Optional[List[int]] = None) -> None:
        """
        Updates each 3D link actor matrix in real time.
        Animates gripper fingers opening and closing with J6 angle (40°=open, 128°=closed).
        """
        # Update links 0..5
        for i, actor in enumerate(self.actors):
            if i < len(pose.link_matrices):
                actor.user_matrix = pose.link_matrices[i]

        # Update Gripper base or Active Tool actor (index 6 in link_matrices)
        mat_gripper = pose.link_matrices[6] if len(pose.link_matrices) > 6 else pose.link_matrices[-1]

        if self.active_tool.tool_type == "GRIPPER":
            if self.gripper_base_actor:
                self.gripper_base_actor.user_matrix = mat_gripper

            # Calculate claw finger opening offset
            open_offset = 14.0
            if joint_angles and len(joint_angles) >= 6:
                j6 = float(joint_angles[5])
                # 40 deg = fully open (~44mm tip gap), 128 deg = tightly closed (~0mm tip gap)
                progress = max(0.0, min(1.0, (j6 - 40.0) / (128.0 - 40.0)))
                # Model default tip gap is 11.68mm. Offset -5.8mm closes it; +16.0mm opens it wide.
                open_offset = (1.0 - progress) * 21.8 - 5.8

            if self.gripper_left_actor and self.gripper_right_actor:
                # Shift left and right fingers laterally along X
                mat_l = mat_gripper @ np.array([
                    [1.0, 0.0, 0.0, -open_offset],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ], dtype=np.float64)

                mat_r = mat_gripper @ np.array([
                    [1.0, 0.0, 0.0, open_offset],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ], dtype=np.float64)

                self.gripper_left_actor.user_matrix = mat_l
                self.gripper_right_actor.user_matrix = mat_r
        else:
            if self.active_tool_actor:
                self.active_tool_actor.user_matrix = mat_gripper

        # Update TCP point
        self._current_tcp = [pose.tcp_x, pose.tcp_y, pose.tcp_z]
        if self.tcp_actor:
            t_tcp = np.eye(4, dtype=np.float64)
            t_tcp[0, 3] = pose.tcp_x
            t_tcp[1, 3] = pose.tcp_y
            t_tcp[2, 3] = pose.tcp_z
            self.tcp_actor.user_matrix = t_tcp

        # Update 3D orientation triad at tool tip
        mat_flange = pose.link_matrices[-1]
        for act in self.tcp_triad_actors:
            act.user_matrix = mat_flange

        # If Gizmo is active, sync it to TCP when not actively being dragged (watchdog < 70ms)
        if self.gizmo_widget:
            is_actively_dragging = getattr(self, "_is_dragging_gizmo", False) and (time.perf_counter() - getattr(self, "_last_drag_time", 0.0) < 0.07)
            if not is_actively_dragging:
                self._is_dragging_gizmo = False
                try:
                    self.gizmo_widget.SetCenter(tuple(self._current_tcp))
                except Exception:
                    pass

        self.plotter.render()

    def set_gizmo_enabled(self,
                           enabled: bool,
                           on_jog_callback: Optional[any] = None,
                           kinematics: Optional[any] = None,
                           get_angles_fn: Optional[any] = None) -> None:
        """
        Activates or deactivates the 3D Interactive TCP Jog Gizmo.
        When active, the user can click and drag the 3D sphere directly with the mouse
        in the 3D viewport to manipulate the robot arm using Inverse Kinematics.
        Equipped with reach boundary clamping and 60 FPS event throttling to eliminate
        freezing, lag, and dead zones.
        """
        try:
            self.plotter.clear_sphere_widgets()
        except Exception:
            pass
        self.gizmo_widget = None
        self._is_dragging_gizmo = False
        self._last_drag_time = 0.0
        self._last_solved_drag_time = 0.0

        if not enabled or not on_jog_callback or not kinematics or not get_angles_fn:
            self.plotter.render()
            return

        def _on_gizmo_drag(center):
            now = time.perf_counter()
            self._is_dragging_gizmo = True
            self._last_drag_time = now

            # Throttle drag updates to ~60 FPS (16ms) to prevent UI event queue starvation
            if now - getattr(self, "_last_solved_drag_time", 0.0) < 0.016:
                return
            self._last_solved_drag_time = now

            x, y, z = center
            angles = get_angles_fn()

            sol, actual_pt = kinematics.solve_ik_clamped(
                float(x), float(y), float(z),
                current_angles=angles
            )
            if sol is not None:
                on_jog_callback(sol)

        def _on_gizmo_start(widget, event):
            self._is_dragging_gizmo = True
            self._last_drag_time = time.perf_counter()

        def _on_gizmo_end(widget, event):
            self._is_dragging_gizmo = False
            # On release, perform final solve and snap the gizmo sphere to the actual reached TCP
            try:
                center = widget.GetCenter()
                angles = get_angles_fn()
                sol, actual_pt = kinematics.solve_ik_clamped(
                    float(center[0]), float(center[1]), float(center[2]),
                    current_angles=angles
                )
                if sol is not None:
                    on_jog_callback(sol)
                widget.SetCenter(tuple(self._current_tcp))
                self.plotter.render()
            except Exception:
                pass

        try:
            self.gizmo_widget = self.plotter.add_sphere_widget(
                _on_gizmo_drag,
                center=tuple(self._current_tcp),
                radius=14.0,
                color="#00e5ff",
                selected_color="#ff4081",
                interaction_event="always"
            )
            if self.gizmo_widget:
                try:
                    self.gizmo_widget.AddObserver("StartInteractionEvent", _on_gizmo_start)
                    self.gizmo_widget.AddObserver("EndInteractionEvent", _on_gizmo_end)
                except Exception:
                    pass
        except Exception as e:
            # Fallback with interaction_event='end'
            try:
                self.gizmo_widget = self.plotter.add_sphere_widget(
                    _on_gizmo_drag,
                    center=tuple(self._current_tcp),
                    radius=14.0,
                    color="#00e5ff"
                )
            except Exception:
                pass
        self.plotter.render()

    def update_trajectory_path(self, statements_or_keyframes, kinematics) -> None:
        """
        Renders 3D trajectory path for Offline Programming (Visual Components style).
        Features:
        - Waypoint coordinate frames (tool orientation RGB axes)
        - Collision warning detection (floor penetration Z < 0 turns red)
        - Semi-transparent PTP cyan arcs and LINEAR orange welding seam tubes
        """
        # 1. Clear previous trajectory actors
        for act in self.trajectory_actors:
            try:
                self.plotter.remove_actor(act)
            except Exception:
                pass
        self.trajectory_actors.clear()

        # Extract only motion items
        motion_items = []
        for item in (statements_or_keyframes or []):
            if hasattr(item, "angles") and getattr(item, "statement_type", getattr(item, "motion_type", "")) in ("PTP", "LIN"):
                motion_items.append(item)

        if not motion_items:
            self.plotter.render()
            return

        # 2. Extract TCP points and orientation frames for each waypoint
        tcp_points = []
        has_collision = False

        for i, item in enumerate(motion_items):
            pose = kinematics.forward_kinematics([float(a) for a in item.angles])
            pt = np.array([pose.tcp_x, pose.tcp_y, pose.tcp_z], dtype=np.float64)
            tcp_points.append(pt)

            m_type = getattr(item, "statement_type", getattr(item, "motion_type", "PTP"))
            is_collision = (pt[2] < 2.0)  # Collision with table floor
            if is_collision:
                has_collision = True

            # Waypoint Marker Sphere
            if is_collision:
                color = "#ff1744"  # Bright Red for collision
            else:
                color = "#ff9800" if m_type == "LIN" else "#00e5ff"

            sphere = pv.Sphere(radius=5.5, center=pt)
            sphere_act = self.plotter.add_mesh(
                sphere,
                color=color,
                ambient=0.9,
                render_points_as_spheres=True,
                show_edges=False
            )
            self.trajectory_actors.append(sphere_act)

            # Draw Visual Components 3-axis tool orientation frame at waypoint (RGB)
            mat_tcp = pose.link_matrices[-1]
            frame_len = 16.0
            r_x = pt + mat_tcp[0:3, 0] * frame_len
            r_y = pt + mat_tcp[0:3, 1] * frame_len
            r_z = pt + mat_tcp[0:3, 2] * frame_len

            # X = Red, Y = Green, Z = Blue
            for end_pt, ax_color in [(r_x, "#ff5252"), (r_y, "#69f0ae"), (r_z, "#448aff")]:
                axis_line = pv.Line(pt, end_pt).tube(radius=0.7)
                act_ax = self.plotter.add_mesh(axis_line, color=ax_color, ambient=0.9)
                self.trajectory_actors.append(act_ax)

        # 3. Draw connecting trajectory segments between consecutive waypoints
        for i in range(1, len(motion_items)):
            item_prev = motion_items[i - 1]
            item_curr = motion_items[i]
            p_start = tcp_points[i - 1]
            p_end = tcp_points[i]

            dist = float(np.linalg.norm(p_end - p_start))
            if dist < 0.5:
                continue

            m_type_curr = getattr(item_curr, "statement_type", getattr(item_curr, "motion_type", "PTP"))

            if m_type_curr == "LIN":
                # LINEAR straight line: precision Cartesian tube (orange welding seam)
                line = pv.Line(p_start, p_end)
                tube = line.tube(radius=2.2, n_sides=16)
                tube_act = self.plotter.add_mesh(
                    tube,
                    color="#ff9800",
                    ambient=0.85,
                    opacity=0.88,
                    smooth_shading=True
                )
                self.trajectory_actors.append(tube_act)
            else:
                # PTP arc: sample intermediate robot joint angles via S-curve
                num_substeps = max(6, int(dist / 14.0))
                sub_pts = []
                for s in range(num_substeps + 1):
                    prog = s / float(num_substeps)
                    t_curv = (1.0 - np.cos(prog * np.pi)) / 2.0
                    sub_angles = [
                        float(a1 + (a2 - a1) * t_curv)
                        for a1, a2 in zip(item_prev.angles, item_curr.angles)
                    ]
                    sub_pose = kinematics.forward_kinematics(sub_angles)
                    sub_pts.append([sub_pose.tcp_x, sub_pose.tcp_y, sub_pose.tcp_z])

                pts_arr = np.array(sub_pts, dtype=np.float64)
                poly = pv.lines_from_points(pts_arr)
                tube = poly.tube(radius=1.3, n_sides=12)
                tube_act = self.plotter.add_mesh(
                    tube,
                    color="#00e5ff",
                    ambient=0.7,
                    opacity=0.7,
                    smooth_shading=True
                )
                self.trajectory_actors.append(tube_act)

        # 4. Draw Waypoint Billboard Labels (P1, P2, P3...) in 3D scene
        if tcp_points:
            try:
                pts_arr = np.array(tcp_points, dtype=np.float32)
                pts_lbl = pts_arr.copy()
                pts_lbl[:, 2] += 12.0
                labels = []
                for idx, it in enumerate(motion_items):
                    lbl_id = getattr(it, "target_id", getattr(it, "comment", ""))
                    if not lbl_id or lbl_id.startswith("P"):
                        m_type = getattr(it, "statement_type", getattr(it, "motion_type", "PTP"))
                        lbl_id = f"P{idx+1} ({m_type})"
                    else:
                        lbl_id = f"P{idx+1}: {lbl_id[:10]}"
                    labels.append(lbl_id)
                lbl_act = self.plotter.add_point_labels(
                    pts_lbl,
                    labels,
                    point_size=0,
                    font_size=10,
                    text_color="#ffffff",
                    shape="rounded_rect",
                    shape_color="#212121",
                    shape_opacity=0.85,
                    always_visible=False
                )
                self.trajectory_actors.append(lbl_act)
            except Exception:
                pass

        self.plotter.render()

    def reset_camera(self) -> None:
        """Positions camera in an optimal isometric angle framing the entire robot."""
        self.plotter.camera_position = [
            (1150.0, -1150.0, 700.0),  # Eye position
            (0.0, 0.0, 260.0),          # Focal point (middle of arm)
            (0.0, 0.0, 1.0)             # Up vector
        ]
        self.plotter.reset_camera_clipping_range()
        self.plotter.render()

    def view_top(self) -> None:
        self.plotter.view_xy()
        self.plotter.render()

    def view_side(self) -> None:
        self.plotter.view_xz()
        self.plotter.render()

    def load_workpiece(self, file_path: str) -> bool:
        """Loads a CAD STL workpiece onto the robot workcell table for seam planning."""
        try:
            self.clear_workpiece()
            mesh = pv.read(file_path)
            # Normalize so bottom of mesh is at Z=0, and center is at (0, 0)
            mesh = mesh.translate((-mesh.center[0], -mesh.center[1], -mesh.bounds[4]), inplace=False)
            mesh = mesh.compute_normals(auto_orient_normals=True)
            self.raw_workpiece_mesh = mesh
            self.set_workpiece_transform(260.0, 0.0, 0.0, 0.0)
            return True
        except Exception:
            return False

    def clear_workpiece(self) -> None:
        """Removes the imported workpiece from the 3D scene."""
        if self.workpiece_actor:
            try:
                self.plotter.remove_actor(self.workpiece_actor)
            except Exception:
                pass
            self.workpiece_actor = None
            self.workpiece_bounds = None
            self.raw_workpiece_mesh = None
            self.clear_temp_markers()
            self.plotter.render()

    def set_workpiece_transform(self, x: float = 260.0, y: float = 0.0, z: float = 0.0, rot_z_deg: float = 0.0) -> None:
        """Positions and rotates the active CAD workpiece anywhere on the robot workcell table."""
        if self.raw_workpiece_mesh is None:
            return
        self.workpiece_pos = [float(x), float(y), float(z)]
        self.workpiece_rot_z = float(rot_z_deg)

        mesh = self.raw_workpiece_mesh.copy()
        if rot_z_deg != 0.0:
            mesh = mesh.rotate_z(rot_z_deg, inplace=False)
        mesh = mesh.translate((x, y, z), inplace=False)
        mesh = mesh.compute_normals(auto_orient_normals=True)

        self.workpiece_bounds = tuple(mesh.bounds)
        if self.workpiece_actor:
            try:
                self.plotter.remove_actor(self.workpiece_actor)
            except Exception:
                pass
        self.workpiece_actor = self.plotter.add_mesh(
            mesh,
            color="#78909c",
            specular=0.5,
            specular_power=25,
            smooth_shading=True,
            show_edges=True,
            edge_color="#37474f"
        )
        self.plotter.render()

    def load_demo_workpiece(self, fixture_name: str = "WELD_PLATE") -> bool:
        """
        Loads built-in CAD workpiece fixtures (V-seam plates, pipe T-joint, or corner bracket)
        onto the workcell table for instant offline programming & seam teaching.
        """
        try:
            self.clear_workpiece()
            if fixture_name == "PIPE_JOINT":
                p1 = pv.Cylinder(center=(0, 0, 30), direction=(0, 1, 0), radius=25, height=140, resolution=40)
                p2 = pv.Cylinder(center=(0, 0, 55), direction=(0, 0, 1), radius=20, height=50, resolution=40)
                mesh = p1 + p2
            elif fixture_name == "CORNER_BRACKET":
                b1 = pv.Cube(center=(0, -30, 10), x_length=80, y_length=60, z_length=20)
                b2 = pv.Cube(center=(0, 5, 40), x_length=80, y_length=20, z_length=60)
                mesh = b1 + b2
            else:  # WELD_PLATE (Default)
                p_left = pv.Cube(center=(0, -42, 10), x_length=120, y_length=80, z_length=20)
                p_right = pv.Cube(center=(0, 42, 10), x_length=120, y_length=80, z_length=20)
                mesh = p_left + p_right

            mesh = mesh.translate((-mesh.center[0], -mesh.center[1], -mesh.bounds[4]), inplace=False)
            mesh = mesh.compute_normals(auto_orient_normals=True)
            self.raw_workpiece_mesh = mesh
            self.set_workpiece_transform(260.0, 0.0, 0.0, 0.0)
            return True
        except Exception:
            return False

    def enable_surface_snapping(self, on_snap_callback: any) -> None:
        """Enables 3D click-to-snap: clicking workpiece passes exact surface XYZ to callback."""
        self.is_snapping_active = True
        def _on_pick(picked_point):
            if picked_point is not None:
                on_snap_callback(float(picked_point[0]), float(picked_point[1]), float(picked_point[2]))

        self.plotter.enable_surface_point_picking(
            callback=_on_pick,
            left_clicking=True,
            show_point=True,
            color="#00e5ff",
            point_size=12,
            show_message=False,
            picker="cell"
        )

    def disable_surface_snapping(self) -> None:
        """Disables surface point picking and returns to orbit camera control."""
        self.is_snapping_active = False
        try:
            self.plotter.disable_picking()
        except Exception:
            pass
        self.clear_temp_markers()
        try:
            self.plotter.render()
        except Exception:
            pass

    def add_temp_marker(self, pt: Tuple[float, float, float], color: str = "#00e676", radius: float = 5.0) -> None:
        """Adds a temporary visual marker sphere at the specified 3D coordinate."""
        try:
            sphere = pv.Sphere(radius=radius, center=pt)
            actor = self.plotter.add_mesh(sphere, color=color, smooth_shading=True)
            self.temp_marker_actors.append(actor)
            self.plotter.render()
        except Exception:
            pass

    def clear_temp_markers(self) -> None:
        """Clears all temporary 3D markers."""
        for act in self.temp_marker_actors:
            try:
                self.plotter.remove_actor(act)
            except Exception:
                pass
        self.temp_marker_actors.clear()
        try:
            self.plotter.render()
        except Exception:
            pass

    def add_weld_point(self, pt: List[float]) -> None:
        """Deposits a glowing molten weld seam bead element at the current TCP position."""
        if not self.weld_points:
            self.weld_points.append(list(pt))
            return

        p_prev = self.weld_points[-1]
        dist = math.sqrt((pt[0] - p_prev[0])**2 + (pt[1] - p_prev[1])**2 + (pt[2] - p_prev[2])**2)
        if dist < 0.8:
            return  # Filter micro jitter

        try:
            line = pv.Line(p_prev, pt)
            tube = line.tube(radius=2.2, n_sides=10)
            actor = self.plotter.add_mesh(
                tube,
                color="#ff9100",
                ambient=0.9,
                specular=0.8,
                specular_power=30,
                smooth_shading=True
            )
            self.weld_bead_actors.append(actor)
            self.weld_points.append(list(pt))
        except Exception:
            pass

    def clear_weld_seams(self) -> None:
        """Clears all deposited weld beads from the 3D scene."""
        for act in self.weld_bead_actors:
            try:
                self.plotter.remove_actor(act)
            except Exception:
                pass
        self.weld_bead_actors.clear()
        self.weld_points.clear()
        self.plotter.render()

    def set_reach_envelope_visible(self, visible: bool) -> None:
        """Toggles Visual Components -style semi-transparent hemispherical reach volume."""
        if self.envelope_actor:
            try:
                self.plotter.remove_actor(self.envelope_actor)
            except Exception:
                pass
            self.envelope_actor = None

        if visible:
            sphere = pv.Sphere(radius=570.0, center=(0, 0, self.l0), end_phi=90)
            self.envelope_actor = self.plotter.add_mesh(
                sphere,
                color="#00e5ff",
                opacity=0.12,
                show_edges=False
            )
        self.plotter.render()

    def set_collision_highlight(self, is_collision: bool) -> None:
        """Highlights the robot arm links in collision warning red."""
        if getattr(self, "_last_collision_state", None) == is_collision:
            return
        self._last_collision_state = is_collision

        orig_colors = ["#25282e", "#374151", "#0284c7", "#e2e8f0", "#ea580c"]
        for i, act in enumerate(self.actors):
            if hasattr(act, "prop"):
                try:
                    if is_collision:
                        act.prop.color = "#ff1744"
                    else:
                        if i < len(orig_colors):
                            act.prop.color = orig_colors[i]
                except Exception:
                    pass
        self.plotter.render()
