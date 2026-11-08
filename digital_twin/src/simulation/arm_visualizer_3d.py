"""
3D Simulation Viewport for 6-DOF Robot Arm Digital Twin.
Integrates PyVistaQt QtInteractor into PyQt, rendering calibrated CAD STL models
with animated opening/closing gripper fingers and hardware-accelerated 60 FPS posing.
"""
from pathlib import Path
from typing import Dict, List, Optional
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor

from src.config import (
    LINK_0_HEIGHT, LINK_1_LENGTH, LINK_2_LENGTH, LINK_3_LENGTH,
    MODELS_DIR, STL_FILES
)
from src.core.kinematics import PoseData
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
        self.plotter.set_background("#181a1f", top="#282c34")  # Modern dark gradient
        self.plotter.show_axes()
        self.plotter.enable_anti_aliasing()

        # Actors storage
        self.actors: List[pv.Actor] = []
        self.gripper_base_actor: Optional[pv.Actor] = None
        self.gripper_left_actor: Optional[pv.Actor] = None
        self.gripper_right_actor: Optional[pv.Actor] = None
        self.tcp_actor: Optional[pv.Actor] = None

        # Add floor grid plane
        self._add_floor()

        # Build robot links and gripper
        self._init_robot_actors()

        # Initial camera view
        self.reset_camera()

    def _add_floor(self) -> None:
        """Adds workspace baseplate and reach circle."""
        floor = pv.Disc(center=(0, 0, -2), inner=0, outer=550, normal=(0, 0, 1), r_res=60)
        self.plotter.add_mesh(floor, color="#21252b", opacity=0.85, show_edges=False)

        reach_ring = pv.Disc(center=(0, 0, -1), inner=575, outer=580, normal=(0, 0, 1), r_res=80)
        self.plotter.add_mesh(reach_ring, color="#00bcd4", opacity=0.3)

    def _init_robot_actors(self) -> None:
        """Loads STL models with exact kinematic alignment and splits gripper for animation."""
        MODELS_DIR.mkdir(parents=True, exist_ok=True)

        # 1. Base
        base_f = MODELS_DIR / "base.stl"
        if base_f.is_file():
            base_m = pv.read(str(base_f))
            base_m = base_m.translate((-base_m.center[0], -base_m.center[1], -base_m.bounds[4]), inplace=False)
        else:
            base_m = self._make_procedural_base()
        self.actors.append(self.plotter.add_mesh(base_m, color="#37474f", smooth_shading=True))

        # 2. Link 1 (Turntable)
        l1_f = MODELS_DIR / "link1.stl"
        if l1_f.is_file():
            l1_m = pv.read(str(l1_f))
            l1_m = l1_m.translate((-l1_m.center[0], -l1_m.center[1], 100.0), inplace=False)
        else:
            l1_m = self._make_procedural_link1()
        self.actors.append(self.plotter.add_mesh(l1_m, color="#455a64", smooth_shading=True))

        # 3. Link 2 (Upper Arm - arm1)
        l2_f = MODELS_DIR / "link2.stl"
        if l2_f.is_file():
            l2_m = pv.read(str(l2_f))
            # Align bottom hole center at (0, 0, 0), hinge pin along Y, length along +Z
            l2_m = l2_m.translate((0, 18.55, 0), inplace=False)
            l2_m = l2_m.rotate_z(90, point=(0, 0, 0), inplace=False)
            l2_m = l2_m.rotate_y(90, point=(0, 0, 0), inplace=False)
        else:
            l2_m = self._make_procedural_link2()
        self.actors.append(self.plotter.add_mesh(l2_m, color="#00bcd4", smooth_shading=True))

        # 4. Link 3 (Forearm - arm1)
        l3_f = MODELS_DIR / "link3.stl"
        if not l3_f.is_file() and l2_f.is_file():
            l3_f = l2_f
        if l3_f.is_file():
            l3_m = pv.read(str(l3_f))
            l3_m = l3_m.translate((0, 18.55, 0), inplace=False)
            l3_m = l3_m.rotate_z(90, point=(0, 0, 0), inplace=False)
            l3_m = l3_m.rotate_y(90, point=(0, 0, 0), inplace=False)
        else:
            l3_m = self._make_procedural_link3()
        self.actors.append(self.plotter.add_mesh(l3_m, color="#eceff1", smooth_shading=True))

        # 5. Link 4 (Ranne)
        l4_f = MODELS_DIR / "link4.stl"
        if l4_f.is_file():
            l4_m = pv.read(str(l4_f))
            l4_m = l4_m.rotate_z(90, point=(0, 0, 0), inplace=False)
            l4_m = l4_m.rotate_y(-90, point=(0, 0, 0), inplace=False)
            l4_m = l4_m.translate((0, 0, 25.0), inplace=False)
        else:
            l4_m = self._make_procedural_link4()
        self.actors.append(self.plotter.add_mesh(l4_m, color="#ff9800", smooth_shading=True))

        # 6. Link 5 (Tarttujan runkolevy)
        l5_f = MODELS_DIR / "link5.stl"
        if l5_f.is_file():
            l5_m = pv.read(str(l5_f))
            l5_m = l5_m.translate((-l5_m.center[0], -l5_m.center[1], -l5_m.bounds[4]), inplace=False)
            l5_m = l5_m.translate((0, 0, 28.0), inplace=False)
        else:
            l5_m = self._make_procedural_link5()
        self.actors.append(self.plotter.add_mesh(l5_m, color="#263238", smooth_shading=True))

        # 7. Gripper (Split into animated components)
        grip_f = MODELS_DIR / "gripper.stl"
        if grip_f.is_file():
            grip_m = pv.read(str(grip_f))
            grip_m = grip_m.translate((-grip_m.center[0], -grip_m.bounds[3], -grip_m.center[2]), inplace=False)
            grip_m = grip_m.rotate_x(-90, point=(0, 0, 0), inplace=False)
            grip_m = grip_m.translate((0, 0, 35.0), inplace=False)

            bodies = grip_m.split_bodies()
            base_poly = pv.PolyData()
            left_poly = pv.PolyData()
            right_poly = pv.PolyData()

            for b in bodies:
                cx = b.center[0]
                if -5 < cx < 5:
                    base_poly = base_poly + b
                elif cx <= -5:
                    left_poly = left_poly + b
                else:
                    right_poly = right_poly + b

            self.gripper_base_actor = self.plotter.add_mesh(base_poly, color="#37474f", smooth_shading=True)
            self.gripper_left_actor = self.plotter.add_mesh(left_poly, color="#cfd8dc", smooth_shading=True)
            self.gripper_right_actor = self.plotter.add_mesh(right_poly, color="#cfd8dc", smooth_shading=True)
        else:
            grip_m = self._make_procedural_gripper()
            self.gripper_base_actor = self.plotter.add_mesh(grip_m, color="#cfd8dc", smooth_shading=True)

        # TCP Coordinate Indicator Sphere
        tcp_sphere = pv.Sphere(radius=10, center=(0, 0, 0))
        self.tcp_actor = self.plotter.add_mesh(
            tcp_sphere,
            color="#ff4081",
            ambient=0.8,
            render_points_as_spheres=True
        )

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

        # Update Gripper and animated fingers
        mat_wrist = pose.link_matrices[4]  # Mounts onto wrist frame

        if self.gripper_base_actor:
            self.gripper_base_actor.user_matrix = mat_wrist

        # Calculate claw finger opening offset
        open_offset = 15.0
        if joint_angles and len(joint_angles) >= 6:
            j6 = float(joint_angles[5])
            # 40 deg = fully open (15mm offset), 128 deg = fully closed (0mm offset)
            progress = max(0.0, min(1.0, (j6 - 40.0) / (128.0 - 40.0)))
            open_offset = (1.0 - progress) * 15.0

        if self.gripper_left_actor and self.gripper_right_actor:
            # Shift left and right fingers laterally along X
            mat_l = mat_wrist @ np.array([
                [1.0, 0.0, 0.0, -open_offset],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float64)

            mat_r = mat_wrist @ np.array([
                [1.0, 0.0, 0.0, open_offset],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float64)

            self.gripper_left_actor.user_matrix = mat_l
            self.gripper_right_actor.user_matrix = mat_r

        # Update TCP point
        if self.tcp_actor:
            t_tcp = np.eye(4, dtype=np.float64)
            t_tcp[0, 3] = pose.tcp_x
            t_tcp[1, 3] = pose.tcp_y
            t_tcp[2, 3] = pose.tcp_z
            self.tcp_actor.user_matrix = t_tcp

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
