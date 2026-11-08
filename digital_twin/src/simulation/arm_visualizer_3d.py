"""
3D Simulation Viewport for 6-DOF Robot Arm Digital Twin.
Integrates PyVistaQt QtInteractor into PyQt, rendering either imported STL CAD meshes
or sleek procedural fallback meshes with 60 FPS real-time kinematics updates.
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
        self.tcp_actor: Optional[pv.Actor] = None
        self.using_stl_map: Dict[str, bool] = {}

        # Add floor grid plane
        self._add_floor()

        # Build robot links
        self._init_robot_actors()

        # Initial camera view
        self.reset_camera()

    def _add_floor(self) -> None:
        """Adds a subtle circular workspace baseplate."""
        floor = pv.Disc(center=(0, 0, -2), inner=0, outer=550, normal=(0, 0, 1), r_res=60)
        self.plotter.add_mesh(floor, color="#21252b", opacity=0.85, show_edges=False)

        # Reach boundary circle (580mm max reach)
        reach_ring = pv.Disc(center=(0, 0, -1), inner=575, outer=580, normal=(0, 0, 1), r_res=80)
        self.plotter.add_mesh(reach_ring, color="#00bcd4", opacity=0.3)

    def _init_robot_actors(self) -> None:
        """Loads STL models from assets/models or generates parametric procedural meshes."""
        MODELS_DIR.mkdir(parents=True, exist_ok=True)

        parts = [
            ("base", "#37474f", self._make_procedural_base),
            ("link1", "#455a64", self._make_procedural_link1),
            ("link2", "#00bcd4", self._make_procedural_link2),
            ("link3", "#546e7a", self._make_procedural_link3),
            ("link4", "#ff9800", self._make_procedural_link4),
            ("link5", "#263238", self._make_procedural_link5),
            ("gripper", "#cfd8dc", self._make_procedural_gripper),
        ]

        for name, color, procedural_fn in parts:
            stl_name = STL_FILES.get(name, f"{name}.stl")
            stl_path = MODELS_DIR / stl_name

            if stl_path.is_file() and stl_path.stat().st_size > 0:
                try:
                    mesh = pv.read(str(stl_path))
                    self.using_stl_map[name] = True
                except Exception:
                    mesh = procedural_fn()
                    self.using_stl_map[name] = False
            else:
                mesh = procedural_fn()
                self.using_stl_map[name] = False

            actor = self.plotter.add_mesh(
                mesh,
                color=color,
                smooth_shading=True,
                ambient=0.3,
                diffuse=0.7,
                specular=0.5,
                pbr=False
            )
            self.actors.append(actor)

        # TCP Coordinate Indicator Sphere
        tcp_sphere = pv.Sphere(radius=10, center=(0, 0, 0))
        self.tcp_actor = self.plotter.add_mesh(
            tcp_sphere,
            color="#ff4081",
            ambient=0.8,
            render_points_as_spheres=True
        )

    # ---------------- Procedural Link Generators ----------------
    def _make_procedural_base(self) -> pv.PolyData:
        """Cylindrical heavy baseplate sitting on the desk."""
        plate = pv.Cylinder(center=(0, 0, 10), direction=(0, 0, 1), radius=90, height=20, resolution=50)
        ring = pv.Cylinder(center=(0, 0, 25), direction=(0, 0, 1), radius=60, height=10, resolution=50)
        return plate + ring

    def _make_procedural_link1(self) -> pv.PolyData:
        """Turntable turret rotating about Z, extending from Z=0 to L0=140mm."""
        turret = pv.Cylinder(center=(0, 0, 35), direction=(0, 0, 1), radius=50, height=50, resolution=50)
        pylon_left = pv.Cube(center=(-25, 0, 85), x_length=16, y_length=45, z_length=90)
        pylon_right = pv.Cube(center=(25, 0, 85), x_length=16, y_length=45, z_length=90)
        return turret + pylon_left + pylon_right

    def _make_procedural_link2(self) -> pv.PolyData:
        """Upper arm link: pivot at (0,0,0), extends along local Z by L1=200mm."""
        joint_bearing = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=30, height=45, resolution=40)
        beam = pv.Cube(center=(0, 0, self.l1 / 2.0), x_length=30, y_length=35, z_length=self.l1)
        elbow_bearing = pv.Cylinder(center=(0, 0, self.l1), direction=(0, 1, 0), radius=26, height=45, resolution=40)
        return joint_bearing + beam + elbow_bearing

    def _make_procedural_link3(self) -> pv.PolyData:
        """Forearm link: pivot at (0,0,0), extends along local Z by L2=200mm."""
        joint_bearing = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=24, height=40, resolution=40)
        beam = pv.Cube(center=(0, 0, self.l2 / 2.0), x_length=24, y_length=28, z_length=self.l2)
        wrist_bearing = pv.Cylinder(center=(0, 0, self.l2), direction=(0, 1, 0), radius=20, height=36, resolution=40)
        return joint_bearing + beam + wrist_bearing

    def _make_procedural_link4(self) -> pv.PolyData:
        """Wrist pitch knuckle: pivot at (0,0,0), extends along local Z by 45mm."""
        knuckle = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=18, height=32, resolution=30)
        shaft = pv.Cylinder(center=(0, 0, 22), direction=(0, 0, 1), radius=15, height=45, resolution=30)
        return knuckle + shaft

    def _make_procedural_link5(self) -> pv.PolyData:
        """Wrist roll collar: pivot at (0,0,0), extends along local Z by 35mm."""
        collar = pv.Cylinder(center=(0, 0, 15), direction=(0, 0, 1), radius=22, height=30, resolution=30)
        flange = pv.Cylinder(center=(0, 0, 32), direction=(0, 0, 1), radius=28, height=6, resolution=30)
        return collar + flange

    def _make_procedural_gripper(self) -> pv.PolyData:
        """Gripper claw body + dual fingers extending along local Z."""
        palm = pv.Cube(center=(0, 0, 15), x_length=55, y_length=25, z_length=30)
        finger_l = pv.Cube(center=(-18, 0, 55), x_length=8, y_length=16, z_length=50)
        finger_r = pv.Cube(center=(18, 0, 55), x_length=8, y_length=16, z_length=50)
        tip_l = pv.Cube(center=(-14, 0, 80), x_length=12, y_length=14, z_length=10)
        tip_r = pv.Cube(center=(14, 0, 80), x_length=12, y_length=14, z_length=10)
        return palm + finger_l + finger_r + tip_l + tip_r

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

    def update_pose(self, pose: PoseData) -> None:
        """
        Updates each 3D link actor matrix in real time.
        Hardware-accelerated 60 FPS update via actor.user_matrix.
        """
        for i, actor in enumerate(self.actors):
            if i < len(pose.link_matrices):
                actor.user_matrix = pose.link_matrices[i]

        if self.tcp_actor:
            # Place TCP marker at (tcp_x, tcp_y, tcp_z)
            t_tcp = np.eye(4, dtype=np.float64)
            t_tcp[0, 3] = pose.tcp_x
            t_tcp[1, 3] = pose.tcp_y
            t_tcp[2, 3] = pose.tcp_z
            self.tcp_actor.user_matrix = t_tcp

        self.plotter.render()

    def reset_camera(self) -> None:
        """Positions camera in an optimal isometric angle."""
        self.plotter.camera_position = [
            (650.0, -650.0, 550.0),  # Eye position
            (0.0, 0.0, 180.0),       # Focal point (middle of arm)
            (0.0, 0.0, 1.0)          # Up vector
        ]
        self.plotter.reset_camera_clipping_range()
        self.plotter.render()

    def view_top(self) -> None:
        self.plotter.view_xy()
        self.plotter.render()

    def view_side(self) -> None:
        self.plotter.view_xz()
        self.plotter.render()
