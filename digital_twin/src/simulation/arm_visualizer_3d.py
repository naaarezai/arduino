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
        self.plotter.set_background("#16181c", top="#242830")  # Professional dark studio gradient
        self.plotter.show_axes()
        self.plotter.enable_anti_aliasing()

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
        self.tcp_actor: Optional[pv.Actor] = None
        self.trajectory_actors: List[pv.Actor] = []
        self.gizmo_widget = None
        self._current_tcp = [0.0, 0.0, 706.0]

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

        # Coordinate alignment matrix for arm links: X_orig -> Y_new, Y_orig -> Z_new, Z_orig -> X_new
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
        self.actors.append(self.plotter.add_mesh(base_m, color="#2a2e33", specular=0.3, specular_power=15, smooth_shading=True))

        # 2. Link 1 (Turntable top plate sitting at Z = 100 on top of base)
        l1_f = MODELS_DIR / "link1.stl"
        if l1_f.is_file():
            l1_m = pv.read(str(l1_f))
            l1_m = l1_m.translate((-l1_m.center[0], -l1_m.center[1], 100.0 - l1_m.bounds[4]), inplace=False)
            l1_m = l1_m.compute_normals(auto_orient_normals=True)
        else:
            l1_m = self._make_procedural_link1()
        self.actors.append(self.plotter.add_mesh(l1_m, color="#373d44", specular=0.4, specular_power=20, smooth_shading=True))

        # 3. Link 2 (Upper Arm - arm1.stl): Industrial high-vis cyan
        l2_f = MODELS_DIR / "link2.stl"
        if l2_f.is_file():
            l2_m = pv.read(str(l2_f))
            l2_m = l2_m.translate((0, 15.0, 0), inplace=False)
            l2_m = l2_m.transform(mat_align, inplace=False)
            l2_m = l2_m.compute_normals(auto_orient_normals=True)
        else:
            l2_m = self._make_procedural_link2()
        self.actors.append(self.plotter.add_mesh(l2_m, color="#00838f", specular=0.5, specular_power=25, smooth_shading=True))

        # 4. Link 3 (Forearm - arm1.stl): Precision silver-white
        l3_f = MODELS_DIR / "link3.stl"
        if not l3_f.is_file() and l2_f.is_file():
            l3_f = l2_f
        if l3_f.is_file():
            l3_m = pv.read(str(l3_f))
            l3_m = l3_m.translate((0, 15.0, 0), inplace=False)
            l3_m = l3_m.transform(mat_align, inplace=False)
            l3_m = l3_m.compute_normals(auto_orient_normals=True)
        else:
            l3_m = self._make_procedural_link3()
        self.actors.append(self.plotter.add_mesh(l3_m, color="#eceff1", specular=0.45, specular_power=20, smooth_shading=True))

        # 5. Link 4 (Ranne - ranne.stl): Industrial orange safety accent
        l4_f = MODELS_DIR / "link4.stl"
        if l4_f.is_file():
            l4_m = pv.read(str(l4_f))
            l4_m = l4_m.transform(mat_align, inplace=False)
            l4_m = l4_m.compute_normals(auto_orient_normals=True)
        else:
            l4_m = self._make_procedural_link4()
        self.actors.append(self.plotter.add_mesh(l4_m, color="#e65100", specular=0.4, specular_power=20, smooth_shading=True))

        # 6. Link 5 (Tarttujan runkolevy - link5.stl): Anodized dark bracket
        l5_f = MODELS_DIR / "link5.stl"
        if l5_f.is_file():
            l5_m = pv.read(str(l5_f))
            l5_m = l5_m.translate((-l5_m.center[0], -l5_m.center[1], -l5_m.bounds[4]), inplace=False)
            l5_m = l5_m.compute_normals(auto_orient_normals=True)
        else:
            l5_m = self._make_procedural_link5()
        self.actors.append(self.plotter.add_mesh(l5_m, color="#212529", specular=0.3, specular_power=15, smooth_shading=True))

        # 7. Gripper (Split into animated components: center mount, left finger, right finger)
        grip_f = MODELS_DIR / "gripper.stl"
        if grip_f.is_file():
            grip_m = pv.read(str(grip_f))
            grip_m = grip_m.translate((-16.2, -46.3, -30.0), inplace=False)
            grip_m = grip_m.rotate_x(-90, point=(0, 0, 0), inplace=False)
            grip_m = grip_m.compute_normals(auto_orient_normals=True)

            bodies = grip_m.split_bodies()
            base_poly = pv.PolyData()
            left_poly = pv.PolyData()
            right_poly = pv.PolyData()

            for b in bodies:
                cx = b.center[0]
                if cx < -5.0:
                    left_poly = left_poly + b
                elif cx > 5.0:
                    right_poly = right_poly + b
                else:
                    base_poly = base_poly + b

            self.gripper_base_actor = self.plotter.add_mesh(base_poly, color="#37474f", specular=0.4, smooth_shading=True)
            self.gripper_left_actor = self.plotter.add_mesh(left_poly, color="#b0bec5", specular=0.6, specular_power=30, smooth_shading=True)
            self.gripper_right_actor = self.plotter.add_mesh(right_poly, color="#b0bec5", specular=0.6, specular_power=30, smooth_shading=True)
        else:
            grip_m = self._make_procedural_gripper()
            self.gripper_base_actor = self.plotter.add_mesh(grip_m, color="#b0bec5", smooth_shading=True)

        # TCP Coordinate Indicator Sphere
        tcp_sphere = pv.Sphere(radius=7, center=(0, 0, 0))
        self.tcp_actor = self.plotter.add_mesh(
            tcp_sphere,
            color="#00e5ff",
            ambient=0.85,
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

        # Update Gripper base and animated fingers (index 6 in link_matrices)
        mat_gripper = pose.link_matrices[6] if len(pose.link_matrices) > 6 else pose.link_matrices[-1]

        if self.gripper_base_actor:
            self.gripper_base_actor.user_matrix = mat_gripper

        # Calculate claw finger opening offset
        open_offset = 15.0
        if joint_angles and len(joint_angles) >= 6:
            j6 = float(joint_angles[5])
            # 40 deg = fully open (15mm offset), 128 deg = fully closed (0mm offset)
            progress = max(0.0, min(1.0, (j6 - 40.0) / (128.0 - 40.0)))
            open_offset = (1.0 - progress) * 15.0

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

        # Update TCP point
        self._current_tcp = [pose.tcp_x, pose.tcp_y, pose.tcp_z]
        if self.tcp_actor:
            t_tcp = np.eye(4, dtype=np.float64)
            t_tcp[0, 3] = pose.tcp_x
            t_tcp[1, 3] = pose.tcp_y
            t_tcp[2, 3] = pose.tcp_z
            self.tcp_actor.user_matrix = t_tcp

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
        """
        try:
            self.plotter.clear_sphere_widgets()
        except Exception:
            pass
        self.gizmo_widget = None

        if not enabled or not on_jog_callback or not kinematics or not get_angles_fn:
            self.plotter.render()
            return

        def _on_gizmo_drag(center):
            x, y, z = center
            angles = get_angles_fn()
            roll = float(angles[4]) if len(angles) > 4 else 90.0
            grip = float(angles[5]) if len(angles) > 5 else 40.0
            safe_z = max(5.0, float(z))

            sol = kinematics.inverse_kinematics(
                float(x), float(y), safe_z,
                pitch_deg=0.0, roll_deg=roll, gripper_deg=grip,
                auto_adjust_pitch=True
            )
            if sol is not None:
                on_jog_callback(sol)

        try:
            self.gizmo_widget = self.plotter.add_sphere_widget(
                _on_gizmo_drag,
                center=tuple(self._current_tcp),
                radius=14.0,
                color="#00e5ff",
                selected_color="#ff4081",
                interaction_event="always"
            )
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
