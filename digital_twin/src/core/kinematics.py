"""
Kinematics Engine for 6-DOF Robot Arm Digital Twin.
Computes Forward Kinematics (FK), Analytical Inverse Kinematics (IK),
PTP (Point-to-Point) S-curve interpolation, and LINEAR (Cartesian straight-line) path generation.
"""
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple
import numpy as np

from src.config import LINK_0_HEIGHT, LINK_1_LENGTH, LINK_2_LENGTH, LINK_3_LENGTH


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """Returns a 4x4 homogeneous translation matrix."""
    return np.array([
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)


def rot_x_matrix(angle_rad: float) -> np.ndarray:
    """Returns a 4x4 rotation matrix around X axis."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0,   c,  -s, 0.0],
        [0.0,   s,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)


def rot_y_matrix(angle_rad: float) -> np.ndarray:
    """Returns a 4x4 rotation matrix around Y axis."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([
        [  c, 0.0,   s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [ -s, 0.0,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)


def rot_z_matrix(angle_rad: float) -> np.ndarray:
    """Returns a 4x4 rotation matrix around Z axis."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([
        [  c,  -s, 0.0, 0.0],
        [  s,   c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)


@dataclass
class PoseData:
    tcp_x: float
    tcp_y: float
    tcp_z: float
    pitch_deg: float
    link_matrices: List[np.ndarray]  # 4x4 matrices for base, link1, link2, link3, link4, link5, gripper


class RobotKinematics:
    """
    Computes Forward and Inverse Kinematics for a 6-DOF articulated robot arm.
    Supports PTP and LINEAR trajectory planning for welding, dispensing, and pick & place.
    """
    def __init__(self,
                 l0: float = LINK_0_HEIGHT,
                 l1: float = LINK_1_LENGTH,
                 l2: float = LINK_2_LENGTH,
                 l3: float = LINK_3_LENGTH):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def forward_kinematics(self, joint_angles_deg: List[float]) -> PoseData:
        """
        Computes forward kinematics for given joint angles (J1..J6 in degrees).
        Returns PoseData containing TCP position and global transform matrix for each link.
        """
        j1, j2, j3, j4, j5, j6 = [math.radians(a) for a in joint_angles_deg]

        # Base frame T0 (world origin)
        t_base = np.eye(4, dtype=np.float64)

        # Link 1 (Turntable base): Rotates around Z by (J1 - 90°)
        r1 = rot_z_matrix(j1 - math.pi / 2.0)
        t_link1 = t_base @ r1

        # Shoulder joint is at height L0 = 140mm
        t_shoulder_joint = t_link1 @ translation_matrix(0, 0, self.l0)

        # Link 2 (Upper arm): Rotates around Y axis by (J2 - 90°)
        r2 = rot_y_matrix(j2 - math.pi / 2.0)
        t_link2 = t_shoulder_joint @ r2

        # Elbow joint is at distance 206mm along link 2
        t_elbow_joint = t_link2 @ translation_matrix(0, 0, self.l1)

        # Link 3 (Forearm): Rotates around Y axis by (J3 - 90°)
        r3 = rot_y_matrix(j3 - math.pi / 2.0)
        t_link3 = t_elbow_joint @ r3

        # Wrist pitch joint is at distance 206mm along link 3
        t_wrist_joint = t_link3 @ translation_matrix(0, 0, self.l2)

        # Link 4 (Wrist pitch bracket / ranne): Rotates around Y axis by (J4 - 90°)
        r4 = rot_y_matrix(j4 - math.pi / 2.0)
        t_link4 = t_wrist_joint @ r4

        # Link 5 (Tarttujan runko / roll): Mounted at end of Link 4 (+45mm)
        t_wrist_flange = t_link4 @ translation_matrix(0, 0, 45.0)
        r5 = rot_z_matrix(j5 - math.pi / 2.0)
        t_link5 = t_wrist_flange @ r5

        # Gripper base: Mounted on Link 5 flange
        t_gripper = t_link5

        # Gripper Tool Center Point (TCP): fingertip contact point
        tool_reach = max(10.0, self.l3 - 45.0)
        t_tcp = t_gripper @ translation_matrix(0, 0, tool_reach)

        # Extract TCP Cartesian coordinates
        tcp_x = float(t_tcp[0, 3])
        tcp_y = float(t_tcp[1, 3])
        tcp_z = float(t_tcp[2, 3])

        # Pitch angle relative to horizontal plane
        total_angle_rad = (j2 - math.pi / 2.0) + (j3 - math.pi / 2.0) + (j4 - math.pi / 2.0)
        pitch_deg = 90.0 - math.degrees(total_angle_rad)

        return PoseData(
            tcp_x=tcp_x,
            tcp_y=tcp_y,
            tcp_z=tcp_z,
            pitch_deg=pitch_deg,
            link_matrices=[
                t_base,      # 0
                t_link1,     # 1
                t_link2,     # 2
                t_link3,     # 3
                t_link4,     # 4
                t_link5,     # 5
                t_gripper,   # 6
                t_tcp        # 7
            ]
        )

    def inverse_kinematics(self,
                           x: float,
                           y: float,
                           z: float,
                           pitch_deg: float = 0.0,
                           roll_deg: float = 90.0,
                           gripper_deg: float = 40.0,
                           auto_adjust_pitch: bool = True) -> Optional[List[int]]:
        """
        Analytical Inverse Kinematics solver.
        Calculates J1..J6 joint angles to place TCP at (X, Y, Z) with given pitch angle.
        Supports automatic pitch fallback if exact orientation is out of reach.
        """
        res = self._solve_exact_ik(x, y, z, pitch_deg, roll_deg, gripper_deg)
        if res is not None:
            return res

        if auto_adjust_pitch:
            # Try searching nearby pitch angles (±5° .. ±40°)
            for delta in range(5, 45, 5):
                for sign in (-1, 1):
                    alt_pitch = pitch_deg + delta * sign
                    res = self._solve_exact_ik(x, y, z, alt_pitch, roll_deg, gripper_deg)
                    if res is not None:
                        return res

        return None

    def _solve_exact_ik(self,
                        x: float,
                        y: float,
                        z: float,
                        pitch_deg: float,
                        roll_deg: float,
                        gripper_deg: float) -> Optional[List[int]]:
        """Core analytical IK for a 3-link planar arm with turntable base."""
        # 1. Turntable base (J1)
        r_target = math.sqrt(x * x + y * y)
        if r_target < 0.001:
            j1 = 90.0
        else:
            j1 = 90.0 + math.degrees(math.atan2(y, x))

        if not (0.0 <= j1 <= 180.0):
            return None

        # 2. Wrist Center Position
        tool_angle_rad = math.radians(90.0 - pitch_deg)
        wr = r_target - self.l3 * math.sin(tool_angle_rad)
        wz = z - self.l0 - self.l3 * math.cos(tool_angle_rad)

        dist_sq = wr * wr + wz * wz
        dist = math.sqrt(dist_sq)

        # Check reach boundary
        max_reach = self.l1 + self.l2
        if dist > max_reach or dist < 10.0:
            return None

        # 3. Elbow angle (J3) by Law of Cosines
        cos_th3 = (dist_sq - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2)
        cos_th3 = max(-1.0, min(1.0, cos_th3))
        th3 = math.acos(cos_th3)

        # 4. Shoulder angle (J2)
        alpha = math.atan2(wr, wz)
        cos_beta = (self.l1 * self.l1 + dist_sq - self.l2 * self.l2) / (2.0 * self.l1 * dist)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        # Branch 1: Natural elbow configuration
        th2 = alpha - beta
        th4 = tool_angle_rad - th2 - th3

        s1 = int(round(j1))
        s2 = int(round(90.0 + math.degrees(th2)))
        s3 = int(round(90.0 + math.degrees(th3)))
        s4 = int(round(90.0 + math.degrees(th4)))
        s5 = int(round(max(0.0, min(180.0, roll_deg))))
        s6 = int(round(max(40.0, min(128.0, gripper_deg))))

        sol = [s1, s2, s3, s4, s5, s6]
        if all(0 <= sol[i] <= 180 for i in range(5)):
            return sol

        # Branch 2: Inverted elbow configuration
        th2_alt = alpha + beta
        th3_alt = -th3
        th4_alt = tool_angle_rad - th2_alt - th3_alt

        s2_alt = int(round(90.0 + math.degrees(th2_alt)))
        s3_alt = int(round(90.0 + math.degrees(th3_alt)))
        s4_alt = int(round(90.0 + math.degrees(th4_alt)))

        sol_alt = [s1, s2_alt, s3_alt, s4_alt, s5, s6]
        if all(0 <= sol_alt[i] <= 180 for i in range(5)):
            return sol_alt

        return None

    def generate_linear_path(self,
                             start_angles: List[int],
                             target_angles: List[int],
                             step_size_mm: float = 6.0) -> List[List[int]]:
        """
        Generates a straight 3D Cartesian line path (LINEAR motion) between two poses.
        Interpolates (X, Y, Z) and solves Inverse Kinematics for every intermediate point.
        Ideal for welding seams, dispensing glue, and straight-line machining.
        """
        p_start = self.forward_kinematics([float(a) for a in start_angles])
        p_end = self.forward_kinematics([float(a) for a in target_angles])

        dx = p_end.tcp_x - p_start.tcp_x
        dy = p_end.tcp_y - p_start.tcp_y
        dz = p_end.tcp_z - p_start.tcp_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 0.001:
            return [list(start_angles)]

        steps = max(2, min(80, int(math.ceil(dist / step_size_mm))))
        path: List[List[int]] = []

        roll_start = float(start_angles[4]) if len(start_angles) > 4 else 90.0
        roll_end = float(target_angles[4]) if len(target_angles) > 4 else 90.0
        grip_start = float(start_angles[5]) if len(start_angles) > 5 else 40.0
        grip_end = float(target_angles[5]) if len(target_angles) > 5 else 40.0

        for i in range(steps + 1):
            t = i / float(steps)
            x_interp = p_start.tcp_x + dx * t
            y_interp = p_start.tcp_y + dy * t
            z_interp = p_start.tcp_z + dz * t
            pitch_interp = p_start.pitch_deg + (p_end.pitch_deg - p_start.pitch_deg) * t
            roll_interp = roll_start + (roll_end - roll_start) * t
            grip_interp = grip_start + (grip_end - grip_start) * t

            ik_sol = self.inverse_kinematics(
                x_interp, y_interp, z_interp,
                pitch_deg=pitch_interp,
                roll_deg=roll_interp,
                gripper_deg=grip_interp,
                auto_adjust_pitch=True
            )

            if ik_sol is not None:
                path.append(ik_sol)
            else:
                # Fallback to joint interpolation if point temporarily singularity
                ptp_step = [
                    int(round(start_angles[j] + (target_angles[j] - start_angles[j]) * t))
                    for j in range(6)
                ]
                path.append(ptp_step)

        return path
