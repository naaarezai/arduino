"""
Kinematics Engine for 6-DOF Robot Arm Digital Twin.
Computes 4x4 homogeneous transformation matrices for each link in a parent-child chain,
and provides Forward Kinematics (FK) to calculate TCP (Tool Center Point) coordinates.
"""
from dataclasses import dataclass
import math
from typing import Dict, List, Tuple
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
    Computes Forward Kinematics and local-to-global transformation matrices
    for a 6-DOF articulated robot arm.
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
        # Home 90° points forward along +X
        r1 = rot_z_matrix(j1 - math.pi / 2.0)
        t_link1 = t_base @ r1

        # Shoulder joint is at height L0
        t_shoulder_joint = t_link1 @ translation_matrix(0, 0, self.l0)

        # Link 2 (Upper arm): Rotates around Y axis by (J2 - 90°)
        r2 = rot_y_matrix(j2 - math.pi / 2.0)
        t_link2 = t_shoulder_joint @ r2

        # Elbow joint is at distance L1 along link 2 (pointing up at 90°)
        t_elbow_joint = t_link2 @ translation_matrix(0, 0, self.l1)

        # Link 3 (Forearm): Rotates around Y axis by (J3 - 90°)
        r3 = rot_y_matrix(j3 - math.pi / 2.0)
        t_link3 = t_elbow_joint @ r3

        # Wrist pitch joint is at distance L2 along link 3
        t_wrist_joint = t_link3 @ translation_matrix(0, 0, self.l2)

        # Link 4 (Wrist pitch): Rotates around Y axis by (J4 - 90°)
        r4 = rot_y_matrix(j4 - math.pi / 2.0)
        t_link4 = t_wrist_joint @ r4

        # Link 5 (Wrist roll): Rotates around local Z tool axis by (J5 - 90°)
        r5 = rot_z_matrix(j5 - math.pi / 2.0)
        t_link5 = t_link4 @ r5

        # Gripper (Tool Center Point TCP) is at distance L3 along tool axis
        t_tcp = t_link5 @ translation_matrix(0, 0, self.l3)

        # Extract TCP Cartesian coordinates
        tcp_x = float(t_tcp[0, 3])
        tcp_y = float(t_tcp[1, 3])
        tcp_z = float(t_tcp[2, 3])

        # Compute pitch angle relative to horizontal plane
        # (J2 + J3 + J4 sum of pitch angles)
        total_pitch_rad = (j2 - math.pi / 2.0) + (j3 - math.pi / 2.0) + (j4 - math.pi / 2.0)
        pitch_deg = math.degrees(total_pitch_rad)

        return PoseData(
            tcp_x=tcp_x,
            tcp_y=tcp_y,
            tcp_z=tcp_z,
            pitch_deg=pitch_deg,
            link_matrices=[
                t_base,
                t_link1,
                t_link2,
                t_link3,
                t_link4,
                t_link5,
                t_tcp
            ]
        )
