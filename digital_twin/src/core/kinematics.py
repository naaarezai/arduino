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
from src.core.tool_data import ToolProfile


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
                 l3: float = LINK_3_LENGTH,
                 active_tool: Optional[ToolProfile] = None):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.active_tool = active_tool or ToolProfile()
        self.l3 = 50.0 + self.active_tool.tcp_z if self.active_tool else l3

    def set_active_tool(self, tool: ToolProfile) -> None:
        """Sets active tool profile and updates TCP calculations."""
        self.active_tool = tool
        self.l3 = 50.0 + tool.tcp_z

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

        # Link 5 (Tarttujan runko / roll): Mounted at end of Link 4 (+50.0mm ranne flange)
        t_wrist_flange = t_link4 @ translation_matrix(0, 0, 50.0)
        r5 = rot_z_matrix(j5 - math.pi / 2.0)
        t_link5 = t_wrist_flange @ r5

        # Tool Flange (t_flange = t_link5)
        t_gripper = t_link5

        # Tool Center Point (TCP) with 6-DOF tool offset (X, Y, Z, Pitch, Roll)
        r_pitch = rot_y_matrix(math.radians(self.active_tool.tcp_pitch_deg))
        r_roll = rot_z_matrix(math.radians(self.active_tool.tcp_roll_deg))
        t_tool_offset = translation_matrix(
            self.active_tool.tcp_x,
            self.active_tool.tcp_y,
            self.active_tool.tcp_z
        ) @ r_pitch @ r_roll

        t_tcp = t_gripper @ t_tool_offset

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
                           auto_adjust_pitch: bool = True,
                           current_angles: Optional[List[int]] = None) -> Optional[List[int]]:
        """
        Analytical Inverse Kinematics solver.
        Calculates J1..J6 joint angles to place TCP at (X, Y, Z) with given pitch angle.
        Supports automatic pitch fallback across full physical range (-85° .. +85°).
        Minimizes joint angular displacement from current_angles if provided to ensure smooth motion.
        """
        res = self._solve_exact_ik(x, y, z, pitch_deg, roll_deg, gripper_deg, current_angles=current_angles)
        if res is not None:
            if current_angles is None:
                return res
            d = sum((res[i] - current_angles[i]) ** 2 for i in range(min(5, len(current_angles))))
            if d < 400:  # < 20 deg per joint average
                return res

        if auto_adjust_pitch:
            best_sol = res
            min_dist = float("inf") if res is None else sum((res[i] - current_angles[i]) ** 2 for i in range(min(5, len(current_angles)))) if current_angles else 0.0

            # Search nearby pitch angles across full physical range (-85° .. +85°) in 5° steps
            for delta in range(5, 90, 5):
                for sign in (1, -1):
                    alt_pitch = pitch_deg + delta * sign
                    if -85.0 <= alt_pitch <= 85.0:
                        alt_res = self._solve_exact_ik(x, y, z, alt_pitch, roll_deg, gripper_deg, current_angles=current_angles)
                        if alt_res is not None:
                            if current_angles is None:
                                return alt_res
                            dist = sum((alt_res[i] - current_angles[i]) ** 2 for i in range(min(5, len(current_angles))))
                            if dist < min_dist:
                                min_dist = dist
                                best_sol = alt_res
                            # Early return if solution is very close to current posture
                            if dist < 400:
                                return best_sol

            if best_sol is not None:
                return best_sol

        return None

    def _solve_exact_ik(self,
                        x: float,
                        y: float,
                        z: float,
                        pitch_deg: float,
                        roll_deg: float,
                        gripper_deg: float,
                        current_angles: Optional[List[int]] = None) -> Optional[List[int]]:
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

        # Branch 2: Inverted elbow configuration
        th2_alt = alpha + beta
        th3_alt = -th3
        th4_alt = tool_angle_rad - th2_alt - th3_alt

        s2_alt = int(round(90.0 + math.degrees(th2_alt)))
        s3_alt = int(round(90.0 + math.degrees(th3_alt)))
        s4_alt = int(round(90.0 + math.degrees(th4_alt)))

        sol_alt = [s1, s2_alt, s3_alt, s4_alt, s5, s6]

        valid_sols = []
        if all(0 <= sol[i] <= 180 for i in range(5)):
            valid_sols.append(sol)
        if all(0 <= sol_alt[i] <= 180 for i in range(5)):
            valid_sols.append(sol_alt)

        if not valid_sols:
            return None

        if len(valid_sols) == 1 or current_angles is None:
            return valid_sols[0]

        # Choose the branch closest to current joint angles to prevent jumping
        def dist_sq_fn(s):
            return sum((s[i] - current_angles[i]) ** 2 for i in range(min(5, len(current_angles))))
        return min(valid_sols, key=dist_sq_fn)

    def solve_ik_clamped(self,
                         x: float,
                         y: float,
                         z: float,
                         current_angles: List[int],
                         preferred_pitch: Optional[float] = None) -> Tuple[Optional[List[int]], Tuple[float, float, float]]:
        """
        Robust IK solver with reach boundary clamping for Interactive 3D Gizmo dragging.
        If target (X, Y, Z) is outside reach, smoothly projects/clamps the target along
        the ray from the current TCP to find the furthest reachable point toward the cursor.
        The robot arm NEVER freezes or locks up when dragging outside workspace limits.
        Returns: (solution_angles, (actual_x, actual_y, actual_z))
        """
        pose = self.forward_kinematics([float(a) for a in current_angles])
        cur_x, cur_y, cur_z = pose.tcp_x, pose.tcp_y, pose.tcp_z
        pitch = preferred_pitch if preferred_pitch is not None else pose.pitch_deg
        roll = float(current_angles[4]) if len(current_angles) > 4 else 90.0
        grip = float(current_angles[5]) if len(current_angles) > 5 else 40.0

        # Safe Z floor limit (table height protection)
        safe_z = max(5.0, float(z))
        target_x = float(x)
        target_y = float(y)

        # 1. Try target directly
        sol = self.inverse_kinematics(
            target_x, target_y, safe_z,
            pitch_deg=pitch, roll_deg=roll, gripper_deg=grip,
            auto_adjust_pitch=True, current_angles=current_angles
        )
        if sol is not None:
            return sol, (target_x, target_y, safe_z)

        # 2. Target is outside reach: binary search along line segment from current TCP to target
        p_cur = np.array([cur_x, cur_y, cur_z], dtype=np.float64)
        p_tgt = np.array([target_x, target_y, safe_z], dtype=np.float64)

        low = 0.0
        high = 1.0
        best_sol = None
        best_pt = p_cur

        for _ in range(8):
            mid = (low + high) * 0.5
            p_mid = (1.0 - mid) * p_cur + mid * p_tgt
            s = self.inverse_kinematics(
                float(p_mid[0]), float(p_mid[1]), float(p_mid[2]),
                pitch_deg=pitch, roll_deg=roll, gripper_deg=grip,
                auto_adjust_pitch=True, current_angles=current_angles
            )
            if s is not None:
                best_sol = s
                best_pt = p_mid
                low = mid  # Try reaching further toward target
            else:
                high = mid # Pull back toward known reachable position

        if best_sol is not None:
            return best_sol, (float(best_pt[0]), float(best_pt[1]), float(best_pt[2]))

        # Fallback to current angles
        return list(current_angles), (cur_x, cur_y, cur_z)

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

    def check_collision(self, pose: PoseData, workpiece_bounds: Optional[Tuple[float, float, float, float, float, float]] = None) -> Tuple[bool, str]:
        """
        Evaluates potential collisions for the given pose:
        1. Floor/Table penetration (Z < 1 mm).
        2. Self-collision (Elbow / Wrist / Gripper folding into base).
        3. Workpiece collision (Body links penetrating workpiece bounding box).
        Returns (has_collision, description).
        """
        # 1. Floor collision check
        # Check TCP and all joint origins
        for i, m in enumerate(pose.link_matrices):
            pos_z = float(m[2, 3])
            # Links 2..7 must remain above table floor Z=0
            if i >= 2 and pos_z < 0.0:
                link_name = f"Link {i}" if i < 4 else ("Ranne" if i == 4 else "Gripper")
                return True, f"Törmäys työpöytään ({link_name} Z={pos_z:.1f} mm < 0 mm)"

        if pose.tcp_z < 0.0:
            return True, f"TCP törmää työpöytään (Z={pose.tcp_z:.1f} mm < 0 mm)"

        # 2. Self-collision check (folded configurations)
        # Distance from Wrist joint (matrix 4) to Base origin (0, 0, L0)
        p_wrist = pose.link_matrices[4][:3, 3]
        dist_wrist_base = math.sqrt(p_wrist[0]**2 + p_wrist[1]**2 + (p_wrist[2] - self.l0)**2)
        if dist_wrist_base < 65.0:
            return True, f"Itsetörmäys: Ranne osuu jalustaan (etäisyys {dist_wrist_base:.1f} mm < 65 mm)"

        # 3. Workpiece collision check
        if workpiece_bounds:
            xmin, xmax, ymin, ymax, zmin, zmax = workpiece_bounds
            # Check if Link 2 or Link 3 origin falls inside workpiece
            for i in (2, 3, 4):
                lp = pose.link_matrices[i][:3, 3]
                if (xmin <= lp[0] <= xmax) and (ymin <= lp[1] <= ymax) and (zmin <= lp[2] <= zmax):
                    return True, f"Törmäys työkappaleeseen (Link {i} sisällä)"

        return False, "Ei törmäystä (Clear)"

    def check_singularity(self, joint_angles: List[int]) -> Optional[str]:
        """
        Monitors proximity to kinematic singularities:
        - Boundary / Reach Singularity (arm stretched to absolute maximum reach).
        - Wrist Alignment Singularity (wrist pitch/roll collinearity).
        """
        pose = self.forward_kinematics([float(a) for a in joint_angles])
        # Horizontal reach from shoulder
        r_reach = math.sqrt(pose.tcp_x**2 + pose.tcp_y**2 + (pose.tcp_z - self.l0)**2)
        max_possible = self.l1 + self.l2 + self.l3

        if r_reach > 0.96 * max_possible:
            return f"Äärirajas singulariteetti: Käsivarsi lähes suorana ({r_reach:.1f} mm / {max_possible:.1f} mm)"

        j3 = joint_angles[2]
        if abs(j3 - 90) > 85:
            return f"Kyynärnivelen ääriasento: J3={j3}° lähellä mekaanista rajaa"

        return None

