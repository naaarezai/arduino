"""
Industrial Welding Planner & Trajectory Generator for 6-DOF Robot Arm.
Implements Visual Components / Robotmaster standard features:
1. Weave Motion Generator (Zigzag & Sine wave weaving for V-grooves and fillets)
2. Multi-Pass Seam Generator (Root pass, Fill pass, Cap pass)
3. Torch Work & Travel Angle orientation (Push / Drag angle adjustment)
4. One-Click Seam Reversal (Inverts motion order while maintaining approach/retract)
"""
import copy
import math
from typing import Dict, List, Optional, Tuple
import numpy as np

from src.core.kinematics import RobotKinematics
from src.core.program import Statement, MotionStatement, DelayStatement, GripperStatement


def generate_weave_path(p_start: Tuple[float, float, float],
                        p_end: Tuple[float, float, float],
                        amplitude: float = 3.0,
                        pitch: float = 6.0,
                        weave_type: str = "ZIGZAG") -> List[Tuple[float, float, float]]:
    """
    Computes 3D Cartesian coordinates for a weaving weld bead along a seam vector.
    Supports ZIGZAG, SINE, and TRIANGLE weave patterns.
    """
    p_a = np.array(p_start, dtype=np.float64)
    p_b = np.array(p_end, dtype=np.float64)
    v = p_b - p_a
    length = float(np.linalg.norm(v))
    if length < 1.0 or amplitude <= 0.0 or pitch <= 0.0 or weave_type == "NONE":
        return [tuple(p_a), tuple(p_b)]

    t_hat = v / length

    # Transverse vector perpendicular to seam travel
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    n = np.cross(t_hat, up)
    if np.linalg.norm(n) < 0.01:
        # Seam is vertical along Z -> use Y axis for transverse vector
        n = np.cross(t_hat, np.array([0.0, 1.0, 0.0], dtype=np.float64))
    n_hat = n / np.linalg.norm(n)

    num_half_steps = max(4, int(round(2.0 * length / max(1.0, pitch))))
    points = []
    for k in range(num_half_steps + 1):
        s = (k / num_half_steps) * length
        center_pt = p_a + s * t_hat

        # Taper to exact centerline at start and end
        if k == 0 or k == num_half_steps:
            offset = 0.0
        elif weave_type == "SINE":
            offset = amplitude * math.sin(2.0 * math.pi * s / pitch)
        else:  # ZIGZAG (default)
            offset = amplitude if (k % 2 == 1) else -amplitude

        pt = center_pt + offset * n_hat
        points.append((float(pt[0]), float(pt[1]), float(pt[2])))

    return points


def generate_industrial_weld_sequence(p_start: Tuple[float, float, float],
                                      p_end: Tuple[float, float, float],
                                      kinematics: RobotKinematics,
                                      current_angles: List[int],
                                      approach_h: float = 25.0,
                                      speed_percent: int = 25,
                                      weave_type: str = "NONE",
                                      amplitude: float = 3.0,
                                      pitch: float = 6.0,
                                      drag_angle_deg: float = 10.0) -> Tuple[Optional[List[Statement]], Optional[str]]:
    """
    Compiles a complete 5-step (or N-step weaving) industrial welding sequence.
    Sequence:
    1. PTP Approach (+approach_h mm above seam start)
    2. LIN Touchdown at seam start (ARC ON)
    3. LIN Linear or Weave weld motions to seam end
    4. DELAY Crater fill dwell (500 ms, ARC OFF)
    5. LIN Retract (+approach_h mm above seam end)
    """
    xA, yA, zA = p_start
    xB, yB, zB = p_end

    seam_len = math.sqrt((xB - xA)**2 + (yB - yA)**2 + (zB - zA)**2)
    if seam_len < 2.0:
        return None, "Sauman pituus on liian lyhyt (< 2 mm)."

    # Determine preferred pitch and drag angle
    cur_pose = kinematics.forward_kinematics([float(a) for a in current_angles])
    pref_pitch = cur_pose.pitch_deg - drag_angle_deg

    # 1. Approach waypoint
    sol_app, _ = kinematics.solve_ik_clamped(xA, yA, zA + approach_h, current_angles=current_angles, preferred_pitch=pref_pitch)
    if sol_app is None:
        return None, "Lähestymispiste ylittää robotin ulottuvuusalueen."

    # 2. Touchdown waypoint
    sol_start, _ = kinematics.solve_ik_clamped(xA, yA, zA, current_angles=sol_app, preferred_pitch=pref_pitch)
    if sol_start is None:
        return None, "Sauman alkupiste ylittää robotin ulottuvuusalueen."

    statements: List[Statement] = []
    statements.append(MotionStatement(
        statement_type="PTP",
        angles=sol_app,
        speed_percent=100,
        comment=f"Lähestyminen saumaan (+{int(approach_h)}mm)"
    ))
    statements.append(MotionStatement(
        statement_type="LIN",
        angles=sol_start,
        speed_percent=50,
        comment="Kosketus / Valokaari syttyy (ARC ON)"
    ))

    # 3. Weld path (Straight or Weave)
    if weave_type in ("ZIGZAG", "SINE") and amplitude > 0.0:
        weave_pts = generate_weave_path(p_start, p_end, amplitude=amplitude, pitch=pitch, weave_type=weave_type)
        prev_sol = sol_start
        for i, wpt in enumerate(weave_pts[1:]):
            sol_w, _ = kinematics.solve_ik_clamped(wpt[0], wpt[1], wpt[2], current_angles=prev_sol, preferred_pitch=pref_pitch)
            if sol_w is None:
                return None, f"Pendlausliikkeen piste {i+1} ylittää robotin ulottuvuusalueen."
            statements.append(MotionStatement(
                statement_type="LIN",
                angles=sol_w,
                speed_percent=speed_percent,
                comment=f"Pendlaus {weave_type} ({i+1}/{len(weave_pts)-1})"
            ))
            prev_sol = sol_w
        last_sol = prev_sol
    else:
        # Standard straight LIN seam
        sol_end, _ = kinematics.solve_ik_clamped(xB, yB, zB, current_angles=sol_start, preferred_pitch=pref_pitch)
        if sol_end is None:
            return None, "Sauman loppupiste ylittää robotin ulottuvuusalueen."
        statements.append(MotionStatement(
            statement_type="LIN",
            angles=sol_end,
            speed_percent=speed_percent,
            comment=f"Hitsaussauma LIN ({seam_len:.1f} mm)"
        ))
        last_sol = sol_end

    # 4. Crater fill dwell
    statements.append(DelayStatement(
        duration_ms=500,
        comment="Kraatterin täyttö / Valokaari sammuu (ARC OFF)"
    ))

    # 5. Retract waypoint
    sol_retract, _ = kinematics.solve_ik_clamped(xB, yB, zB + approach_h, current_angles=last_sol, preferred_pitch=pref_pitch)
    if sol_retract is None:
        sol_retract = last_sol

    statements.append(MotionStatement(
        statement_type="LIN",
        angles=sol_retract,
        speed_percent=80,
        comment=f"Nosto sauman päältä (+{int(approach_h)}mm)"
    ))

    return statements, None


def generate_multipass_routine(p_start: Tuple[float, float, float],
                               p_end: Tuple[float, float, float],
                               kinematics: RobotKinematics,
                               current_angles: List[int],
                               num_passes: int = 3,
                               lateral_offset_mm: float = 2.5,
                               vertical_offset_mm: float = 1.8,
                               approach_h: float = 25.0,
                               speed_percent: int = 30) -> Tuple[Optional[List[Statement]], Optional[str]]:
    """
    Generates multi-pass welding sequence (e.g. 3 passes: Root, Fill Left, Cap Right).
    """
    p_a = np.array(p_start, dtype=np.float64)
    p_b = np.array(p_end, dtype=np.float64)
    v = p_b - p_a
    length = float(np.linalg.norm(v))
    if length < 2.0:
        return None, "Sauman pituus on liian lyhyt (< 2 mm)."

    t_hat = v / length
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    n = np.cross(t_hat, up)
    if np.linalg.norm(n) < 0.01:
        n = np.cross(t_hat, np.array([0.0, 1.0, 0.0], dtype=np.float64))
    n_hat = n / np.linalg.norm(n)

    # Pass definitions: (lateral_factor, vertical_factor, name)
    pass_configs = [
        (0.0, 0.0, "Juuripalko (Pass 1 Root)"),
        (-1.0, 0.4, "Täyttöpalko vasen (Pass 2 Fill)"),
        (1.0, 0.8, "Pintapalko oikea (Pass 3 Cap)")
    ]

    all_statements: List[Statement] = []
    cur_angles = list(current_angles)

    for i in range(min(num_passes, len(pass_configs))):
        lat_fac, vert_fac, pass_name = pass_configs[i]
        offset_vec = lat_fac * lateral_offset_mm * n_hat + np.array([0.0, 0.0, vert_fac * vertical_offset_mm])
        pass_a = tuple(p_a + offset_vec)
        pass_b = tuple(p_b + offset_vec)

        pass_stmts, err = generate_industrial_weld_sequence(
            pass_a, pass_b,
            kinematics=kinematics,
            current_angles=cur_angles,
            approach_h=approach_h,
            speed_percent=speed_percent,
            weave_type="NONE"
        )
        if err:
            return None, f"Virhe palkossa {i+1} ({pass_name}): {err}"

        # Update comments with pass identifier
        for st in pass_stmts:
            st.comment = f"[{pass_name}] " + st.comment
            all_statements.append(st)

        # Track end angles for next pass approach
        if pass_stmts and isinstance(pass_stmts[-1], MotionStatement):
            cur_angles = pass_stmts[-1].angles

    return all_statements, None


def reverse_seam_statements(statements: List[Statement]) -> List[Statement]:
    """
    Reverses an offline programming motion routine.
    Inverts the waypoint order so welding runs B -> A instead of A -> B.
    """
    motion_stmts = [s for s in statements if isinstance(s, MotionStatement)]
    if len(motion_stmts) < 2:
        return list(statements)

    # Invert the angles order
    reversed_motions = list(reversed(motion_stmts))

    # Update comments and types
    new_statements: List[Statement] = []
    for i, m in enumerate(reversed_motions):
        new_m = copy.deepcopy(m)
        if i == 0:
            new_m.statement_type = "PTP"
            new_m.speed_percent = 100
            new_m.comment = "Lähestyminen (Käännetty sauma)"
        elif i == 1:
            new_m.statement_type = "LIN"
            new_m.speed_percent = 50
            new_m.comment = "Kosketus / ARC ON (Käännetty sauma)"
        elif i == len(reversed_motions) - 1:
            new_m.statement_type = "LIN"
            new_m.speed_percent = 80
            new_m.comment = "Nosto (Käännetty sauma)"
        else:
            new_m.statement_type = "LIN"
            new_m.comment = f"Hitsaussauma LIN (Käännetty {i})"
        new_statements.append(new_m)

    # Insert a crater delay before the final retract
    delay = DelayStatement(duration_ms=500, comment="Kraatterin täyttö / ARC OFF")
    new_statements.insert(len(new_statements) - 1, delay)

    return new_statements
