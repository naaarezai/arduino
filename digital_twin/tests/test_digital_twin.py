"""
Automated Integration and Regression Tests for 6-DOF Robot Arm Digital Twin.
Verifies kinematics, calibration manager, post-processor, and 3D CAD transformations.
"""
import unittest
import math
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.config import (
    LINK_0_HEIGHT, LINK_1_LENGTH, LINK_2_LENGTH, LINK_3_LENGTH,
    MODELS_DIR, JOINT_LIMITS
)
from src.core.kinematics import RobotKinematics
from src.core.calibration import CalibrationManager
from src.core.post_processor import RobotPostProcessor
from src.core.program import RobotProgram, MotionStatement, GripperStatement, DelayStatement
from src.core.tool_data import ToolProfile, ToolManager, TOOL_PRESETS


class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.kin = RobotKinematics(
            l0=LINK_0_HEIGHT,
            l1=LINK_1_LENGTH,
            l2=LINK_2_LENGTH,
            l3=LINK_3_LENGTH
        )

    def test_home_pose_forward_kinematics(self):
        """Home pose (all 90 deg) points straight up."""
        pose = self.kin.forward_kinematics([90, 90, 90, 90, 90, 40])
        expected_z = LINK_0_HEIGHT + LINK_1_LENGTH + LINK_2_LENGTH + LINK_3_LENGTH
        self.assertAlmostEqual(pose.tcp_x, 0.0, places=1)
        self.assertAlmostEqual(pose.tcp_y, 0.0, places=1)
        self.assertAlmostEqual(pose.tcp_z, expected_z, places=1)
        self.assertAlmostEqual(pose.pitch_deg, 90.0, places=1)

    def test_seamless_joint_connections(self):
        """Verifies that each link connects with 0.0 mm gap to its parent link."""
        angles = [180, 0, 180, 0, 20, 122]
        pose = self.kin.forward_kinematics(angles)
        
        # Link 4 wrist flange to Gripper mount face must be exactly 0.0 mm
        t_link4 = pose.link_matrices[4]
        t_grip = pose.link_matrices[6]
        flange_world = t_link4 @ np.array([0, 0, 50.0, 1.0])
        grip_base_world = t_grip @ np.array([0, 0, 0.0, 1.0])
        dist = np.linalg.norm(flange_world[:3] - grip_base_world[:3])
        self.assertAlmostEqual(dist, 0.0, places=3)

    def test_collision_detection(self):
        """Verifies table floor and reach collision detection."""
        # Arm reaching into table floor (Z < 0)
        floor_colliding_pose = self.kin.forward_kinematics([90, 170, 170, 170, 90, 40])
        has_coll, reason = self.kin.check_collision(floor_colliding_pose)
        self.assertTrue(has_coll)
        self.assertIn("työpöyt", reason.lower())

        # Safe upright pose
        safe_pose = self.kin.forward_kinematics([90, 90, 90, 90, 90, 40])
        has_coll_safe, _ = self.kin.check_collision(safe_pose)
        self.assertFalse(has_coll_safe)

    def test_inverse_kinematics_full_pitch_range(self):
        """Verifies that full pitch range (-85° .. +85°) enables solutions at close radii."""
        # Radius 260 at Z=200 requires pitch ~ -70° to -80°
        sol = self.kin.inverse_kinematics(260.0, 0.0, 200.0, pitch_deg=0.0, auto_adjust_pitch=True)
        self.assertIsNotNone(sol, "Should find IK solution using expanded pitch search")
        self.assertTrue(all(0 <= sol[i] <= 180 for i in range(5)))

    def test_solve_ik_clamped_boundary_projection(self):
        """Verifies that out-of-reach drag targets never freeze and clamp to valid boundary."""
        cur = [90, 90, 90, 90, 90, 40]
        # Target far out of reach (X=700 mm)
        sol_far, pt_far = self.kin.solve_ik_clamped(700.0, 0.0, 200.0, current_angles=cur)
        self.assertIsNotNone(sol_far, "Clamped IK must never return None on out-of-reach target")
        self.assertLess(pt_far[0], 700.0, "Clamped point should be projected closer")
        self.assertTrue(all(0 <= sol_far[i] <= 180 for i in range(5)))

        # Target in inner dead zone (X=30 mm, Z=80 mm)
        sol_inner, pt_inner = self.kin.solve_ik_clamped(30.0, 0.0, 80.0, current_angles=cur)
        self.assertIsNotNone(sol_inner, "Clamped IK must never return None on inner dead zone target")
        self.assertTrue(all(0 <= sol_inner[i] <= 180 for i in range(5)))


class TestCalibrationManager(unittest.TestCase):
    def test_trim_and_inversion(self):
        cm = CalibrationManager()
        cm.set_offset(1, 5)   # +5 deg mechanical trim on J2
        cm.set_inverted(1, True)
        
        # Simulation angle 100 -> Inverted: (180 - 100) = 80 -> Trim: 80 + 5 = 85
        phys = cm.to_physical([90, 100, 90, 90, 90, 40])
        self.assertEqual(phys[1], 85)

        # Clamping J6 (Servo 6) to safe boundary [40, 128]
        cm.set_offset(5, 50)
        phys_clamped = cm.to_physical([90, 90, 90, 90, 90, 120])
        self.assertEqual(phys_clamped[5], 128)


class TestPostProcessor(unittest.TestCase):
    def test_arduino_sketch_generation(self):
        prog = RobotProgram(name="Weld Routine")
        prog.add_motion([90, 90, 90, 90, 90, 40], motion_type="PTP", target_id="Approach", comment="Approach")
        prog.add_motion([90, 120, 60, 90, 90, 40], motion_type="LIN", target_id="Weld Touchdown", comment="Weld Touchdown")
        
        sketch = RobotPostProcessor.to_arduino_sketch(prog)
        self.assertIn("#include <Servo.h>", sketch)
        # Check Uno R4 WiFi servo pins: 3, 5, 6, 9, 10, 11
        self.assertIn("const int servoPins[6] = {3, 5, 6, 9, 10, 11};", sketch)
        self.assertIn("Approach", sketch)
        self.assertIn("Weld Touchdown", sketch)


class TestToolManagerAndTCP(unittest.TestCase):
    def test_tool_presets_and_persistence(self):
        tm = ToolManager()
        # Set welding torch preset
        torch = tm.set_preset("WELDING_TORCH")
        self.assertEqual(torch.tool_type, "WELDING_TORCH")
        self.assertEqual(torch.tcp_z, 135.0)
        self.assertEqual(torch.tcp_x, 35.0)
        
        # Load from disk
        loaded = tm.load_tool()
        self.assertEqual(loaded.tool_type, "WELDING_TORCH")
        self.assertEqual(loaded.tcp_z, 135.0)

        # Reset back to gripper
        tm.set_preset("GRIPPER")

    def test_kinematics_with_active_tool(self):
        kin = RobotKinematics()
        
        # 1. With standard gripper (tcp_z = 99.3)
        gripper = TOOL_PRESETS["GRIPPER"]
        kin.set_active_tool(gripper)
        p_grip = kin.forward_kinematics([90, 90, 90, 90, 90, 40])
        self.assertAlmostEqual(p_grip.tcp_z, 688.3, places=1)
        self.assertAlmostEqual(p_grip.tcp_x, 0.0, places=1)

        # 2. With welding torch (tcp_z = 135.0, tcp_x = 35.0, pitch = 45°)
        torch = TOOL_PRESETS["WELDING_TORCH"]
        kin.set_active_tool(torch)
        p_torch = kin.forward_kinematics([90, 90, 90, 90, 90, 40])
        # Tool flange is at Z = 589.0 in upright home pose.
        # Torch has tcp_x = 35.0, tcp_z = 135.0 with 45 deg pitch
        # Flange normal is along +Z, so tool offset is directly added
        self.assertGreater(p_torch.tcp_z, p_grip.tcp_z)
        self.assertNotEqual(p_torch.tcp_x, 0.0)


class TestUXAndSafety(unittest.TestCase):
    def test_serial_worker_emergency_stop(self):
        from src.communication.serial_worker import SerialWorker
        worker = SerialWorker()
        worker.send_angles([90, 100, 110, 90, 90, 40])
        self.assertIsNotNone(worker._pending_angles)
        worker.emergency_stop()
        self.assertIsNone(worker._pending_angles, "Emergency stop must immediately flush pending serial queue")

    def test_port_metadata_structure(self):
        from src.communication.serial_worker import SerialWorker
        details = SerialWorker.list_port_details()
        self.assertIsInstance(details, list)
        for p in details:
            self.assertIn("device", p)
            self.assertIn("description", p)
            self.assertIn("is_arduino", p)


if __name__ == "__main__":
    unittest.main()
