import 'package:flutter_test/flutter_test.dart';
import 'package:robot_arm_app/kinematics/arm_kinematics.dart';

void main() {
  final kinematics = ArmKinematics(
    dims: const ArmDimensions(l0: 140, l1: 200, l2: 200, l3: 180),
  );

  test('FK calculates home pose correctly', () {
    final pose = kinematics.forwardKinematics([90, 90, 90, 90, 90, 40]);
    expect(pose.x.abs() < 1.0, isTrue);
    expect(pose.y.abs() < 1.0, isTrue);
    expect((pose.z - 720.0).abs() < 1.0, isTrue);
    expect((pose.pitch - 90.0).abs() < 1.0, isTrue);
  });

  test('IK can solve realistic picking pose', () {
    final target = const CartesianPose(x: 200, y: 0, z: 250, pitch: -60, roll: 90, gripper: 40);
    final angles = kinematics.inverseKinematics(target);
    expect(angles, isNotNull);

    final fk = kinematics.forwardKinematics(angles!);
    expect((fk.x - target.x).abs() < 3.0, isTrue);
    expect((fk.y - target.y).abs() < 3.0, isTrue);
    expect((fk.z - target.z).abs() < 3.0, isTrue);
    expect(angles[5] >= 40 && angles[5] <= 128, isTrue);
  });

  test('Linear path generator produces multiple steps', () {
    final start = const CartesianPose(x: 250, y: -50, z: 200, pitch: 0);
    final target = const CartesianPose(x: 250, y: 50, z: 200, pitch: 0);
    final path = kinematics.generateLinearPath(start: start, target: target, stepSizeMm: 10.0);
    expect(path.length >= 5, isTrue);
  });
}
