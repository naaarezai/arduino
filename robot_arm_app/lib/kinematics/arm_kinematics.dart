import 'dart:math' as math;

/// 3D Karteesinen piste ja asento avaruudessa
class CartesianPose {
  final double x; // mm (+X eteenpäin, -X taaksepäin)
  final double y; // mm (+Y vasemmalle, -Y oikealle)
  final double z; // mm (korkeus pöytätasosta: +Z ylöspäin, 0 = pöytä)
  final double pitch; // astetta (-90° suoraan alas, 0° vaakasuoraan eteen, +90° suoraan ylös)
  final double roll;  // astetta (ranteen kierto 0..180)
  final double gripper; // astetta (kouran avautuma 40..128)

  const CartesianPose({
    required this.x,
    required this.y,
    required this.z,
    this.pitch = 0.0,
    this.roll = 90.0,
    this.gripper = 40.0,
  });

  CartesianPose copyWith({
    double? x,
    double? y,
    double? z,
    double? pitch,
    double? roll,
    double? gripper,
  }) {
    return CartesianPose(
      x: x ?? this.x,
      y: y ?? this.y,
      z: z ?? this.z,
      pitch: pitch ?? this.pitch,
      roll: roll ?? this.roll,
      gripper: gripper ?? this.gripper,
    );
  }

  @override
  String toString() =>
      'XYZ(X: ${x.toStringAsFixed(1)}mm, Y: ${y.toStringAsFixed(1)}mm, Z: ${z.toStringAsFixed(1)}mm, Pitch: ${pitch.toStringAsFixed(1)}°)';
}

/// Robottikäsivarren mitat (millimetreinä)
class ArmDimensions {
  final double l0; // Jalustan korkeus pöydästä olkaniveleen (140 mm)
  final double l1; // Olkavarsi: olkanivelestä kyynärniveleen (200 mm)
  final double l2; // Kyynärvarsi: kyynärnivelestä ranteeseen (200 mm)
  final double l3; // Koura: ranteesta kouran kärkeen (180 mm)

  const ArmDimensions({
    this.l0 = 140.0,
    this.l1 = 200.0,
    this.l2 = 200.0,
    this.l3 = 180.0,
  });

  /// Maksimi ulottuvuus säteenä jalustasta
  double get maxReachRadius => l1 + l2 + l3; // 580 mm
}

/// Kinematiikkalaskin: Suora kinematiikka (FK), Käänteinen kinematiikka (IK), PTP ja Linear
class ArmKinematics {
  final ArmDimensions dims;

  ArmKinematics({this.dims = const ArmDimensions()});

  // =========================================================================
  // 1. SUORA KINEMATIIKKA (Forward Kinematics - FK)
  // Laskee servokulmista (0-180°) kouran todellisen (X, Y, Z) -sijainnin mm
  // =========================================================================
  CartesianPose forwardKinematics(List<double> angles) {
    if (angles.length < 6) {
      return const CartesianPose(x: 0, y: 0, z: 0);
    }

    final s1 = angles[0]; // Jalusta 0..180 (90 = suoraan eteen)
    final s2 = angles[1]; // Olka 0..180 (90 = pystyssä, 180 = eteen vaaka)
    final s3 = angles[2]; // Kyynär 0..180 (90 = suora, >90 = koukku eteen)
    final s4 = angles[3]; // Ranne pysty 0..180 (90 = suora)
    final s5 = angles[4]; // Ranne kierto
    final s6 = angles[5]; // Koura

    // Jalustan kulma radiaaneina (90° = 0 rad, kierto vastapäivään +Y)
    final thetaBase = (s1 - 90.0) * (math.pi / 180.0);

    // Segmenttien kulmat suhteessa pystyakseliin
    final theta2 = (s2 - 90.0) * (math.pi / 180.0);
    final theta3 = (s3 - 90.0) * (math.pi / 180.0);
    final theta4 = (s4 - 90.0) * (math.pi / 180.0);

    final psi1 = theta2;
    final psi2 = theta2 + theta3;
    final psi3 = theta2 + theta3 + theta4;

    // Säde ja korkeus 2D-profiilitasossa
    final r = dims.l1 * math.sin(psi1) +
        dims.l2 * math.sin(psi2) +
        dims.l3 * math.sin(psi3);

    final z = dims.l0 +
        dims.l1 * math.cos(psi1) +
        dims.l2 * math.cos(psi2) +
        dims.l3 * math.cos(psi3);

    // 3D Karteesiset koordinaatit
    final x = r * math.cos(thetaBase);
    final y = r * math.sin(thetaBase);

    // Kouran pitch-kulma horisontista (-90° alas, 0° suoraan eteen, +90° ylös)
    final pitch = 90.0 - (psi3 * 180.0 / math.pi);

    return CartesianPose(
      x: x,
      y: y,
      z: z,
      pitch: pitch,
      roll: s5,
      gripper: s6.clamp(40.0, 128.0),
    );
  }

  // =========================================================================
  // 2. KÄÄNTEINEN KINEMATIIKKA (Inverse Kinematics - IK)
  // Laskee (X, Y, Z, Pitch) -koordinaateista tarvittavat 6 servon kulmaa (0..180)
  // =========================================================================
  List<double>? inverseKinematics(
    CartesianPose target, {
    bool? elbowUp,
    bool autoAdjustPitch = true,
  }) {
    // Kokeillaan ensin annettua elbowUp-konfiguraatiota (tai false ja true)
    if (elbowUp != null) {
      final res = _solveExactIK(target, elbowUp);
      if (res != null) return res;
    } else {
      final resDown = _solveExactIK(target, false);
      if (resDown != null) return resDown;

      final resUp = _solveExactIK(target, true);
      if (resUp != null) return resUp;
    }

    // Jos suora kulma ei tavoita pistettä ja autoAdjustPitch on sallittu,
    // etsitään lähin toimiva pitch-kulma (±35° säteellä)
    if (autoAdjustPitch) {
      for (double delta = 5.0; delta <= 35.0; delta += 5.0) {
        for (double sign in [-1.0, 1.0]) {
          final testPitch = target.pitch + (delta * sign);
          final adjustedPose = target.copyWith(pitch: testPitch);

          final resDown = _solveExactIK(adjustedPose, false);
          if (resDown != null) return resDown;

          final resUp = _solveExactIK(adjustedPose, true);
          if (resUp != null) return resUp;
        }
      }
    }

    return null; // Työalueen ulkopuolella
  }

  List<double>? _solveExactIK(CartesianPose target, bool elbowUp) {
    // 1. Jalustan kulma (Servo 1)
    final rTarget = math.sqrt(target.x * target.x + target.y * target.y);
    double s1;
    if (rTarget < 0.001) {
      s1 = 90.0;
    } else {
      final baseRad = math.atan2(target.y, target.x);
      s1 = 90.0 + (baseRad * 180.0 / math.pi);
    }

    if (s1 < 0.0 || s1 > 180.0) {
      return null;
    }

    // 2. Rannenivelen sijainti (Wrist Center)
    final psi3 = (90.0 - target.pitch) * (math.pi / 180.0);
    final rWrist = rTarget - dims.l3 * math.sin(psi3);
    final zWrist = target.z - dims.l3 * math.cos(psi3);

    final rRel = rWrist;
    final zRel = zWrist - dims.l0;

    final distSq = rRel * rRel + zRel * zRel;
    final dist = math.sqrt(distSq);

    final maxDist = dims.l1 + dims.l2; // 400 mm
    final minDist = (dims.l1 - dims.l2).abs(); // 0 mm
    if (dist > maxDist || dist < minDist || dist < 1.0) {
      return null;
    }

    // 3. Kyynärnivelen kulma (Servo 3) kosinilauseella
    final cosTheta3 = (distSq - dims.l1 * dims.l1 - dims.l2 * dims.l2) /
        (2.0 * dims.l1 * dims.l2);
    final clampedCos3 = cosTheta3.clamp(-1.0, 1.0);
    double theta3 = math.acos(clampedCos3);

    if (elbowUp) {
      theta3 = -theta3;
    }

    // 4. Olkanivelen kulma (Servo 2)
    final alpha = math.atan2(rRel, zRel);
    final cosBeta = (dims.l1 * dims.l1 + distSq - dims.l2 * dims.l2) /
        (2.0 * dims.l1 * dist);
    final clampedCosBeta = cosBeta.clamp(-1.0, 1.0);
    final beta = math.acos(clampedCosBeta);

    final theta2 = elbowUp ? (alpha + beta) : (alpha - beta);

    // 5. Rannenivelen pystykulma (Servo 4)
    final theta4 = psi3 - theta2 - theta3;

    final s2 = 90.0 + (theta2 * 180.0 / math.pi);
    final s3 = 90.0 + (theta3 * 180.0 / math.pi);
    final s4 = 90.0 + (theta4 * 180.0 / math.pi);

    if (s2 < 0.0 || s2 > 180.0 ||
        s3 < 0.0 || s3 > 180.0 ||
        s4 < 0.0 || s4 > 180.0) {
      return null;
    }

    final s5 = target.roll.clamp(0.0, 180.0);
    final s6 = target.gripper.clamp(40.0, 128.0); // Servo 6 rajoitus 40-128°

    return [s1, s2, s3, s4, s5, s6];
  }

  // =========================================================================
  // 3. LINEAR LIIKE (Karteesinen suora linja 3D-avaruudessa)
  // =========================================================================
  List<List<int>> generateLinearPath({
    required CartesianPose start,
    required CartesianPose target,
    double stepSizeMm = 8.0,
  }) {
    final dx = target.x - start.x;
    final dy = target.y - start.y;
    final dz = target.z - start.z;
    final totalDist = math.sqrt(dx * dx + dy * dy + dz * dz);

    final steps = (totalDist / stepSizeMm).ceil().clamp(2, 60);
    final List<List<int>> trajectory = [];

    for (int i = 0; i <= steps; i++) {
      final t = i / steps.toDouble();
      final interpolatedPose = CartesianPose(
        x: start.x + dx * t,
        y: start.y + dy * t,
        z: start.z + dz * t,
        pitch: start.pitch + (target.pitch - start.pitch) * t,
        roll: start.roll + (target.roll - start.roll) * t,
        gripper: start.gripper + (target.gripper - start.gripper) * t,
      );

      final anglesDouble = inverseKinematics(interpolatedPose, autoAdjustPitch: true);
      if (anglesDouble != null) {
        trajectory.add(anglesDouble.map((a) => a.round()).toList());
      } else {
        trajectory.add(trajectory.isNotEmpty
            ? trajectory.last
            : forwardKinematicsAsInt(start));
      }
    }

    return trajectory;
  }

  // =========================================================================
  // 4. PTP LIIKE (Point-to-Point Nivel-interpolaatio)
  // =========================================================================
  List<List<int>> generatePtpPath({
    required List<int> startAngles,
    required List<int> targetAngles,
    int steps = 25,
  }) {
    if (startAngles.length < 6 || targetAngles.length < 6) return [targetAngles];

    final List<List<int>> trajectory = [];
    for (int i = 0; i <= steps; i++) {
      final t = i / steps.toDouble();
      final smoothT = 0.5 * (1.0 - math.cos(t * math.pi));

      final stepAngles = <int>[];
      for (int j = 0; j < 6; j++) {
        final val = startAngles[j] + (targetAngles[j] - startAngles[j]) * smoothT;
        if (j == 5) {
          stepAngles.add(val.round().clamp(40, 128)); // Servo 6 rajoitus
        } else {
          stepAngles.add(val.round().clamp(0, 180));
        }
      }
      trajectory.add(stepAngles);
    }
    return trajectory;
  }

  List<int> forwardKinematicsAsInt(CartesianPose pose) {
    final ik = inverseKinematics(pose);
    if (ik != null) return ik.map((a) => a.round()).toList();
    return [90, 90, 90, 90, 90, 40];
  }
}
