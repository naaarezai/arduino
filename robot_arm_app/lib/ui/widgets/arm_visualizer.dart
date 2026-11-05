import 'dart:math' as math;
import 'package:flutter/material.dart';
import '../../kinematics/arm_kinematics.dart';

/// 2D Kaavio / visualisointi robottikäsivarren asennosta reaaliajassa
class ArmVisualizer extends StatelessWidget {
  final List<double> angles; // [base, shoulder, elbow, wristPitch, wristRoll, gripper]
  final CartesianPose? cartesianPose;
  final double height;

  const ArmVisualizer({
    super.key,
    required this.angles,
    this.cartesianPose,
    this.height = 220,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      height: height,
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        color: const Color(0xFF1E222D),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.cyanAccent.withValues(alpha: 0.2)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: 0.4),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Stack(
        children: [
          // Ruudukko taustalla
          CustomPaint(
            size: Size.infinite,
            painter: _GridBackgroundPainter(),
          ),
          // Robottikäsivarren kinematiikka
          ClipRRect(
            borderRadius: BorderRadius.circular(16),
            child: CustomPaint(
              size: Size.infinite,
              painter: _ArmKinematicsPainter(angles: angles),
            ),
          ),
          // Yläkulman info-otsikko
          Positioned(
            top: 12,
            left: 14,
            child: Row(
              children: [
                const Icon(Icons.precision_manufacturing, size: 18, color: Colors.cyanAccent),
                const SizedBox(width: 6),
                Text(
                  "KÄSIVARREN ASENTO (KINEMATIIKKA)",
                  style: TextStyle(
                    color: Colors.cyanAccent.withValues(alpha: 0.9),
                    fontSize: 11,
                    fontWeight: FontWeight.bold,
                    letterSpacing: 1.1,
                  ),
                ),
              ],
            ),
          ),
          // Jalustan kiertoindikaattori (Base Azimuth)
          Positioned(
            top: 10,
            right: 14,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                color: Colors.black.withValues(alpha: 0.5),
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.white24),
              ),
              child: Row(
                children: [
                  const Icon(Icons.rotate_right, size: 14, color: Colors.orangeAccent),
                  const SizedBox(width: 4),
                  Text(
                    "Jalusta: ${angles.isNotEmpty ? angles[0].round() : 90}°",
                    style: const TextStyle(
                      color: Colors.white70,
                      fontSize: 11,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ],
              ),
            ),
          ),
          // Alakulman koordinaattinäyttö (XYZ)
          if (cartesianPose != null)
            Positioned(
              bottom: 10,
              left: 14,
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: Colors.black.withValues(alpha: 0.65),
                  borderRadius: BorderRadius.circular(6),
                  border: Border.all(color: Colors.cyanAccent.withValues(alpha: 0.3)),
                ),
                child: Text(
                  "X: ${cartesianPose!.x.round()}  Y: ${cartesianPose!.y.round()}  Z: ${cartesianPose!.z.round()} mm",
                  style: const TextStyle(
                    color: Colors.cyanAccent,
                    fontSize: 11,
                    fontFamily: 'monospace',
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}

class _GridBackgroundPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white.withValues(alpha: 0.04)
      ..strokeWidth = 1.0;

    const step = 20.0;
    for (double x = 0; x < size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    for (double y = 0; y < size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

class _ArmKinematicsPainter extends CustomPainter {
  final List<double> angles;

  _ArmKinematicsPainter({required this.angles});

  @override
  void paint(Canvas canvas, Size size) {
    if (angles.length < 6) return;

    final base = angles[0];
    final shoulder = angles[1];
    final elbow = angles[2];
    final wristPitch = angles[3];
    final wristRoll = angles[4];
    final gripper = angles[5];

    final origin = Offset(size.width * 0.45, size.height * 0.85);

    // Piirretään maapinta / jalusta
    final groundPaint = Paint()
      ..color = Colors.blueGrey.shade700
      ..strokeWidth = 4.0
      ..strokeCap = StrokeCap.round;

    canvas.drawLine(
      Offset(origin.dx - 60, origin.dy + 8),
      Offset(origin.dx + 60, origin.dy + 8),
      groundPaint,
    );

    // Jalustan runko
    final basePaint = Paint()
      ..color = const Color(0xFF2C3242)
      ..style = PaintingStyle.fill;
    final baseOutline = Paint()
      ..color = Colors.cyanAccent.withValues(alpha: 0.5)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;

    final baseRect = RRect.fromRectAndRadius(
      Rect.fromCenter(center: Offset(origin.dx, origin.dy), width: 50, height: 16),
      const Radius.circular(4),
    );
    canvas.drawRRect(baseRect, basePaint);
    canvas.drawRRect(baseRect, baseOutline);

    // Pituudet skaalattuna näyttöön
    final link1Len = size.height * 0.28; // Olkavarsi
    final link2Len = size.height * 0.24; // Kyynärvarsi
    final link3Len = size.height * 0.14; // Ranne
    final clawLen = size.height * 0.08;  // Koura

    final radShoulder = (shoulder - 90.0) * (math.pi / 180.0);
    final j1 = Offset(origin.dx, origin.dy - 8);
    final j2 = Offset(
      j1.dx + link1Len * math.sin(radShoulder),
      j1.dy - link1Len * math.cos(radShoulder),
    );

    final radElbow = radShoulder + (elbow - 90.0) * (math.pi / 180.0);
    final j3 = Offset(
      j2.dx + link2Len * math.sin(radElbow),
      j2.dy - link2Len * math.cos(radElbow),
    );

    final radWrist = radElbow + (wristPitch - 90.0) * (math.pi / 180.0);
    final j4 = Offset(
      j3.dx + link3Len * math.sin(radWrist),
      j3.dy - link3Len * math.cos(radWrist),
    );

    // Piirretään segmentit (Linkit)
    _drawLink(canvas, j1, j2, Colors.cyanAccent, 7.0);
    _drawLink(canvas, j2, j3, Colors.lightBlueAccent, 6.0);
    _drawLink(canvas, j3, j4, Colors.orangeAccent, 5.0);

    // Jalustan kääntölevy (Base turntable suunta)
    final baseTurntablePaint = Paint()
      ..color = Colors.cyanAccent.withValues(alpha: 0.7)
      ..strokeWidth = 3.0
      ..strokeCap = StrokeCap.round;
    final baseRad = (base - 90.0) * (math.pi / 180.0);
    final baseIndicatorEnd = Offset(
      origin.dx + 25 * math.sin(baseRad),
      origin.dy + 2,
    );
    canvas.drawLine(Offset(origin.dx, origin.dy + 2), baseIndicatorEnd, baseTurntablePaint);

    // Ranteen kierto (Wrist Roll indikaattori j4:ssä)
    final rollRad = (wristRoll - 90.0) * (math.pi / 180.0);
    final rollIndicatorEnd = Offset(
      j4.dx + 8 * math.cos(rollRad),
      j4.dy + 8 * math.sin(rollRad),
    );
    canvas.drawLine(j4, rollIndicatorEnd, Paint()..color = Colors.yellow..strokeWidth = 2);

    // Piirretään nivelet (Kiekot)
    _drawJoint(canvas, j1, "S1:Olka", Colors.cyan);
    _drawJoint(canvas, j2, "S2:Kyynär", Colors.lightBlue);
    _drawJoint(canvas, j3, "S3:Ranne", Colors.orange);

    // Piirretään koura (Gripper 40° = auki, 128° = kiinni)
    final clampedGripper = gripper.clamp(40.0, 128.0);
    final clawProgress = (clampedGripper - 40.0) / (128.0 - 40.0);
    final clawAngleOffset = (1.0 - clawProgress) * 0.42 + 0.12;

    final claw1End = Offset(
      j4.dx + clawLen * math.sin(radWrist - clawAngleOffset),
      j4.dy - clawLen * math.cos(radWrist - clawAngleOffset),
    );
    final claw2End = Offset(
      j4.dx + clawLen * math.sin(radWrist + clawAngleOffset),
      j4.dy - clawLen * math.cos(radWrist + clawAngleOffset),
    );

    final clawPaint = Paint()
      ..color = Colors.amberAccent
      ..strokeWidth = 3.5
      ..strokeCap = StrokeCap.round;

    canvas.drawLine(j4, claw1End, clawPaint);
    canvas.drawLine(j4, claw2End, clawPaint);

    // Kouran kärjet
    canvas.drawCircle(claw1End, 3, Paint()..color = Colors.white);
    canvas.drawCircle(claw2End, 3, Paint()..color = Colors.white);
  }

  void _drawLink(Canvas canvas, Offset p1, Offset p2, Color color, double width) {
    final shadowPaint = Paint()
      ..color = color.withValues(alpha: 0.3)
      ..strokeWidth = width + 4
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(p1, p2, shadowPaint);

    final linkPaint = Paint()
      ..color = color
      ..strokeWidth = width
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(p1, p2, linkPaint);
  }

  void _drawJoint(Canvas canvas, Offset p, String label, Color color) {
    final outerPaint = Paint()
      ..color = const Color(0xFF151821)
      ..style = PaintingStyle.fill;
    final ringPaint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.5;
    final centerPaint = Paint()..color = Colors.white;

    canvas.drawCircle(p, 8, outerPaint);
    canvas.drawCircle(p, 8, ringPaint);
    canvas.drawCircle(p, 2.5, centerPaint);
  }

  @override
  bool shouldRepaint(covariant _ArmKinematicsPainter oldDelegate) {
    return oldDelegate.angles != angles;
  }
}
