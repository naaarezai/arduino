import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../../kinematics/arm_kinematics.dart';

/// Karteesisen XYZ-ohjauksen, PTP/Linear-liiketilan ja joggauksen käyttöliittymäkomponentti
class XyzControllerCard extends StatefulWidget {
  final CartesianPose currentPose;
  final Function(CartesianPose newPose, bool isLinear) onMoveRequested;
  final ArmKinematics kinematics;
  final bool isMoving;

  const XyzControllerCard({
    super.key,
    required this.currentPose,
    required this.onMoveRequested,
    required this.kinematics,
    this.isMoving = false,
  });

  @override
  State<XyzControllerCard> createState() => _XyzControllerCardState();
}

class _XyzControllerCardState extends State<XyzControllerCard> {
  // Askelkoko millimetreinä
  double _stepMm = 10.0;
  // Liiketila: true = Linear (Suora linja), false = PTP (Point-to-Point)
  bool _isLinearMode = false;

  void _jog({double dx = 0.0, double dy = 0.0, double dz = 0.0, double dpitch = 0.0}) {
    HapticFeedback.selectionClick();

    final target = widget.currentPose.copyWith(
      x: (widget.currentPose.x + dx).roundToDouble(),
      y: (widget.currentPose.y + dy).roundToDouble(),
      z: (widget.currentPose.z + dz).clamp(0.0, 750.0).roundToDouble(),
      pitch: (widget.currentPose.pitch + dpitch).clamp(-90.0, 90.0).roundToDouble(),
    );

    // Tarkistetaan onko uusi piste ulottuvuusalueella
    final ikSolution = widget.kinematics.inverseKinematics(target);
    if (ikSolution == null) {
      HapticFeedback.heavyImpact();
      ScaffoldMessenger.of(context).hideCurrentSnackBar();
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text("⚠️ Piste on käsivarren ulottuvuusalueen tai servorajojen ulkopuolella!"),
          backgroundColor: Colors.orangeAccent,
          duration: Duration(milliseconds: 1500),
          behavior: SnackBarBehavior.floating,
        ),
      );
      return;
    }

    widget.onMoveRequested(target, _isLinearMode);
  }

  @override
  Widget build(BuildContext context) {
    final pose = widget.currentPose;

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: const Color(0xFF1B202D),
        borderRadius: BorderRadius.circular(18),
        border: Border.all(color: Colors.cyanAccent.withValues(alpha: 0.25)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: 0.3),
            blurRadius: 8,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          // Ylärivi: Nykyiset koordinaatit laatikossa
          Row(
            children: [
              Expanded(child: _buildCoordTile("X (Eteen)", "${pose.x.round()} mm", Colors.cyanAccent)),
              const SizedBox(width: 8),
              Expanded(child: _buildCoordTile("Y (Sivu)", "${pose.y.round()} mm", Colors.lightBlueAccent)),
              const SizedBox(width: 8),
              Expanded(child: _buildCoordTile("Z (Korkeus)", "${pose.z.round()} mm", Colors.greenAccent)),
              const SizedBox(width: 8),
              Expanded(child: _buildCoordTile("Pitch", "${pose.pitch.round()}°", Colors.orangeAccent)),
            ],
          ),

          const SizedBox(height: 14),

          // Liiketila (PTP / Linear)
          Row(
            children: [
              const Text(
                "Liiketila: ",
                style: TextStyle(color: Colors.white54, fontSize: 12, fontWeight: FontWeight.w600),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Container(
                  decoration: BoxDecoration(
                    color: const Color(0xFF10141E),
                    borderRadius: BorderRadius.circular(10),
                    border: Border.all(color: Colors.white12),
                  ),
                  child: Row(
                    children: [
                      Expanded(
                        child: _buildModeButton("PTP (Nivelet)", !_isLinearMode, () {
                          setState(() => _isLinearMode = false);
                          HapticFeedback.lightImpact();
                        }, tooltip: "Point-to-Point: Synkronoitu nopea nivel-liike"),
                      ),
                      Expanded(
                        child: _buildModeButton("LINEAR (Suora)", _isLinearMode, () {
                          setState(() => _isLinearMode = true);
                          HapticFeedback.lightImpact();
                        }, tooltip: "Linear: Koura liikkuu täysin suoraa 3D-viivaa"),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),

          const SizedBox(height: 10),

          // Askelkoko: 1mm, 5mm, 10mm, 25mm (Expanded, jotta skaalautuu aina eikä ylivuoda)
          Row(
            children: [
              const Text(
                "Askel: ",
                style: TextStyle(color: Colors.white54, fontSize: 12, fontWeight: FontWeight.w600),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Row(
                  children: [1.0, 5.0, 10.0, 25.0].map((step) {
                    final isSelected = _stepMm == step;
                    return Expanded(
                      child: Container(
                        margin: const EdgeInsets.symmetric(horizontal: 2),
                        child: InkWell(
                          onTap: () {
                            setState(() => _stepMm = step);
                            HapticFeedback.selectionClick();
                          },
                          borderRadius: BorderRadius.circular(8),
                          child: Container(
                            padding: const EdgeInsets.symmetric(vertical: 8),
                            decoration: BoxDecoration(
                              color: isSelected ? Colors.cyanAccent : const Color(0xFF10141E),
                              borderRadius: BorderRadius.circular(8),
                              border: Border.all(
                                color: isSelected ? Colors.cyanAccent : Colors.white12,
                              ),
                            ),
                            alignment: Alignment.center,
                            child: Text(
                              "${step.round()}mm",
                              style: TextStyle(
                                color: isSelected ? Colors.black : Colors.white70,
                                fontWeight: FontWeight.bold,
                                fontSize: 11,
                              ),
                            ),
                          ),
                        ),
                      ),
                    );
                  }).toList(),
                ),
              ),
            ],
          ),

          const SizedBox(height: 16),
          const Divider(color: Colors.white12, height: 1),
          const SizedBox(height: 14),

          // Ohjausristikko (X/Y vaakatasossa, Z pystytasossa, Pitch kallistus)
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              // 1. X/Y -suuntapainikkeet (Eteen, Taakse, Vasen, Oikea)
              Column(
                children: [
                  const Text(
                    "VAAKATASO (X / Y)",
                    style: TextStyle(color: Colors.white54, fontSize: 11, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 8),
                  _buildJogButton(
                    icon: Icons.keyboard_arrow_up,
                    label: "+X",
                    onPressed: () => _jog(dx: _stepMm),
                  ),
                  Row(
                    children: [
                      _buildJogButton(
                        icon: Icons.keyboard_arrow_left,
                        label: "+Y (Vas)",
                        onPressed: () => _jog(dy: _stepMm),
                      ),
                      const SizedBox(width: 8),
                      Container(
                        width: 48,
                        height: 48,
                        decoration: BoxDecoration(
                          color: const Color(0xFF10141E),
                          borderRadius: BorderRadius.circular(10),
                          border: Border.all(color: Colors.cyanAccent.withValues(alpha: 0.3)),
                        ),
                        child: const Center(
                          child: Icon(Icons.open_with, color: Colors.cyanAccent, size: 20),
                        ),
                      ),
                      const SizedBox(width: 8),
                      _buildJogButton(
                        icon: Icons.keyboard_arrow_right,
                        label: "-Y (Oik)",
                        onPressed: () => _jog(dy: -_stepMm),
                      ),
                    ],
                  ),
                  _buildJogButton(
                    icon: Icons.keyboard_arrow_down,
                    label: "-X",
                    onPressed: () => _jog(dx: -_stepMm),
                  ),
                ],
              ),

              // 2. Z-korkeuspainikkeet (Ylös / Alas)
              Column(
                children: [
                  const Text(
                    "KORKEUS (Z)",
                    style: TextStyle(color: Colors.white54, fontSize: 11, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 8),
                  _buildJogButton(
                    icon: Icons.arrow_upward,
                    label: "+Z",
                    color: Colors.greenAccent,
                    onPressed: () => _jog(dz: _stepMm),
                  ),
                  const SizedBox(height: 8),
                  Container(
                    width: 48,
                    height: 48,
                    decoration: BoxDecoration(
                      color: const Color(0xFF10141E),
                      borderRadius: BorderRadius.circular(10),
                      border: Border.all(color: Colors.greenAccent.withValues(alpha: 0.3)),
                    ),
                    child: Center(
                      child: Text(
                        "${pose.z.round()}",
                        style: const TextStyle(
                          color: Colors.greenAccent,
                          fontWeight: FontWeight.bold,
                          fontSize: 12,
                          fontFamily: 'monospace',
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(height: 8),
                  _buildJogButton(
                    icon: Icons.arrow_downward,
                    label: "-Z",
                    color: Colors.greenAccent,
                    onPressed: () => _jog(dz: -_stepMm),
                  ),
                ],
              ),

              // 3. Kouran Pitch -kallistus (Ylös / Alas)
              Column(
                children: [
                  const Text(
                    "KALLISTUS",
                    style: TextStyle(color: Colors.white54, fontSize: 11, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 8),
                  _buildJogButton(
                    icon: Icons.rotate_90_degrees_ccw,
                    label: "+10°",
                    color: Colors.orangeAccent,
                    onPressed: () => _jog(dpitch: 10.0),
                  ),
                  const SizedBox(height: 8),
                  Container(
                    width: 48,
                    height: 48,
                    decoration: BoxDecoration(
                      color: const Color(0xFF10141E),
                      borderRadius: BorderRadius.circular(10),
                      border: Border.all(color: Colors.orangeAccent.withValues(alpha: 0.3)),
                    ),
                    child: Center(
                      child: Text(
                        "${pose.pitch.round()}°",
                        style: const TextStyle(
                          color: Colors.orangeAccent,
                          fontWeight: FontWeight.bold,
                          fontSize: 12,
                          fontFamily: 'monospace',
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(height: 8),
                  _buildJogButton(
                    icon: Icons.rotate_90_degrees_cw,
                    label: "-10°",
                    color: Colors.orangeAccent,
                    onPressed: () => _jog(dpitch: -10.0),
                  ),
                ],
              ),
            ],
          ),

          const SizedBox(height: 12),

          // Pikanapit: Pöytätaso (Z=0), Nosto (Z=250), Kurotus (X=350)
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildPresetPoseButton(
                label: "Pöydälle (Z=40)",
                onTap: () {
                  final target = pose.copyWith(z: 40.0, pitch: -60.0);
                  widget.onMoveRequested(target, _isLinearMode);
                },
              ),
              _buildPresetPoseButton(
                label: "Nosta (Z=250)",
                onTap: () {
                  final target = pose.copyWith(z: 250.0);
                  widget.onMoveRequested(target, _isLinearMode);
                },
              ),
              _buildPresetPoseButton(
                label: "Eteen (X=350)",
                onTap: () {
                  final target = pose.copyWith(x: 350.0, y: 0.0, z: 150.0, pitch: -20.0);
                  widget.onMoveRequested(target, _isLinearMode);
                },
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildCoordTile(String label, String value, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 4),
      decoration: BoxDecoration(
        color: const Color(0xFF10141E),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: color.withValues(alpha: 0.3)),
      ),
      child: Column(
        children: [
          Text(label, style: const TextStyle(color: Colors.white54, fontSize: 10)),
          const SizedBox(height: 4),
          Text(
            value,
            style: TextStyle(
              color: color,
              fontWeight: FontWeight.bold,
              fontSize: 13,
              fontFamily: 'monospace',
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildModeButton(String label, bool isSelected, VoidCallback onTap, {required String tooltip}) {
    return Tooltip(
      message: tooltip,
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(9),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 8),
          alignment: Alignment.center,
          decoration: BoxDecoration(
            color: isSelected ? Colors.cyanAccent : Colors.transparent,
            borderRadius: BorderRadius.circular(9),
          ),
          child: Text(
            label,
            textAlign: TextAlign.center,
            style: TextStyle(
              color: isSelected ? Colors.black : Colors.white60,
              fontWeight: FontWeight.bold,
              fontSize: 11,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildJogButton({
    required IconData icon,
    required String label,
    required VoidCallback onPressed,
    Color color = Colors.cyanAccent,
  }) {
    return ElevatedButton(
      style: ElevatedButton.styleFrom(
        backgroundColor: const Color(0xFF141A28),
        foregroundColor: color,
        elevation: 2,
        padding: const EdgeInsets.all(12),
        minimumSize: const Size(54, 48),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(10),
          side: BorderSide(color: color.withValues(alpha: 0.3)),
        ),
      ),
      onPressed: widget.isMoving ? null : onPressed,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 20),
          Text(label, style: const TextStyle(fontSize: 9, fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }

  Widget _buildPresetPoseButton({required String label, required VoidCallback onTap}) {
    return OutlinedButton(
      style: OutlinedButton.styleFrom(
        foregroundColor: Colors.white70,
        side: const BorderSide(color: Colors.white24),
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
        minimumSize: const Size(0, 32),
      ),
      onPressed: widget.isMoving ? null : onTap,
      child: Text(label, style: const TextStyle(fontSize: 11)),
    );
  }
}

