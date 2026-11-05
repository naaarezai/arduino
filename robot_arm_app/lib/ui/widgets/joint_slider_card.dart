import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../../models/arm_state.dart';

class JointSliderCard extends StatelessWidget {
  final JointInfo joint;
  final ValueChanged<double> onChanged;
  final ValueChanged<double> onChangeEnd;
  final VoidCallback onReset;

  const JointSliderCard({
    super.key,
    required this.joint,
    required this.onChanged,
    required this.onChangeEnd,
    required this.onReset,
  });

  @override
  Widget build(BuildContext context) {
    final isGripper = joint.key == 'gripper';
    final currentAngleInt = joint.angle.round();

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: const Color(0xFF1B202D),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(
          color: isGripper
              ? Colors.amberAccent.withValues(alpha: 0.3)
              : Colors.cyanAccent.withValues(alpha: 0.2),
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: 0.25),
            blurRadius: 6,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Ylärivi: Ikoni, Nimi, Kulmanäyttö
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: isGripper
                      ? Colors.amberAccent.withValues(alpha: 0.15)
                      : Colors.cyanAccent.withValues(alpha: 0.15),
                  borderRadius: BorderRadius.circular(10),
                ),
                child: Icon(
                  joint.icon,
                  color: isGripper ? Colors.amberAccent : Colors.cyanAccent,
                  size: 20,
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      "${joint.id + 1}. ${joint.nameFi}",
                      style: const TextStyle(
                        color: Colors.white,
                        fontWeight: FontWeight.bold,
                        fontSize: 15,
                      ),
                    ),
                    Text(
                      joint.nameEn,
                      style: const TextStyle(
                        color: Colors.white38,
                        fontSize: 11,
                      ),
                    ),
                  ],
                ),
              ),
              // Kulmanäyttö laatikossa
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                decoration: BoxDecoration(
                  color: const Color(0xFF10141E),
                  borderRadius: BorderRadius.circular(10),
                  border: Border.all(
                    color: isGripper ? Colors.amberAccent : Colors.cyanAccent,
                    width: 1.2,
                  ),
                ),
                child: Text(
                  "$currentAngleInt°",
                  style: TextStyle(
                    color: isGripper ? Colors.amberAccent : Colors.cyanAccent,
                    fontWeight: FontWeight.bold,
                    fontSize: 16,
                    fontFamily: 'monospace',
                  ),
                ),
              ),
              const SizedBox(width: 6),
              // Reset nappi oletuskulmaan
              IconButton(
                icon: const Icon(Icons.refresh, size: 20, color: Colors.white54),
                tooltip: "Palauta oletuskulmaan (${joint.defaultAngle.round()}°)",
                onPressed: () {
                  HapticFeedback.lightImpact();
                  onReset();
                },
              ),
            ],
          ),

          const SizedBox(height: 8),

          // Slider
          SliderTheme(
            data: SliderTheme.of(context).copyWith(
              activeTrackColor: isGripper ? Colors.amberAccent : Colors.cyanAccent,
              inactiveTrackColor: Colors.white12,
              thumbColor: isGripper ? Colors.amberAccent : Colors.cyanAccent,
              overlayColor: (isGripper ? Colors.amberAccent : Colors.cyanAccent)
                  .withValues(alpha: 0.2),
              trackHeight: 6.0,
              thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 10),
            ),
            child: Slider(
              value: joint.angle,
              min: joint.minAngle,
              max: joint.maxAngle,
              divisions: (joint.maxAngle - joint.minAngle).round().clamp(1, 180),
              onChanged: (val) {
                if (val.round() != currentAngleInt) {
                  HapticFeedback.selectionClick();
                }
                onChanged(val);
              },
              onChangeEnd: onChangeEnd,
            ),
          ),

          // Pikasäädöt (-5°, -1°, +1°, +5°)
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              _buildStepButton(context, "-5°", -5),
              _buildStepButton(context, "-1°", -1),
              if (isGripper) ...[
                // Kouran pikanapit Auki / Kiinni
                ElevatedButton.icon(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.amberAccent.withValues(alpha: 0.2),
                    foregroundColor: Colors.amberAccent,
                    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                    minimumSize: const Size(0, 32),
                    elevation: 0,
                  ),
                  icon: const Icon(Icons.lock_open, size: 14),
                  label: const Text("Auki", style: TextStyle(fontSize: 12)),
                  onPressed: () {
                    HapticFeedback.mediumImpact();
                    onChanged(joint.minAngle);
                    onChangeEnd(joint.minAngle);
                  },
                ),
                ElevatedButton.icon(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.amberAccent.withValues(alpha: 0.2),
                    foregroundColor: Colors.amberAccent,
                    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                    minimumSize: const Size(0, 32),
                    elevation: 0,
                  ),
                  icon: const Icon(Icons.lock, size: 14),
                  label: const Text("Kiinni", style: TextStyle(fontSize: 12)),
                  onPressed: () {
                    HapticFeedback.mediumImpact();
                    onChanged(joint.maxAngle);
                    onChangeEnd(joint.maxAngle);
                  },
                ),
              ],
              _buildStepButton(context, "+1°", 1),
              _buildStepButton(context, "+5°", 5),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildStepButton(BuildContext context, String text, int delta) {
    return InkWell(
      onTap: () {
        HapticFeedback.selectionClick();
        final newAngle = (joint.angle + delta).clamp(joint.minAngle, joint.maxAngle);
        onChanged(newAngle);
        onChangeEnd(newAngle);
      },
      borderRadius: BorderRadius.circular(8),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
        decoration: BoxDecoration(
          color: const Color(0xFF10141E),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.white12),
        ),
        child: Text(
          text,
          style: const TextStyle(
            color: Colors.white70,
            fontSize: 12,
            fontWeight: FontWeight.w600,
          ),
        ),
      ),
    );
  }
}

