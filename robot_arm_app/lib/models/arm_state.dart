import 'package:flutter/material.dart';

/// Yhden servonivelen tiedot
class JointInfo {
  final int id; // 0..5
  final String key;
  final String nameFi;
  final String nameEn;
  final IconData icon;
  double angle; // 0..180
  final double minAngle;
  final double maxAngle;
  final double defaultAngle;

  JointInfo({
    required this.id,
    required this.key,
    required this.nameFi,
    required this.nameEn,
    required this.icon,
    required this.angle,
    this.minAngle = 0.0,
    this.maxAngle = 180.0,
    required this.defaultAngle,
  });

  JointInfo copyWith({double? angle}) {
    return JointInfo(
      id: id,
      key: key,
      nameFi: nameFi,
      nameEn: nameEn,
      icon: icon,
      angle: angle ?? this.angle,
      minAngle: minAngle,
      maxAngle: maxAngle,
      defaultAngle: defaultAngle,
    );
  }
}

/// Esiasetettu tai tallennettu robottikäden asento
class ArmPreset {
  final String id;
  final String name;
  final List<int> angles; // 6 kulmaa [0..180]
  final IconData icon;

  ArmPreset({
    required this.id,
    required this.name,
    required this.angles,
    this.icon = Icons.bookmark_border,
  });

  Map<String, dynamic> toJson() => {
        'id': id,
        'name': name,
        'angles': angles,
      };

  factory ArmPreset.fromJson(Map<String, dynamic> json) {
    return ArmPreset(
      id: json['id'] as String,
      name: json['name'] as String,
      angles: List<int>.from(json['angles'] as List),
    );
  }
}

/// Sekvenssin yksi askel (waypoint)
class SequenceStep {
  final String id;
  final String name;
  final List<int> angles;
  int delayMs; // Viive millisekunteina ennen seuraavaa askelta

  SequenceStep({
    required this.id,
    required this.name,
    required this.angles,
    this.delayMs = 1500,
  });

  Map<String, dynamic> toJson() => {
        'id': id,
        'name': name,
        'angles': angles,
        'delayMs': delayMs,
      };

  factory SequenceStep.fromJson(Map<String, dynamic> json) {
    return SequenceStep(
      id: json['id'] as String,
      name: json['name'] as String,
      angles: List<int>.from(json['angles'] as List),
      delayMs: json['delayMs'] as int? ?? 1500,
    );
  }
}
