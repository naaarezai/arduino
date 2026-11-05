import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import '../models/arm_state.dart';

class StorageService {
  static const String _keyPresets = 'arm_saved_presets';
  static const String _keySequences = 'arm_saved_sequences';
  static const String _keyInversions = 'arm_servo_inversions';

  /// Lataa servojen suuntakäännöt (true = käännetty, 180 - kulma)
  /// Oletuksena Servo 2 (Olkavarsi) on käännetty käyttäjän ilmoituksen perusteella.
  static Future<List<bool>> loadInversions() async {
    final prefs = await SharedPreferences.getInstance();
    final list = prefs.getStringList(_keyInversions);
    if (list == null || list.length < 6) {
      return [false, true, false, false, false, false];
    }
    return list.map((e) => e == 'true').toList();
  }

  static Future<void> saveInversions(List<bool> inversions) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setStringList(
      _keyInversions,
      inversions.map((e) => e.toString()).toList(),
    );
  }

  static Future<List<ArmPreset>> loadPresets() async {
    final prefs = await SharedPreferences.getInstance();
    final jsonStr = prefs.getString(_keyPresets);
    if (jsonStr == null || jsonStr.isEmpty) {
      return _defaultPresets();
    }
    try {
      final List decoded = jsonDecode(jsonStr);
      final list = decoded.map((e) => ArmPreset.fromJson(e)).toList();
      // Varmistetaan että mekaaninen rajoitus 40-128° pätee myös aiemmin tallennettuihin
      for (var p in list) {
        if (p.angles.length >= 6) {
          p.angles[5] = p.angles[5].clamp(40, 128);
        }
      }
      return list;
    } catch (_) {
      return _defaultPresets();
    }
  }

  static Future<void> savePresets(List<ArmPreset> presets) async {
    final prefs = await SharedPreferences.getInstance();
    final jsonList = presets.map((e) => e.toJson()).toList();
    await prefs.setString(_keyPresets, jsonEncode(jsonList));
  }

  static Future<List<SequenceStep>> loadSequence() async {
    final prefs = await SharedPreferences.getInstance();
    final jsonStr = prefs.getString(_keySequences);
    if (jsonStr == null || jsonStr.isEmpty) {
      return _defaultSequence();
    }
    try {
      final List decoded = jsonDecode(jsonStr);
      final list = decoded.map((e) => SequenceStep.fromJson(e)).toList();
      // Varmistetaan että mekaaninen rajoitus 40-128° pätee myös aiemmin tallennettuihin
      for (var s in list) {
        if (s.angles.length >= 6) {
          s.angles[5] = s.angles[5].clamp(40, 128);
        }
      }
      return list;
    } catch (_) {
      return _defaultSequence();
    }
  }

  static Future<void> saveSequence(List<SequenceStep> steps) async {
    final prefs = await SharedPreferences.getInstance();
    final jsonList = steps.map((e) => e.toJson()).toList();
    await prefs.setString(_keySequences, jsonEncode(jsonList));
  }

  static List<ArmPreset> _defaultPresets() {
    return [
      ArmPreset(
        id: 'home',
        name: 'Lepoasento (Home)',
        angles: [90, 90, 90, 90, 90, 40],
      ),
      ArmPreset(
        id: 'ready',
        name: 'Valmiustila (Ready)',
        angles: [90, 110, 70, 90, 90, 50],
      ),
      ArmPreset(
        id: 'pick_low',
        name: 'Poiminta lattialta',
        angles: [90, 140, 40, 110, 90, 40],
      ),
      ArmPreset(
        id: 'lift_high',
        name: 'Nosto ylös',
        angles: [90, 70, 110, 90, 90, 128],
      ),
      ArmPreset(
        id: 'reach_forward',
        name: 'Kurotus eteen',
        angles: [90, 130, 90, 90, 90, 84],
      ),
      ArmPreset(
        id: 'box_left',
        name: 'Kääntö vasemmalle',
        angles: [30, 90, 90, 90, 90, 40],
      ),
      ArmPreset(
        id: 'box_right',
        name: 'Kääntö oikealle',
        angles: [150, 90, 90, 90, 90, 40],
      ),
    ];
  }

  static List<SequenceStep> _defaultSequence() {
    return [
      SequenceStep(
        id: 'step_1',
        name: '1. Aloitus: Lepoasento',
        angles: [90, 90, 90, 90, 90, 40],
        delayMs: 1500,
      ),
      SequenceStep(
        id: 'step_2',
        name: '2. Laskeudu alas (Koura auki)',
        angles: [90, 135, 45, 100, 90, 40],
        delayMs: 1500,
      ),
      SequenceStep(
        id: 'step_3',
        name: '3. Tartu kappaleeseen (Sulje koura)',
        angles: [90, 135, 45, 100, 90, 128],
        delayMs: 1200,
      ),
      SequenceStep(
        id: 'step_4',
        name: '4. Nosta ilmaan',
        angles: [90, 80, 100, 90, 90, 128],
        delayMs: 1500,
      ),
      SequenceStep(
        id: 'step_5',
        name: '5. Käännä oikealle',
        angles: [150, 80, 100, 90, 90, 128],
        delayMs: 1500,
      ),
      SequenceStep(
        id: 'step_6',
        name: '6. Laskeudu ja vapauta kappale',
        angles: [150, 135, 45, 100, 90, 40],
        delayMs: 1500,
      ),
      SequenceStep(
        id: 'step_7',
        name: '7. Palaa kotiin',
        angles: [90, 90, 90, 90, 90, 40],
        delayMs: 1500,
      ),
    ];
  }
}
