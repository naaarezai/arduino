import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../../kinematics/arm_kinematics.dart';
import '../../models/arm_state.dart';
import '../../services/ble_service.dart';
import '../../services/storage_service.dart';
import '../widgets/arm_visualizer.dart';
import '../widgets/joint_slider_card.dart';
import '../widgets/xyz_controller_card.dart';
import 'ble_scan_dialog.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen>
    with SingleTickerProviderStateMixin {
  final BleService _bleService = BleService();
  final ArmKinematics _kinematics = ArmKinematics();
  late TabController _tabController;

  // 6 Servonivelet (0..180 astetta)
  late List<JointInfo> _joints;

  // Esiasetetut asennot
  List<ArmPreset> _presets = [];

  // Sekvensseri
  List<SequenceStep> _sequence = [];
  bool _isSequenceRunning = false;
  int _activeSequenceIndex = -1;
  bool _sequenceLoop = false;
  Timer? _sequenceTimer;

  // Trajektoriliikkeen tila (Linear streaming)
  bool _isMovingTrajectory = false;
  Timer? _trajectoryTimer;

  // Servojen pyörimissuuntien kääntö (true = käännetty 180 - kulma)
  // Oletuksena Servo 2 (Olkavarsi) on käännetty käyttäjän asennustavan mukaan
  List<bool> _servoInversions = [false, true, false, false, false, false];

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 4, vsync: this);
    _bleService.init();

    _joints = [
      JointInfo(
        id: 0,
        key: 'base',
        nameFi: 'Jalusta (Kierto)',
        nameEn: 'Base (Rotation) - Pin D3',
        icon: Icons.sync,
        angle: 90.0,
        defaultAngle: 90.0,
      ),
      JointInfo(
        id: 1,
        key: 'shoulder',
        nameFi: 'Olkavarsi',
        nameEn: 'Shoulder (Pitch) - Pin D5',
        icon: Icons.fitness_center,
        angle: 90.0,
        defaultAngle: 90.0,
      ),
      JointInfo(
        id: 2,
        key: 'elbow',
        nameFi: 'Kyynärvarsi',
        nameEn: 'Elbow (Pitch) - Pin D6',
        icon: Icons.timeline,
        angle: 90.0,
        defaultAngle: 90.0,
      ),
      JointInfo(
        id: 3,
        key: 'wrist_pitch',
        nameFi: 'Ranne (Pysty)',
        nameEn: 'Wrist (Pitch) - Pin D9',
        icon: Icons.pan_tool_alt,
        angle: 90.0,
        defaultAngle: 90.0,
      ),
      JointInfo(
        id: 4,
        key: 'wrist_roll',
        nameFi: 'Ranne (Kierto)',
        nameEn: 'Wrist (Roll) - Pin D10',
        icon: Icons.rotate_right,
        angle: 90.0,
        defaultAngle: 90.0,
      ),
      JointInfo(
        id: 5,
        key: 'gripper',
        nameFi: 'Koura (Tarttuja)',
        nameEn: 'Gripper (Claw) - Pin D11 [40°-128°]',
        icon: Icons.back_hand,
        angle: 40.0,
        minAngle: 40.0,
        maxAngle: 128.0,
        defaultAngle: 40.0,
      ),
    ];

    _loadData();
  }

  Future<void> _loadData() async {
    final p = await StorageService.loadPresets();
    final s = await StorageService.loadSequence();
    final inv = await StorageService.loadInversions();
    if (mounted) {
      setState(() {
        _presets = p;
        _sequence = s;
        _servoInversions = inv;
      });
    }
  }

  List<int> get _currentAnglesInt =>
      _joints.map((j) => j.angle.round()).toList();
  List<double> get _currentAnglesDouble => _joints.map((j) => j.angle).toList();

  CartesianPose get _currentCartesianPose =>
      _kinematics.forwardKinematics(_currentAnglesDouble);

  /// Muuntaa loogiset kulmat (0..180°) fyysisen servon kulmiksi suuntakäännön mukaan
  List<int> toPhysicalAngles(List<int> logical) {
    final res = <int>[];
    for (int i = 0; i < 6 && i < logical.length; i++) {
      final a = logical[i];
      if (_servoInversions[i]) {
        if (i == 5) {
          // Koura 40..128
          res.add((40 + 128 - a).clamp(40, 128));
        } else {
          res.add((180 - a).clamp(0, 180));
        }
      } else {
        res.add(a);
      }
    }
    return res;
  }

  List<int> get _currentPhysicalAnglesInt => toPhysicalAngles(_currentAnglesInt);

  void _onXyzMoveRequested(CartesianPose target, bool isLinear) {
    if (_isMovingTrajectory) return;

    if (!isLinear) {
      // PTP: Suora IK-kulmalaskenta ja nopea synkronoitu siirto
      final ikAngles = _kinematics.inverseKinematics(target);
      if (ikAngles == null) return;

      setState(() {
        for (int i = 0; i < 6; i++) {
          _joints[i].angle = ikAngles[i];
        }
      });
      _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);
    } else {
      // LINEAR: Suora 3D-viiva avaruudessa väliaskeleilla
      final startPose = _currentCartesianPose;
      final trajectory = _kinematics.generateLinearPath(
        start: startPose,
        target: target,
        stepSizeMm: 6.0,
      );

      if (trajectory.isEmpty) return;

      setState(() => _isMovingTrajectory = true);
      int stepIdx = 0;

      _trajectoryTimer?.cancel();
      _trajectoryTimer = Timer.periodic(const Duration(milliseconds: 35), (timer) {
        if (!mounted || stepIdx >= trajectory.length) {
          timer.cancel();
          setState(() => _isMovingTrajectory = false);
          return;
        }

        final angles = trajectory[stepIdx];
        setState(() {
          for (int i = 0; i < 6; i++) {
            _joints[i].angle = angles[i].toDouble();
          }
        });
        _bleService.sendAnglesImmediate(toPhysicalAngles(angles));
        stepIdx++;
      });
    }
  }

  void _onJointAngleChanged(int index, double newAngle) {
    setState(() {
      _joints[index].angle = newAngle;
    });
    // Lähetetään reaaliaikaisesti Arduinolle suunnat huomioiden
    _bleService.sendAnglesThrottled(_currentPhysicalAnglesInt);
  }

  void _onJointAngleChangeEnd(int index, double newAngle) {
    setState(() {
      _joints[index].angle = newAngle;
    });
    _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);
  }

  void _resetJoint(int index) {
    setState(() {
      _joints[index].angle = _joints[index].defaultAngle;
    });
    _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);
  }

  void _applyPreset(ArmPreset preset) {
    HapticFeedback.mediumImpact();
    setState(() {
      for (int i = 0; i < 6 && i < preset.angles.length; i++) {
        _joints[i].angle = preset.angles[i].toDouble();
      }
    });
    _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text("Asetettu: ${preset.name}"),
        duration: const Duration(seconds: 1),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  void _emergencyStop() {
    HapticFeedback.heavyImpact();
    _trajectoryTimer?.cancel();
    _isMovingTrajectory = false;
    _stopSequence();
    // Asetetaan lepoasento
    setState(() {
      for (var j in _joints) {
        j.angle = j.defaultAngle;
      }
    });
    _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text(
          "⚠️ HÄTÄSEIS: Robottikäsivarsi pysäytetty ja palautettu lepoasentoon",
        ),
        backgroundColor: Colors.redAccent,
        duration: Duration(seconds: 2),
      ),
    );
  }

  // --- SEKVENSSERIN TOIMINNOT ---
  Future<void> _startSequence() async {
    if (_sequence.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text("Lisää vähintään yksi askel sekvenssiin!"),
        ),
      );
      return;
    }

    setState(() {
      _isSequenceRunning = true;
      _activeSequenceIndex = 0;
    });

    _runNextSequenceStep();
  }

  void _runNextSequenceStep() {
    if (!_isSequenceRunning || !mounted) return;

    if (_activeSequenceIndex >= _sequence.length) {
      if (_sequenceLoop) {
        _activeSequenceIndex = 0;
      } else {
        _stopSequence();
        return;
      }
    }

    final step = _sequence[_activeSequenceIndex];
    setState(() {
      for (int i = 0; i < 6 && i < step.angles.length; i++) {
        _joints[i].angle = step.angles[i].toDouble();
      }
    });
    _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);

    _sequenceTimer = Timer(Duration(milliseconds: step.delayMs), () {
      if (!_isSequenceRunning || !mounted) return;
      setState(() {
        _activeSequenceIndex++;
      });
      _runNextSequenceStep();
    });
  }

  void _stopSequence() {
    _sequenceTimer?.cancel();
    setState(() {
      _isSequenceRunning = false;
      _activeSequenceIndex = -1;
    });
  }

  // Servojen suuntien (Invert) asetusdialogi
  void _showInversionSettingsDialog() {
    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: const Color(0xFF151924),
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (ctx) {
        return StatefulBuilder(
          builder: (context, setSheetState) {
            final names = [
              "1. Jalusta (Base)",
              "2. Olkavarsi (Shoulder - Eteen/Taakse)",
              "3. Kyynärvarsi (Elbow)",
              "4. Ranne pysty (Wrist Pitch)",
              "5. Ranne kierto (Wrist Roll)",
              "6. Koura (Gripper)",
            ];
            final descriptions = [
              "Kääntää jalustan pyörimissuunnan (vasen/oikea)",
              "Kääntää olkanivelen (eteen/taakse - oletuksena käännetty)",
              "Kääntää kyynärnivelen suunnan",
              "Kääntää ranteen pystykulman",
              "Kääntää ranteen pyörimissuunnan",
              "Kääntää kouran avautumissuunnan",
            ];

            return DraggableScrollableSheet(
              initialChildSize: 0.75,
              minChildSize: 0.4,
              maxChildSize: 0.95,
              expand: false,
              builder: (context, scrollController) {
                return ListView(
                  controller: scrollController,
                  padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
                  children: [
                    Center(
                      child: Container(
                        width: 40,
                        height: 4,
                        decoration: BoxDecoration(
                          color: Colors.white24,
                          borderRadius: BorderRadius.circular(2),
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        const Row(
                          children: [
                            Icon(Icons.swap_horiz, color: Colors.cyanAccent),
                            SizedBox(width: 8),
                            Text(
                              "Servojen pyörimissuunnat",
                              style: TextStyle(color: Colors.white, fontSize: 17, fontWeight: FontWeight.bold),
                            ),
                          ],
                        ),
                        IconButton(
                          icon: const Icon(Icons.close, color: Colors.white54),
                          onPressed: () => Navigator.pop(ctx),
                        ),
                      ],
                    ),
                    const SizedBox(height: 6),
                    const Text(
                      "Jos robotti liikkuu johonkin suuntaan väärinpäin servon mekaanisen asennustavan vuoksi, käännä suunta (Invert) tästä:",
                      style: TextStyle(color: Colors.white60, fontSize: 12),
                    ),
                    const Divider(color: Colors.white12, height: 24),
                    ...List.generate(6, (i) {
                      return Container(
                        margin: const EdgeInsets.only(bottom: 8),
                        decoration: BoxDecoration(
                          color: const Color(0xFF1B202D),
                          borderRadius: BorderRadius.circular(12),
                          border: Border.all(
                            color: _servoInversions[i]
                                ? Colors.cyanAccent.withValues(alpha: 0.4)
                                : Colors.white10,
                          ),
                        ),
                        child: SwitchListTile(
                          activeThumbColor: Colors.cyanAccent,
                          title: Text(
                            names[i],
                            style: TextStyle(
                              color: Colors.white,
                              fontWeight: _servoInversions[i] ? FontWeight.bold : FontWeight.normal,
                              fontSize: 14,
                            ),
                          ),
                          subtitle: Text(
                            "${descriptions[i]}\nTila: ${_servoInversions[i] ? "KÄÄNNETTY (Inverted)" : "Normaali"}",
                            style: TextStyle(
                              color: _servoInversions[i] ? Colors.cyanAccent : Colors.white38,
                              fontSize: 11,
                            ),
                          ),
                          value: _servoInversions[i],
                          onChanged: (val) {
                            HapticFeedback.selectionClick();
                            setSheetState(() {
                              _servoInversions[i] = val;
                            });
                            setState(() {
                              _servoInversions[i] = val;
                            });
                            StorageService.saveInversions(_servoInversions);
                            _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);
                          },
                        ),
                      );
                    }),
                  ],
                );
              },
            );
          },
        );
      },
    );
  }

  // Uuden esiasennon tallennusdialogi
  void _showSavePresetDialog() {
    final controller = TextEditingController(
      text: "Asento ${_presets.length + 1}",
    );
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF1E2433),
        title: const Text(
          "Tallenna nykyinen asento",
          style: TextStyle(color: Colors.white),
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              "Tallenna robottikäsivarren nykyiset servokulmat esiasetukseksi:",
              style: TextStyle(color: Colors.white70, fontSize: 13),
            ),
            const SizedBox(height: 12),
            TextField(
              controller: controller,
              style: const TextStyle(color: Colors.white),
              decoration: InputDecoration(
                labelText: "Asennon nimi",
                labelStyle: const TextStyle(color: Colors.cyanAccent),
                filled: true,
                fillColor: const Color(0xFF151822),
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(10),
                ),
              ),
            ),
            const SizedBox(height: 12),
            Text(
              "Kulmat: ${_currentAnglesInt.join('°, ')}°",
              style: const TextStyle(
                color: Colors.cyanAccent,
                fontFamily: 'monospace',
                fontSize: 12,
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            child: const Text(
              "Peruuta",
              style: TextStyle(color: Colors.white54),
            ),
            onPressed: () => Navigator.of(ctx).pop(),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.cyanAccent,
              foregroundColor: Colors.black,
            ),
            child: const Text("Tallenna"),
            onPressed: () {
              final name = controller.text.trim();
              if (name.isNotEmpty) {
                final newPreset = ArmPreset(
                  id: DateTime.now().millisecondsSinceEpoch.toString(),
                  name: name,
                  angles: List.from(_currentAnglesInt),
                );
                setState(() {
                  _presets.add(newPreset);
                });
                StorageService.savePresets(_presets);
                Navigator.of(ctx).pop();
              }
            },
          ),
        ],
      ),
    );
  }

  // Uuden sekvenssiaskeleen lisäysdialogi
  void _addCurrentPoseToSequence() {
    final newStep = SequenceStep(
      id: DateTime.now().millisecondsSinceEpoch.toString(),
      name: "Askel ${_sequence.length + 1}",
      angles: List.from(_currentAnglesInt),
      delayMs: 1500,
    );
    setState(() {
      _sequence.add(newStep);
    });
    StorageService.saveSequence(_sequence);
    HapticFeedback.lightImpact();

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          "Lisätty askeleeksi ${_sequence.length} (${_currentAnglesInt.join(', ')})",
        ),
        duration: const Duration(seconds: 1),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF10131B),
      appBar: AppBar(
        backgroundColor: const Color(0xFF151924),
        elevation: 0,
        title: Row(
          children: [
            Container(
              width: 34,
              height: 34,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                border: Border.all(
                  color: Colors.cyanAccent.withValues(alpha: 0.7),
                  width: 1.5,
                ),
              ),
              child: ClipOval(
                child: Image.asset(
                  'assets/images/logo.jpeg',
                  fit: BoxFit.cover,
                  errorBuilder: (ctx, err, stack) => const Icon(
                    Icons.precision_manufacturing,
                    color: Colors.cyanAccent,
                    size: 20,
                  ),
                ),
              ),
            ),
            const SizedBox(width: 10),
            const Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  "Arduino Robotti",
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  "Uno R4 WiFi • 6 Servoa",
                  style: TextStyle(color: Colors.white54, fontSize: 11),
                ),
              ],
            ),
          ],
        ),
        actions: [
          // Bluetooth-yhteyspilleri
          ListenableBuilder(
            listenable: _bleService,
            builder: (context, _) {
              final isConnected = _bleService.isConnected;
              final isConnecting =
                  _bleService.status == BleConnectionStatus.connecting;

              return InkWell(
                onTap: () => BleScanDialog.show(context, _bleService),
                borderRadius: BorderRadius.circular(20),
                child: Container(
                  margin: const EdgeInsets.only(right: 8),
                  padding: const EdgeInsets.symmetric(
                    horizontal: 10,
                    vertical: 6,
                  ),
                  decoration: BoxDecoration(
                    color: isConnected
                        ? Colors.greenAccent.withValues(alpha: 0.15)
                        : (isConnecting
                              ? Colors.orangeAccent.withValues(alpha: 0.15)
                              : Colors.redAccent.withValues(alpha: 0.15)),
                    borderRadius: BorderRadius.circular(20),
                    border: Border.all(
                      color: isConnected
                          ? Colors.greenAccent
                          : (isConnecting
                                ? Colors.orangeAccent
                                : Colors.redAccent.withValues(alpha: 0.5)),
                      width: 1,
                    ),
                  ),
                  child: Row(
                    children: [
                      Container(
                        width: 8,
                        height: 8,
                        decoration: BoxDecoration(
                          shape: BoxShape.circle,
                          color: isConnected
                              ? Colors.greenAccent
                              : (isConnecting
                                    ? Colors.orangeAccent
                                    : Colors.redAccent),
                        ),
                      ),
                      const SizedBox(width: 6),
                      Text(
                        isConnected
                            ? "Yhdistetty"
                            : (isConnecting ? "Yhdistetään..." : "Ei yhteyttä"),
                        style: TextStyle(
                          color: isConnected
                              ? Colors.greenAccent
                              : (isConnecting
                                    ? Colors.orangeAccent
                                    : Colors.redAccent),
                          fontSize: 11,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      const SizedBox(width: 4),
                      Icon(
                        Icons.bluetooth,
                        size: 14,
                        color: isConnected
                            ? Colors.greenAccent
                            : (isConnecting
                                  ? Colors.orangeAccent
                                  : Colors.redAccent),
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
          // Servojen pyörimissuunnat (Invert) -asetusnappi
          IconButton(
            icon: const Icon(Icons.swap_horiz, color: Colors.cyanAccent),
            tooltip: "Servojen pyörimissuunnat (Invert)",
            onPressed: _showInversionSettingsDialog,
          ),
          // Hätäseis-nappi
          IconButton(
            icon: const Icon(Icons.dangerous, color: Colors.redAccent),
            tooltip: "Hätäseis",
            onPressed: _emergencyStop,
          ),
        ],
        bottom: TabBar(
          controller: _tabController,
          indicatorColor: Colors.cyanAccent,
          labelColor: Colors.cyanAccent,
          unselectedLabelColor: Colors.white54,
          labelStyle: const TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 12,
          ),
          tabs: const [
            Tab(icon: Icon(Icons.control_camera, size: 18), text: "XYZ (IK)"),
            Tab(icon: Icon(Icons.tune, size: 18), text: "Nivelet"),
            Tab(icon: Icon(Icons.bookmark, size: 18), text: "Asennot"),
            Tab(icon: Icon(Icons.playlist_play, size: 18), text: "Sekvenssit"),
          ],
        ),
      ),
      body: TabBarView(
        controller: _tabController,
        children: [
          _buildXyzTab(),
          _buildControlTab(),
          _buildPresetsTab(),
          _buildSequenceTab(),
        ],
      ),
      bottomNavigationBar: _buildBottomStatusBar(),
    );
  }

  // --- VÄLILEHTI 1: KARTEESINEN XYZ-OHJAUS (IK / PTP / LINEAR) ---
  Widget _buildXyzTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.only(bottom: 24),
      child: Column(
        children: [
          ArmVisualizer(
            angles: _currentAnglesDouble,
            cartesianPose: _currentCartesianPose,
            height: 190,
          ),
          XyzControllerCard(
            currentPose: _currentCartesianPose,
            kinematics: _kinematics,
            isMoving: _isMovingTrajectory,
            onMoveRequested: _onXyzMoveRequested,
          ),
        ],
      ),
    );
  }

  // --- VÄLILEHTI 2: NIVELTEN OHJAUS (MANUAALINEN JOGGAUS) ---
  Widget _buildControlTab() {
    return Column(
      children: [
        // Kinematiikan 2D visualisointi
        ArmVisualizer(
          angles: _currentAnglesDouble,
          cartesianPose: _currentCartesianPose,
          height: 190,
        ),

        // 6 Nivelen Sliderit vieritettävässä listassa
        Expanded(
          child: ListView.builder(
            itemCount: _joints.length,
            padding: const EdgeInsets.only(bottom: 24),
            itemBuilder: (context, index) {
              return JointSliderCard(
                joint: _joints[index],
                onChanged: (val) => _onJointAngleChanged(index, val),
                onChangeEnd: (val) => _onJointAngleChangeEnd(index, val),
                onReset: () => _resetJoint(index),
              );
            },
          ),
        ),
      ],
    );
  }

  // --- VÄLILEHTI 2: ASIANASETUKSET (PRESETS) ---
  Widget _buildPresetsTab() {
    return Column(
      children: [
        Padding(
          padding: const EdgeInsets.all(16.0),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              const Text(
                "Tallennetut asennot",
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 16,
                  fontWeight: FontWeight.bold,
                ),
              ),
              ElevatedButton.icon(
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.cyanAccent,
                  foregroundColor: Colors.black,
                  padding: const EdgeInsets.symmetric(
                    horizontal: 14,
                    vertical: 8,
                  ),
                ),
                icon: const Icon(Icons.add, size: 16),
                label: const Text("Tallenna nykyinen"),
                onPressed: _showSavePresetDialog,
              ),
            ],
          ),
        ),
        Expanded(
          child: GridView.builder(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
              crossAxisCount: 2,
              childAspectRatio: 1.5,
              crossAxisSpacing: 12,
              mainAxisSpacing: 12,
            ),
            itemCount: _presets.length,
            itemBuilder: (context, index) {
              final preset = _presets[index];
              return InkWell(
                onTap: () => _applyPreset(preset),
                borderRadius: BorderRadius.circular(14),
                child: Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: const Color(0xFF1B202D),
                    borderRadius: BorderRadius.circular(14),
                    border: Border.all(color: Colors.white12),
                  ),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          Icon(preset.icon, color: Colors.cyanAccent, size: 20),
                          if (index >= 7) // Poistettavissa jos käyttäjän luoma
                            InkWell(
                              onTap: () {
                                setState(() {
                                  _presets.removeAt(index);
                                });
                                StorageService.savePresets(_presets);
                              },
                              child: const Icon(
                                Icons.close,
                                size: 16,
                                color: Colors.white38,
                              ),
                            ),
                        ],
                      ),
                      Text(
                        preset.name,
                        style: const TextStyle(
                          color: Colors.white,
                          fontWeight: FontWeight.bold,
                          fontSize: 13,
                        ),
                        maxLines: 2,
                        overflow: TextOverflow.ellipsis,
                      ),
                      Text(
                        "${preset.angles.join('°, ')}°",
                        style: const TextStyle(
                          color: Colors.white38,
                          fontSize: 10,
                          fontFamily: 'monospace',
                        ),
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      ],
    );
  }

  // --- VÄLILEHTI 3: SEKVENSSERIT (OPETETUT LIIKERADAT) ---
  Widget _buildSequenceTab() {
    return Column(
      children: [
        // Ohjauspaneeli: Toista, Pysäytä, Loop, Lisää askel
        Container(
          margin: const EdgeInsets.all(16),
          padding: const EdgeInsets.all(14),
          decoration: BoxDecoration(
            color: const Color(0xFF1B202D),
            borderRadius: BorderRadius.circular(16),
            border: Border.all(color: Colors.cyanAccent.withValues(alpha: 0.2)),
          ),
          child: Column(
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Row(
                    children: [
                      if (_isSequenceRunning)
                        ElevatedButton.icon(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.redAccent,
                            foregroundColor: Colors.white,
                          ),
                          icon: const Icon(Icons.stop),
                          label: const Text("Pysäytä"),
                          onPressed: _stopSequence,
                        )
                      else
                        ElevatedButton.icon(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.greenAccent,
                            foregroundColor: Colors.black,
                          ),
                          icon: const Icon(Icons.play_arrow),
                          label: const Text("Toista sarja"),
                          onPressed: _startSequence,
                        ),
                      const SizedBox(width: 8),
                      // Loop-kytkin
                      FilterChip(
                        label: const Text("Jatkuva (Loop)"),
                        selected: _sequenceLoop,
                        selectedColor: Colors.cyanAccent.withValues(alpha: 0.2),
                        checkmarkColor: Colors.cyanAccent,
                        labelStyle: TextStyle(
                          color: _sequenceLoop
                              ? Colors.cyanAccent
                              : Colors.white60,
                          fontSize: 12,
                        ),
                        onSelected: (val) {
                          setState(() {
                            _sequenceLoop = val;
                          });
                        },
                      ),
                    ],
                  ),
                  IconButton(
                    icon: const Icon(
                      Icons.add_location_alt,
                      color: Colors.cyanAccent,
                    ),
                    tooltip: "Tallenna nykyinen asento askeleeksi",
                    onPressed: _addCurrentPoseToSequence,
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Text(
                _isSequenceRunning
                    ? "Toistetaan askelta ${_activeSequenceIndex + 1} / ${_sequence.length}..."
                    : "Sekvenssissä on ${_sequence.length} askelta.",
                style: const TextStyle(color: Colors.white70, fontSize: 12),
              ),
            ],
          ),
        ),

        // Askelten lista
        Expanded(
          child: _sequence.isEmpty
              ? const Center(
                  child: Text(
                    "Ei tallennettuja askeleita.\nLiikuta käsivarsi haluttuun asentoon ja paina '+'-nappia!",
                    textAlign: TextAlign.center,
                    style: TextStyle(color: Colors.white38),
                  ),
                )
              : ReorderableListView.builder(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  itemCount: _sequence.length,
                  // ignore: deprecated_member_use
                  onReorder: (oldIdx, newIdx) {
                    setState(() {
                      if (newIdx > oldIdx) newIdx -= 1;
                      final item = _sequence.removeAt(oldIdx);
                      _sequence.insert(newIdx, item);
                    });
                    StorageService.saveSequence(_sequence);
                  },
                  itemBuilder: (context, index) {
                    final step = _sequence[index];
                    final isActive = _activeSequenceIndex == index;

                    return Container(
                      key: ValueKey(step.id),
                      margin: const EdgeInsets.only(bottom: 8),
                      decoration: BoxDecoration(
                        color: isActive
                            ? const Color(0xFF1E3838)
                            : const Color(0xFF181D29),
                        borderRadius: BorderRadius.circular(12),
                        border: Border.all(
                          color: isActive ? Colors.cyanAccent : Colors.white10,
                          width: isActive ? 1.8 : 1.0,
                        ),
                      ),
                      child: ListTile(
                        leading: CircleAvatar(
                          backgroundColor: isActive
                              ? Colors.cyanAccent
                              : Colors.white12,
                          foregroundColor: isActive
                              ? Colors.black
                              : Colors.white,
                          child: Text("${index + 1}"),
                        ),
                        title: Text(
                          step.name,
                          style: TextStyle(
                            color: Colors.white,
                            fontWeight: isActive
                                ? FontWeight.bold
                                : FontWeight.normal,
                          ),
                        ),
                        subtitle: Text(
                          "Kulmat: ${step.angles.join('°, ')}° • Viive: ${(step.delayMs / 1000).toStringAsFixed(1)}s",
                          style: const TextStyle(
                            color: Colors.white38,
                            fontSize: 11,
                          ),
                        ),
                        trailing: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            // Kokeile tätä asentoa heti
                            IconButton(
                              icon: const Icon(
                                Icons.arrow_circle_right_outlined,
                                color: Colors.cyanAccent,
                              ),
                              tooltip: "Aja tähän asentoon",
                              onPressed: () {
                                setState(() {
                                  for (
                                    int i = 0;
                                    i < 6 && i < step.angles.length;
                                    i++
                                  ) {
                                    _joints[i].angle = step.angles[i]
                                        .toDouble();
                                  }
                                });
                                _bleService.sendAnglesImmediate(
                                  _currentPhysicalAnglesInt,
                                );
                              },
                            ),
                            // Poista askel
                            IconButton(
                              icon: const Icon(
                                Icons.delete_outline,
                                color: Colors.white38,
                              ),
                              onPressed: () {
                                setState(() {
                                  _sequence.removeAt(index);
                                });
                                StorageService.saveSequence(_sequence);
                              },
                            ),
                            const Icon(
                              Icons.drag_handle,
                              color: Colors.white24,
                            ),
                          ],
                        ),
                      ),
                    );
                  },
                ),
        ),
      ],
    );
  }

  // Alareunan tilapalkki ja pikatoiminnot
  Widget _buildBottomStatusBar() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: const BoxDecoration(
        color: Color(0xFF141722),
        border: Border(top: BorderSide(color: Colors.white10)),
      ),
      child: SafeArea(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            // Pikakoura auki / kiinni (40° - 128°)
            ElevatedButton.icon(
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.amberAccent.withValues(alpha: 0.2),
                foregroundColor: Colors.amberAccent,
                padding: const EdgeInsets.symmetric(
                  horizontal: 12,
                  vertical: 8,
                ),
              ),
              icon: Icon(
                _joints[5].angle > 84 ? Icons.lock_open : Icons.lock,
                size: 16,
              ),
              label: Text(
                _joints[5].angle > 84
                    ? "Avaa koura (40°)"
                    : "Sulje koura (128°)",
              ),
              onPressed: () {
                final newAngle = _joints[5].angle > 84 ? 40.0 : 128.0;
                _onJointAngleChanged(5, newAngle);
                _onJointAngleChangeEnd(5, newAngle);
              },
            ),

            // Pikapainike Lepoasento (Home)
            OutlinedButton.icon(
              style: OutlinedButton.styleFrom(
                foregroundColor: Colors.cyanAccent,
                side: const BorderSide(color: Colors.cyanAccent),
                padding: const EdgeInsets.symmetric(
                  horizontal: 12,
                  vertical: 8,
                ),
              ),
              icon: const Icon(Icons.home, size: 16),
              label: const Text("Kotiin (Home)"),
              onPressed: () {
                setState(() {
                  for (var j in _joints) {
                    j.angle = j.defaultAngle;
                  }
                });
                _bleService.sendAnglesImmediate(_currentPhysicalAnglesInt);
              },
            ),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    _trajectoryTimer?.cancel();
    _sequenceTimer?.cancel();
    _tabController.dispose();
    super.dispose();
  }
}
