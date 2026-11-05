import 'dart:async';
import 'dart:io';
import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

enum BleConnectionStatus {
  disconnected,
  scanning,
  connecting,
  connected,
  disconnecting,
}

class BleService extends ChangeNotifier {
  static final BleService _instance = BleService._internal();
  factory BleService() => _instance;
  BleService._internal();

  // UUID:t jotka vastaavat Arduino Uno R4 WiFi koodia
  static const String serviceUuidStr = "19B10000-E8F2-537E-4F6C-D104768A1214";
  static const String charUuidStr = "19B10001-E8F2-537E-4F6C-D104768A1214";

  final Guid targetServiceUuid = Guid(serviceUuidStr);
  final Guid targetCharUuid = Guid(charUuidStr);

  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _armCharacteristic;

  BleConnectionStatus _status = BleConnectionStatus.disconnected;
  String? _statusMessage;
  List<ScanResult> _scanResults = [];
  bool _isScanning = false;

  StreamSubscription<List<ScanResult>>? _scanSub;
  StreamSubscription<bool>? _isScanningSub;
  StreamSubscription<BluetoothConnectionState>? _connSub;
  StreamSubscription<BluetoothAdapterState>? _adapterSub;

  // Throttling datan lähetykselle (max 20 pakettia/s = 50ms)
  Timer? _throttleTimer;
  Uint8List? _pendingData;
  DateTime _lastSentTime = DateTime.fromMillisecondsSinceEpoch(0);
  final int _throttleMs = 45;

  BleConnectionStatus get status => _status;
  bool get isConnected => _status == BleConnectionStatus.connected && _armCharacteristic != null;
  bool get isScanning => _isScanning;
  String? get statusMessage => _statusMessage;
  List<ScanResult> get scanResults => _scanResults;
  BluetoothDevice? get connectedDevice => _connectedDevice;

  void init() {
    if (kIsWeb || Platform.environment.containsKey('FLUTTER_TEST')) return;
    try {
      _adapterSub?.cancel();
      _adapterSub = FlutterBluePlus.adapterState.listen((state) {
        if (state != BluetoothAdapterState.on) {
          if (_status == BleConnectionStatus.connected) {
            _status = BleConnectionStatus.disconnected;
            _statusMessage = "Bluetooth kytketty pois päältä";
            notifyListeners();
          }
        }
      });
    } catch (_) {}
  }

  /// Pyydä tarvittavat Bluetooth- ja sijaintiluvat (Android)
  Future<bool> requestPermissions() async {
    if (kIsWeb) return true;

    if (Platform.isAndroid) {
      Map<Permission, PermissionStatus> statuses = await [
        Permission.bluetoothScan,
        Permission.bluetoothConnect,
        Permission.locationWhenInUse,
      ].request();

      bool granted = statuses.values.every((status) =>
          status.isGranted || status.isLimited);
      return granted;
    }
    return true;
  }

  /// Aloita BLE-laitteiden haku
  Future<void> startScan({Duration timeout = const Duration(seconds: 10)}) async {
    await requestPermissions();

    if (_isScanning) {
      await FlutterBluePlus.stopScan();
    }

    _scanResults.clear();
    _status = BleConnectionStatus.scanning;
    _statusMessage = "Etsitään robottikäsivartta...";
    notifyListeners();

    _scanSub?.cancel();
    _scanSub = FlutterBluePlus.scanResults.listen((results) {
      _scanResults = results;
      notifyListeners();
    });

    _isScanningSub?.cancel();
    _isScanningSub = FlutterBluePlus.isScanning.listen((scanning) {
      _isScanning = scanning;
      if (!scanning && _status == BleConnectionStatus.scanning) {
        _status = BleConnectionStatus.disconnected;
        _statusMessage = "Haku päättyi";
      }
      notifyListeners();
    });

    try {
      await FlutterBluePlus.startScan(
        timeout: timeout,
        androidUsesFineLocation: true,
      );
    } catch (e) {
      _status = BleConnectionStatus.disconnected;
      _statusMessage = "Haku epäonnistui: $e";
      notifyListeners();
    }
  }

  /// Pysäytä haku
  Future<void> stopScan() async {
    try {
      await FlutterBluePlus.stopScan();
    } catch (_) {}
    _isScanning = false;
    notifyListeners();
  }

  /// Yhdistä valittuun BLE-laitteeseen
  Future<bool> connect(BluetoothDevice device) async {
    await stopScan();

    _status = BleConnectionStatus.connecting;
    _statusMessage = "Yhdistetään laitteeseen ${device.platformName.isNotEmpty ? device.platformName : device.remoteId.str}...";
    notifyListeners();

    try {
      // Kuuntele yhteystilan muutoksia
      _connSub?.cancel();
      _connSub = device.connectionState.listen((state) {
        if (state == BluetoothConnectionState.connected) {
          _status = BleConnectionStatus.connected;
          _statusMessage = "Yhdistetty: ${device.platformName.isNotEmpty ? device.platformName : 'Arduino Uno R4'}";
          notifyListeners();
        } else if (state == BluetoothConnectionState.disconnected) {
          _status = BleConnectionStatus.disconnected;
          _statusMessage = "Yhteys katkesi";
          _armCharacteristic = null;
          _connectedDevice = null;
          notifyListeners();
        }
      });

      await device.connect(
        license: License.nonprofit,
        timeout: const Duration(seconds: 15),
        autoConnect: false,
      );
      _connectedDevice = device;

      // Etsi palvelut ja karakteristiikat
      _statusMessage = "Etsitään ohjauspalvelua...";
      notifyListeners();

      List<BluetoothService> services = await device.discoverServices();
      BluetoothCharacteristic? foundChar;

      for (var service in services) {
        if (service.uuid == targetServiceUuid ||
            service.uuid.str128.toLowerCase() == targetServiceUuid.str128.toLowerCase()) {
          for (var c in service.characteristics) {
            if (c.uuid == targetCharUuid ||
                c.uuid.str128.toLowerCase() == targetCharUuid.str128.toLowerCase()) {
              foundChar = c;
              break;
            }
          }
        }
      }

      // Jos täsmällistä UUID:tä ei löytynyt (esim. eri laite), etsitään ensimmäinen kirjoitettava
      if (foundChar == null) {
        for (var service in services) {
          for (var c in service.characteristics) {
            if (c.properties.write || c.properties.writeWithoutResponse) {
              foundChar = c;
              break;
            }
          }
          if (foundChar != null) break;
        }
      }

      if (foundChar != null) {
        _armCharacteristic = foundChar;
        _status = BleConnectionStatus.connected;
        _statusMessage = "Robottikäsivarsi valmis ohjattavaksi!";
        notifyListeners();
        return true;
      } else {
        _status = BleConnectionStatus.disconnected;
        _statusMessage = "Ohjauskarakteristiikkaa ei löytynyt laitteesta";
        await device.disconnect();
        notifyListeners();
        return false;
      }
    } catch (e) {
      _status = BleConnectionStatus.disconnected;
      _statusMessage = "Yhteysvirhe: $e";
      _connectedDevice = null;
      notifyListeners();
      return false;
    }
  }

  /// Katkaise yhteys
  Future<void> disconnect() async {
    _status = BleConnectionStatus.disconnecting;
    _statusMessage = "Katkaistaan yhteys...";
    notifyListeners();

    try {
      await _connectedDevice?.disconnect();
    } catch (_) {}

    _connSub?.cancel();
    _connectedDevice = null;
    _armCharacteristic = null;
    _status = BleConnectionStatus.disconnected;
    _statusMessage = "Yhteys katkaistu";
    notifyListeners();
  }

  /// Lähetä 6 kulmaa Arduinolle reaaliaikaisesti (Throttled)
  void sendAnglesThrottled(List<int> angles) {
    if (angles.length != 6) return;
    _pendingData = Uint8List.fromList(angles.map((a) => a.clamp(0, 180)).toList());

    final now = DateTime.now();
    final elapsed = now.difference(_lastSentTime).inMilliseconds;

    if (elapsed >= _throttleMs) {
      _executeSend();
    } else {
      _throttleTimer?.cancel();
      _throttleTimer = Timer(Duration(milliseconds: _throttleMs - elapsed), () {
        _executeSend();
      });
    }
  }

  /// Pakota välitön lähetys (esim. napin painallus tai esiasento)
  Future<bool> sendAnglesImmediate(List<int> angles) async {
    if (angles.length != 6) return false;
    _pendingData = null;
    _throttleTimer?.cancel();
    final bytes = Uint8List.fromList(angles.map((a) => a.clamp(0, 180)).toList());
    return await _writeBytes(bytes);
  }

  void _executeSend() {
    if (_pendingData == null) return;
    final data = _pendingData!;
    _pendingData = null;
    _lastSentTime = DateTime.now();
    _writeBytes(data);
  }

  Future<bool> _writeBytes(Uint8List bytes) async {
    if (_armCharacteristic == null) return false;
    try {
      await _armCharacteristic!.write(
        bytes,
        withoutResponse: _armCharacteristic!.properties.writeWithoutResponse,
      );
      return true;
    } catch (e) {
      debugPrint("Virhe BLE-datan lähetyksessä: $e");
      return false;
    }
  }

  @override
  void dispose() {
    _adapterSub?.cancel();
    _scanSub?.cancel();
    _isScanningSub?.cancel();
    _connSub?.cancel();
    _throttleTimer?.cancel();
    super.dispose();
  }
}
