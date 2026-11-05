import 'package:flutter/material.dart';
import '../../services/ble_service.dart';

class BleScanDialog extends StatefulWidget {
  final BleService bleService;

  const BleScanDialog({super.key, required this.bleService});

  static Future<void> show(BuildContext context, BleService bleService) {
    return showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: const Color(0xFF151822),
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(24)),
      ),
      builder: (ctx) => BleScanDialog(bleService: bleService),
    );
  }

  @override
  State<BleScanDialog> createState() => _BleScanDialogState();
}

class _BleScanDialogState extends State<BleScanDialog> {
  @override
  void initState() {
    super.initState();
    // Käynnistetään haku heti kun dialogi avataan
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!widget.bleService.isScanning) {
        widget.bleService.startScan();
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
      listenable: widget.bleService,
      builder: (context, _) {
        final ble = widget.bleService;
        final results = ble.scanResults;

        return DraggableScrollableSheet(
          initialChildSize: 0.75,
          minChildSize: 0.5,
          maxChildSize: 0.95,
          expand: false,
          builder: (context, scrollController) {
            return Column(
              children: [
                // Yläpalkki ja kahva
                Container(
                  margin: const EdgeInsets.symmetric(vertical: 12),
                  width: 40,
                  height: 4,
                  decoration: BoxDecoration(
                    color: Colors.white24,
                    borderRadius: BorderRadius.circular(2),
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 20),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      Row(
                        children: [
                          const Icon(Icons.bluetooth, color: Colors.cyanAccent),
                          const SizedBox(width: 8),
                          const Text(
                            "Bluetooth-laitteet",
                            style: TextStyle(
                              color: Colors.white,
                              fontSize: 18,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                        ],
                      ),
                      // Haku / Päivitä nappi
                      if (ble.isScanning)
                        const SizedBox(
                          width: 20,
                          height: 20,
                          child: CircularProgressIndicator(
                            strokeWidth: 2,
                            color: Colors.cyanAccent,
                          ),
                        )
                      else
                        IconButton(
                          icon: const Icon(Icons.refresh, color: Colors.cyanAccent),
                          tooltip: "Päivitä haku",
                          onPressed: () => ble.startScan(),
                        ),
                    ],
                  ),
                ),

                // Tilaviesti
                if (ble.statusMessage != null)
                  Container(
                    margin: const EdgeInsets.fromLTRB(20, 8, 20, 8),
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                    decoration: BoxDecoration(
                      color: const Color(0xFF1E2433),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: Row(
                      children: [
                        const Icon(Icons.info_outline, size: 16, color: Colors.white54),
                        const SizedBox(width: 8),
                        Expanded(
                          child: Text(
                            ble.statusMessage!,
                            style: const TextStyle(color: Colors.white70, fontSize: 12),
                          ),
                        ),
                      ],
                    ),
                  ),

                const Divider(color: Colors.white12),

                // Laitelista
                Expanded(
                  child: results.isEmpty
                      ? Center(
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Icon(
                                ble.isScanning ? Icons.bluetooth_searching : Icons.bluetooth_disabled,
                                size: 48,
                                color: Colors.white24,
                              ),
                              const SizedBox(height: 12),
                              Text(
                                ble.isScanning
                                    ? "Etsitään lähellä olevia Arduino-laitteita..."
                                    : "Laitteita ei löytynyt.",
                                style: const TextStyle(color: Colors.white54),
                              ),
                              const SizedBox(height: 16),
                              ElevatedButton.icon(
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.cyanAccent.withValues(alpha: 0.15),
                                  foregroundColor: Colors.cyanAccent,
                                ),
                                icon: const Icon(Icons.search),
                                label: const Text("Aloita haku"),
                                onPressed: () => ble.startScan(),
                              ),
                            ],
                          ),
                        )
                      : ListView.builder(
                          controller: scrollController,
                          itemCount: results.length,
                          itemBuilder: (context, index) {
                            final result = results[index];
                            final device = result.device;
                            final name = device.platformName.isNotEmpty
                                ? device.platformName
                                : result.advertisementData.advName;
                            final isArduino = name.toLowerCase().contains("arduino") ||
                                name.toLowerCase().contains("arm");
                            final isConnected = ble.connectedDevice?.remoteId == device.remoteId;

                            return Container(
                              margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                              decoration: BoxDecoration(
                                color: isArduino
                                    ? const Color(0xFF1A2838)
                                    : const Color(0xFF1B202D),
                                borderRadius: BorderRadius.circular(12),
                                border: Border.all(
                                  color: isArduino
                                      ? Colors.cyanAccent.withValues(alpha: 0.5)
                                      : Colors.white10,
                                ),
                              ),
                              child: ListTile(
                                leading: Container(
                                  padding: const EdgeInsets.all(8),
                                  decoration: BoxDecoration(
                                    color: isArduino
                                        ? Colors.cyanAccent.withValues(alpha: 0.2)
                                        : Colors.white10,
                                    borderRadius: BorderRadius.circular(8),
                                  ),
                                  child: Icon(
                                    isArduino ? Icons.precision_manufacturing : Icons.bluetooth,
                                    color: isArduino ? Colors.cyanAccent : Colors.white70,
                                  ),
                                ),
                                title: Row(
                                  children: [
                                    Expanded(
                                      child: Text(
                                        name.isNotEmpty ? name : "Tuntematon laite",
                                        style: TextStyle(
                                          color: Colors.white,
                                          fontWeight: isArduino ? FontWeight.bold : FontWeight.normal,
                                        ),
                                      ),
                                    ),
                                    if (isArduino)
                                      Container(
                                        padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                                        decoration: BoxDecoration(
                                          color: Colors.cyanAccent.withValues(alpha: 0.2),
                                          borderRadius: BorderRadius.circular(4),
                                        ),
                                        child: const Text(
                                          "UNO R4",
                                          style: TextStyle(
                                            color: Colors.cyanAccent,
                                            fontSize: 10,
                                            fontWeight: FontWeight.bold,
                                          ),
                                        ),
                                      ),
                                  ],
                                ),
                                subtitle: Text(
                                  "${device.remoteId.str} • Signaali: ${result.rssi} dBm",
                                  style: const TextStyle(color: Colors.white38, fontSize: 11),
                                ),
                                trailing: isConnected
                                    ? OutlinedButton(
                                        style: OutlinedButton.styleFrom(
                                          foregroundColor: Colors.redAccent,
                                          side: const BorderSide(color: Colors.redAccent),
                                        ),
                                        onPressed: () => ble.disconnect(),
                                        child: const Text("Katkaise"),
                                      )
                                    : ElevatedButton(
                                        style: ElevatedButton.styleFrom(
                                          backgroundColor: isArduino
                                              ? Colors.cyanAccent
                                              : Colors.blueGrey,
                                          foregroundColor: Colors.black,
                                        ),
                                        onPressed: ble.status == BleConnectionStatus.connecting
                                            ? null
                                            : () async {
                                                final ok = await ble.connect(device);
                                                if (ok && context.mounted) {
                                                  Navigator.of(context).pop();
                                                }
                                              },
                                        child: const Text("Yhdistä"),
                                      ),
                              ),
                            );
                          },
                        ),
                ),
              ],
            );
          },
        );
      },
    );
  }
}
