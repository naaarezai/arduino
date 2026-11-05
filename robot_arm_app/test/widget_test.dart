import 'package:flutter_test/flutter_test.dart';
import 'package:robot_arm_app/main.dart';

void main() {
  testWidgets('RobotArmApp smoke test', (WidgetTester tester) async {
    await tester.pumpWidget(const RobotArmApp());
    expect(find.text('ROBOTTIKÄSI'), findsOneWidget);
    await tester.pumpAndSettle(const Duration(seconds: 4));
  });
}
