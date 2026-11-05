/*
  ========================================================================
  Arduino Uno R4 WiFi - 6-Akselinen Robottikäsivarren BLE-ohjain
  ========================================================================
  Tämä ohjelma vastaanottaa kulmatiedot (0 - 180 astetta) kuudelle
  servolle Bluetooth Low Energy (BLE) -yhteydellä Flutter-sovelluksesta.

  Servojen kytkentä Arduino Uno R4 WiFi -levyyn:
    Servo 1: Jalusta (Base)         -> Pin D3  (PWM)
    Servo 2: Olkavarsi (Shoulder)   -> Pin D5  (PWM)
    Servo 3: Kyynärvarsi (Elbow)     -> Pin D6  (PWM)
    Servo 4: Ranne pysty (Pitch)    -> Pin D9  (PWM)
    Servo 5: Ranne kierto (Roll)    -> Pin D10 (PWM)
    Servo 6: Koura (Gripper)        -> Pin D11 (PWM)

  TÄRKEÄÄ VIRTALÄHTEESTÄ:
    ÄLÄ ota virtaa kuudelle servolle suoraan Arduinon 5V-pinnistä!
    Käytä erillistä 5V - 6V virtalähdettä (vähintään 3A - 5A) servoille.
    Muista yhdistää ulkoisen virtalähteen GND Arduinon GND-pinniin!
  ========================================================================
*/

#include <ArduinoBLE.h>
#include <Servo.h>
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

// Määritellään servojen pinnit (Uno R4 WiFi PWM-pinnit)
const int PIN_SERVO_1 = 3;   // Jalusta
const int PIN_SERVO_2 = 5;   // Olkavarsi
const int PIN_SERVO_3 = 6;   // Kyynärvarsi
const int PIN_SERVO_4 = 9;   // Ranne pysty
const int PIN_SERVO_5 = 10;  // Ranne kierto
const int PIN_SERVO_6 = 11;  // Koura

const int NUM_SERVOS = 6;
Servo servos[NUM_SERVOS];
const int servoPins[NUM_SERVOS] = {
  PIN_SERVO_1, PIN_SERVO_2, PIN_SERVO_3,
  PIN_SERVO_4, PIN_SERVO_5, PIN_SERVO_6
};

// Nykyiset ja tavoitekulmat (0 - 180 astetta, Servo 6 rajattu 40 - 128 astetta)
float currentAngles[NUM_SERVOS] = {90.0, 90.0, 90.0, 90.0, 90.0, 40.0};
float targetAngles[NUM_SERVOS]  = {90.0, 90.0, 90.0, 90.0, 90.0, 40.0};

// Mekaaniset kulmarajat (Servo 6 koura rajattu välille 40 - 128 astetta)
const int minAngles[NUM_SERVOS] = {0, 0, 0, 0, 0, 40};
const int maxAngles[NUM_SERVOS] = {180, 180, 180, 180, 180, 128};

// Liikkeen pehmennysnopeus (asteina per askel)
const float SPEED_STEP = 1.0; 
const unsigned long STEP_INTERVAL_MS = 15; // Päivitysväli millisekunteina
unsigned long lastStepTime = 0;

// BLE Palvelu ja Karakteristiikka
const char* SERVICE_UUID        = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char* CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214";

BLEService armService(SERVICE_UUID);
BLECharacteristic armCharacteristic(
  CHARACTERISTIC_UUID, 
  BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify, 
  20
);

// Sisäänrakennettu LED (Pin 13)
const int LED_PIN = LED_BUILTIN;

// Tila: liikkuvatko servot parhaillaan
bool isMoving = false;
unsigned long lastCmdTime = 0;

// LED Matrix kuvakkeet (12x8)
// Bluetooth-ikoni odotustilassa
const uint32_t iconBluetooth[] = {
  0x0400c014,
  0x24424014,
  0x0c004000
};

// Robottikäsi / Nuoli liikkeen aikana
const uint32_t iconMoving[] = {
  0x1fc02004,
  0x08010020,
  0x0401fc00
};

// OK / Valmis -ruutu kun asento saavutettu
const uint32_t iconReady[] = {
  0x00000200,
  0x40081040,
  0x20000000
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Arduino Uno R4 WiFi Robottikäsivarsi ---");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Alustetaan Uno R4 WiFi:n 12x8 LED-matriisinäyttö
  matrix.begin();
  matrix.loadFrame(iconBluetooth);

  // Alustetaan servot ja asetetaan ne alkuasentoon
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write((int)currentAngles[i]);
  }

  // Alustetaan BLE
  if (!BLE.begin()) {
    Serial.println("VIRHE: BLE:n alustus epaonnistui!");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }

  // Määritellään BLE-laite ja mainostettavat tiedot
  BLE.setLocalName("Arduino-Arm");
  BLE.setDeviceName("Arduino-Arm");
  BLE.setAdvertisedService(armService);
  armService.addCharacteristic(armCharacteristic);
  BLE.addService(armService);

  uint8_t initialData[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    initialData[i] = (uint8_t)currentAngles[i];
  }
  armCharacteristic.writeValue(initialData, NUM_SERVOS);

  BLE.advertise();
  Serial.println("BLE kaynnistetty! Mainostetaan nimella: Arduino-Arm");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Yhdistetty: ");
    Serial.println(central.address());
    matrix.loadFrame(iconReady);

    while (central.connected()) {
      if (armCharacteristic.written()) {
        int len = armCharacteristic.valueLength();
        const uint8_t* val = armCharacteristic.value();

        if (len == 6) {
          for (int i = 0; i < NUM_SERVOS; i++) {
            targetAngles[i] = constrain(val[i], minAngles[i], maxAngles[i]);
          }
        } else if (len > 0) {
          char buffer[32];
          int copyLen = min(len, 31);
          memcpy(buffer, val, copyLen);
          buffer[copyLen] = '\0';
          parseTextCommand(buffer);
        }
        lastCmdTime = millis();
      }

      updateServoPositions();
    }

    Serial.print("Yhteys katkesi: ");
    Serial.println(central.address());
    digitalWrite(LED_PIN, LOW);
    matrix.loadFrame(iconBluetooth);
  } else {
    // Vilkutetaan hitaasti LEDiä paritustilassa
    if ((millis() / 500) % 2 == 0) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }

  updateServoPositions();
}

void updateServoPositions() {
  unsigned long now = millis();
  if (now - lastStepTime < STEP_INTERVAL_MS) {
    return;
  }
  lastStepTime = now;

  bool movingNow = false;

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (abs(currentAngles[i] - targetAngles[i]) > 0.05) {
      movingNow = true;
      if (currentAngles[i] < targetAngles[i]) {
        currentAngles[i] += SPEED_STEP;
        if (currentAngles[i] > targetAngles[i]) currentAngles[i] = targetAngles[i];
      } else {
        currentAngles[i] -= SPEED_STEP;
        if (currentAngles[i] < targetAngles[i]) currentAngles[i] = targetAngles[i];
      }
      servos[i].write((int)round(currentAngles[i]));
    }
  }

  // Valojen ohjaus liikkeen mukaan
  if (movingNow) {
    // Kun servot liikkuvat: sytytetään LED_BUILTIN ja näytetään liikesymboli matriisissa
    digitalWrite(LED_PIN, HIGH);
    if (!isMoving) {
      matrix.loadFrame(iconMoving);
      isMoving = true;
    }
  } else {
    // Kun liike on pysähtynyt ja tavoiteasento saavutettu
    if (isMoving) {
      digitalWrite(LED_PIN, LOW);
      matrix.loadFrame(iconReady);
      isMoving = false;
      // Lähetetään kuittaus BLE:n kautta takaisin puhelimeen
      uint8_t reportData[NUM_SERVOS];
      for (int i = 0; i < NUM_SERVOS; i++) reportData[i] = (uint8_t)currentAngles[i];
      armCharacteristic.writeValue(reportData, NUM_SERVOS);
    }
  }
}

void parseTextCommand(const char* str) {
  int a0, a1, a2, a3, a4, a5;
  if (sscanf(str, "%d,%d,%d,%d,%d,%d", &a0, &a1, &a2, &a3, &a4, &a5) == 6) {
    targetAngles[0] = constrain(a0, minAngles[0], maxAngles[0]);
    targetAngles[1] = constrain(a1, minAngles[1], maxAngles[1]);
    targetAngles[2] = constrain(a2, minAngles[2], maxAngles[2]);
    targetAngles[3] = constrain(a3, minAngles[3], maxAngles[3]);
    targetAngles[4] = constrain(a4, minAngles[4], maxAngles[4]);
    targetAngles[5] = constrain(a5, minAngles[5], maxAngles[5]);
    return;
  }

  int servoIdx, angle;
  if (sscanf(str, "S%d:%d", &servoIdx, &angle) == 2) {
    if (servoIdx >= 1 && servoIdx <= NUM_SERVOS) {
      int idx = servoIdx - 1;
      targetAngles[idx] = constrain(angle, minAngles[idx], maxAngles[idx]);
    }
  }
}
