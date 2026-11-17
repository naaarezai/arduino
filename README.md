# 🦾 6-Akselinen Teollisuusrobottikäsivarsi (6-DOF)
### Digital Twin & OLP Studio | Flutter BLE Controller | Arduino Uno R4 WiFi Firmware

[![Python Tests](https://img.shields.io/badge/Python%20Tests-23%20Passed-brightgreen)](digital_twin/tests)
[![Flutter Tests](https://img.shields.io/badge/Flutter%20Tests-4%20Passed-brightgreen)](robot_arm_app/test)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20Android%20%7C%20Arduino-orange)]()

Tämä repositorio sisältää täydellisen, avoimen lähdekoodin teollisuusrobottijärjestelmän 6-akseliselle nivelvarsirobotille. Järjestelmä yhdistää fysiikkaperusteisen **3D-digitaalisen kaksosen (Digital Twin)**, ammattitasoisen **etäohjelmoinnin (Offline Programming, OLP)**, modernin **Android-mobiiliohjaimen (Flutter BLE)** sekä sulavan **Arduino Uno R4 WiFi -laiteohjelmiston**.

---

## 📑 Sisällysluettelo
1. [Järjestelmän yleiskuva](#-järjestelmän-yleiskuva)
2. [Laitteisto- ja järjestelmävaatimukset (Requirements)](#-laitteisto--ja-järjestelmävaatimukset-requirements)
3. [Asennus ja pika-aloitus](#-asennus-ja-pika-aloitus)
   - [Työpöytä: Robot OLP Studio](#1-työpöytä-robot-olp-studio-3d-digital-twin)
   - [Mobiili: Flutter Android App (APK)](#2-mobiili-flutter-android-sovellus)
   - [Laiteohjelmisto: Arduino Uno R4 WiFi](#3-laiteohjelmisto-arduino-uno-r4-wifi)
4. [Sähkökytkennät, pinnijako ja virtalähde](#-sähkökytkennät-pinnijako-ja-virtalähde)
5. [Kinematiikka, mitat ja servojen suunnat (Inversions)](#-kinematiikka-mitat-ja-servojen-suunnat-inversions)
6. [Robot OLP Studio -ominaisuudet](#-robot-olp-studio--ominaisuudet)
7. [Automaattiset testit ja laadunvarmistus](#-automaattiset-testit-ja-laadunvarmistus)
8. [Projektin hakemistorakenne](#-projektin-hakemistorakenne)
9. [Ongelmanratkaisu ja UKK (FAQ)](#-ongelmanratkaisu-ja-ukk-faq)
10. [Lisenssi ja tekijät](#-lisenssi-ja-tekijät)

---

## 🌟 Järjestelmän yleiskuva

Kokonaisuus koostuu kolmesta toisiaan täydentävästä komponentista:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        6-DOF ROBOTTIJÄRJESTELMÄ                        │
├────────────────────────────────┬───────────────────────────────────────┤
│ 🖥️ Robot OLP Studio (Desktop) │ 📱 Robot Arm App (Android / Mobile)   │
│ - Python 3.10+, PyQt, PyVista  │ - Flutter SDK, Dart, reactive BLE     │
│ - 3D VTK -visualisointi        │ - 2D-kinematiikkavektorigrafiikka     │
│ - FK / IK matriisimoottori     │ - Karteesinen XYZ-hölkkäajo (Jog)     │
│ - PTP, LIN & Weave -liikeradat │ - Kosketussekvensseri (Teach & Play)  │
│ - KUKA KRL, ABB RAPID, G-Code  │ - Ready/Home/Pick -pika-asennot       │
│ - USB (115200) & BLE -lähetys  │ - Valmis asennuspaketti (APK)         │
└────────────────────────────────┴───────────────────────────────────────┘
                                ▲
                                │ USB-sarjaportti (115200 baud) TAI
                                │ Bluetooth Low Energy (BLE: "Arduino-Arm")
                                ▼
┌────────────────────────────────────────────────────────────────────────┐
│ ⚡ Arduino Uno R4 WiFi -laiteohjelmisto (Firmware)                     │
│ - Renesas RA4M1 (48 MHz) + ESP32-S3 BLE -radio                        │
│ - S-curve / kosini-interpolaatio servoliikkeiden pehmennykseen        │
│ - 12x8 LED-matriisi tilasymboleille (ᛒ Bluetooth, Nuoli liike, Neliö) │
│ - 6x PWM-servomoottorin laitteisto-ohjaus turvarajoilla                │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 💻 Laitteisto- ja järjestelmävaatimukset (Requirements)

Ennen järjestelmän käyttöä varmista, että ympäristösi täyttää seuraavat vaatimukset:

### 1. Työpöytätietokone (Robot OLP Studio)
* **Käyttöjärjestelmä:** Windows 10/11 (64-bit), Linux (Ubuntu 20.04+) tai macOS (12+).
* **Python-versio:** **Python 3.10, 3.11 tai 3.12** (64-bit suositeltu).
* **Näytönohjain:** OpenGL 3.3+ -yhteensopiva GPU sujuvaa 3D VTK -renderöintiä varten.
* **Python-riippuvuudet (`digital_twin/requirements.txt`):**
  * `PyQt6>=6.6.0` tai `PyQt5>=5.15.0` (GUI-käyttöliittymäkehys)
  * `pyvista>=0.43.0` ja `pyvistaqt>=0.11.0` (3D VTK -moottori)
  * `numpy>=1.24.0` (Kinematiikka ja matriisilaskenta)
  * `pyserial>=3.5` (USB-sarjaliikenne 115200 baud)
  * `bleak>=0.22.0` (Langaton Bluetooth Low Energy -yhteys)

### 2. Mobiililaite (Robot Arm App)
* **Käyttöjärjestelmä:** Android 8.0 (API level 26) tai uudempi.
* **Yhteys:** Bluetooth 4.2+ / Bluetooth Low Energy (BLE).
* **Lähdekoodin kääntämiseen:** Flutter SDK 3.24+ ja Android SDK.

### 3. Elektroniikka ja laitteisto (Hardware)
* **Mikrokontrolleri:** **Arduino Uno R4 WiFi** (virallinen Renesas RA4M1 + ESP32-S3 -malli).
* **Servomoottorit:** 6 kpl standardikokoisia servomoottoreita (esim. MG996R, DS3218 tai vastaavia metallirattaisia $15-20\,\text{kg}\cdot\text{cm}$ servoja).
* **Virtalähde (KRIITTINEN):**
  * **Ulkoinen stabiloitu 5V – 6V virtalähde, vähintään 4A – 6A jatkuvaa virtaa.**
  * ⚠️ *Kuutta servoa EI SAA KOSKAAN syöttää suoraan Arduinon 5V-nastasta!*
  * Yhteinen maadoitus (**Common GND**) Arduinon ja ulkoisen virtalähteen välillä on pakollinen.
* **Mekaniikka:** 3D-tulostetut tai CNC-koneistetut runko-osat. CAD-lähdetiedostot löytyvät kansiosta `KasiVarsi/` (Autodesk Inventor `.ipt`, `.iam`, `.step`) sekä `digital_twin/assets/models/` (`.stl`).

---

## 🚀 Asennus ja pika-aloitus

### 1. Työpöytä: Robot OLP Studio (3D Digital Twin)

#### Pika-aloitus yhdellä klikkauksella (Windows):
Kaksoisnapsauta tiedostoa:
👉 **`run_digital_twin.bat`**

#### Asennus komentoriviltä:
```powershell
# 1. Kloonaa repositorio
git clone https://github.com/naaarezai/arduino.git
cd arduino

# 2. Asenna riippuvuudet
cd digital_twin
python -m pip install -r requirements.txt

# 3. Käynnistä OLP Studio
python main.py
```

---

### 2. Mobiili: Flutter Android -sovellus

#### Vaihtoehto A: Asenna valmis APK suoraan puhelimeen (Suositeltu)
1. Lataa valmis asennuspaketti:
   👉 **[Lataa app-release.apk tästä](https://github.com/naaarezai/arduino/raw/main/robot_arm_app/app-release.apk)**
2. Siirrä tiedosto puhelimeen tai avaa linkki puhelimen selaimella ja asenna sovellus.
3. Myönnä sovellukselle Bluetooth-oikeudet ja yhdistä laitteeseen **Arduino-Arm**.

#### Vaihtoehto B: Käännä lähdekoodista (Kehittäjille)
```bash
cd robot_arm_app
flutter pub get
flutter run --release
```

---

### 3. Laiteohjelmisto: Arduino Uno R4 WiFi

1. Avaa **Arduino IDE** (versio 2.2+ suositeltu).
2. Avaa *Tools -> Board -> Boards Manager* ja asenna **Arduino UNO R4 Boards**.
3. Avaa *Tools -> Manage Libraries* ja asenna:
   * **`ArduinoBLE`**
   * **`Servo`**
4. Avaa tiedosto:
   `arduino_firmware/arduino_arm_controller/arduino_arm_controller.ino`
5. Valitse laudaksi **Arduino UNO R4 WiFi** ja kytketty COM-portti.
6. Paina **Upload** (Lataa).
7. Serial Monitor (115200 baud) tulostaa: `BLE kaynnistetty! Mainostetaan nimella: Arduino-Arm`.

---

## ⚡ Sähkökytkennät, pinnijako ja virtalähde

### Arduino Uno R4 WiFi Pinnijako

| Nivel | Nimi (Fi / En) | Uno R4 Pinni | Oletuskulma | Sallittu liikealue | Suunta (Inversion) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Servo 1** | Jalusta (Base) | **D3** (PWM) | 90° | 0° – 180° | Normaali |
| **Servo 2** | Olkapää (Shoulder) | **D5** (PWM) | 90° | 0° – 180° | **Käännetty (`Inv = True`)** |
| **Servo 3** | Kyynärpää (Elbow) | **D6** (PWM) | 90° | 0° – 180° | **Käännetty (`Inv = True`)** |
| **Servo 4** | Ranne pysty (Wrist Pitch) | **D9** (PWM) | 90° | 0° – 180° | **Käännetty (`Inv = True`)** |
| **Servo 5** | Ranne kierto (Wrist Roll) | **D10** (PWM) | 90° | 0° – 180° | Normaali |
| **Servo 6** | Koura (Gripper) | **D11** (PWM) | 20° (Auki) | **20° – 180°** | Normaali |

> [!CAUTION]
> **Kouran mekaaninen turvaraja:** Kouran servon minimikulma on ohjelmallisesti rajoitettu **20 asteeseen**. Alle 20° kulmassa tarttujan hammasrattaat hyppäävät pois paikoiltaan mekaanisesti.

### Kytkentäkaavio (Virtalähde & Signaalit)

```
[ Ulkoinen 5V-6V 5A Virtalähde ]
  (+) 5V/6V ──────────────────────> Kaikkien 6 servon punaiset johdot (VCC)
  (-) GND  ────────┬─────────────> Kaikkien 6 servon mustat/ruskeat johdot (GND)
                   │
                   └─────────────> Arduino Uno R4 WiFi GND (YHTEINEN MAA!)

[ Arduino Uno R4 WiFi ]
  Pin D3 (PWM)   ────────────────> Servo 1 (Jalusta) kelta/oranssi signaali
  Pin D5 (PWM)   ────────────────> Servo 2 (Olkapää) kelta/oranssi signaali
  Pin D6 (PWM)   ────────────────> Servo 3 (Kyynärpää) kelta/oranssi signaali
  Pin D9 (PWM)   ────────────────> Servo 4 (Ranne pysty) kelta/oranssi signaali
  Pin D10 (PWM)  ────────────────> Servo 5 (Ranne kierto) kelta/oranssi signaali
  Pin D11 (PWM)  ────────────────> Servo 6 (Koura) kelta/oranssi signaali
```

---

## 📐 Kinematiikka, mitat ja servojen suunnat (Inversions)

### Käsivarren nivelmitat
* **$L_0$ (Jalustan korkeus):** $140\,\text{mm}$ (pöydän pinnasta olkanivelen akselille)
* **$L_1$ (Olkavarsi):** $200\,\text{mm}$
* **$L_2$ (Kyynärvarsi):** $200\,\text{mm}$
* **$L_3$ (Ranne $\rightarrow$ TCP):** $180\,\text{mm}$ (työkalulaipasta kouran tartuntakeskipisteeseen)
* **Maksimiulottuvuus:** $\approx 580\,\text{mm}$ säteellä

### Servojen mekaaninen suunnanvaihto (Inversions)
Robottirungon mekaanisen vivuston vuoksi servot 2, 3 ja 4 kääntyvät fyysisesti vastakkaiseen suuntaan kuin puhtaan matemaattisen kinematiikan positiivinen kulma. Järjestelmässä on sisäänrakennettu automaattinen inversiomatriisi:

$$\text{Kulma}_{\text{fyysinen}} = 180^\circ - \text{Kulma}_{\text{virtuaalinen}} \quad (\text{servot 2, 3 ja 4})$$

Tämä takaa, että:
1. Kun vedät virtuaalimallin 3D-kärkeä eteenpäin, fyysinen robotti kurkottaa eteenpäin.
2. Kun painat mobiilisovelluksessa $+X$, varsi ojentuu suoraviivaisesti eteenpäin.
3. Nollapisteen kalibrointi ja trimmi tallentuvat tiedostoon `digital_twin/calibration.json`.

---

## 🖥️ Robot OLP Studio – Ominaisuudet

Täysi visuaalinen kuvitettu käyttöohje:
👉 **[Lue kattava käyttöohje kuvineen tästä (digital_twin_kayttoohje.md)](digital_twin_kayttoohje.md)**

* **Ribbon-työtilat:**
  * `Program (OLP)`: Liikeratojen opetus, lausekepuu, liikekäskyt.
  * `Simulation`: Reaaliaikainen toisto, aikajana, sykliaikalaskuri ja fyysisen robotin suoratoisto (`Stream to Arm`).
  * `Layout & Connections`: BLE- ja USB-yhteyksien hallinta, täysi nivelten liukusäädinpaneeli, pika-asennot.
* **Monipuoliset liikekäskyt:**
  * `PTP (Point-to-Point)`: Nivelakseleita optimoiva nopea siirtymäliike.
  * `LIN (Linear)`: Suoraviivainen karteesinen hitsaus- tai työstörata.
  * `Weave (Pujotus)`: Hitsauksen siksak- tai aaltoliike säädettävällä leveydellä ja aallonpituudella.
  * `Gripper / ToolAction`: Kouran ja työkalujen ohjaus.
* **3D-työkalurivi ja interaktiivisuus:**
  * **Interactive Gizmo:** Raahaa kouran kärkeä vapaasti 3D-avaruudessa; kinematiikkamoottori ratkaisee nivelkulmat 60 FPS:llä.
  * **Snap TCP:** Kohdista kärki automaattisesti työkappaleen saumoihin tai kulmiin.
  * **Reach Envelope:** Läpikuultava kinemaattinen työaluepallo.
  * **Törmäystarkastelu:** Varoittaa, jos kärki tai nivelet osuvat työpöytään tai kiinnittimiin.
* **Koodin kääntäjät (Post-Processors):**
  * KUKA KRL (`.src`)
  * ABB RAPID (`.mod`)
  * ISO G-Code (`.nc`)
  * Arduino C++ (`.ino`)
* **Turvallisuus:** Yläpalkin punainen **⛔ HÄTÄ-SEIS (Esc)** pysäyttää simulaation ja katkaisee servojen liikkeen välittömästi.

---

## 🧪 Automaattiset testit ja laadunvarmistus

Projekti sisältää kattavat automatisoidut yksikkö- ja integraatiotestit sekä Python- että Flutter-puolella.

### Python-testien suoritus (Digital Twin):
```powershell
python -m unittest discover -s digital_twin/tests
```
*Tuloste:*
```text
Ran 23 tests in 0.141s
OK (FK/IK, törmäystarkastus, OLP-lausekkeet, koodikääntäjät, kalibrointi)
```

### Flutter-testien suoritus (Mobiilisovellus):
```bash
cd robot_arm_app
flutter test
```
*Tuloste:*
```text
00:01 +4: All tests passed! (FK, realistinen poiminta-IK, lineaaripolku, widget-testi)
```

---

## 📂 Projektin hakemistorakenne

```text
Arduino/
├── .gitignore                                 # Git-ohitussäännöt (rakennusartefaktit, Inventor OldVersions)
├── README.md                                  # Pääohje ja järjestelmäkuvaus (Tämä tiedosto)
├── run_digital_twin.bat                       # 1-klikkauksen käynnistys Windowsissa
│
├── digital_twin/                              # 🖥️ Robot OLP Studio (Python + PyQt + PyVista)
│   ├── README.md                              # OLP Studion oma syventävä dokumentaatio
│   ├── requirements.txt                       # Python-kirjastoriippuvuudet
│   ├── main.py                                # Pääkäynnistystiedosto
│   ├── calibration.json                       # Tallennetut servojen nollapistetrimmit ja suunnat
│   ├── assets/models/                         # 3D CAD STL -mallit (käsivarren linkit, koura, hitsauspoltin)
│   ├── src/
│   │   ├── config.py                          # Mitat, työpöydän rajat ja konfiguraatio
│   │   ├── core/                              # Kinematiikka, OLP-ohjelmamalli, kääntäjät, projektit
│   │   ├── communication/                     # USB pySerial ja Bluetooth bleak -taustasäikeet
│   │   ├── simulation/                        # PyVista 3D-moottori, Gizmo, törmäystarkastelu
│   │   └── ui/                                # PyQt-käyttöliittymä, työtilat, dialogit
│   └── tests/                                 # 23 automatisoitua yksikkö- ja integraatiotestiä
│
├── robot_arm_app/                             # 📱 Flutter-mobiilisovellus (Android)
│   ├── app-release.apk                        # Valmiiksi käännetty asennuspaketti puhelimeen
│   ├── pubspec.yaml                           # Flutter-riippuvuudet
│   ├── lib/                                   # Dart-lähdekoodi (BLE, Kinematiikka, UI, Sekvensseri)
│   └── test/                                  # Kinematiikan ja käyttöliittymän testit
│
├── arduino_firmware/                          # ⚡ Arduino Uno R4 WiFi -laiteohjelmisto
│   └── arduino_arm_controller/
│       └── arduino_arm_controller.ino         # C++ ohjain: BLE + USB 115200, S-curve, LED-matriisi
│
└── KasiVarsi/                                 # 📐 Alkuperäiset mekaaniset CAD-mallit
    ├── .ipt .iam .step -tiedostot/            # Autodesk Inventor -kokoonpanot ja osamallit
    └── Omat/                                  # Muokatut osat ja 3D-tulostustiedostot
```

---

## ❓ Ongelmanratkaisu ja UKK (FAQ)

### 1. Servot nykivät tai Arduino sammuu heti liikkeen alkaessa
* **Syy:** Servot ottavat liikaa virtaa ja aiheuttavat jännitteen romahduksen (brownout).
* **Ratkaisu:** Älä syötä servoja Arduinon 5V-nastasta. Käytä erillistä 5V–6V (vähintään 4A) virtalähdettä ja kytke yhteinen maadoitus (**GND**).

### 2. Robotti liikkuu väärään suuntaan kuin käyttöliittymässä
* **Syy:** Servon vipu tai asennussuunta on peilikuvana.
* **Ratkaisu:** Siirry Robot OLP Studion *Layout & Connections* -näkymään tai avaa *Hardware Zero-Offset & Trim Calibration* ja laita rasti kyseisen servon **Inv**-ruutuun. Oletuksena servot 2, 3 ja 4 ovat käännettyjä.

### 3. Bluetooth (BLE) ei löydä laitetta
* **Syy:** Sijainti- tai Bluetooth-oikeuksia ei ole myönnetty puhelimessa / PC:llä, tai toinen laite on jo varannut yhteyden.
* **Ratkaisu:** Varmista, että mikään muu laite ei ole paritettu Arduinoon (vain 1 aktiivinen BLE-yhteys kerrallaan). Tarkista, että Uno R4:n Serial Monitor näyttää: `BLE kaynnistetty! Mainostetaan nimella: Arduino-Arm`.

### 4. PyQt / DLL load failed Windowsissa
* **Syy:** Python-ympäristöstä puuttuu Visual C++ Redistributable tai Qt-kirjastoissa on ristiriita.
* **Ratkaisu:** Asenna uusin [Visual C++ Redistributable 2015-2022](https://aka.ms/vs/17/release/vc_redist.x64.exe) ja asenna kirjastot puhtaaseen virtuaaliympäristöön:
  ```powershell
  python -m pip install --upgrade pip
  python -m pip install -r digital_twin/requirements.txt
  ```

---

## 👥 Lisenssi ja tekijät

Tämä projekti on julkaistu avoimena lähdekoodina **MIT-lisenssillä**. Voit vapaasti käyttää, muokata ja jatkokehittää sitä opetuskäyttöön, opinnäytetöihin, tutkimukseen ja omiin robotiikkaprojekteihisi.

🤖 **Tervetuloa kehittämään ja kokeilemaan!** Voit jättää parannusehdotuksia tai raportoida havainnoista avaamalla GitHub Issuen tai Pull Requestin.
