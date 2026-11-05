# 🦾 6-Akselinen Robottikäsivarsi (6-DOF) — Flutter & Arduino Uno R4 WiFi

Tämä repositorio sisältää täydellisen, testatun ja tuotantovalmiin robottikäsivarren langattoman ohjausjärjestelmän. Järjestelmä yhdistää modernin **Flutter (Dart)** -mobiilisovelluksen ja **Arduino Uno R4 WiFi** -mikrokontrollerin käyttäen nopeaa ja vähävirtaista **Bluetooth Low Energy (BLE)** -tiedonsiirtoa.

Projektissa on toteutettu teollisuusrobottien ohjauksesta tutut ominaisuudet: **3D-avaruuden Karteesinen XYZ-ohjaus**, **käänteinen kinematiikka (Inverse Kinematics / IK)**, **PTP (Point-to-Point)** ja **LINEAR (suoraviivainen)** liikeratasuunnittelu, **sekvensseri (Teach & Play)** sekä servojen mekaaniset turvarajat.

---

## 📱 Pika-aloitus: Lataa valmis sovellus puhelimeen (APK)

Etkö halua asentaa kehitysympäristöä tai koodata? Voit ladata valmiin ja testatun asennuspaketin suoraan Android-puhelimeesi:

### 📥 [Lataa valmis APK tästä (app-release.apk)](https://github.com/naaarezai/arduino/raw/main/robot_arm_app/app-release.apk)

### 📲 Asennus- ja käyttöohje (3 helppoa vaihetta):
1. **Lataa tiedosto:** Avaa yllä oleva latauslinkki suoraan puhelimen selaimessa.
2. **Asenna sovellus:**
   * Avaa ladattu `app-release.apk` puhelimen ilmoituspalkista tai *Tiedostot / Lataukset* -kansiosta.
   * Valitse **Asenna** (jos puhelin kysyy lupaa, salli asennus tuntemattomista lähteistä tälle selaimelle).
3. **Aloita käsivarren ohjaus:**
   * Kytke virta Arduino Uno R4 WiFiin (ja ulkoinen virtalähde servoille).
   * Avaa **Robot Arm App** puhelimessasi.
   * Myönnä sovellukselle Bluetooth-oikeudet.
   * Napauta yläpalkin **"Ei yhteyttä"** -> Valitse listalta **Arduino-Arm** -> **Yhdistä**.
   * Robottikäsivarsi on heti käyttövalmis!

---

## 📸 Visuaalinen Identiteetti ja Käyttöliittymä

* **Sovelluksen logo & Launcher Icon:** Mukautettu sovelluskuvake puhelimen aloitusnäytöllä.
* **Animoitu Splash Screen:** Cyberpunk-tyylinen tumma käynnistysruutu hehkuvalla logolla ja kinematiikkamoottorin alustusindikaattorilla.
* **2D-Kinematiikkavisualisoija:** Reaaliaikainen vektorikaavio, joka näyttää käsivarren asennon, jalustan kääntökulman, kouran tilan sekä tarkan $(X, Y, Z)$ -koordinaatin millimetreinä.

---

## 🚀 Pääominaisuudet

### 1. Karteesinen XYZ-ohjaus & Käänteinen Kinematiikka (IK)
Ohjaa käsivartta kuten ammattirobottia: älä säädä jokaista servoa erikseen, vaan **kerro kouralle mihin pisteeseen avaruudessa sen halutaan liikkuvan**.
* **X-akseli (mm):** Eteenpäin ($+X$) ja taaksepäin ($-X$).
* **Y-akseli (mm):** Vasemmalle ($+Y$) ja oikealle ($-Y$).
* **Z-akseli (mm):** Korkeus pöytätasosta ylöspäin (ohjelmallisesti suojattu: $Z \ge 0$, estää törmäämisen pöytään).
* **Pitch (astetta):** Kouran työkalu- ja tartuntakulma ($-90^\circ$ suoraan alas pöytää kohti, $0^\circ$ vaakatasossa eteenpäin).
* **Säädettävä askelkoko (Jogging):** `1 mm` (mikrosäätö), `5 mm`, `10 mm` ja `25 mm`.

### 2. Liiketilat: PTP vs. LINEAR
* **PTP (Point-to-Point):**
  * Kaikki servot liikkuvat samanaikaisesti ja saapuvat tavoitekulmiin yhtä aikaa sujuvan S-curve/kosiniprofiilin mukaisesti.
  * Nopein ja luonnollisin tapa siirtää robotti asennosta toiseen.
* **LINEAR (Suoraviivainen karteesinen liike):**
  * Kouran kärki liikkuu 3D-avaruudessa **täysin suoraa viivaa** pisteestä A pisteeseen B.
  * Kinematiikkamoottori jakaa suoran linjan pieniksi $6\,\text{mm}$ väliaskeleiksi, laskee jokaiselle pisteelle käänteisen kinematiikan ja suoratoistaa asennot Arduinolle $35\,\text{ms}$ välein.

### 3. Servojen pyörimissuuntien kääntö (Inversion Management)
Robottirungoissa servot voidaan asentaa fyysisesti kummalle puolelle tahansa. Jos servon kulma kasvaa, se voi mekaanisen asennuksen vuoksi kääntää vartta väärään suuntaan.
* Yläpalkissa on **`⇄` (Suunnat)** -asetusvalikko.
* Voit kääntää minkä tahansa servon suunnan ($180^\circ - \text{kulma}$) päälle/pois yhdellä kytkimellä.
* Valinnat tallentuvat automaattisesti laitteen pysyvään muistiin (`SharedPreferences`).

### 4. Mekaaniset turvarajat (Servo Limits)
* **Servo 6 (Koura / Tarttuja):** Rajattu tiukasti välille **`40° – 128°`** sekä sovelluksessa että suoraan Arduinon laiteohjelmistossa.
  * Tämä estää servon moottoria ja kouran muovisia/metallisia rattaita vääntymästä rikki ääriasennoissa.

### 5. Sekvensseri (Teach & Play / Opetustoiminto)
* Tallenna nykyinen asento sekvenssiin yhdellä napautuksella.
* Järjestä vaiheet uudelleen raahaamalla (`ReorderableListView`).
* Määritä jokaiselle askeleelle oma viiveaika millisekunteina.
* Toista liikesarja kerran tai jatkuvana silmukkana (**Loop**).

### 6. Valmiit asennot (Presets)
* Yhden napautuksen pikapainikkeet: **Lepoasento (Home)**, **Valmiustila (Ready)**, **Poiminta lattialta**, **Nosto ylös**, **Kurotus eteen** sekä mahdollisuus tallentaa omia asentoja.

### 7. Hätäseis (Emergency Stop)
* Yläpalkin punainen hätäpainike keskeyttää välittömästi aktiivisen liikesarjan/suoratoiston ja ajaa käsivarren turvallisesti lepoasentoon.

---

## 📐 Käsivarren Kinemaattiset Mitat

Käänteinen kinematiikka perustuu robottikäsivarren todellisiin nivelvälimittoihin (akselin keskeltä seuraavan akselin keskelle):

| Mitta | Osa | Arvo | Kuvaus |
| :--- | :--- | :--- | :--- |
| **$L_0$** | Jalusta (Base) | **140 mm** | Pöydän pinnasta olkaservon (Servo 2) akselin keskelle |
| **$L_1$** | Olkavarsi (Upper arm) | **200 mm** | Olkaservosta (S2) kyynärservon (S3) akselille |
| **$L_2$** | Kyynärvarsi (Forearm) | **200 mm** | Kyynärservosta (S3) ranneservon (S4) akselille |
| **$L_3$** | Ranne $\rightarrow$ Koura | **180 mm** | Ranneservosta (S4) kouran kärkeen / tartuntapisteeseen |

* **Maksimiulottuvuus eteenpäin:** $\approx 580\,\text{mm}$ (58 cm)
* **Maksimikorkeus ylöspäin:** $\approx 720\,\text{mm}$ (72 cm)

---

## ⚡ Servojen Kytkentä ja Elektroniikka

### Arduino Uno R4 WiFi Pinnijako

| Nivel | Nimi (Fi / En) | Uno R4 Pinni | Oletuskulma | Sallittu alue |
| :--- | :--- | :--- | :--- | :--- |
| **Servo 1** | Jalusta (Base) | **D3** (PWM) | 90° | 0° – 180° |
| **Servo 2** | Olkavarsi (Shoulder) | **D5** (PWM) | 90° | 0° – 180° *(Inverted)* |
| **Servo 3** | Kyynärvarsi (Elbow) | **D6** (PWM) | 90° | 0° – 180° |
| **Servo 4** | Ranne pysty (Wrist Pitch) | **D9** (PWM) | 90° | 0° – 180° |
| **Servo 5** | Ranne kierto (Wrist Roll) | **D10** (PWM) | 90° | 0° – 180° |
| **Servo 6** | Koura (Gripper) | **D11** (PWM) | 40° (Auki) | **40° – 128°** |

### ⚠️ KRIITTINEN VIRTALÄHDEOHJE (Lue ennen kytkentää!)
1. **ÄLÄ KOSKAAN syötä virtaa kuudelle servolle suoraan Arduinon 5V-nastasta!**
   * Kuusi servoa kuluttaa liikkuessaan helposti **2A – 4A** virtaa.
   * Jos servot ottavat virran Arduinon lineaarisäätimestä, Arduino sammuu (brownout) heti liikkeen alkaessa tai mikrokontrolleri vaurioituu pysyvästi.
2. **Käytä erillistä 5V – 6V (vähintään 3A – 5A) virtalähdettä** servoille (esim. hakkurivirtalähde, BEC tai erillinen virtakisko/PCA9685).
3. **Yhteinen maa (Common Ground):**
   * Kytke ulkoisen virtalähteen miinusnapa (**GND**) **Arduinon GND-pinniin**. Ilman yhteistä maata PWM-ohjaussignaali ei toimi!

---

## 💡 Valomerkit ja Tilailmaisut (Arduino Uno R4 WiFi)

Ohjelmisto hyödyntää Uno R4 WiFi:n omia valojärjestelmiä:
1. **LED_BUILTIN (Pin 13):**
   * *Vilkkuu rauhallisesti:* Odottaa Bluetooth-yhteyttä puhelimesta.
   * *Palaa yhtäjaksoisesti:* Servot liikkuvat parhaillaan.
   * *Sammuu:* Käsivarsi on saavuttanut kohdeasennon ja seisoo paikallaan.
2. **12x8 LED-matriisinäyttö:**
   * *Bluetooth-ikoni (ᛒ):* Odottaa paritusta.
   * *Liikesymboli / Nuoli:* Servot liikkuvat kohti tavoiteasentoa.
   * *Valmis-piste / Ruutu:* Asento saavutettu, järjestelmä valmis uuteen komentoon.

---

## 🛠️ Käyttöönotto-ohje

### 1. Arduino-koodin lataus
1. Avaa **Arduino IDE** (v2 suositeltu).
2. Asenna *Boards Managerista*: **Arduino UNO R4 Boards** (`arduino:renesas_uno`).
3. Asenna *Library Managerista*:
   * `ArduinoBLE`
   * `Servo`
4. Avaa tiedosto:
   `arduino_firmware/arduino_arm_controller/arduino_arm_controller.ino`
5. Valitse laudaksi **Arduino Uno R4 WiFi** ja valitse oikea sarjaportti (esim. COM4).
6. Klikkaa **Upload**. Serial Monitor (115200 baud) vahvistaa: `BLE kaynnistetty! Mainostetaan nimella: Arduino-Arm`.

### 2. Flutter-sovelluksen ajaminen
1. Varmista, että koneellasi on asennettuna Flutter SDK (v3.24+).
2. Siirry projektikansioon:
   ```bash
   cd robot_arm_app
   flutter pub get
   ```
3. Kytke Android-puhelin USB:llä (USB-vianetsintä päällä) ja käynnistä:
   ```bash
   flutter run
   ```
   *Tai rakenna valmis asennuspaketti:*
   ```bash
   flutter build apk --release
   ```
   Valmis APK: `build/app/outputs/flutter-apk/app-release.apk`.
4. Käynnistä sovellus puhelimessa, hyväksy Bluetooth- ja sijaintioikeudet ja napauta yläpalkin **"Ei yhteyttä"** -> Valitse listalta **Arduino-Arm** -> **Yhdistä**.

---

## 🧠 Miten haasteet ja virheet ratkaistiin (Problem Solving Log)

Seuraavalle kehitystiimille tiedoksi projektin aikana ratkaistut kriittiset tekniset kohdat:

1. **Gradle / Android SDK 37 AAR-virhe:**
   * *Ongelma:* Uusin `permission_handler` pakotti Android SDK 37:n, jota nykyinen Gradle-laajennus ei vielä tukenut.
   * *Ratkaisu:* `permission_handler` lukittiin vakaaseen versioon `11.3.1` tiedostossa `pubspec.yaml`.
2. **Servo 6 mekaaninen jumiutuminen:**
   * *Ongelma:* Kouran vivusto ei fyysisesti kykene 0°–180° liikkeeseen ilman rattaiden hakkaamista reunoihin.
   * *Ratkaisu:* Koura rajattiin sekä ohjelmiston kinematiikassa että Arduinon laiteohjelmistossa tiukasti välille `40° – 128°`.
3. **Liikesuunnan peilaus (Eteen-nappi vei taaksepäin):**
   * *Ongelma:* Olkaservon fyysinen asennustapa käänsi servoa vastakkaiseen suuntaan kulman kasvaessa.
   * *Ratkaisu:* Luotiin ohjelmallinen suuntamatriisi (`toPhysicalAngles`), joka peilaa signaalin ($180 - \text{kulma}$) lähetysvaiheessa. Lisättiin sovellukseen interaktiivinen suuntien kääntövalikko pysyvällä muistitallennuksella.
4. **Käyttöliittymän ylivuoto (Overflow 40px):**
   * *Ongelma:* Liiketilapainikkeet ja 25mm askelnappi ylittivät kapeilla näytöillä ruudun leveyden.
   * *Ratkaisu:* Erotettiin liiketila ja askelkoko omille riveilleen ja tehtiin askelnäppäimistä dynaamisesti skaalautuvia (`Expanded`).
5. **BLE-tiedonsiirron pehmennys (Throttling):**
   * *Ongelma:* Sliderin nopea liikuttaminen tukki BLE-puskurin sadoilla paketeilla sekunnissa.
   * *Ratkaisu:* Sovellukseen lisättiin $45\,\text{ms}$ lähetyskuristin (Throttling) manuaaliajoon ja tasainen $35\,\text{ms}$ sykli lineaarisuoratoistoon.

---

## 📂 Projektirakenne

```text
Arduino/
├── .gitignore
├── README.md                                  # Tämä dokumentti
├── arduino_firmware/
│   └── arduino_arm_controller/
│       └── arduino_arm_controller.ino         # Arduino Uno R4 WiFi firmware (BLE, LED Matrix, Smooth Easing)
└── robot_arm_app/                             # Flutter-mobiilisovellus
    ├── assets/
    │   └── images/
    │       └── logo.jpeg                      # Sovelluksen logo
    ├── lib/
    │   ├── main.dart                          # Sovelluksen pääkäynnistys ja teemat
    │   ├── kinematics/
    │   │   └── arm_kinematics.dart            # FK, IK, PTP ja Linear kinematiikkamoottori
    │   ├── models/
    │   │   └── arm_state.dart                 # Tietomallit (Nivelet, Asennot, Sekvenssit)
    │   ├── services/
    │   │   ├── ble_service.dart               # Bluetooth Low Energy -palvelu ja kuristus
    │   │   └── storage_service.dart           # Pysyväistallennus (SharedPreferences)
    │   └── ui/
    │       ├── screens/
    │       │   ├── splash_screen.dart         # Animoitu käynnistysruutu
    │       │   ├── home_screen.dart           # Päänäkymä (välilehdet, tilarivi, hätäseis)
    │       │   └── ble_scan_dialog.dart       # BLE-laitteiden skannaus ja paritusdialogi
    │       └── widgets/
    │           ├── arm_visualizer.dart        # 2D-vektorikinematiikan visualisoija ja HUD
    │           ├── joint_slider_card.dart     # Servokohtaiset säätökortit
    │           └── xyz_controller_card.dart   # Karteesinen XYZ-ohjainpaneeli ja askelvalitsin
    ├── test/
    │   ├── kinematics_test.dart               # Kinematiikan matemaattiset yksikkötestit
    │   └── widget_test.dart                   # Savutestit
    └── pubspec.yaml                           # Riippuvuudet ja asetukset
```

---

## 👥 Tekijät ja Lisenssi

Projekti on rakennettu ja optimoitu avoimen lähdekoodin periaatteita noudattaen robotiikan opetukseen, tutkimukseen ja harrasteprojekteihin. Vapaasti jatkokehitettävissä ja muokattavissa! 🤖✨
