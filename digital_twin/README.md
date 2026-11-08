# 🦾 Robot OLP Studio — 6-DOF Industrial Twin & Offline Programming

Teollisuustason **Digital Twin ja Offline Programming (OLP)** -ohjelmisto 6-akseliselle robottikäsivarrelle. Kehitetty Python-, PyQt- ja PyVista (VTK) -teknologioilla noudattaen johtavien robottisimulaattoreiden (**Visual Components, ABB RobotStudio, KUKA Sim, Robotmaster**) ammattimaisia työnkulkuja ja arkkitehtuuria.

Sovelluksella voidaan suunnitella liikeradat, simuloida robottisolun toiminta 3D-tilassa, tarkistaa ulottuvuudet ja törmäykset, generoida automaattisia hitsaus- ja kappaleenkäsittelyrutiineja sekä kääntää liikeradat suoraan fyysiselle robotille tai teollisuusohjaimille (**Arduino C++, G-Koodi, ABB RAPID**).

---

## 🚀 Pika-aloitus

### Tapa 1: Yhdellä klikkauksella (Windows)
Kaksoisnapsauta projektin juurikansiosta löytyvää käynnistintä:
👉 **`run_digital_twin.bat`**

### Tapa 2: Komentoriviltä
```powershell
cd digital_twin
python -m pip install -r requirements.txt
python main.py
```

---

## 🌟 Pääominaisuudet

### 1. 🗂️ Teollisuuden Ribbon-työtilat (Workspaces)
Yläpalkin välilehdistä voit siirtyä eri suunnitteluvaiheiden välillä:
* **📋 Program (OLP):** Päätila robottiohjelman opettamiseen, liikeratapuun muokkaukseen ja dynaamisten työkalutoimintojen käyttöön.
* **▶️ Simulation:** Simulaation toisto, nopeuskertoimet (0.5x, 1x, 2x, 4x), aikajana ja sykliaikalaskuri.
* **🔌 Layout & Connections:** Sarjaporttiyhteys (USB 115200 baud), servojen kalibrointi ja laiteyhteyden hallinta.

---

### 2. 🎛️ Dynaaminen työkalupalkki (Tool Context Awareness)
Ohjelmapuun yläpuolella oleva toimintopalkki mukautuu **automaattisesti** robotin aktiivisen työkalun mukaan. Ei enää sekavia nappirivejä eikä vääriä toimintoja:

#### A. Yleiset perusliikkeet (Aina näkyvillä):
* `+ PTP Motion`: Lisää nykyisen nivelasennon PTP-siirtona (Point-to-Point kosini-interpolaatiolla).
* `+ Linear Motion`: Lisää suoraviivaisen Cartesian-siirron (Linear LIN).
* `+ Dwell`: Lisää säädettävän pito-/odotusajan (ms).

#### B. Työkalukohtaiset toiminnot:

| Työkalu | Käytettävissä olevat toiminnot | Erityisominaisuudet |
| :--- | :--- | :--- |
| **🦾 Koura (Gripper)** | `🤏 Tartu (Close)`, `🖐️ Päästä (Open)`, `📐 Leukaväli...`, `📦 Pick & Place -apuri` | Ei koskaan tuota hitsausefektejä. Siirrot näkyvät siistinä sinisenä robottiliikkeenä. |
| **🔥 Hitsauspoltin (Welding Torch 45°)** | `🔥 Weld Wizard`, `⚡ 2-Click Seam`, `🧬 Multi-Pass`, `🔄 Käännä sauma` | Generoi valokaaren, kraatteriviiveen ja oranssit hitsipalkot. |
| **💨 Imukuppi (Vacuum)** | `💨 Imu PÄÄLLE`, `⚪ Imu POIS`, `📦 Pick & Place -apuri` | Pneumaattinen kappaleenkäsittely. |
| **🧪 Annostelija (Dispenser)** | `🧪 Pursotus PÄÄLLE`, `⚪ Pursotus POIS` | Liimaus- ja tiivistesaumat. |

---

### 3. 🔥 Teollisuushitsauksen erikoistyökalut (Welding Suite)
Hitsauspolttimella työskenneltäessä käytössäsi ovat alan kehittyneimmät OLP-työkalut:
* **⚡ 2-Click Seam Generator:** Klikkaa 3D-ikkunassa työkappaleen pinnalta sauman alku- ja loppupiste. Sovellus laskee käänteisen kinematiikan ja tuottaa automaattisen 5-vaiheisen teollisuusrutiinin (Lähestyminen $\rightarrow$ Sytytys/Arc On $\rightarrow$ Hitsaus $\rightarrow$ Kraatterin täyttö $\rightarrow$ Nosto).
* **🌊 Pendlausliike (Weave Motion):**
  * **Siksak (Zigzag)** tai **Sini (Sine wave)** -aaltoliike V-railoille ja pienasaumoille.
  * Säädettävä amplitudi (leveys mm) ja aallonpituus (pitch mm).
  * Sauman alku ja loppu kapenevat automaattisesti keskiviivalle siistin liitoksen varmistamiseksi.
* **🧬 Monipalkohitsaus (Multi-Pass Generator):**
  * Tuottaa paksuille materiaaleille monipalkoiset saumat (**Juuripalko / Root, Täyttöpalko / Fill, Pintapalko / Cap**).
  * Automaattiset sivuttais- ja korkeussiirtymät.
* **📐 Polttimen ajo- ja jätökulma (Push / Drag Angle):**
  * Poltin kallistuu sauman suuntaan säädettävällä kulmalla ($+10^\circ \dots +15^\circ$) oikean tunkeuman takaamiseksi.
* **🔄 Sauman kääntö:**
  * Kääntää liikeradan suunnan päinvastaiseksi ($A \leftrightarrow B$) yhdellä klikkauksella säilyttäen turvallisen lähestymisen ja poistumisen.

---

### 4. 📦 Automaattinen Pick & Place -apuri (Kouralle & Imukupille)
Yhdellä painikkeella avautuva velho tuottaa täyden 10-vaiheisen kappaleensiirtorutiinin:
1. PTP-lähestyminen nostokorkeudelle
2. LIN-laskeutuminen kappaleeseen
3. Koura kiinni / Imu päälle
4. Odotus / otteen vakiintuminen
5. LIN-nosto pöydästä
6. PTP-siirto asennuspaikan ylle
7. LIN-lasku asennuspintaan
8. Koura auki / Imu pois
9. Odotus / vapautus
10. LIN-nosto turvakorkeuteen

---

### 5. 💻 Reaaliaikainen koodiesikatselu (Live Code Preview)
Ohjelmapuun viereisellä välilehdellä näet heti suoritettavan ohjelman koodin valmiina:
* **Arduino C++ (`.ino`):** Täysi autonominen C++-luonnos, joka voidaan ladata suoraan Arduino Uno R4 WiFiin Arduino IDE:llä ilman PC-yhteyttä.
* **G-Koodi (`.nc`):** Standardi ISO 6983 -koodi CNC-, hitsaus- ja annosteluroboteille.
* **ABB RAPID (`.mod`):** Standardit teollisuuskäskyt (`MoveJ`, `MoveL`, `SetDO`).
* Toiminnot: 1-klikkauksen leikepöydälle kopiointi vihreällä kuitilla ja suora tiedostovienti.

---

### 6. 🎯 3D-työtila & Interaktiivinen Gizmo
* **3D Interactive Jog Gizmo:** Raahaa TCP-palloa hiirellä 3D-avaruudessa. 60 FPS:n suodatus, ulottuvuusrajojen leikkaus ja 70 ms watchdog takaavat, että robotti ei koskaan jumiudu tai karkaa avaruuteen.
* **🎯 Snap TCP:** Klikkaa mitä tahansa kohtaa 3D-työkappaleen pinnalla, ja polttimen tai kouran kärki siirtyy suoraan kohteeseen.
* **Reach Envelope:** Puoliläpinäkyvä 570 mm ulottuvuuspallo visualisoi robotin maksimiulottuvuusalueen.
* **Törmäystarkistus:** Reaaliaikainen pöytätason ($Z < 0$) ja kappaleen törmäysvaroitus varoitusväreillä.

---

### 7. 💾 Projektien tallennus (`.project.json`)
Voit tallentaa koko robottisolun tilan yhdellä kertaa:
* OLP-ohjelman kaikki käskyt, nopeudet ja viiveet
* Aktiivisen työkalun ja sen TCP-koordinaatit
* 3D-työkappaleen (esimerkkikappale tai oma STL) ja sen sijainnin sekä asennon pöydällä
* Robotin nivelasennot

Pikanäppäimet: **`Ctrl + Shift + S`** (Tallenna) ja **`Ctrl + Shift + O`** (Avaa).

---

## ⌨️ Pikanäppäimet (Keyboard Shortcuts)

| Pikanäppäin | Toiminto |
| :--- | :--- |
| **`Space`** | Simulaation käynnistys / pysäytys (Play / Pause) |
| **`Esc`** | **⛔ HÄTÄ-SEIS (E-STOP)** — pysäyttää välittömästi kaiken liikkeen ja sarjaliikenteen |
| **`Ctrl + T`** | Avaa työkalu- ja TCP-hallintadialogin |
| **`Ctrl + H`** | Aja robotti koti-asentoon (Home Pose: 90°, 90°, 90°, 90°, 90°, 40°) |
| **`Ctrl + Shift + S`** | Tallenna koko työtilaprojekti (`.project.json`) |
| **`Ctrl + Shift + O`** | Avaa aiemmin tallennettu työtilaprojekti |
| **`Ctrl + S`** | Tallenna OLP-ohjelma JSON-tiedostoon |
| **`Ctrl + O`** | Avaa OLP-ohjelma JSON-tiedostosta |
| **`Ctrl + Z`** / **`Ctrl + Y`** | Kumoa / Tee uudelleen (Undo / Redo, 40 askeleen puskuri) |
| **`Ctrl + D`** | Monista valittu käsky (Duplicate) |
| **`Delete`** | Poista valittu käsky |

---

## 🛠️ Työkalut & TCP-esiasetukset

Sovellus sisältää valmiit teollisuustyökalut tarkasti lasketuilla TCP-offseteillä:

1. **2-Leukakoura (Standard Gripper):** TCP Z=+99.3 mm, leukakulma 40° (auki) – 128° (kiinni).
2. **Hitsauspoltin (MIG/MAG Torch 45°):** TCP X=+35.0 mm, Z=+135.0 mm, kallistus 45.0°.
3. **Annostelusuutin / Liimakynä (Dispenser):** TCP Z=+115.0 mm.
4. **Imukuppitarttuja (Vacuum Gripper):** TCP Z=+65.0 mm.
5. **Mukautettu työkalu (Custom STL):** Voit tuoda minkä tahansa oman STL-työkalun ja asettaa sille vapaat XYZ- ja kulmaoffsetit.

---

## ⚡ Suorituskyky ja Optimointi

Sovellukseen on tehty laaja teollisuusluokan suorituskykyoptimointi:
* **Yksivaiheinen GPU-renderöinti:** Poistettu turhat tuplarenderöinnit, jolloin 3D-näkymä toimii vakaasti 60 FPS:llä.
* **FXAA (Fast Approximate Anti-Aliasing):** Sahalaitojen poisto ilman GPU-kuormitusta.
* **CSS-välimuisti:** Qt-tyylimäärittelyt päivittyvät vain tilan vaihtuessa, mikä vapauttaa CPU-resursseja.
* **Turvallinen sarjaliikenne:** Taustasäie (`QThread`) hoitaa USB-tiedonsiirron Arduinolle 30 Hz kuristuksella ilman käyttöliittymän nykimistä.

---

## 📁 Tiedostorakenne

```text
digital_twin/
├── assets/
│   └── models/               # Robotin CAD STL -osat (base, link1..link5, gripper)
├── src/
│   ├── config.py             # Mitat, servorajat, baudinopeus ja asetukset
│   ├── communication/
│   │   └── serial_worker.py  # Säieturvallinen QThread pySerial -kommunikaattori
│   ├── core/
│   │   ├── kinematics.py     # 6-DOF matriisikinematiikka, FK, IK ja törmäystarkistus
│   │   ├── program.py        # OLP-ohjelmamalli (PTP, LIN, Gripper, ToolAction, Delay)
│   │   ├── post_processor.py # Kääntäjä: Arduino C++, G-Code, ABB RAPID
│   │   ├── project_manager.py# Koko työtilan tallennus/lataus (.project.json)
│   │   ├── tool_data.py      # Työkalujen ja TCP-profiilien hallinta
│   │   ├── weld_planner.py   # Hitsauksen liikesuunnittelu (Weave, Multi-Pass, Reverse)
│   │   └── sequencer.py      # S-curve liikeratainterpolaatio
│   ├── simulation/
│   │   └── arm_visualizer_3d.py # PyVista 3D-moottori, Gizmo, valot ja radat
│   └── ui/
│       ├── main_window.py    # Pääikkuna, Ribbon-työtilat ja 3D-työkalupalkit
│       ├── statement_tree_panel.py # Ohjelmapuu, dynaamiset työkalupalkit ja velhot
│       ├── code_preview_panel.py   # Reaaliaikainen koodiesikatselupaneeli
│       ├── compact_jog_panel.py    # Kompakti nivel- ja XYZ-ohjauspaneeli
│       ├── simulation_control_panel.py # Simulaation toisto, nopeus ja syklit
│       ├── tool_dialog.py    # Työkalujen ja TCP-offsetien asennusdialogi
│       ├── workpiece_dialog.py# Työkappaleen 3D-asemointidialogi
│       ├── calibration_dialog.py # Servojen fyysinen kalibrointi
│       └── qt_compat.py      # PyQt6 / PyQt5 -yhteensopivuuskerros
├── tests/
│   └── test_digital_twin.py  # Kattavat yksikkö- ja integraatiotestit (20 testiä)
├── main.py                   # Sovelluksen pääkäynnistystiedosto
├── requirements.txt          # Python-riippuvuudet
└── README.md                 # Tämä käyttö- ja kehittäjädokumentaatio
```

---

## 🧪 Testien suoritus

Aja kattavat yksikkö- ja kinematiikkatestit komentoriviltä:
```powershell
$env:PYTHONPATH="digital_twin"
python -m unittest discover -s digital_twin/tests -p "test_*.py"
```
Kaikki 20 testiä suoritetaan automaattisesti alle 0.15 sekunnissa.
