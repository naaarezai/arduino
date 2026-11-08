# 🦾 6-DOF Robot Arm Digital Twin & OLP -järjestelmä

Tämä on **Python + PyQt + PyVista** -pohjainen teollisuustason työpöytäsovellus (Digital Twin / Offline Programming OLP), jolla voidaan simuloida ja ohjata 6-akselista robottikäsivartta.

---

## 🌟 Pääominaisuudet

1. **Offline-tila (Simulaatio):**
   * Täysi interaktiivinen 3D-ympäristö PyVistalla: vapaa kameran pyöritys, zoomaus ja panarointi hiirellä.
   * Kameran pikapainikkeet: Isometrinen 3D, Ylhäältä (XY) ja Sivulta (XZ).
   * 6 liukusäädintä (J1–J6) ja numeeriset astekentät (0°–180°).
   * Reaaliaikainen **käänteinen kinematiikka ja Tool Center Point (TCP)**: näet jatkuvasti kouran $(X, Y, Z)$ -millimetrikoordinaatit ja pitch-kulman.
   * Koura (J6) on mekaanisesti rajattu turvallisesti välille `40° – 128°`.
2. **Online-tila (Fyysinen ohjaus):**
   * Yhdistää Arduinon USB-sarjaporttiin nopeudella **115200 baud**.
   * Säieturvallinen taustaprosessi (`QThread`), joka ei koskaan jäädytä käyttöliittymää.
   * Lähettää standardia ASCII-pakettia: `<J1,J2,J3,J4,J5,J6>\n`.
   * Puskurin kuristus (Throttling ~30 Hz) estää sarjapuskurin ruuhkautumisen.
   * **Vikaturvallinen:** Jos USB-johto irrotetaan, sovellus palaa automaattisesti Offline-tilaan kaatumatta.
3. **Liikesarjasekvensseri (Teach & Play):**
   * Tallenna nykyinen asento yhdellä painikkeella (`➕ Tallenna nykyinen asento`).
   * Aseta siirtymän kesto millisekunteina ja viive asennossa.
   * `▶️ Simuloi ruudulla`: ajaa liikesarjan läpi sulavalla kosini-interpolaatiolla (S-curve).
   * Tallenna ja avaa liikesarjoja `.json`-tiedostoina.

---

## 🚀 Käynnistys

Voit käynnistää sovelluksen kahdella tavalla:

### Tapa 1: Kaksoisnapsautus
Tuplaklikkaa suoraan projektin juurikansiosta:
👉 **`run_digital_twin.bat`**

### Tapa 2: Komentoriviltä
```bash
cd digital_twin
python main.py
```

---

## 📂 Omien STL CAD -osien lisääminen

Kun olet tallentanut Inventorista osat STL-muotoon, sijoita ne kansioon:
`digital_twin/assets/models/`

Nimeä tiedostot seuraavasti:
* `base.stl` — Kiinteä pöytäjalusta
* `link1.stl` — Jalustan kääntyvä alusta (J1)
* `link2.stl` — Olkavarsi (J2)
* `link3.stl` — Kyynärvarsi (J3)
* `link4.stl` — Ranne pysty (J4)
* `link5.stl` — Ranne kierto (J5)
* `gripper.stl` — Koura / tarttuja (J6)

Kun tiedostot löytyvät kansiosta, sovellus lataa automaattisesti oikeat CAD-mallit väliaikaisten geometristen tilapäismallien tilalle!

---

## 📁 Tiedostorakenne

```text
digital_twin/
├── assets/
│   └── models/               # Tähän sijoitetaan STL-tiedostot
├── src/
│   ├── config.py             # Mitat, servorajat, baudinopeus, polut
│   ├── communication/
│   │   └── serial_worker.py  # QThread pySerial taustasäie
│   ├── core/
│   │   ├── kinematics.py     # 4x4 muunnosmatriisit ja Forward Kinematics
│   │   └── sequencer.py      # Opetussekvensseri ja S-curve interpolaatio
│   ├── simulation/
│   │   └── arm_visualizer_3d.py # PyVista 3D-moottori ja visualisointi
│   └── ui/
│       ├── qt_compat.py      # PyQt6 / PyQt5 -yhteensopivuuskerros
│       ├── main_window.py    # Pääikkuna
│       ├── connection_bar.py # COM-valikko ja tilarivi
│       ├── slider_panel.py   # Liukusäätimet ja esiasennot
│       └── sequencer_panel.py# Liikesarjojen opetus ja ajo
├── main.py                   # Käynnistystiedosto
├── requirements.txt          # Riippuvuudet
└── README.md                 # Tämä ohje
```
