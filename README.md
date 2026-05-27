# Wave Rover – Grüner Pfad Folger

Der Wave Rover folgt automatisch einem grünen Streifen auf dem Boden.

## Setup

### 1. Abhängigkeiten installieren
```bash
pip install -r requirements.txt
```

### 2. Konfiguration anpassen (`config.py`)
| Parameter | Beschreibung | Standard |
|---|---|---|
| `ROVER_IP` | IP-Adresse des Rovers | `192.168.4.1` |
| `CAMERA_INDEX` | Kamera-Index | `0` |
| `SPEED_FORWARD` | Grundgeschwindigkeit (0.0–1.0) | `0.40` |
| `GREEN_HSV_LOW/HIGH` | HSV-Bereich für Grün | `(35,80,60)–(85,255,255)` |

### 3. Starten
```bash
# Mit dem Rover verbunden (Rover-WLAN aktiv)
python main.py

# Ohne Rover (nur Kamera-Debug)
# → Bei Verbindungsfehler "j" eingeben
```

## Grün-Farbe kalibrieren

Falls der Pfad nicht erkannt wird, `GREEN_HSV_LOW` und `GREEN_HSV_HIGH` in `config.py` anpassen:

```python
# Grünes Klebeband (satt grün):
GREEN_HSV_LOW  = (35,  80,  60)
GREEN_HSV_HIGH = (85, 255, 255)

# Helleres Gelbgrün:
GREEN_HSV_LOW  = (25,  60,  80)
GREEN_HSV_HIGH = (75, 255, 255)
```

Mit `M`-Taste im Debug-Fenster die Maske einblenden → grüner Bereich sollte weiß werden.

## Tastenkürzel
| Taste | Funktion |
|---|---|
| `Q` / `ESC` | Beenden |
| `P` | Pause (Rover stoppt) |
| `M` | Grün-Maske einblenden |
| `+` | Geschwindigkeit erhöhen (+0.05) |
| `-` | Geschwindigkeit verringern (-0.05) |

## Dateistruktur
```
wave_rover_follower/
├── main.py              # Hauptprogramm + Zustandsmaschine
├── path_detector.py     # Grün-Erkennung mit OpenCV
├── rover_controller.py  # HTTP-API für den Wave Rover
├── config.py            # Alle Einstellungen
└── requirements.txt
```

## Zustandsmaschine
```
         Pfad gefunden
    ┌─────────────────────┐
    │                     ▼
[PAUSED] ──P──► [FOLLOWING] ──Pfad verloren──► [SEARCHING]
    ▲                                               │
    │                     Pfad gefunden             │
    └───────────────────────────────────────────────┘
```

## Wave Rover HTTP-API
Der Rover wird über `POST http://192.168.4.1/js` gesteuert:
```json
{"T": 1, "L": 0.5, "R": 0.5}
```
- `T=1` → Motorsteuerung
- `L`/`R` → Linke/Rechte Seite: `-1.0` (zurück) bis `1.0` (vorwärts)
