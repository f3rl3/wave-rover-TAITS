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
