"""
Konfiguration für den Wave Rover Pfadfolger
============================================
Alle Parameter hier anpassen, bevor das Programm gestartet wird.
"""

# ── Rover Netzwerk ──────────────────────────────────────────────────────────
ROVER_IP   = "192.168.50.11"          # Standard-IP wenn Rover als Hotspot läuft
ROVER_PORT = 80
ROVER_URL  = f"http://{ROVER_IP}:{ROVER_PORT}/js"
HTTP_TIMEOUT = 0.3                  # Sekunden – kurz halten für Echtzeit-Kontrolle

# ── Kamera ──────────────────────────────────────────────────────────────────
CAMERA_INDEX   = 0                  # 0 = erste USB-Kamera (Logitech Brio)
FRAME_WIDTH    = 1280                # Auflösung reduzieren für schnellere Verarbeitung
FRAME_HEIGHT   = 620
TARGET_FPS     = 30

# ── Grün-Erkennung (HSV-Farbraum) ───────────────────────────────────────────
# Tipp: Mit dem Debug-Modus die Werte live anpassen
# OpenCV HSV: H=0-180, S=0-255, V=0-255
GREEN_HSV_LOW  = (35,  80,  60)     # Minimaler Hue, Saturation, Value
GREEN_HSV_HIGH = (85, 255, 255)     # Maximaler Hue, Saturation, Value

# Mindestfläche in Pixeln damit ein grüner Bereich als Pfad gilt
MIN_GREEN_AREA = 2000

# ── Region of Interest (ROI) ─────────────────────────────────────────────────
# Kamera zeigt NACH UNTEN auf den Boden.
#
# Frame-Koordinaten bei nach-unten-gerichteter Kamera:
#
#   y=0%  ┌────────────────┐ ← FERN-Zone: Pfad knapp vor dem Rover
#         │  (voraus)      │
#   y=40% ├────────────────┤ ← NAH-Zone Anfang
#         │  (unter Rover) │
#   y=75% ├────────────────┤ ← NAH-Zone Ende
#         │  (hinter Rover)│
#   y=90% └────────────────┘ ← ROI-Ende (unterste 10% = bereits abgefahren)
#
# ROI deckt fast den ganzen Frame ab – oben = voraus, unten = bereits hinter dem Rover.
ROI_TOP_RATIO    = 0.03             # Kleiner Rand oben
ROI_BOTTOM_RATIO = 0.90             # Unterste 10% abschneiden (bereits abgefahrener Pfad)

# ── Fahrgeschwindigkeiten ────────────────────────────────────────────────────
# Wave Rover Geschwindigkeiten: -1.0 (rückwärts) bis 1.0 (vorwärts)
SPEED_FORWARD   = 0.20             # Grundgeschwindigkeit beim Folgen
SPEED_TURN_MAX  = 0.10             # Maximale Differenz links/rechts beim Lenken
SPEED_SEARCH    = 0.15             # Geschwindigkeit beim Suchen (Rotation)

# ── Lenkung ──────────────────────────────────────────────────────────────────
# Toter Bereich um die Mitte – in diesem Bereich fährt der Rover gerade
DEAD_ZONE_RATIO = 0.10              # 10% der Framebreite links/rechts = "gerade"

# PD-Regler Koeffizienten (P = Proportional, D = Differenzial)
# offset_normalized ist -1.0…+1.0, daher müssen KP/KD in dieser Größenordnung sein.
KP = 0.55                          # Proportionaler Anteil  (war 0.0025 → viel zu klein!)
KD = 0.10                          # Differenzialer Anteil (dämpft Überschwingen)

# ── Knick-Erkennung & Geschwindigkeitsanpassung ──────────────────────────────
# Der Knickwinkel wird aus dem Unterschied zwischen dem nahen und fernen
# Pfad-Schwerpunkt im ROI berechnet (in Grad).
#
#   0°–BEND_SLOW_DEG   → normale Geschwindigkeit
#   BEND_SLOW_DEG–BEND_STOP_DEG → linear bremsen bis SPEED_MIN_FACTOR
#   > BEND_STOP_DEG    → Stopp + Ausrichtung (ALIGNING)

BEND_SLOW_DEG     = 15.0           # Ab diesem Winkel: langsamer werden
BEND_STOP_DEG     = 32.0           # Ab diesem Winkel: Stopp und Ausrichten
BEND_ALIGN_DEG    =  8.0           # Ausrichtung abgeschlossen wenn Winkel < X°
SPEED_MIN_FACTOR  =  0.30          # Minimaler Geschwindigkeitsfaktor beim Bremsen
                                   # (0.30 = 30% der Grundgeschwindigkeit)
ALIGN_ROTATE_SPD  =  0.22          # Rotationsgeschwindigkeit beim Ausrichten
ALIGN_TIMEOUT_S   =  6.0           # Maximale Ausrichtungszeit (Sicherheits-Stop)

# ── Rotations-Tracking (Rückwärtsfahren verhindern) ──────────────────────────
# Der Rover hat keinen Kompass – die Rotation wird über Zeit × Winkelgeschwindigkeit
# geschätzt. ROTATE_DEG_PER_SEC kalibrieren: Rover 5s drehen lassen und messen
# wie viele Grad er sich gedreht hat, dann durch 5 teilen.
ROTATE_DEG_PER_SEC   = 50.0       # Grad/Sekunde bei turn_in_place (kalibrieren!)

# ALIGNING: Nie mehr als diesen Winkel drehen – sonst rückwärts!
# Pfadkurven sind max. ~90°, daher ist 90° eine sichere Grenze.
MAX_ALIGN_ROTATION_DEG = 90.0

# SEARCHING: Pro Richtung maximal diesen Winkel drehen, dann umkehren.
# < 180° garantiert, dass der Rover nie rückwärts schaut.
MAX_SEARCH_ROTATION_DEG = 150.0

# ── Verhalten bei Pfadverlust ────────────────────────────────────────────────
SEARCH_TIMEOUT_S   = 5.0           # Nach X Sekunden ohne Pfad: Suche starten
SEARCH_ROTATION    = 0.3           # Rotationsgeschwindigkeit beim Suchen
SEARCH_DIRECTION   = "left"        # "left" oder "right" – erste Suchrichtung

# ── Debug / Visualisierung ───────────────────────────────────────────────────
DEBUG_WINDOW      = False          # OpenCV-Fenster (nur mit Monitor am Pi sinnvoll)
DEBUG_SHOW_MASK   = False          # Grün-Maske als zweites OpenCV-Fenster
DEBUG_PRINT_SPEED = True           # Geschwindigkeitswerte in der Konsole ausgeben

# ── Web-Debug-Server ──────────────────────────────────────────────────────────
# Öffne im Browser:  http://192.168.4.1:5000  (wenn Rover als Hotspot läuft)
DEBUG_WEB_SERVER  = True           # Web-Dashboard aktivieren
DEBUG_SERVER_PORT = 5000           # Port des Web-Servers
DEBUG_STREAM_FPS  = 15             # Stream-FPS (15 reicht, spart Bandbreite)
