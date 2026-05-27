"""
Konfiguration für den Wave Rover Pfadfolger
============================================
Alle Parameter hier anpassen, bevor das Programm gestartet wird.
"""

# ── Rover Netzwerk ──────────────────────────────────────────────────────────
ROVER_IP   = "192.168.4.1"          # Standard-IP wenn Rover als Hotspot läuft
ROVER_PORT = 80
ROVER_URL  = f"http://{ROVER_IP}:{ROVER_PORT}/js"
HTTP_TIMEOUT = 0.3                  # Sekunden – kurz halten für Echtzeit-Kontrolle

# ── Kamera ──────────────────────────────────────────────────────────────────
CAMERA_INDEX   = 0                  # 0 = erste USB-Kamera (Logitech Brio)
FRAME_WIDTH    = 640                # Auflösung reduzieren für schnellere Verarbeitung
FRAME_HEIGHT   = 480
TARGET_FPS     = 30

# ── Grün-Erkennung (HSV-Farbraum) ───────────────────────────────────────────
# Tipp: Mit dem Debug-Modus die Werte live anpassen
# OpenCV HSV: H=0-180, S=0-255, V=0-255
GREEN_HSV_LOW  = (35,  80,  60)     # Minimaler Hue, Saturation, Value
GREEN_HSV_HIGH = (85, 255, 255)     # Maximaler Hue, Saturation, Value

# Mindestfläche in Pixeln damit ein grüner Bereich als Pfad gilt
MIN_GREEN_AREA = 2000

# ── Region of Interest (ROI) ─────────────────────────────────────────────────
# Anteil des Frames der ausgewertet wird (von unten, da Kamera vorwärts schaut)
# 0.0 = ganz unten, 1.0 = ganzer Frame
ROI_TOP_RATIO    = 0.55             # ROI beginnt bei 55% der Framehöhe
ROI_BOTTOM_RATIO = 0.95             # ROI endet bei 95% (etwas Rand lassen)

# ── Fahrgeschwindigkeiten ────────────────────────────────────────────────────
# Wave Rover Geschwindigkeiten: -1.0 (rückwärts) bis 1.0 (vorwärts)
SPEED_FORWARD   = 0.40             # Grundgeschwindigkeit beim Folgen
SPEED_TURN_MAX  = 0.35             # Maximale Differenz links/rechts beim Lenken
SPEED_SEARCH    = 0.25             # Geschwindigkeit beim Suchen (Rotation)

# ── Lenkung ──────────────────────────────────────────────────────────────────
# Toter Bereich um die Mitte – in diesem Bereich fährt der Rover gerade
DEAD_ZONE_RATIO = 0.10              # 10% der Framebreite links/rechts = "gerade"

# PD-Regler Koeffizienten (P = Proportional, D = Differenzial)
KP = 0.0025                        # Proportionaler Anteil
KD = 0.0005                        # Differenzialer Anteil (dämpft Überschwingen)

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

# ── Verhalten bei Pfadverlust ────────────────────────────────────────────────
SEARCH_TIMEOUT_S   = 5.0           # Nach X Sekunden ohne Pfad: Suche starten
SEARCH_ROTATION    = 0.3           # Rotationsgeschwindigkeit beim Suchen
SEARCH_DIRECTION   = "left"        # "left" oder "right" – erste Suchrichtung

# ── Debug / Visualisierung ───────────────────────────────────────────────────
DEBUG_WINDOW      = True           # Kamerabild mit Overlay anzeigen
DEBUG_SHOW_MASK   = False          # Grün-Maske separat anzeigen
DEBUG_PRINT_SPEED = True           # Geschwindigkeitswerte in der Konsole ausgeben
