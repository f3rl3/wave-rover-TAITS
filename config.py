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
FRAME_WIDTH    = 640                # 640×480 reicht für Pfaderkennung; 1280×620 kostet 3× mehr CPU
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
# Toter Bereich um die Mitte – in diesem Bereich fährt der Rover geradeaus.
#
# Ziel: Pfad stets im MITTLEREN DRITTEL des Bildes halten.
#   Mittleres Drittel = Bereich x ∈ [W/3 … 2W/3] um die Bildmitte.
#   Abstand zur Mitte: W/6 = Frame_Width × (1/6) ≈ 0.167
#
#   DEAD_ZONE_RATIO = 0.167  → Rover fährt gerade wenn Pfad im mittleren Drittel
#   DEAD_ZONE_RATIO = 0.10   → ältere engere Einstellung (nur zentrales Fünftel)
DEAD_ZONE_RATIO = 0.167             # mittleres Drittel = ±1/6 der Framebreite

# PD-Regler Koeffizienten (P = Proportional, D = Differenzial)
# offset_normalized ist -1.0…+1.0, daher müssen KP/KD in dieser Größenordnung sein.
KP = 0.55                          # Proportionaler Anteil  (war 0.0025 → viel zu klein!)
KD = 0.10                          # Differenzialer Anteil (dämpft Überschwingen)

# ── Streifen-Ausrichtung (Winkelkorrektur) ────────────────────────────────────
# Drei-Phasen-Lenkung:
#   Phase 1 – ANNÄHERN  (Streifen außerhalb Mitte): nur lateral lenken, vorwärts.
#   Phase 2 – AUSRICHTEN (Streifen zentriert, Winkel zu groß): AUF DER STELLE drehen.
#             Rover stoppt Vorwärtsbewegung und dreht sich bis Streifen senkrecht steht.
#   Phase 3 – FOLGEN     (Streifen mittig + ausgerichtet): normale Fahrt + Knick-Check.
#
# STRIPE_ALIGN_TOL_DEG:  Phase-2-EINGANG: Rover dreht sich auf der Stelle wenn Winkel > X°.
# STRIPE_ALIGN_EXIT_DEG: Phase-2-AUSGANG: Rover fährt erst weiter wenn Winkel < X°.
#   Hysterese: Eingang > Ausgang verhindert Pendeln an der Schwelle.
#   Beispiel: Rover dreht wenn > 12°, hört erst auf wenn < 6° → keine Zickzack-Bewegung.
# STRIPE_ALIGN_SPD:      Rotationsgeschwindigkeit beim Auf-der-Stelle-Drehen (Phase 2).
STRIPE_ALIGN_TOL_DEG  = 12.0         # Phase-2-Eingang (Grad)
STRIPE_ALIGN_EXIT_DEG =  6.0         # Phase-2-Ausgang (Grad) – strenger als Eingang!
STRIPE_ALIGN_SPD      =  0.18        # Drehgeschwindigkeit Phase 2 (0.0–1.0)

# Nicht mehr benötigt (wurde durch auf-der-Stelle-Drehen ersetzt):
# STRIPE_ALIGN_KP      = 0.30
STRIPE_ALIGN_KP      = 0.30          # Rückwärtskompatibilität – wird in main.py nicht mehr verwendet

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
# Pfadkurven sind max. ~90°, daher ist 100° eine sichere Grenze.
MAX_ALIGN_ROTATION_DEG = 180.0

# SEARCHING: Pro Richtung maximal diesen Winkel drehen, dann umkehren.
# < 180° garantiert, dass der Rover nie rückwärts schaut.
MAX_SEARCH_ROTATION_DEG = 180.0

# ── Verhalten bei Pfadverlust ────────────────────────────────────────────────
SEARCH_TIMEOUT_S   = 5.0           # Nach X Sekunden ohne Pfad: Suche starten
SEARCH_ROTATION    = 0.3           # Rotationsgeschwindigkeit beim Suchen
SEARCH_DIRECTION   = "left"        # "left" oder "right" – erste Suchrichtung

# ── Rote Stop-Markierung ─────────────────────────────────────────────────────
# Der Rover hält auf der ersten roten Fläche an und wartet auf ein Signal.
# Dann dreht er 180° und fährt zurück. Auf der zweiten roten Fläche terminiert er.
#
# RED_STOP_WAIT_S:      Dummy-Wartezeit in Sekunden (später: echtes Signal-Modul).
# TURN_180_SPD:         Rotationsgeschwindigkeit beim 180°-Drehen (0.0–1.0).
#
# RED_DETECT_Y_START:   Rot wird nur erkannt wenn es im UNTEREN Teil des Frames ist.
#                       Wert ist ein Anteil der Frame-Höhe (0.0 = ganz oben, 1.0 = unten).
#                       0.55 → roter Bereich muss ab 55 % Frame-Höhe sichtbar sein.
#                       Kamera zeigt nach unten: y > 55 % ≈ direkt unter / hinter Rover.
#                       Damit hält der Rover erst wenn er AUF der Markierung steht,
#                       nicht schon wenn er sie von weitem sieht.
#
# RED_COOLDOWN_S:       Sperre nach dem Wiederanfahren – Rot wird für diese Dauer
#                       NICHT erkannt. Verhindert sofortige Re-Auslösung während
#                       der Rover noch über die erste Markierung hinwegfährt.
RED_STOP_WAIT_S    = 10.0            # Sekunden bis "Signal" empfangen (Dummy)
TURN_180_SPD       = 0.20            # Rotationsgeschwindigkeit für 180°-Drehung
RED_DETECT_Y_START = 0.55            # Rot nur im unteren Teil (≥ 55 % Frame-Höhe)
RED_COOLDOWN_S     = 15.0            # Sekunden Rot-Sperre nach dem Wiederanfahren

# ── Debug / Visualisierung ───────────────────────────────────────────────────
DEBUG_WINDOW      = False          # OpenCV-Fenster (nur mit Monitor am Pi sinnvoll)
DEBUG_SHOW_MASK   = False          # Grün-Maske als zweites OpenCV-Fenster
DEBUG_PRINT_SPEED = True           # Geschwindigkeitswerte in der Konsole ausgeben

# ── Web-Debug-Server ──────────────────────────────────────────────────────────
# Öffne im Browser:  http://192.168.4.1:5000  (wenn Rover als Hotspot läuft)
DEBUG_WEB_SERVER  = True           # Web-Dashboard aktivieren
DEBUG_SERVER_PORT = 5000           # Port des Web-Servers
DEBUG_STREAM_FPS  = 15             # Stream-FPS (15 reicht, spart Bandbreite)
