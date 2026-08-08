# --- Rover Netzwerk ---
ROVER_IP   = "192.168.50.11"
ROVER_PORT = 80
ROVER_URL  = f"http://{ROVER_IP}:{ROVER_PORT}/js"
HTTP_TIMEOUT = 0.3

# --- Kamera --- 
CAMERA_INDEX   = 0                  # 0 = erste USB-Kamera
FRAME_WIDTH    = 640
FRAME_HEIGHT   = 480
TARGET_FPS     = 30

# --- Grün-Erkennung (HSV-Farbraum) --- 
GREEN_HSV_LOW  = (35,  80,  60)     # Minimaler Hue, Saturation, Value
GREEN_HSV_HIGH = (85, 255, 255)

MIN_GREEN_AREA = 2000 # in Pixel

ROI_TOP_RATIO    = 0.03             # Kleiner Rand oben
ROI_BOTTOM_RATIO = 0.90             # Unterste 10% abschneiden

# --- Fahrgeschwindigkeiten ---
# Wave Rover Geschwindigkeiten: -1.0 bis 1.0
SPEED_FORWARD   = 0.20
SPEED_TURN_MAX  = 0.10
SPEED_SEARCH    = 0.15

# --- Lenkung ---
# in diesem Bereich fährt der Rover geradeaus
DEAD_ZONE_RATIO = 0.167

# PD-Regler Koeffizienten
KP = 0.55                          # Proportionaler Anteil
KD = 0.10                          # Differenzialer Anteil (dämpft Überschwingen)

# --- Streifen-Ausrichtung (Winkelkorrektur) ---
STRIPE_ALIGN_TOL_DEG  = 12.0         # Phase-2-EINGANG  (Grad): dreht wenn Winkel > X°
STRIPE_ALIGN_EXIT_DEG =  6.0         # Phase-2-AUSGANG  (Grad): fährt weiter wenn Winkel < X°
STRIPE_ALIGN_TIMEOUT_S = 1.5         # Deadlock-Schutz: nach X Sek. Phase 2 abbrechen
STRIPE_ALIGN_BOOST_S   = 1.5         # Nach Timeout: X Sek. volle Geschwindigkeit fahren
STRIPE_ALIGN_SPD      =  0.18        # Drehgeschwindigkeit Phase 2 (0.0–1.0)

# --- Stuck-Erkennung ---
# Wenn der Rover X Sekunden lang keine Vorwärtsbewegung macht,
# wird das Pathfinding komplett zurückgesetzt.
STUCK_RESET_S = 10.0                 # Sekunden bis Pathfinding-Reset

# --- Knick-Erkennung & Geschwindigkeitsanpassung ---
BEND_SLOW_DEG     = 15.0           # Ab diesem Winkel: langsamer werden
BEND_STOP_DEG     = 32.0           # Ab diesem Winkel: Stopp und Ausrichten
BEND_ALIGN_DEG    =  8.0           # Ausrichtung abgeschlossen wenn Winkel < X°
SPEED_MIN_FACTOR  =  0.30          # Minimaler Geschwindigkeitsfaktor beim Bremsen
ALIGN_ROTATE_SPD  =  0.22          # Rotationsgeschwindigkeit beim Ausrichten
ALIGN_TIMEOUT_S   =  6.0           # Maximale Ausrichtungszeit

# --- Rotations-Tracking ---
ROTATE_DEG_PER_SEC   = 50.0       # Grad/Sekunde bei turn_in_place

# ALIGNING: Nie mehr als diesen Winkel drehen
MAX_ALIGN_ROTATION_DEG = 180.0

# SEARCHING: Pro Richtung maximal diesen Winkel drehen, dann umkehren
MAX_SEARCH_ROTATION_DEG = 180.0

# --- Verhalten bei Pfadverlust ---
SEARCH_TIMEOUT_S   = 5.0           # Nach X Sekunden ohne Pfad: Suche starten
SEARCH_ROTATION    = 0.3           # Rotationsgeschwindigkeit beim Suchen
SEARCH_DIRECTION   = "left"        # "left" oder "right" – erste Suchrichtung

# --- Rote Stop-Markierung ---
RED_STOP_WAIT_S    = 10.0            # Sekunden bis "Signal" empfangen (Dummy)
TURN_180_SPD       = 0.20            # Rotationsgeschwindigkeit für 180°-Drehung
RED_DETECT_Y_START = 0.55            # Rot nur im unteren Teil (≥ 55 % Frame-Höhe)
RED_COOLDOWN_S     = 15.0            # Sekunden Rot-Sperre nach dem Wiederanfahren

# --- Debug / Visualisierung ---
DEBUG_WINDOW      = False          # OpenCV-Fenster (nur mit Monitor am Pi sinnvoll)
DEBUG_SHOW_MASK   = False          # Grün-Maske als zweites OpenCV-Fenster
DEBUG_PRINT_SPEED = True           # Geschwindigkeitswerte in der Konsole ausgeben

# --- Web-Debug-Server ---
# aufrufbar unter: http://192.168.4.1:5000
DEBUG_WEB_SERVER  = True
DEBUG_SERVER_PORT = 5000
DEBUG_STREAM_FPS  = 15