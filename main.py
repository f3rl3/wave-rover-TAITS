"""
Wave Rover – Grüner-Pfad-Folger
=================================
Hauptprogramm: Verbindet Kamera, Pfaderkennung und Rover-Steuerung.

Zustandsmaschine:
                    Pfad gefunden
         ┌──────────────────────────────────────┐
         │                                      ▼
      [PAUSED] ──P──► [FOLLOWING] ──Knick≥STOP──► [ALIGNING]
                           │                         │
                           │  Pfad verloren          │ ausgerichtet
                           ▼                         │
                      [SEARCHING] ◄──────────────────┘
                           │  Pfad gefunden
                           └──────────────────────────────► [FOLLOWING]

Tastenkürzel im Debug-Fenster:
    Q / ESC → Beenden
    P       → Pause (Rover stoppt, Bild läuft weiter)
    M       → Grün-Maske ein-/ausblenden
    +/-     → Grundgeschwindigkeit erhöhen/verringern
"""

import cv2
import time
import logging
import sys

from config import (
    CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, TARGET_FPS,
    SPEED_FORWARD, SPEED_TURN_MAX,
    SEARCH_TIMEOUT_S, SEARCH_ROTATION, SEARCH_DIRECTION,
    KP, KD,
    BEND_STOP_DEG, BEND_ALIGN_DEG,
    ALIGN_ROTATE_SPD, ALIGN_TIMEOUT_S,
    DEBUG_WINDOW, DEBUG_SHOW_MASK, DEBUG_PRINT_SPEED,
)
from rover_controller import RoverController
from path_detector    import PathDetector

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")


# ── Zustände ─────────────────────────────────────────────────────────────────
class State:
    FOLLOWING = "FOLLOWING"   # Pfad folgen
    ALIGNING  = "ALIGNING"    # Stopp + Ausrichten bei scharfem Knick
    SEARCHING = "SEARCHING"   # Pfad verloren, suchen
    PAUSED    = "PAUSED"      # Manuell pausiert


# ── Hilfsfunktionen ───────────────────────────────────────────────────────────

def open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)   # Linux / Raspberry Pi
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        logger.error("Kamera %d konnte nicht geöffnet werden!", CAMERA_INDEX)
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    logger.info("Kamera: %dx%d @ %.0f FPS", actual_w, actual_h, actual_fps)
    return cap


STATE_COLORS = {
    State.FOLLOWING: (0, 220, 0),
    State.ALIGNING:  (0, 100, 255),
    State.SEARCHING: (0, 165, 255),
    State.PAUSED:    (0, 0, 220),
}

def draw_hud(frame, state: str, speed: float, extra: str = ""):
    """Zeichnet Status-HUD unten im Frame."""
    h, w = frame.shape[:2]
    color = STATE_COLORS.get(state, (200, 200, 200))

    # Hintergrundbalken
    cv2.rectangle(frame, (0, h - 55), (w, h), (30, 30, 30), -1)

    label = f"[{state}]"
    if extra:
        label += f"  {extra}"
    cv2.putText(frame, label, (10, h - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, color, 2)
    cv2.putText(frame, f"Speed-Basis: {speed:.2f}   Q=Beenden  P=Pause  +/-=Speed  M=Maske",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)


# ── Hauptprogramm ─────────────────────────────────────────────────────────────

def main():
    logger.info("=" * 55)
    logger.info("  Wave Rover – Grüner Pfad Folger")
    logger.info("=" * 55)

    # Rover verbinden
    rover = RoverController()
    if not rover.connect():
        answer = input("Rover nicht erreichbar. Trotzdem fortfahren (nur Kamera)? [j/N]: ")
        if answer.strip().lower() not in ("j", "y", "ja", "yes"):
            sys.exit(1)
        logger.warning("Simulation-Modus – keine Rover-Befehle werden gesendet")

    cap      = open_camera()
    detector = PathDetector()

    # ── Zustandsvariablen ─────────────────────────────────────────────────────
    state          = State.FOLLOWING
    base_speed     = SPEED_FORWARD
    search_dir     = SEARCH_DIRECTION
    show_mask      = DEBUG_SHOW_MASK
    prev_error     = 0.0

    last_seen_t    = time.time()    # Zeitpunkt: Pfad zuletzt gesehen
    search_flip_t  = time.time()    # Zeitpunkt: letzte Suchrichtungsumkehr
    align_start_t  = 0.0            # Zeitpunkt: Ausrichtung begonnen
    align_dir      = "left"         # Richtung der aktuellen Ausrichtung

    frame_count    = 0
    fps_t          = time.time()
    fps_display    = 0.0

    logger.info("▶  Läuft – grünen Streifen vor die Kamera legen.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.03)
                continue

            frame_count += 1
            now = time.time()

            # ── Pfaderkennung ─────────────────────────────────────────────────
            result, debug_frame = detector.process(frame)

            # ── Zustandsmaschine ──────────────────────────────────────────────
            hud_extra = ""

            if state == State.PAUSED:
                rover.stop()
                hud_extra = "P drücken zum Fortfahren"

            elif state == State.ALIGNING:
                # ── ALIGNING: Stopp → drehen bis Knick weg ───────────────────
                elapsed = now - align_start_t

                if not result.found:
                    # Pfad während Ausrichtung verloren → Suche
                    logger.warning("Pfad während Ausrichtung verloren – wechsle zu SEARCHING")
                    state = State.SEARCHING
                    rover.stop()
                elif result.bend_angle_deg < BEND_ALIGN_DEG:
                    # Erfolgreich ausgerichtet
                    logger.info("✅ Ausgerichtet (Winkel=%.1f°) – weiterfahren", result.bend_angle_deg)
                    state      = State.FOLLOWING
                    prev_error = 0.0
                elif elapsed > ALIGN_TIMEOUT_S:
                    # Timeout – trotzdem weiterfahren
                    logger.warning("Ausrichtungs-Timeout (%.1fs) – fahre trotzdem weiter", elapsed)
                    state = State.FOLLOWING
                else:
                    # Weiter drehen in Richtung des Knicks
                    rover.turn_in_place(ALIGN_ROTATE_SPD, direction=align_dir)
                    remaining = ALIGN_TIMEOUT_S - elapsed
                    hud_extra = (f"dreht {align_dir.upper()}  "
                                 f"Winkel={result.bend_angle_deg:.1f}°  "
                                 f"Timeout in {remaining:.1f}s")

                    if DEBUG_PRINT_SPEED:
                        logger.debug("ALIGN  dir=%s  winkel=%.1f°", align_dir, result.bend_angle_deg)

            elif state == State.SEARCHING:
                # ── SEARCHING: Pfad verloren → drehen und suchen ──────────────
                if result.found:
                    logger.info("Pfad wiedergefunden – weiterfahren")
                    state      = State.FOLLOWING
                    last_seen_t = now
                    prev_error  = 0.0
                else:
                    # Alle 3 s Richtung wechseln
                    if now - search_flip_t > 3.0:
                        search_dir    = "right" if search_dir == "left" else "left"
                        search_flip_t = now
                        logger.info("Suchrichtung → %s", search_dir)
                    rover.turn_in_place(SEARCH_ROTATION, direction=search_dir)
                    hud_extra = f"suche  dir={search_dir.upper()}"

            else:
                # ── FOLLOWING: Pfad gefunden → folgen ────────────────────────
                if not result.found:
                    lost = now - last_seen_t
                    if lost < SEARCH_TIMEOUT_S * 0.3:
                        rover.forward(base_speed * 0.4)   # kurz weiterrollen
                    else:
                        logger.info("Pfad verloren (%.1fs) – wechsle zu SEARCHING", lost)
                        state = State.SEARCHING
                        rover.stop()
                else:
                    last_seen_t = now

                    # Scharfer Knick → ALIGNING starten
                    if result.is_sharp_bend:
                        logger.info(
                            "⚠ Scharfer Knick erkannt (%.1f°, %s) – halte an und richte aus",
                            result.bend_angle_deg, result.bend_direction
                        )
                        state         = State.ALIGNING
                        align_start_t = now
                        align_dir     = result.bend_direction if result.bend_direction != "none" else "left"
                        rover.stop()
                        time.sleep(0.15)   # kurz stehenbleiben bevor Rotation
                    else:
                        # Geschwindigkeit anpassen: langsamer bei Kurve
                        current_speed = base_speed * result.speed_factor
                        prev_error    = result.offset_normalized

                        if result.in_dead_zone:
                            rover.forward(current_speed)
                        else:
                            rover.steer(
                                current_speed,
                                result.offset_normalized,
                                turn_max=SPEED_TURN_MAX,
                                kp=KP, kd=KD,
                                prev_error=prev_error,
                            )

                        if result.speed_factor < 1.0:
                            hud_extra = f"Kurve {result.bend_angle_deg:.0f}° → speed x{result.speed_factor:.2f}"

                        if DEBUG_PRINT_SPEED:
                            logger.debug(
                                "FOLLOW  off=%+.3f  winkel=%.1f°  sf=%.2f  spd=%.2f",
                                result.offset_normalized, result.bend_angle_deg,
                                result.speed_factor, current_speed
                            )

            # ── FPS ───────────────────────────────────────────────────────────
            if frame_count % 30 == 0:
                fps_display = 30.0 / max(now - fps_t, 1e-9)
                fps_t = now
            cv2.putText(debug_frame,
                        f"FPS {fps_display:.1f}",
                        (debug_frame.shape[1] - 95, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # ── Debug-Ausgabe ──────────────────────────────────────────────────
            if DEBUG_WINDOW:
                draw_hud(debug_frame, state, base_speed, hud_extra)
                cv2.imshow("Wave Rover – Pfadfolger", debug_frame)

                if show_mask:
                    cv2.imshow("Grün-Maske", detector.get_mask_only(frame))
                else:
                    try:
                        cv2.destroyWindow("Grün-Maske")
                    except Exception:
                        pass

                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), ord('Q'), 27):
                    logger.info("Benutzer hat beendet.")
                    break
                elif key in (ord('p'), ord('P')):
                    state = State.PAUSED if state != State.PAUSED else State.FOLLOWING
                    logger.info("Zustand: %s", state)
                elif key in (ord('m'), ord('M')):
                    show_mask = not show_mask
                elif key == ord('+'):
                    base_speed = min(0.90, base_speed + 0.05)
                    logger.info("Geschwindigkeit: %.2f", base_speed)
                elif key == ord('-'):
                    base_speed = max(0.10, base_speed - 0.05)
                    logger.info("Geschwindigkeit: %.2f", base_speed)
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("Ctrl+C – beende.")
    finally:
        logger.info("Stoppe Rover…")
        rover.stop()
        time.sleep(0.2)
        cap.release()
        cv2.destroyAllWindows()
        logger.info("Fertig.")


if __name__ == "__main__":
    main()
