"""
Wave Rover – Grüner-Pfad-Folger
=================================
Hauptprogramm: Verbindet Kamera, Pfaderkennung und Rover-Steuerung.

Starten:
    python main.py

Tastenkürzel im Debug-Fenster:
    Q oder ESC → Beenden
    P          → Pause (Rover stoppt, Kamerabild läuft weiter)
    M          → Grün-Maske ein-/ausblenden
    +/-        → Grundgeschwindigkeit erhöhen/verringern
"""

import cv2
import time
import logging
import sys
from pathlib import Path

# Eigene Module
from config import (
    CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, TARGET_FPS,
    SPEED_FORWARD, SPEED_TURN_MAX, SPEED_SEARCH, SPEED_SEARCH as SEARCH_SPEED,
    SEARCH_TIMEOUT_S, SEARCH_ROTATION, SEARCH_DIRECTION,
    KP, KD,
    DEBUG_WINDOW, DEBUG_SHOW_MASK, DEBUG_PRINT_SPEED,
)
from rover_controller import RoverController
from path_detector    import PathDetector

# ── Logging einrichten ───────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")


# ── Zustände der Zustandsmaschine ────────────────────────────────────────────
class State:
    FOLLOWING = "FOLLOWING"    # Pfad gefunden → folgen
    SEARCHING = "SEARCHING"    # Pfad verloren → suchen
    PAUSED    = "PAUSED"       # Manuell pausiert


def open_camera() -> cv2.VideoCapture:
    """Öffnet die Kamera und setzt Auflösung + FPS."""
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)  # CAP_DSHOW für Windows
    if not cap.isOpened():
        # Fallback ohne Backend-Flag
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        logger.error("Kamera %d konnte nicht geöffnet werden!", CAMERA_INDEX)
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    # Puffergröße auf 1 setzen → immer aktuellster Frame
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    logger.info("Kamera geöffnet: %dx%d @ %.0f FPS", actual_w, actual_h, actual_fps)
    return cap


def draw_state_overlay(frame, state: str, speed: float, search_dir: str):
    """Zeichnet Zustand und Geschwindigkeit oben rechts."""
    h, w = frame.shape[:2]
    color_map = {
        State.FOLLOWING: (0, 255, 0),
        State.SEARCHING: (0, 165, 255),
        State.PAUSED:    (0, 0, 255),
    }
    color = color_map.get(state, (255, 255, 255))

    label = f"[{state}]"
    if state == State.SEARCHING:
        label += f"  dir={search_dir}"
    cv2.putText(frame, label, (10, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.putText(frame, f"Speed: {speed:.2f}", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    # Steuerungshinweise
    hints = "Q=Beenden  P=Pause  M=Maske  +/-=Speed"
    cv2.putText(frame, hints, (w - 340, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)


def main():
    logger.info("=" * 55)
    logger.info("  Wave Rover – Grüner Pfad Folger")
    logger.info("=" * 55)

    # ── Rover verbinden ──────────────────────────────────────────────────
    rover = RoverController()
    logger.info("Verbinde mit Rover (%s)…", rover._session.headers)
    if not rover.connect():
        answer = input("Rover nicht erreichbar. Trotzdem fortfahren (Simulation)? [j/N]: ")
        if answer.strip().lower() not in ("j", "y", "ja", "yes"):
            sys.exit(1)
        logger.warning("Rover-Verbindung übersprungen – Simulation-Modus")

    # ── Kamera öffnen ────────────────────────────────────────────────────
    cap       = open_camera()
    detector  = PathDetector()

    # ── Zustandsvariablen ────────────────────────────────────────────────
    state           = State.FOLLOWING
    base_speed      = SPEED_FORWARD
    search_dir      = SEARCH_DIRECTION
    show_mask       = DEBUG_SHOW_MASK
    prev_error      = 0.0
    last_seen_time  = time.time()
    search_flip_t   = time.time()   # Zeitpunkt des letzten Richtungswechsels
    frame_count     = 0
    fps_time        = time.time()
    fps_display     = 0.0

    logger.info("▶  Gestartet – Grünen Pfad vor die Kamera legen.")

    try:
        while True:
            # ── Frame lesen ──────────────────────────────────────────────
            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning("Kein Frame erhalten – überspringe.")
                time.sleep(0.05)
                continue

            frame_count += 1

            # ── Pfaderkennung ─────────────────────────────────────────────
            result, debug_frame = detector.process(frame)

            # ── Zustandsmaschine ──────────────────────────────────────────
            now = time.time()

            if state != State.PAUSED:
                if result.found:
                    # ── Pfad gefunden: FOLLOWING ──────────────────────────
                    state          = State.FOLLOWING
                    last_seen_time = now
                    prev_error     = result.offset_normalized

                    if result.in_dead_zone:
                        # Gerade genug → Geradeaus fahren
                        rover.forward(base_speed)
                        if DEBUG_PRINT_SPEED:
                            logger.debug("GERADE  offset=%.3f", result.offset_normalized)
                    else:
                        # Lenken: PD-Regler
                        rover.steer(
                            base_speed,
                            result.offset_normalized,
                            turn_max=SPEED_TURN_MAX,
                            kp=KP,
                            kd=KD,
                            prev_error=prev_error,
                        )
                        if DEBUG_PRINT_SPEED:
                            logger.debug(
                                "LENKE   offset=%+.3f  dir=%s",
                                result.offset_normalized,
                                "RECHTS" if result.offset_normalized > 0 else "LINKS",
                            )

                else:
                    # ── Pfad verloren ──────────────────────────────────────
                    lost_secs = now - last_seen_time

                    if lost_secs < SEARCH_TIMEOUT_S * 0.3:
                        # Kurz verloren → kurz weiterfahren (Inertia)
                        rover.forward(base_speed * 0.5)
                    else:
                        # Länger verloren → Suche starten
                        state = State.SEARCHING

                        # Alle 3 Sekunden Suchrichtung wechseln
                        if now - search_flip_t > 3.0:
                            search_dir   = "right" if search_dir == "left" else "left"
                            search_flip_t = now
                            logger.info("Suchrichtung gewechselt → %s", search_dir)

                        rover.turn_in_place(SEARCH_ROTATION, direction=search_dir)

                        if DEBUG_PRINT_SPEED:
                            logger.debug("SUCHE   verloren=%.1fs  dir=%s", lost_secs, search_dir)

            else:
                # ── PAUSED ────────────────────────────────────────────────
                rover.stop()

            # ── FPS berechnen ──────────────────────────────────────────────
            if frame_count % 30 == 0:
                fps_display = 30.0 / (now - fps_time + 1e-9)
                fps_time    = now
            cv2.putText(debug_frame, f"FPS: {fps_display:.1f}",
                        (debug_frame.shape[1] - 100, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # ── Debug-Fenster ──────────────────────────────────────────────
            if DEBUG_WINDOW:
                draw_state_overlay(debug_frame, state, base_speed, search_dir)
                cv2.imshow("Wave Rover – Pfadfolger", debug_frame)

                if show_mask:
                    mask_vis = detector.get_mask_only(frame)
                    cv2.imshow("Grün-Maske", mask_vis)
                else:
                    # Fenster schließen falls es offen war
                    try:
                        cv2.destroyWindow("Grün-Maske")
                    except Exception:
                        pass

                # ── Tastatureingaben ───────────────────────────────────────
                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), ord('Q'), 27):         # Q / ESC → Beenden
                    logger.info("Benutzer hat beendet.")
                    break

                elif key in (ord('p'), ord('P')):            # P → Pause umschalten
                    state = State.PAUSED if state != State.PAUSED else State.FOLLOWING
                    logger.info("Zustand: %s", state)

                elif key in (ord('m'), ord('M')):            # M → Maske umschalten
                    show_mask = not show_mask
                    logger.info("Masken-Anzeige: %s", show_mask)

                elif key == ord('+'):                        # + → Schneller
                    base_speed = min(0.9, base_speed + 0.05)
                    logger.info("Geschwindigkeit: %.2f", base_speed)

                elif key == ord('-'):                        # - → Langsamer
                    base_speed = max(0.1, base_speed - 0.05)
                    logger.info("Geschwindigkeit: %.2f", base_speed)

            else:
                # Kein Debug-Fenster → kurz schlafen
                time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("Ctrl+C gedrückt – beende.")

    finally:
        logger.info("Stoppe Rover und schließe Kamera…")
        rover.stop()
        time.sleep(0.2)
        cap.release()
        cv2.destroyAllWindows()
        logger.info("Fertig.")


if __name__ == "__main__":
    main()
