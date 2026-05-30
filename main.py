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

import argparse
import cv2
import time
import logging
import sys
from typing import Optional

from config import (
    CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, TARGET_FPS,
    SPEED_FORWARD, SPEED_TURN_MAX,
    SEARCH_TIMEOUT_S, SEARCH_ROTATION, SEARCH_DIRECTION,
    KP, KD,
    BEND_STOP_DEG, BEND_ALIGN_DEG,
    ALIGN_ROTATE_SPD, ALIGN_TIMEOUT_S,
    ROTATE_DEG_PER_SEC, MAX_ALIGN_ROTATION_DEG, MAX_SEARCH_ROTATION_DEG,
    DEBUG_WINDOW, DEBUG_SHOW_MASK, DEBUG_PRINT_SPEED,
    DEBUG_WEB_SERVER, DEBUG_SERVER_PORT, DEBUG_STREAM_FPS,
)
from rover_controller import RoverController
from path_detector    import PathDetector
from debug_server     import DebugServer

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


# ── Heading-Tracker ───────────────────────────────────────────────────────────
class HeadingTracker:
    """
    Schätzt die kumulierte Rotation des Rovers seit dem letzten reset().

    Da der Rover keinen Kompass hat, wird die Rotation über
        gedrehte_Grad = Zeit × ROTATE_DEG_PER_SEC
    approximiert. Mit dieser Information wird verhindert, dass der Rover
    durch Überrotation rückwärts fährt:

        ALIGNING : nie mehr als MAX_ALIGN_ROTATION_DEG (< 90°) drehen
        SEARCHING: pro Richtung nie mehr als MAX_SEARCH_ROTATION_DEG (< 180°)

    Kalibrierung (ROTATE_DEG_PER_SEC in config.py):
        Rover 5 Sekunden mit turn_in_place drehen lassen,
        gemessene Grad durch 5 teilen.
    """

    def __init__(self, deg_per_sec: float):
        self._dps    = deg_per_sec   # Grad/Sekunde
        self._accum  = 0.0           # kumulierte Rotation (+ = rechts, - = links)
        self._last_t: Optional[float] = None

    def reset(self):
        """Setzt Zähler zurück – markiert aktuelle Richtung als 'vorwärts'."""
        self._accum  = 0.0
        self._last_t = None

    def update(self, direction: str, now: float):
        """
        Jeden Frame aufrufen solange der Rover dreht.
        direction: 'left' oder 'right'
        """
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now
        sign = +1.0 if direction == "right" else -1.0
        self._accum += sign * self._dps * dt

    @property
    def abs_deg(self) -> float:
        """Absoluter Betrag der Gesamtrotation seit letztem reset()."""
        return abs(self._accum)

    @property
    def signed_deg(self) -> float:
        """Rotation mit Vorzeichen (+ = rechts, - = links)."""
        return self._accum


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

def parse_args():
    p = argparse.ArgumentParser(
        description="Wave Rover – Grüner Pfad Folger",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Beispiele:\n"
            "  python main.py                  # Config-Defaults verwenden\n"
            "  python main.py --no-web         # Web-Dashboard deaktivieren (spart RAM/CPU)\n"
            "  python main.py --window         # OpenCV-Fenster anzeigen (nur mit Monitor)\n"
            "  python main.py --no-web --window\n"
        ),
    )
    # BooleanOptionalAction erzeugt automatisch --foo und --no-foo
    p.add_argument(
        "--web",
        default=None,
        action=argparse.BooleanOptionalAction,
        help=(
            "Web-Debug-Dashboard (Flask) ein-/ausschalten.\n"
            "Standard aus config.py: %(default)s"
        ),
    )
    p.add_argument(
        "--window",
        default=None,
        action=argparse.BooleanOptionalAction,
        help=(
            "OpenCV-Debug-Fenster ein-/ausschalten (nur mit Monitor am Pi).\n"
            "Standard aus config.py: %(default)s"
        ),
    )
    return p.parse_args()


def main():
    args = parse_args()

    # Kommandozeile überschreibt config.py, nur wenn Flag explizit gesetzt wurde
    use_web_server = args.web    if args.web    is not None else DEBUG_WEB_SERVER
    use_window     = args.window if args.window is not None else DEBUG_WINDOW

    logger.info("=" * 55)
    logger.info("  Wave Rover – Grüner Pfad Folger")
    logger.info("  Web-Dashboard: %s  |  OpenCV-Fenster: %s",
                "AN" if use_web_server else "AUS",
                "AN" if use_window     else "AUS")
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

    # ── Web-Debug-Server starten ──────────────────────────────────────────────
    debug_srv: Optional[DebugServer] = None
    if use_web_server:
        try:
            debug_srv = DebugServer(port=DEBUG_SERVER_PORT, stream_fps=DEBUG_STREAM_FPS)
            debug_srv.start()
        except ImportError as e:
            logger.warning("Web-Debug deaktiviert: %s", e)

    # ── Zustandsvariablen ─────────────────────────────────────────────────────
    state          = State.FOLLOWING
    base_speed     = SPEED_FORWARD
    search_dir     = SEARCH_DIRECTION
    show_mask      = DEBUG_SHOW_MASK
    prev_error     = 0.0

    last_seen_t    = time.time()    # Zeitpunkt: Pfad zuletzt gesehen
    align_start_t  = 0.0            # Zeitpunkt: Ausrichtung begonnen
    align_dir      = "left"         # Richtung der aktuellen Ausrichtung
    search_dir_t   = time.time()    # Zeitpunkt: aktuelle Suchrichtung begonnen

    # Auf welcher Seite des Sichtfeldes der Pfad zuletzt gesehen wurde.
    # Wird bei jedem Frame aktualisiert solange der Pfad sichtbar ist.
    # Beim Pfadverlust wird diese Seite als erste Suchrichtung verwendet,
    # weil der Pfad wahrscheinlich in diese Richtung weitergeht.
    last_seen_side = SEARCH_DIRECTION  # Fallback: konfigurierter Standardwert

    heading        = HeadingTracker(ROTATE_DEG_PER_SEC)

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
                #
                # Rückwärts-Schutz: Rotationslimit über HeadingTracker.
                # Mehr als MAX_ALIGN_ROTATION_DEG (90°) drehen würde den Rover
                # über 90° von der Vorwärtsrichtung wegbewegen → rückwärts.
                # Deshalb: Limit überschritten → sofort zu FOLLOWING, Rotation stoppen.

                heading.update(align_dir, now)
                elapsed          = now - align_start_t
                rotated_deg      = heading.abs_deg

                if not result.found:
                    logger.warning(
                        "Pfad während Ausrichtung verloren (%.0f° gedreht) – Suche",
                        rotated_deg
                    )
                    heading.reset()
                    state = State.SEARCHING
                    search_dir   = align_dir          # Suche in gleicher Richtung
                    search_dir_t = now
                    rover.stop()

                elif rotated_deg >= MAX_ALIGN_ROTATION_DEG:
                    # Limit erreicht – weiter drehen würde Rückwärtsfahren verursachen
                    logger.warning(
                        "⛔ Rotationslimit (%.0f°/%.0f°) – stoppe Ausrichtung",
                        rotated_deg, MAX_ALIGN_ROTATION_DEG
                    )
                    heading.reset()
                    state      = State.FOLLOWING
                    prev_error = 0.0
                    rover.stop()

                elif result.bend_angle_deg < BEND_ALIGN_DEG:
                    logger.info(
                        "✅ Ausgerichtet (Winkel=%.1f°, %.0f° gedreht) – weiterfahren",
                        result.bend_angle_deg, rotated_deg
                    )
                    heading.reset()
                    state      = State.FOLLOWING
                    prev_error = 0.0

                elif elapsed > ALIGN_TIMEOUT_S:
                    logger.warning(
                        "Ausrichtungs-Timeout (%.1fs, %.0f° gedreht)",
                        elapsed, rotated_deg
                    )
                    heading.reset()
                    state = State.FOLLOWING

                else:
                    rover.turn_in_place(ALIGN_ROTATE_SPD, direction=align_dir)
                    hud_extra = (
                        f"dreht {align_dir.upper()}  "
                        f"{rotated_deg:.0f}°/{MAX_ALIGN_ROTATION_DEG:.0f}°  "
                        f"Knick={result.bend_angle_deg:.1f}°"
                    )
                    if DEBUG_PRINT_SPEED:
                        logger.debug(
                            "ALIGN  dir=%s  rotiert=%.0f°  knick=%.1f°",
                            align_dir, rotated_deg, result.bend_angle_deg
                        )

            elif state == State.SEARCHING:
                # ── SEARCHING: Pfad verloren → drehen und suchen ──────────────
                #
                # Rückwärts-Schutz: Pro Richtung maximal MAX_SEARCH_ROTATION_DEG
                # drehen, dann Richtung umkehren. Da MAX_SEARCH_ROTATION_DEG < 180°,
                # schaut der Rover nie in die Gegenrichtung.

                heading.update(search_dir, now)
                rotated_deg = heading.abs_deg

                if result.found:
                    logger.info(
                        "Pfad wiedergefunden (%.0f° gesucht) – weiterfahren",
                        rotated_deg
                    )
                    heading.reset()
                    state       = State.FOLLOWING
                    last_seen_t = now
                    prev_error  = 0.0

                elif rotated_deg >= MAX_SEARCH_ROTATION_DEG:
                    # Richtungsumkehr – nie über 150° in eine Richtung
                    old_dir    = search_dir
                    search_dir = "right" if search_dir == "left" else "left"
                    search_dir_t = now
                    heading.reset()
                    logger.info(
                        "Suchlimit %.0f° → Richtung %s → %s",
                        MAX_SEARCH_ROTATION_DEG, old_dir, search_dir
                    )
                    rover.stop()

                else:
                    rover.turn_in_place(SEARCH_ROTATION, direction=search_dir)
                    hud_extra = (
                        f"suche {search_dir.upper()}  "
                        f"{rotated_deg:.0f}°/{MAX_SEARCH_ROTATION_DEG:.0f}°"
                    )

            else:
                # ── FOLLOWING: Pfad gefunden → folgen ────────────────────────
                if not result.found:
                    lost = now - last_seen_t
                    if lost < SEARCH_TIMEOUT_S * 0.3:
                        rover.forward(base_speed * 0.4)   # kurz weiterrollen
                    else:
                        logger.info(
                            "Pfad verloren (%.1fs) – suche zuerst %s (zuletzt dort gesehen)",
                            lost, last_seen_side
                        )
                        state        = State.SEARCHING
                        search_dir   = last_seen_side      # zur zuletzt gesehenen Seite drehen
                        search_dir_t = now
                        heading.reset()                    # Von hier aus maximal 150° suchen
                        rover.stop()
                else:
                    last_seen_t = now

                    # Seite merken auf der der Pfad gerade sichtbar ist.
                    # Nur aktualisieren wenn Pfad merklich links oder rechts ist
                    # (nicht im toten Bereich), damit das Ergebnis stabil bleibt.
                    if not result.in_dead_zone:
                        last_seen_side = "left" if result.offset_normalized < 0 else "right"

                    # Scharfer Knick → ALIGNING starten
                    if result.is_sharp_bend:
                        logger.info(
                            "⚠ Scharfer Knick erkannt (%.1f°, %s) – halte an und richte aus",
                            result.bend_angle_deg, result.bend_direction
                        )
                        state         = State.ALIGNING
                        align_start_t = now
                        align_dir     = result.bend_direction if result.bend_direction != "none" else "left"
                        heading.reset()    # Aktuelle Richtung = Vorwärts
                        rover.stop()
                        # KEIN time.sleep() – blockiert den Kontrollloop!
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

            # ── Web-Debug-Server aktualisieren ────────────────────────────────
            if debug_srv is not None:
                current_speed = base_speed * (result.speed_factor if result.found else 1.0)
                debug_srv.push(
                    main_frame=debug_frame,
                    mask_frame=detector.get_last_mask(),
                    status={
                        "state":        state,
                        "speed":        round(base_speed, 3),
                        "eff_speed":    round(current_speed, 3),
                        "path_found":   result.found,
                        "offset":       round(result.offset_normalized, 4) if result.found else 0.0,
                        "in_dead_zone": result.in_dead_zone,
                        "area":         round(result.area, 1),
                        "bend_angle":   round(result.bend_angle_deg, 2) if result.found else 0.0,
                        "bend_dir":     result.bend_direction if result.found else "none",
                        "speed_factor": round(result.speed_factor, 3) if result.found else 1.0,
                        "is_sharp_bend":result.is_sharp_bend if result.found else False,
                        "heading_deg":    round(heading.abs_deg, 1),
                        "heading_limit":  MAX_ALIGN_ROTATION_DEG,
                        "last_seen_side": last_seen_side,
                        "fps":            round(fps_display, 1),
                        "frame_count":    frame_count,
                    }
                )
            cv2.putText(debug_frame,
                        f"FPS {fps_display:.1f}",
                        (debug_frame.shape[1] - 95, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # ── Debug-Ausgabe ──────────────────────────────────────────────────
            if use_window:
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
