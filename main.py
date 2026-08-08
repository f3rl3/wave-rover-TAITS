"""
Wave Rover Path-Follower
Main program: Connects camera, path detection and rover control
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
    ROTATE_DEG_PER_SEC, MAX_ALIGN_ROTATION_DEG, MAX_SEARCH_ROTATION_DEG,
    RED_STOP_WAIT_S, TURN_180_SPD, RED_COOLDOWN_S,
    STRIPE_ALIGN_TOL_DEG, STRIPE_ALIGN_EXIT_DEG, STRIPE_ALIGN_TIMEOUT_S,
    STRIPE_ALIGN_BOOST_S, STRIPE_ALIGN_SPD,
    STUCK_RESET_S,
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

class State:
    FOLLOWING   = "FOLLOWING"
    ALIGNING    = "ALIGNING"
    SEARCHING   = "SEARCHING"
    # PAUSED      = "PAUSED"       # Manually paused
    RED_STOP    = "RED_STOP"
    TURNING_180 = "TURNING_180"
    RETURNING   = "RETURNING"
    TERMINAL    = "TERMINAL"

class HeadingTracker:
    """
    Estimates the accumulated rotation of the rover since the last reset().
    """

    def __init__(self, deg_per_sec: float):
        self._dps    = deg_per_sec   # degrees/second
        self._accum  = 0.0           # accumulated rotation (+ = right, - = left)
        self._last_t: Optional[float] = None

    def reset(self):
        self._accum  = 0.0
        self._last_t = None

    def update(self, direction: str, now: float):
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now
        sign = +1.0 if direction == "right" else -1.0
        self._accum += sign * self._dps * dt

    @property
    def abs_deg(self) -> float:
        return abs(self._accum)

# --- Helper functions ---
def open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        logger.error("Camera %d could not be opened.", CAMERA_INDEX)
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    logger.info("Camera: %dx%d @ %.0f FPS", actual_w, actual_h, actual_fps)
    return cap

STATE_COLORS = {
    State.FOLLOWING:   (0,   220,   0),
    State.SEARCHING:   (0,   165, 255),
    State.PAUSED:      (0,     0, 220),
    State.RED_STOP:    (0,     0, 200),
    State.TURNING_180: (0,   200, 200),
    State.RETURNING:   (180, 220,   0),
    State.TERMINAL:    (60,   60,  60),
}

def draw_hud_on_video(frame, state: str, speed: float, extra: str = ""):
    h, w = frame.shape[:2]
    color = STATE_COLORS.get(state, (200, 200, 200))

    cv2.rectangle(frame, (0, h - 55), (w, h), (30, 30, 30), -1)

    label = f"[{state}]"
    if extra:
        label += f"  {extra}"
    cv2.putText(frame, label, (10, h - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, color, 2)
    cv2.putText(frame, f"Speed-Base: {speed:.2f}",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)


# --- Main program ---
def parse_args():
    p = argparse.ArgumentParser(
        description="Wave Rover Path-Follower",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python main.py                  # Use config defaults,\n"
            "  python main.py --no-web         # Disable web dashboard (saves RAM/CPU)\n"
            "  python main.py --window         # Show OpenCV window (only with monitor)\n"
            "  python main.py --no-web --window\n"
        ),
    )
    p.add_argument(
        "--web",
        default=None,
        action=argparse.BooleanOptionalAction,
        help=(
            "Enable/disable web debug dashboard (Flask).\n"
            "Default from config.py: %(default)s"
        ),
    )
    p.add_argument(
        "--window",
        default=None,
        action=argparse.BooleanOptionalAction,
        help=(
            "Enable/disable OpenCV debug window (only with monitor on Pi).\n"
            "Default from config.py: %(default)s"
        ),
    )
    return p.parse_args()


def main():
    args = parse_args()

    use_web_server = args.web    if args.web    is not None else DEBUG_WEB_SERVER
    use_window     = args.window if args.window is not None else DEBUG_WINDOW

    logger.info("=" * 55)
    logger.info("  Wave Rover Path-Follower")
    logger.info("  Web-Dashboard: %s  |  OpenCV-Window: %s",
                "ON" if use_web_server else "OFF",
                "ON" if use_window     else "OFF")
    logger.info("=" * 55)

    rover = RoverController()
    if not rover.connect():
        answer = input("Rover not reachable. Continue anyway (camera only)? [j/N]: ")
        if answer.strip().lower() not in ("j", "y", "ja", "yes"):
            sys.exit(1)
        logger.warning("Simulation mode - no rover commands will be sent")

    cap      = open_camera()
    detector = PathDetector()

    # --- Start web debug server ---
    debug_srv: Optional[DebugServer] = None
    if use_web_server:
        try:
            debug_srv = DebugServer(port=DEBUG_SERVER_PORT, stream_fps=DEBUG_STREAM_FPS)
            debug_srv.start()
        except ImportError as e:
            logger.warning("Web debug disabled: %s", e)

    # --- State variables ---
    state          = State.FOLLOWING
    base_speed     = SPEED_FORWARD
    search_dir     = SEARCH_DIRECTION
    show_mask      = DEBUG_SHOW_MASK
    show_red_mask  = False
    prev_error          = 0.0
    aligning_stripe     = False   
    align_stripe_t      = 0.0     
    timeout_boost_end_t = 0.0     
    last_forward_t      = time.time()

    last_seen_t    = time.time()
    last_seen_side = SEARCH_DIRECTION

    heading        = HeadingTracker(ROTATE_DEG_PER_SEC)
    follow_state      = State.FOLLOWING
    red_stop_t        = 0.0
    red_area_last     = 0
    red_cooldown_end_t = 0.0

    frame_count    = 0
    fps_t          = time.time()
    fps_display    = 0.0

    logger.info("Running - waiting for stripe...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.03)
                continue

            frame_count += 1
            now = time.time()

            # --- Pfaderkennung ---
            result, debug_frame = detector.process(frame)

            # Check for red marking, only relevant while driving
            red_detected  = False
            red_on_cooldown = now < red_cooldown_end_t
            if state in (State.FOLLOWING, State.RETURNING) and not red_on_cooldown:
                red_detected, red_area_last = detector.detect_red(frame)
                if frame_count % 30 == 0 and red_area_last > 0:
                    logger.debug("red_px=%d  (threshold=%d)", red_area_last, 10_000)

            # Draw red marking overlay on debug frame
            if red_detected or (state in (State.RED_STOP, State.TURNING_180, State.TERMINAL)):
                h_f, w_f = debug_frame.shape[:2]
                cv2.rectangle(debug_frame, (0, 0), (w_f, 30), (0, 0, 180), -1)
                cv2.putText(debug_frame, f"RED detected  ({red_area_last} px)",
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            elif red_on_cooldown:
                remaining_cd = red_cooldown_end_t - now
                h_f, w_f = debug_frame.shape[:2]
                cv2.rectangle(debug_frame, (0, 0), (w_f, 30), (40, 40, 120), -1)
                cv2.putText(debug_frame, f"RED locked  {remaining_cd:.0f}s remaining",
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 255), 2)

            # --- State machine ---
            hud_extra = ""

            if state == State.TERMINAL:
                rover.stop()
                logger.info("TERMINAL - end reached, terminating program...")
                break

            elif state == State.RED_STOP:
                # RED_STOP: First red marking
                rover.stop()
                elapsed = now - red_stop_t
                remaining = max(0.0, RED_STOP_WAIT_S - elapsed)
                hud_extra = f"waiting for signal... {remaining:.1f}s"
                logger.debug("RED_STOP  remaining=%.1fs", remaining)

                if elapsed >= RED_STOP_WAIT_S:
                    logger.info(
                        "Signal recieved after %.0fs), returning...",
                        RED_STOP_WAIT_S
                    )
                    state = State.TURNING_180
                    heading.reset()

            elif state == State.TURNING_180:
                heading.update("left", now)
                rotated = heading.abs_deg
                hud_extra = f"turning 180°  {rotated:.0f}°/180°"

                if rotated >= 175.0:
                    rover.stop()
                    heading.reset()
                    follow_state       = State.RETURNING
                    state              = State.RETURNING
                    last_seen_side     = SEARCH_DIRECTION
                    prev_error         = 0.0
                    red_cooldown_end_t = now + RED_COOLDOWN_S

                    logger.info(
                        "Return trip starts - Red locked for %.0fs",
                        RED_COOLDOWN_S
                    )
                else:
                    rover.turn_in_place(TURN_180_SPD, direction="left")

            # elif state == State.PAUSED:
            #     rover.stop()
            #     hud_extra = "Press P to continue"

            elif state == State.SEARCHING:
                # Rotate and search for path

                heading.update(search_dir, now)
                rotated_deg = heading.abs_deg

                if result.found:
                    logger.info(
                        "Path found again (%.0f° searched) - continuing",
                        rotated_deg
                    )

                    heading.reset()
                    state       = follow_state
                    last_seen_t = now
                    prev_error  = 0.0

                elif rotated_deg >= MAX_SEARCH_ROTATION_DEG:
                    old_dir    = search_dir
                    search_dir = "right" if search_dir == "left" else "left"
                    heading.reset()

                    logger.info(
                        "Search limit %.0f° -> direction %s -> %s",
                        MAX_SEARCH_ROTATION_DEG, old_dir, search_dir
                    )

                    rover.stop()

                else:
                    rover.turn_in_place(SEARCH_ROTATION, direction=search_dir)
                    hud_extra = (
                        f"searching {search_dir.upper()}  "
                        f"{rotated_deg:.0f}°/{MAX_SEARCH_ROTATION_DEG:.0f}°"
                    )

            elif state in (State.FOLLOWING, State.RETURNING):
                # FOLLOWING / RETURNING: Follow path

                if now - last_forward_t > STUCK_RESET_S:
                    logger.warning(
                        "Stuck-Reset: no forward movement for %.0fs - re-aligning "
                        "(drive mode remains: %s)",
                        now - last_forward_t, follow_state,
                    )

                    state               = follow_state
                    aligning_stripe     = False
                    align_stripe_t      = 0.0
                    timeout_boost_end_t = 0.0
                    prev_error          = 0.0
                    last_forward_t      = now
                    heading.reset()
                    rover.stop()
                    continue

                if red_detected:
                    rover.stop()
                    if state == State.FOLLOWING:
                        logger.info(
                            "First red marking detected (%d px) - waiting for signal",
                            red_area_last
                        )
                        state      = State.RED_STOP
                        red_stop_t = now
                    else:   # RETURNING
                        logger.info(
                            "Second red marking detected (%d px) - terminating",
                            red_area_last
                        )
                        state = State.TERMINAL
                    continue   # Skip rest of loop, next frame

                if not result.found:
                    lost = now - last_seen_t
                    if lost < SEARCH_TIMEOUT_S * 0.3:
                        rover.forward(base_speed * 0.4)
                    else:
                        logger.info(
                            "Path lost (%.1fs), looking %s first...",
                            lost, last_seen_side
                        )
                        state        = State.SEARCHING
                        search_dir   = last_seen_side
                        heading.reset()
                        rover.stop()
                else:
                    last_seen_t = now

                    if not result.in_dead_zone:
                        last_seen_side = "left" if result.offset_normalized < 0 else "right"

                    stripe_angle = result.stripe_angle_deg

                    if abs(stripe_angle) > STRIPE_ALIGN_TOL_DEG:
                        if not aligning_stripe:
                            align_stripe_t = now
                        aligning_stripe = True
                    elif abs(stripe_angle) < STRIPE_ALIGN_EXIT_DEG:
                        aligning_stripe = False

                    if aligning_stripe and (now - align_stripe_t) > STRIPE_ALIGN_TIMEOUT_S:
                        logger.warning(
                            "Phase-2 timeout after %.1fs (angle %+.1f°) - %.1fs full speed",
                            STRIPE_ALIGN_TIMEOUT_S, stripe_angle, STRIPE_ALIGN_BOOST_S
                        )
                        aligning_stripe     = False
                        timeout_boost_end_t = now + STRIPE_ALIGN_BOOST_S

                    if not result.in_dead_zone:
                        # aligning_stripe remains unchanged - Phase 2 continues
                        # there once the stripe is centered again.
                        rover.steer(
                            base_speed,
                            result.offset_normalized,
                            turn_max=SPEED_TURN_MAX,
                            kp=KP, kd=KD,
                            prev_error=prev_error,
                        )
                        prev_error     = result.offset_normalized
                        last_forward_t = now
                        hud_extra      = f"Approaching  off={result.offset_normalized:+.2f}"

                    elif aligning_stripe:
                        align_dir_now = "right" if stripe_angle > 0 else "left"
                        rover.turn_in_place(STRIPE_ALIGN_SPD, direction=align_dir_now)
                        prev_error = 0.0
                        hud_extra  = f"Aligning {stripe_angle:+.0f}° ({align_dir_now})"

                    else:
                        boost_active  = now < timeout_boost_end_t
                        current_speed = base_speed if boost_active else base_speed * result.speed_factor

                        rover.steer(
                            current_speed,
                            result.offset_normalized,
                            turn_max=SPEED_TURN_MAX,
                            kp=KP, kd=KD,
                            prev_error=prev_error,
                        )

                        prev_error     = result.offset_normalized
                        last_forward_t = now
                        hud_extra      = (
                            f"Curve {result.bend_angle_deg:.0f}°  off={result.offset_normalized:+.2f}"
                            if result.speed_factor < 1.0 else
                            f"BOOST {timeout_boost_end_t - now:.1f}s" if boost_active else ""
                        )

                    if DEBUG_PRINT_SPEED:
                        phase_name = (
                            "APPROACH" if not result.in_dead_zone else
                            "ALIGN"    if aligning_stripe          else
                            "FOLLOW"
                        )
                        logger.debug(
                            "FOLLOW  off=%+.3f  stripe=%+.1f°  phase=%s  sf=%.2f",
                            result.offset_normalized, stripe_angle,
                            phase_name,
                            result.speed_factor,
                        )

            # --- FPS ---
            if frame_count % 30 == 0:
                fps_display = 30.0 / max(now - fps_t, 1e-9)
                fps_t = now

            # --- Update web debug server ---
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
                        "red_detected":   red_detected,
                        "red_area":       red_area_last,
                        "follow_mode":    follow_state,
                        "fps":            round(fps_display, 1),
                        "frame_count":    frame_count,
                    }
                )
            cv2.putText(debug_frame,
                        f"FPS {fps_display:.1f}",
                        (debug_frame.shape[1] - 95, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # --- Debug output ---
            if use_window:
                draw_hud_on_video(debug_frame, state, base_speed, hud_extra)
                cv2.imshow("Wave Rover Path-Follower", debug_frame)

                if show_mask:
                    cv2.imshow("Green Mask", detector.get_last_mask())
                else:
                    try:
                        cv2.destroyWindow("Green Mask")
                    except Exception:
                        pass

                if show_red_mask:
                    red_dbg = detector.get_last_red_mask()
                    if red_dbg is not None:
                        # Color red mask as BGR image
                        red_vis = cv2.cvtColor(red_dbg, cv2.COLOR_GRAY2BGR)
                        red_vis[red_dbg > 0] = (0, 0, 220)
                        cv2.putText(red_vis,
                                    f"Red-Px: {red_area_last}  Threshold: 10000",
                                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (255, 255, 255), 1)
                        cv2.imshow("Red Mask (R=close)", red_vis)
                else:
                    try:
                        cv2.destroyWindow("Red Mask (R=close)")
                    except Exception:
                        pass

                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), ord('Q'), 27):
                    logger.info("User quit.")
                    break
                elif key in (ord('p'), ord('P')):
                    state = State.PAUSED if state != State.PAUSED else State.FOLLOWING
                    logger.info("State: %s", state)
                elif key in (ord('m'), ord('M')):
                    show_mask = not show_mask
                elif key in (ord('r'), ord('R')):
                    show_red_mask = not show_red_mask
                    logger.info("Red mask: %s", "ON" if show_red_mask else "OFF")
                elif key == ord('+'):
                    base_speed = min(0.90, base_speed + 0.05)
                    logger.info("Speed: %.2f", base_speed)
                elif key == ord('-'):
                    base_speed = max(0.10, base_speed - 0.05)
                    logger.info("Speed: %.2f", base_speed)
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("Ctrl+C - stopping.")
    finally:
        logger.info("Stopping rover...")
        rover.stop()
        time.sleep(0.2)
        cap.release()
        cv2.destroyAllWindows()
        logger.info("Done.")


if __name__ == "__main__":
    main()
