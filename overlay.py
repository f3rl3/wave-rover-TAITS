"""
Debug overlay rendering for Wave Rover
All drawing / HUD logic lives here.
"""

import cv2
import math
import numpy as np

from config import (
    FAR_ZONE_END,
    NEAR_ZONE_START,
    NEAR_ZONE_END,
    BEND_SLOW_DEG,
)

STATE_COLORS = {
    "FOLLOWING":   (0,   220,   0),
    "SEARCHING":   (0,   165, 255),
    "RED_STOP":    (0,     0, 200),
    "TURNING_180": (0,   200, 200),
    "RETURNING":   (180, 220,   0),
    "TERMINAL":    (60,   60,  60),
}


def draw_debug_frame(
    frame: np.ndarray,        # raw camera frame copy (BGR)
    mask: np.ndarray,         # grayscale green mask (from detector.get_last_mask())
    result,                   # PathResult dataclass instance
    *,
    roi_top: int,
    roi_bottom: int,
    dead_zone_px: int,
    state: str,
    base_speed: float,
    hud_extra: str,
    fps: float,
    red_detected: bool,
    red_area_last: int,
    red_on_cooldown: bool,
    red_cooldown_remaining: float,   # seconds remaining; 0 if not on cooldown
) -> np.ndarray:
    """Draw all debug overlays onto the supplied frame and return it."""

    h, w = frame.shape[:2]
    rt   = roi_top
    rb   = roi_bottom
    roi_h = rb - rt

    # 1. Green mask overlay — semi-transparent green where mask > 0 (30 % alpha over ROI)
    roi_region = frame[rt:rb, :]
    overlay    = np.zeros_like(roi_region)
    overlay[mask > 0] = (0, 255, 0)
    cv2.addWeighted(roi_region, 0.7, overlay, 0.3, 0, dst=frame[rt:rb, :])

    # 2. ROI border rectangle (cyan-yellow, 2 px)
    cv2.rectangle(frame, (0, rt), (w, rb), (255, 200, 0), 2)

    # 3. Center line + dead-zone lines
    mid = w // 2
    dz  = dead_zone_px
    cv2.line(frame, (mid,      rt), (mid,      rb), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(frame, (mid - dz, rt), (mid - dz, rb), (100, 100, 255), 1)
    cv2.line(frame, (mid + dz, rt), (mid + dz, rb), (100, 100, 255), 1)

    # 4. Zone lines + labels
    far_y      = rt + int(roi_h * FAR_ZONE_END)
    near_top_y = rt + int(roi_h * NEAR_ZONE_START)
    near_bot_y = rt + int(roi_h * NEAR_ZONE_END)

    # FAR zone line + label
    cv2.line(frame, (0, far_y), (w, far_y), (255, 80, 80), 1)
    cv2.putText(frame, "FAR (ahead)", (5, far_y - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 80, 80), 1)

    # NEAR zone rectangle + label
    cv2.rectangle(frame, (0, near_top_y), (w, near_bot_y), (50, 160, 220), 1)
    cv2.putText(frame, "NEAR (under rover)", (5, near_top_y + 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (50, 160, 220), 1)

    # Forward-travel arrow
    arr_x = w - 22
    cv2.arrowedLine(frame, (arr_x, rb - 10), (arr_x, rt + 10),
                    (180, 180, 180), 1, tipLength=0.15)
    cv2.putText(frame, "fwd", (arr_x - 12, rt + 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)

    # 5. Path-found drawings
    if result.found:
        # Cyan crosshair at centroid (steering origin)
        cv2.drawMarker(frame, (result.centroid_x, result.centroid_y),
                       (0, 255, 255), cv2.MARKER_CROSS, 22, 2)

        # Blue circle at near_cx, red circle at far_cx, arrow NEAR → FAR
        if result.near_cx is not None and result.far_cx is not None:
            near_cy = (near_top_y + near_bot_y) // 2
            far_cy  = rt + int(roi_h * FAR_ZONE_END) // 2

            cv2.circle(frame, (result.near_cx, near_cy), 7, (50, 200, 255), -1)   # blue = NEAR
            cv2.circle(frame, (result.far_cx,  far_cy),  7, (255, 80,  80), -1)   # red  = FAR

            bend_col = (0, 255, 0) if not result.is_sharp_bend else (0, 0, 255)
            cv2.arrowedLine(frame,
                            (result.near_cx, near_cy),
                            (result.far_cx,  far_cy),
                            bend_col, 2, tipLength=0.2)

        # "Offset: +XX.X%" text (green if in_dead_zone, orange otherwise)
        off_pct = result.offset_normalized * 100
        off_col = (0, 255, 0) if result.in_dead_zone else (0, 165, 255)
        cv2.putText(frame, f"Offset: {off_pct:+.1f}%",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.62, off_col, 2)

        # Bend / angle status text line
        sf = result.speed_factor
        if result.is_sharp_bend:
            btxt = f"BEND {result.bend_angle_deg:.1f}° ({result.bend_direction}) ► ALIGN"
            bcol = (0, 0, 255)
        elif result.bend_angle_deg > BEND_SLOW_DEG:
            btxt = f"Curve {result.bend_angle_deg:.1f}°  Speed x{sf:.2f}"
            bcol = (0, 140, 255)
        else:
            btxt = f"Angle {result.bend_angle_deg:.1f}°  Speed x{sf:.2f}"
            bcol = (180, 255, 180)
        cv2.putText(frame, btxt, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, bcol, 2)

        # Stripe alignment line + angle label
        angle     = result.stripe_angle_deg
        angle_rad = math.radians(angle)
        lx = math.sin(angle_rad)
        ly = -math.cos(angle_rad)
        cx_l = result.centroid_x if result.centroid_x is not None else w // 2
        cy_l = (rt + rb) // 2
        arm  = (rb - rt) // 2 - 5
        stripe_col = (0, 220, 220) if result.in_dead_zone else (80, 80, 200)
        cv2.line(frame,
                 (int(cx_l - lx * arm), int(cy_l - ly * arm)),
                 (int(cx_l + lx * arm), int(cy_l + ly * arm)),
                 stripe_col, 2, cv2.LINE_AA)
        cv2.putText(frame, f"{angle:+.0f}deg",
                    (cx_l + 6, cy_l - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, stripe_col, 1)

    # 6. PATH NOT FOUND text
    else:
        cv2.putText(frame, "PATH NOT FOUND",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # 7. Red banner (top 30 px)
    if red_detected:
        cv2.rectangle(frame, (0, 0), (w, 30), (0, 0, 180), -1)
        cv2.putText(frame, f"RED detected  ({red_area_last} px)",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    elif red_on_cooldown:
        cv2.rectangle(frame, (0, 0), (w, 30), (40, 40, 120), -1)
        cv2.putText(frame, f"RED locked  {red_cooldown_remaining:.0f}s remaining",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 255), 2)

    # 8. FPS counter (top-right corner)
    cv2.putText(frame, f"FPS {fps:.1f}",
                (w - 95, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    # 9. HUD bar (bottom 55 px)
    color = STATE_COLORS.get(state, (200, 200, 200))
    cv2.rectangle(frame, (0, h - 55), (w, h), (30, 30, 30), -1)
    label = f"[{state}]"
    if hud_extra:
        label += f"  {hud_extra}"
    cv2.putText(frame, label, (10, h - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, color, 2)
    cv2.putText(frame, f"Speed-Base: {base_speed:.2f}",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)

    return frame
