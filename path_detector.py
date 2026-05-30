"""
Grüner-Pfad-Erkenner mit OpenCV — Kamera zeigt nach UNTEN
===========================================================
Verarbeitet jeden Kameraframe (Vogelperspektive auf den Boden) und gibt zurück:
  - Ob ein Pfad gefunden wurde
  - Den horizontalen Offset des Pfades (Lenkkorrektur)
  - Den Knickwinkel des Pfades in Grad
  - Einen annotierten Debug-Frame

Kamera-Geometrie (nach unten gerichtet):
  ┌─────────────────────┐  y = 0%   (Frame-Oberkante)
  │  FERN-Zone          │           → Pfad knapp VOR dem Rover
  │  (Primär-Lenkung)   │           → wird für Offset-Berechnung genutzt
  ├─────────────────────┤  y ≈ 20%
  │  (Übergang)         │
  ├─────────────────────┤  y ≈ 40%
  │  NAH-Zone           │           → Pfad DIREKT unter dem Rover
  │  (Referenz-Knick)   │           → wird für Knick-Berechnung genutzt
  ├─────────────────────┤  y ≈ 75%
  │  (bereits hinter    │
  │   dem Rover)        │           → wird ignoriert (abgefahrener Pfad)
  └─────────────────────┘  y = 90%  (ROI-Ende, konfigurierbar)

Knick-Erkennung:
  bend_angle = atan2(far_cx − near_cx,  vertikaler_abstand)
  far_cx  = Schwerpunkt X der FERN-Zone  (voraus)
  near_cx = Schwerpunkt X der NAH-Zone   (unter Rover)

  → bend_angle > 0 : Pfad biegt nach RECHTS ab
  → bend_angle < 0 : Pfad biegt nach LINKS ab
  → bend_angle ≈ 0 : Pfad gerade voraus
"""

import cv2
import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
import logging

from config import (
    GREEN_HSV_LOW, GREEN_HSV_HIGH,
    MIN_GREEN_AREA,
    ROI_TOP_RATIO, ROI_BOTTOM_RATIO,
    DEAD_ZONE_RATIO,
    FRAME_WIDTH, FRAME_HEIGHT,
    BEND_SLOW_DEG, BEND_STOP_DEG, SPEED_MIN_FACTOR,
)

logger = logging.getLogger(__name__)

# ── Zonen-Konstanten (relativ zur ROI-Höhe) ───────────────────────────────────
# FERN-Zone: oberste 20% des ROI = Pfad knapp vor dem Rover
FAR_ZONE_END     = 0.20

# NAH-Zone: mittlerer Streifen = Pfad direkt unter dem Rover
NEAR_ZONE_START  = 0.40
NEAR_ZONE_END    = 0.75


@dataclass
class PathResult:
    """Ergebnis einer Pfaderkennung für einen Frame."""
    found: bool

    # ── Offset (Lenkung) ──────────────────────────────────────────────────────
    offset_normalized: float = 0.0   # -1.0 (links) … 0.0 (Mitte) … +1.0 (rechts)
    centroid_x: Optional[int] = None # Lenkursprung X (aus FERN-Zone oder Fallback)
    centroid_y: Optional[int] = None # Y-Position (für Debug-Overlay)
    area: float = 0.0                # Gesamtfläche grüner Pixel (ganzes ROI)
    in_dead_zone: bool = False       # True wenn Offset vernachlässigbar klein

    # ── Knick-Erkennung ───────────────────────────────────────────────────────
    bend_angle_deg: float = 0.0      # Knickwinkel in Grad
    bend_direction: str = "none"     # "left", "right", "none"
    near_cx: Optional[int] = None    # Schwerpunkt NAH-Zone (direkt unter Rover)
    far_cx: Optional[int] = None     # Schwerpunkt FERN-Zone (knapp voraus)
    near_found: bool = False
    far_found: bool = False

    # ── Abgeleitete Steuergrößen ──────────────────────────────────────────────
    is_sharp_bend: bool = False
    speed_factor: float = 1.0        # 1.0 = volle Geschw., 0.0 = Stopp/Ausrichten


class PathDetector:
    """Erkennt grünen Pfad bei nach-unten-gerichteter Kamera."""

    def __init__(self):
        self._lower = np.array(GREEN_HSV_LOW,  dtype=np.uint8)
        self._upper = np.array(GREEN_HSV_HIGH, dtype=np.uint8)
        self._dead_zone_px = int(FRAME_WIDTH * DEAD_ZONE_RATIO)
        self._roi_y_top:    Optional[int] = None
        self._roi_y_bottom: Optional[int] = None
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._last_mask: Optional[np.ndarray] = None   # Cache: Maske des letzten Frames

    # ── Öffentliche API ────────────────────────────────────────────────────────

    def process(self, frame: np.ndarray) -> Tuple[PathResult, np.ndarray]:
        """
        Analysiert einen BGR-Frame (Kamera nach unten gerichtet).

        Returns:
            (PathResult, annotierter Debug-Frame)
        """
        h, w = frame.shape[:2]
        self._update_roi(h, w)

        roi_h = self._roi_y_bottom - self._roi_y_top
        roi   = frame[self._roi_y_top:self._roi_y_bottom, :]

        # Grün-Maske berechnen und cachen – get_last_mask() greift darauf zu,
        # ohne die teure HSV-Konvertierung + Morphologie erneut auszuführen.
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        self._last_mask = mask

        # Offset (Lenkung) aus der FERN-Zone
        result = self._calc_offset(mask, w, roi_h)

        if result.found:
            self._calc_bend(mask, result, w, roi_h)

        debug = self._draw_overlay(frame.copy(), mask, result, w, h, roi_h)
        return result, debug

    def get_last_mask(self) -> Optional[np.ndarray]:
        """Gibt die Maske des zuletzt verarbeiteten Frames zurück – ohne Neuberechnung."""
        return self._last_mask

    def update_hsv_range(self, low: tuple, high: tuple):
        self._lower = np.array(low,  dtype=np.uint8)
        self._upper = np.array(high, dtype=np.uint8)

    # ── Interne Berechnungen ───────────────────────────────────────────────────

    def _update_roi(self, h: int, w: int):
        new_top    = int(h * ROI_TOP_RATIO)
        new_bottom = int(h * ROI_BOTTOM_RATIO)
        if new_top != self._roi_y_top or new_bottom != self._roi_y_bottom:
            self._roi_y_top    = new_top
            self._roi_y_bottom = new_bottom
            self._dead_zone_px = int(w * DEAD_ZONE_RATIO)
            logger.debug(
                "ROI aktualisiert: y=%d…%d  (FERN=top %.0f%%  NAH=%.0f–%.0f%%)",
                new_top, new_bottom,
                FAR_ZONE_END * 100, NEAR_ZONE_START * 100, NEAR_ZONE_END * 100
            )

    def _calc_offset(self, mask: np.ndarray, frame_w: int, roi_h: int) -> PathResult:
        """
        Berechnet den Lenk-Offset primär aus der FERN-Zone (Pfad voraus).

        Fallback auf gesamtes ROI wenn FERN-Zone zu wenig Pixel hat
        (z.B. beim Einfahren auf den Pfad).
        """
        # Gesamtfläche aus dem vollen ROI
        M_full = cv2.moments(mask)
        area   = M_full["m00"]

        if area < MIN_GREEN_AREA:
            return PathResult(found=False, area=area)

        # Primär: FERN-Zone (oben = Pfad voraus)
        far_end  = int(roi_h * FAR_ZONE_END)
        far_mask = mask[:far_end, :]
        M_far    = cv2.moments(far_mask)

        if M_far["m00"] >= MIN_GREEN_AREA * 0.25:
            # FERN-Zone hat genug Pixel → daraus lenken
            cx = int(M_far["m10"] / M_far["m00"])
            cy = int(M_far["m01"] / M_far["m00"])          # ROI-relativ
        else:
            # Fallback: gesamter ROI-Schwerpunkt
            cx = int(M_full["m10"] / area)
            cy = int(M_full["m01"] / area)

        center_x     = frame_w / 2.0
        offset_px    = cx - center_x
        offset_norm  = offset_px / center_x               # -1.0 … +1.0
        in_dead_zone = abs(offset_px) < self._dead_zone_px

        return PathResult(
            found=True,
            offset_normalized=float(offset_norm),
            centroid_x=cx,
            centroid_y=cy + self._roi_y_top,
            area=float(area),
            in_dead_zone=in_dead_zone,
        )

    def _calc_bend(self, mask: np.ndarray, result: PathResult,
                   frame_w: int, roi_h: int):
        """
        Vergleicht FERN-Zone (voraus) mit NAH-Zone (unter Rover) → Knickwinkel.

        NAH-Zone ist bei der nach-unten-Kamera der MITTLERE Streifen des Frames
        (direkt unter dem Rover), nicht der untere Rand.
        """
        far_end    = int(roi_h * FAR_ZONE_END)
        near_start = int(roi_h * NEAR_ZONE_START)
        near_end   = int(roi_h * NEAR_ZONE_END)

        far_mask  = mask[:far_end,           :]
        near_mask = mask[near_start:near_end, :]

        zone_min = MIN_GREEN_AREA * 0.35

        M_far  = cv2.moments(far_mask)
        M_near = cv2.moments(near_mask)
        far_area  = M_far["m00"]
        near_area = M_near["m00"]

        result.near_found = near_area >= zone_min
        result.far_found  = far_area  >= zone_min

        if not result.near_found or not result.far_found:
            result.bend_angle_deg = 0.0
            result.bend_direction = "none"
            result.is_sharp_bend  = False
            result.speed_factor   = 1.0
            return

        near_cx     = int(M_near["m10"] / near_area)
        far_cx      = int(M_far ["m10"] / far_area)
        result.near_cx = near_cx
        result.far_cx  = far_cx

        # Vertikaler Abstand zwischen Zonen-Mittelpunkten
        near_cy_roi = near_start + int(M_near["m01"] / near_area)
        far_cy_roi  = int(M_far["m01"] / far_area)
        dy = max(abs(near_cy_roi - far_cy_roi), 1)

        # Knickwinkel & Richtung
        dx = float(far_cx - near_cx)
        angle_deg = math.degrees(math.atan2(abs(dx), dy))
        result.bend_angle_deg = angle_deg
        result.bend_direction = "right" if dx > 0 else ("left" if dx < 0 else "none")

        # Geschwindigkeitsfaktor
        if angle_deg <= BEND_SLOW_DEG:
            result.speed_factor  = 1.0
            result.is_sharp_bend = False
        elif angle_deg >= BEND_STOP_DEG:
            result.speed_factor  = 0.0
            result.is_sharp_bend = True
        else:
            ratio = (angle_deg - BEND_SLOW_DEG) / (BEND_STOP_DEG - BEND_SLOW_DEG)
            result.speed_factor  = 1.0 - ratio * (1.0 - SPEED_MIN_FACTOR)
            result.is_sharp_bend = False

    # ── Debug-Overlay ──────────────────────────────────────────────────────────

    def _draw_overlay(self, frame: np.ndarray, mask: np.ndarray,
                      result: PathResult, w: int, h: int, roi_h: int) -> np.ndarray:
        """Zeichnet alle Debug-Overlays (angepasst für nach-unten-Kamera)."""

        rt = self._roi_y_top
        rb = self._roi_y_bottom

        # ROI-Rahmen
        cv2.rectangle(frame, (0, rt), (w, rb), (255, 200, 0), 2)

        # Grün-Maske als transparentes Overlay
        roi_region = frame[rt:rb, :]
        overlay    = np.zeros_like(roi_region)
        overlay[mask > 0] = (0, 255, 0)
        cv2.addWeighted(roi_region, 0.7, overlay, 0.3, 0, dst=frame[rt:rb, :])

        # Mittellinie + toter Bereich
        mid = w // 2
        dz  = self._dead_zone_px
        cv2.line(frame, (mid, rt), (mid, rb), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (mid - dz, rt), (mid - dz, rb), (100, 100, 255), 1)
        cv2.line(frame, (mid + dz, rt), (mid + dz, rb), (100, 100, 255), 1)

        # Zonen-Linien & Labels
        far_y       = rt + int(roi_h * FAR_ZONE_END)
        near_top_y  = rt + int(roi_h * NEAR_ZONE_START)
        near_bot_y  = rt + int(roi_h * NEAR_ZONE_END)

        # FERN-Zone (oben = voraus) – blaue Linie
        cv2.line(frame, (0, far_y), (w, far_y), (255, 80, 80), 1)
        cv2.putText(frame, "FERN (voraus)", (5, far_y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 80, 80), 1)

        # NAH-Zone (Mitte = unter Rover) – oranges Rechteck
        cv2.rectangle(frame, (0, near_top_y), (w, near_bot_y), (50, 160, 220), 1)
        cv2.putText(frame, "NAH (unter Rover)", (5, near_top_y + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (50, 160, 220), 1)

        # Fahrtrichtungs-Pfeil (oben = vorwärts)
        arr_x = w - 22
        cv2.arrowedLine(frame, (arr_x, rb - 10), (arr_x, rt + 10),
                        (180, 180, 180), 1, tipLength=0.15)
        cv2.putText(frame, "vor", (arr_x - 12, rt + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)

        if result.found:
            # Lenkursprung (aus FERN-Zone)
            cv2.drawMarker(frame, (result.centroid_x, result.centroid_y),
                           (0, 255, 255), cv2.MARKER_CROSS, 22, 2)

            # NAH/FERN Schwerpunkte + Pfeil (Knick-Richtung)
            if result.near_cx is not None and result.far_cx is not None:
                near_cy = (near_top_y + near_bot_y) // 2
                far_cy  = rt + int(roi_h * FAR_ZONE_END) // 2

                cv2.circle(frame, (result.near_cx, near_cy), 7, (50, 200, 255), -1)  # blau = NAH
                cv2.circle(frame, (result.far_cx,  far_cy),  7, (255, 80,  80), -1)  # rot  = FERN

                bend_col = (0, 255, 0) if not result.is_sharp_bend else (0, 0, 255)
                # Pfeil von NAH → FERN zeigt die Pfadrichtung
                cv2.arrowedLine(frame,
                                (result.near_cx, near_cy),
                                (result.far_cx,  far_cy),
                                bend_col, 2, tipLength=0.2)

            # ── Texte ──────────────────────────────────────────────────────────
            off_pct = result.offset_normalized * 100
            sf      = result.speed_factor

            off_col = (0, 255, 0) if result.in_dead_zone else (0, 165, 255)
            cv2.putText(frame, f"Offset: {off_pct:+.1f}%",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.62, off_col, 2)

            if result.is_sharp_bend:
                btxt = f"KNICK {result.bend_angle_deg:.1f}° ({result.bend_direction}) ► AUSRICHTEN"
                bcol = (0, 0, 255)
            elif result.bend_angle_deg > BEND_SLOW_DEG:
                btxt = f"Kurve {result.bend_angle_deg:.1f}°  Speed x{sf:.2f}"
                bcol = (0, 140, 255)
            else:
                btxt = f"Winkel {result.bend_angle_deg:.1f}°  Speed x{sf:.2f}"
                bcol = (180, 255, 180)

            cv2.putText(frame, btxt, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, bcol, 2)
        else:
            cv2.putText(frame, "PFAD NICHT GEFUNDEN",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        return frame
