"""
Grüner-Pfad-Erkenner mit OpenCV
=================================
Verarbeitet jeden Kameraframe und gibt zurück:
  - Ob ein Pfad gefunden wurde
  - Den horizontalen Offset des Pfades (normalisiert: -1.0 links … +1.0 rechts)
  - Den Knickwinkel des Pfades in Grad
  - Einen annotierten Debug-Frame

Knick-Erkennung – Prinzip:
  Das ROI wird in zwei Zonen aufgeteilt:
    ┌────────────────┐ ← ROI oben (Fern-Zone: wo der Pfad hinführt)
    │   far_cx  ●   │
    │               │
    │   near_cx ●   │
    └────────────────┘ ← ROI unten (Nah-Zone: direkt vor dem Rover)

  bend_angle = atan2(far_cx - near_cx, zonen_abstand_px)
  → positiv = Knick nach rechts, negativ = nach links
"""

import cv2
import math
import numpy as np
from dataclasses import dataclass, field
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

# Anteil des ROIs für Nah- und Fern-Zone
# near = untere 40% des ROI, far = obere 40% (mittlerer Streifen ignoriert)
NEAR_ZONE_RATIO = 0.60   # near-Zone: letzten 40% des ROI (von unten)
FAR_ZONE_RATIO  = 0.40   # far-Zone:  erste 40% des ROI (von oben)


@dataclass
class PathResult:
    """Ergebnis einer Pfaderkennung für einen Frame."""
    found: bool

    # ── Offset (Lenkung) ──────────────────────────────────────────────────────
    offset_normalized: float = 0.0   # -1.0 (links) … 0.0 (Mitte) … +1.0 (rechts)
    centroid_x: Optional[int] = None
    centroid_y: Optional[int] = None
    area: float = 0.0
    in_dead_zone: bool = False

    # ── Knick-Erkennung ───────────────────────────────────────────────────────
    bend_angle_deg: float = 0.0      # Knickwinkel in Grad (+ = rechts, - = links)
    bend_direction: str = "none"     # "left", "right", "none"
    near_cx: Optional[int] = None    # Schwerpunkt der Nah-Zone (px)
    far_cx: Optional[int] = None     # Schwerpunkt der Fern-Zone (px)
    near_found: bool = False         # Nah-Zone hatte genug grüne Pixel
    far_found: bool = False          # Fern-Zone hatte genug grüne Pixel

    # ── Abgeleitete Steuergrößen ──────────────────────────────────────────────
    is_sharp_bend: bool = False      # True wenn Knick ≥ BEND_STOP_DEG
    speed_factor: float = 1.0        # 0.0–1.0: wie stark soll gebremst werden


class PathDetector:
    """Erkennt grünen Pfad und Knicke in Kameraframes."""

    def __init__(self):
        self._lower = np.array(GREEN_HSV_LOW,  dtype=np.uint8)
        self._upper = np.array(GREEN_HSV_HIGH, dtype=np.uint8)
        self._dead_zone_px = int(FRAME_WIDTH * DEAD_ZONE_RATIO)

        # ROI Pixel-Grenzen (beim ersten Frame gesetzt)
        self._roi_y_top:    Optional[int] = None
        self._roi_y_bottom: Optional[int] = None

        # Morphologie-Kernel
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    # ── Öffentliche API ────────────────────────────────────────────────────────

    def process(self, frame: np.ndarray) -> Tuple["PathResult", np.ndarray]:
        """
        Analysiert einen BGR-Frame.

        Returns:
            (PathResult, annotierter Debug-Frame)
        """
        h, w = frame.shape[:2]
        self._update_roi(h, w)

        # ROI ausschneiden
        roi = frame[self._roi_y_top:self._roi_y_bottom, 0:w]

        # Grün-Maske für das gesamte ROI
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        roi_h = self._roi_y_bottom - self._roi_y_top

        # Gesamt-Schwerpunkt (für Lenkung)
        result = self._calc_overall(mask, w)

        if result.found:
            # Zonen-Schwerpunkte (für Knick-Erkennung)
            self._calc_bend(mask, result, w, roi_h)

        # Debug-Frame zeichnen
        debug = self._draw_overlay(frame.copy(), mask, result, w, h, roi_h)

        return result, debug

    def get_mask_only(self, frame: np.ndarray) -> np.ndarray:
        """Gibt nur die Grün-Maske zurück (für separates Debug-Fenster)."""
        h, w = frame.shape[:2]
        self._update_roi(h, w)
        roi  = frame[self._roi_y_top:self._roi_y_bottom, 0:w]
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        return mask

    def update_hsv_range(self, low: tuple, high: tuple):
        """HSV-Bereich zur Laufzeit anpassen."""
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

    def _calc_overall(self, mask: np.ndarray, frame_w: int) -> "PathResult":
        """Gesamter Schwerpunkt der Grün-Maske → Offset für Lenkung."""
        M    = cv2.moments(mask)
        area = M["m00"]

        if area < MIN_GREEN_AREA:
            return PathResult(found=False, area=area)

        cx = int(M["m10"] / area)
        cy = int(M["m01"] / area)

        center_x     = frame_w / 2.0
        offset_px    = cx - center_x
        offset_norm  = offset_px / center_x
        in_dead_zone = abs(offset_px) < self._dead_zone_px

        return PathResult(
            found=True,
            offset_normalized=float(offset_norm),
            centroid_x=cx,
            centroid_y=cy + self._roi_y_top,
            area=float(area),
            in_dead_zone=in_dead_zone,
        )

    def _calc_bend(self, mask: np.ndarray, result: "PathResult",
                   frame_w: int, roi_h: int):
        """
        Berechnet Knickwinkel durch Vergleich von Nah- und Fern-Zone.
        Schreibt direkt in das PathResult-Objekt.
        """
        # Zonen-Grenzen innerhalb der Maske (ROI-Koordinaten)
        far_end   = int(roi_h * FAR_ZONE_RATIO)            # obere Zone: 0 … far_end
        near_start = int(roi_h * NEAR_ZONE_RATIO)          # untere Zone: near_start … roi_h

        far_mask  = mask[:far_end,    :]
        near_mask = mask[near_start:, :]

        # Mindestfläche für Zonen etwas lockerer (halbe Gesamt-Mindestfläche)
        zone_min = MIN_GREEN_AREA * 0.4

        M_far  = cv2.moments(far_mask)
        M_near = cv2.moments(near_mask)

        far_area  = M_far["m00"]
        near_area = M_near["m00"]

        result.near_found = near_area >= zone_min
        result.far_found  = far_area  >= zone_min

        if not result.near_found or not result.far_found:
            # Zu wenig Pixel in einer Zone → Knick-Winkel unbekannt
            result.bend_angle_deg = 0.0
            result.bend_direction = "none"
            result.is_sharp_bend  = False
            result.speed_factor   = 1.0
            return

        # Schwerpunkte der Zonen (in Frame-Koordinaten)
        near_cx = int(M_near["m10"] / near_area)
        far_cx  = int(M_far ["m10"] / far_area)

        result.near_cx = near_cx
        result.far_cx  = far_cx

        # Vertikaler Abstand zwischen den Zonen-Mittelpunkten (px)
        near_cy_roi = near_start + int(M_near["m01"] / near_area)
        far_cy_roi  = int(M_far["m01"] / far_area)
        dy = max(abs(near_cy_roi - far_cy_roi), 1)   # verhindert Division durch 0

        # Knickwinkel
        dx            = float(far_cx - near_cx)
        angle_deg     = math.degrees(math.atan2(abs(dx), dy))
        result.bend_angle_deg = angle_deg
        result.bend_direction = "right" if dx > 0 else ("left" if dx < 0 else "none")

        # Geschwindigkeitsfaktor: 1.0 bei kleinem Knick, linear fallend
        if angle_deg <= BEND_SLOW_DEG:
            result.speed_factor  = 1.0
            result.is_sharp_bend = False
        elif angle_deg >= BEND_STOP_DEG:
            result.speed_factor  = 0.0
            result.is_sharp_bend = True
        else:
            # Lineares Abbremsen zwischen BEND_SLOW und BEND_STOP
            ratio = (angle_deg - BEND_SLOW_DEG) / (BEND_STOP_DEG - BEND_SLOW_DEG)
            result.speed_factor  = 1.0 - ratio * (1.0 - SPEED_MIN_FACTOR)
            result.is_sharp_bend = False

    # ── Debug-Overlay ──────────────────────────────────────────────────────────

    def _draw_overlay(self, frame: np.ndarray, mask: np.ndarray,
                      result: "PathResult", w: int, h: int, roi_h: int) -> np.ndarray:
        """Zeichnet alle Debug-Overlays auf den Frame."""

        roi_top    = self._roi_y_top
        roi_bottom = self._roi_y_bottom

        # ROI-Rahmen
        cv2.rectangle(frame, (0, roi_top), (w, roi_bottom), (255, 200, 0), 2)

        # Grün-Maske als transparentes Overlay
        roi_region = frame[roi_top:roi_bottom, :]
        overlay    = np.zeros_like(roi_region)
        overlay[mask > 0] = (0, 255, 0)
        cv2.addWeighted(roi_region, 0.7, overlay, 0.3, 0,
                        dst=frame[roi_top:roi_bottom, :])

        # Mittellinie + toter Bereich
        mid = w // 2
        dz  = self._dead_zone_px
        cv2.line(frame, (mid, roi_top), (mid, roi_bottom), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (mid - dz, roi_top), (mid - dz, roi_bottom), (100, 100, 255), 1)
        cv2.line(frame, (mid + dz, roi_top), (mid + dz, roi_bottom), (100, 100, 255), 1)

        # Zonen-Trennlinien
        far_y   = roi_top + int(roi_h * FAR_ZONE_RATIO)
        near_y  = roi_top + int(roi_h * NEAR_ZONE_RATIO)
        cv2.line(frame, (0, far_y),  (w, far_y),  (200, 150, 50), 1)   # orangefarben
        cv2.line(frame, (0, near_y), (w, near_y), (200, 150, 50), 1)
        cv2.putText(frame, "FERN",  (5, far_y  - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,150,50), 1)
        cv2.putText(frame, "NAH",   (5, near_y + 14),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,150,50), 1)

        if result.found:
            # Gesamt-Schwerpunkt
            cv2.drawMarker(frame, (result.centroid_x, result.centroid_y),
                           (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            # Nah/Fern-Schwerpunkte und Verbindungslinie (Knick visualisieren)
            if result.near_cx is not None and result.far_cx is not None:
                near_cy = roi_top + int(roi_h * NEAR_ZONE_RATIO) + (roi_bottom - roi_top - int(roi_h * NEAR_ZONE_RATIO)) // 2
                far_cy  = roi_top + int(roi_h * FAR_ZONE_RATIO) // 2

                cv2.circle(frame, (result.near_cx, near_cy), 6, (0, 200, 50),  -1)
                cv2.circle(frame, (result.far_cx,  far_cy),  6, (50, 50, 255), -1)

                # Linie Nah → Fern zeigt den Knick
                bend_color = (0, 255, 0) if not result.is_sharp_bend else (0, 0, 255)
                cv2.arrowedLine(frame,
                                (result.near_cx, near_cy),
                                (result.far_cx,  far_cy),
                                bend_color, 2, tipLength=0.2)

            # ── Texte ──────────────────────────────────────────────────────
            offset_pct = result.offset_normalized * 100
            sfactor    = result.speed_factor

            # Zeile 1: Offset
            off_color = (0, 255, 0) if result.in_dead_zone else (0, 165, 255)
            cv2.putText(frame, f"Offset: {offset_pct:+.1f}%",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.62, off_color, 2)

            # Zeile 2: Knickwinkel + Speed-Faktor
            if result.is_sharp_bend:
                bend_txt   = f"KNICK: {result.bend_angle_deg:.1f}° ► AUSRICHTEN"
                bend_color = (0, 0, 255)
            elif result.bend_angle_deg > BEND_SLOW_DEG:
                bend_txt   = f"Kurve: {result.bend_angle_deg:.1f}°  Speed x{sfactor:.2f}"
                bend_color = (0, 140, 255)
            else:
                bend_txt   = f"Winkel: {result.bend_angle_deg:.1f}°  Speed x{sfactor:.2f}"
                bend_color = (180, 255, 180)

            cv2.putText(frame, bend_txt,
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.62, bend_color, 2)
        else:
            cv2.putText(frame, "PFAD NICHT GEFUNDEN",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        return frame
