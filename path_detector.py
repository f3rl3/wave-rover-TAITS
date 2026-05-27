"""
Grüner-Pfad-Erkenner mit OpenCV
=================================
Verarbeitet jeden Kameraframe und gibt zurück:
  - Ob ein Pfad gefunden wurde
  - Den horizontalen Offset des Pfades (normalisiert: -1.0 links … +1.0 rechts)
  - Einen annotierten Debug-Frame

Strategie (vorwärts schauende Kamera):
  Nur die untere Region des Frames (ROI) auswerten, da das der
  Bereich direkt vor dem Rover ist. So wird Rauschen im Hintergrund ignoriert.
"""

import cv2
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
)

logger = logging.getLogger(__name__)


@dataclass
class PathResult:
    """Ergebnis einer Pfaderkennung für einen Frame."""
    found: bool                          # Wurde ein Pfad gefunden?
    offset_normalized: float = 0.0      # -1.0 (links) … 0.0 (Mitte) … +1.0 (rechts)
    centroid_x: Optional[int] = None    # Pixel-X des Pfad-Zentroids
    centroid_y: Optional[int] = None    # Pixel-Y des Pfad-Zentroids
    area: float = 0.0                   # Fläche des erkannten grünen Bereichs
    in_dead_zone: bool = False          # True wenn Offset vernachlässigbar klein


class PathDetector:
    """Erkennt grünen Pfad in Kameraframes."""

    def __init__(self):
        self._lower = np.array(GREEN_HSV_LOW,  dtype=np.uint8)
        self._upper = np.array(GREEN_HSV_HIGH, dtype=np.uint8)
        self._frame_w = FRAME_WIDTH
        self._frame_h = FRAME_HEIGHT
        self._dead_zone_px = int(FRAME_WIDTH * DEAD_ZONE_RATIO)

        # ROI Pixel-Grenzen (werden beim ersten Frame berechnet)
        self._roi_y_top    : Optional[int] = None
        self._roi_y_bottom : Optional[int] = None

    # ── Öffentliche API ────────────────────────────────────────────────────

    def process(self, frame: np.ndarray) -> Tuple[PathResult, np.ndarray]:
        """
        Analysiert einen BGR-Frame und gibt Pfad-Info + Debug-Frame zurück.

        Args:
            frame: OpenCV BGR-Bild

        Returns:
            (PathResult, annotierter Debug-Frame)
        """
        h, w = frame.shape[:2]
        self._update_roi(h, w)

        # ── ROI ausschneiden ──────────────────────────────────────────────
        roi = frame[self._roi_y_top:self._roi_y_bottom, 0:w]

        # ── Grün-Maske ────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)

        # Morphologisches Rauschen entfernen
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ── Schwerpunkt berechnen ─────────────────────────────────────────
        result = self._calculate_centroid(mask, w)

        # ── Debug-Overlay ──────────────────────────────────────────────────
        debug_frame = self._draw_overlay(frame.copy(), mask, result, w, h)

        return result, debug_frame

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
        logger.info("HSV aktualisiert: %s – %s", low, high)

    # ── Intern ────────────────────────────────────────────────────────────

    def _update_roi(self, h: int, w: int):
        """ROI-Grenzen (Pixel) neu berechnen wenn sich Framegröße ändert."""
        new_top    = int(h * ROI_TOP_RATIO)
        new_bottom = int(h * ROI_BOTTOM_RATIO)
        if new_top != self._roi_y_top or new_bottom != self._roi_y_bottom:
            self._roi_y_top    = new_top
            self._roi_y_bottom = new_bottom
            self._dead_zone_px = int(w * DEAD_ZONE_RATIO)
            logger.debug("ROI: y=%d…%d, Toter-Bereich: ±%dpx", new_top, new_bottom, self._dead_zone_px)

    def _calculate_centroid(self, mask: np.ndarray, frame_width: int) -> PathResult:
        """Berechnet Schwerpunkt aus Grün-Maske."""
        moments = cv2.moments(mask)
        area    = moments["m00"]

        if area < MIN_GREEN_AREA:
            return PathResult(found=False, area=area)

        # Schwerpunkt in ROI-Koordinaten
        cx_roi = int(moments["m10"] / area)
        cy_roi = int(moments["m01"] / area)

        # In Frame-Koordinaten umrechnen
        cx = cx_roi
        cy = cy_roi + self._roi_y_top

        # Normalisierter Offset: 0 = Mitte, -1 = ganz links, +1 = ganz rechts
        center_x        = frame_width / 2.0
        offset_px       = cx - center_x
        offset_norm     = offset_px / center_x           # -1.0 … +1.0
        in_dead_zone    = abs(offset_px) < self._dead_zone_px

        return PathResult(
            found=True,
            offset_normalized=float(offset_norm),
            centroid_x=cx,
            centroid_y=cy,
            area=float(area),
            in_dead_zone=in_dead_zone,
        )

    def _draw_overlay(self, frame: np.ndarray, mask: np.ndarray,
                      result: PathResult, w: int, h: int) -> np.ndarray:
        """Zeichnet Debug-Overlays auf den Frame."""
        # ROI-Rechteck
        cv2.rectangle(
            frame,
            (0, self._roi_y_top),
            (w, self._roi_y_bottom),
            (255, 200, 0), 2
        )

        # Grüne Maske als transparentes Overlay in der ROI
        roi_color = frame[self._roi_y_top:self._roi_y_bottom, :]
        green_overlay = np.zeros_like(roi_color)
        green_overlay[mask > 0] = (0, 255, 0)
        cv2.addWeighted(roi_color, 0.7, green_overlay, 0.3, 0,
                        dst=frame[self._roi_y_top:self._roi_y_bottom, :])

        # Mittellinie
        cv2.line(frame, (w // 2, self._roi_y_top), (w // 2, self._roi_y_bottom),
                 (255, 255, 255), 1, cv2.LINE_AA)

        # Toter-Bereich-Linien
        dz = self._dead_zone_px
        cx = w // 2
        cv2.line(frame, (cx - dz, self._roi_y_top), (cx - dz, self._roi_y_bottom),
                 (100, 100, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (cx + dz, self._roi_y_top), (cx + dz, self._roi_y_bottom),
                 (100, 100, 255), 1, cv2.LINE_AA)

        if result.found:
            # Schwerpunkt-Kreuz
            cv2.drawMarker(
                frame, (result.centroid_x, result.centroid_y),
                (0, 255, 255), cv2.MARKER_CROSS, 20, 2
            )
            # Verbindungslinie Mitte → Schwerpunkt
            cv2.arrowedLine(
                frame,
                (w // 2, (self._roi_y_top + self._roi_y_bottom) // 2),
                (result.centroid_x, result.centroid_y),
                (0, 200, 255), 2, tipLength=0.2
            )
            # Offset-Anzeige
            offset_pct = result.offset_normalized * 100
            color = (0, 255, 0) if result.in_dead_zone else (0, 165, 255)
            label = f"Offset: {offset_pct:+.1f}%  Fläche: {int(result.area)}"
            cv2.putText(frame, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        else:
            cv2.putText(frame, "⚠ PFAD NICHT GEFUNDEN", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        return frame
