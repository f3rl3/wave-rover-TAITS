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
    RED_DETECT_Y_START,
    FAR_ZONE_END, NEAR_ZONE_START, NEAR_ZONE_END,
)

logger = logging.getLogger(__name__)

_RED_LOW1  = np.array((  0, 140,  80), dtype=np.uint8)   # lower red  (H near 0°)
_RED_HIGH1 = np.array(( 10, 255, 255), dtype=np.uint8)
_RED_LOW2  = np.array((165, 140,  80), dtype=np.uint8)   # upper red   (H near 180°)
_RED_HIGH2 = np.array((180, 255, 255), dtype=np.uint8)

MIN_RED_AREA = 10_000

@dataclass
class PathResult:
    found: bool

    # --- Offset (steering) ---
    offset_normalized: float = 0.0   # -1.0 (left) ... 0.0 (center) ... +1.0 (right)
    centroid_x: Optional[int] = None # Steering origin X
    centroid_y: Optional[int] = None # Y-position
    area: float = 0.0                # Total area of green pixels
    in_dead_zone: bool = False       # True if offset is negligibly small

    # --- Bend detection ---
    bend_angle_deg: float = 0.0      # Bend angle in degrees
    bend_direction: str = "none"     # "left", "right", "none"
    near_cx: Optional[int] = None    # Centroid NEAR zone
    far_cx: Optional[int] = None     # Centroid FAR zone
    near_found: bool = False
    far_found: bool = False

    # --- Derived control values ---
    is_sharp_bend: bool = False
    speed_factor: float = 1.0

    # --- Stripe alignment angle ---
    stripe_angle_deg: float = 0.0

class PathDetector:
    def __init__(self):
        self._lower = np.array(GREEN_HSV_LOW,  dtype=np.uint8)
        self._upper = np.array(GREEN_HSV_HIGH, dtype=np.uint8)
        self._dead_zone_px = int(FRAME_WIDTH * DEAD_ZONE_RATIO)
        self._roi_y_top:    Optional[int] = None
        self._roi_y_bottom: Optional[int] = None
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._last_mask: Optional[np.ndarray] = None   # Cache: mask of the last frame

    # --- Public properties ---

    @property
    def roi_top(self) -> int:
        return self._roi_y_top

    @property
    def roi_bottom(self) -> int:
        return self._roi_y_bottom

    @property
    def dead_zone_px(self) -> int:
        return self._dead_zone_px

    # --- API ---

    def process(self, frame: np.ndarray) -> PathResult:
        h, w = frame.shape[:2]
        self._update_roi(h, w)

        roi_h = self._roi_y_bottom - self._roi_y_top
        roi   = frame[self._roi_y_top:self._roi_y_bottom, :]

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        self._last_mask = mask

        result = self._calc_offset(mask, w, roi_h)

        if result.found:
            self._calc_bend(mask, result, w, roi_h)
            result.stripe_angle_deg = self._calc_stripe_angle(mask)

        return result

    def get_last_mask(self) -> Optional[np.ndarray]:
        return self._last_mask

    def detect_red(self, frame: np.ndarray) -> Tuple[bool, int]:
        h, w = frame.shape[:2]
        self._update_roi(h, w)

        red_y_start = int(h * RED_DETECT_Y_START)
        red_y_end   = self._roi_y_bottom
        roi = frame[red_y_start:red_y_end, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, _RED_LOW1, _RED_HIGH1)
        mask2 = cv2.inRange(hsv, _RED_LOW2, _RED_HIGH2)
        mask  = cv2.bitwise_or(mask1, mask2)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        self._last_red_mask = mask

        area = int(cv2.countNonZero(mask))
        return area >= MIN_RED_AREA, area

    def get_last_red_mask(self) -> Optional[np.ndarray]:
        return getattr(self, "_last_red_mask", None)

    def update_hsv_range(self, low: tuple, high: tuple):
        self._lower = np.array(low,  dtype=np.uint8)
        self._upper = np.array(high, dtype=np.uint8)

    # --- Internal calculations ---

    def _calc_stripe_angle(self, mask: np.ndarray) -> float:
        ys, xs = np.where(mask > 0)
        if len(xs) < 30:
            return 0.0

        pts = np.column_stack([xs, ys]).astype(np.float32).reshape(-1, 1, 2)
        out = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        vx, vy = float(out[0]), float(out[1])

        if vy > 0:
            vx, vy = -vx, -vy

        return math.degrees(math.atan2(vx, -vy))

    def _update_roi(self, h: int, w: int):
        new_top    = int(h * ROI_TOP_RATIO)
        new_bottom = int(h * ROI_BOTTOM_RATIO)
        if new_top != self._roi_y_top or new_bottom != self._roi_y_bottom:
            self._roi_y_top    = new_top
            self._roi_y_bottom = new_bottom
            self._dead_zone_px = int(w * DEAD_ZONE_RATIO)
            logger.debug(
                "ROI updated: y=%d...%d  (FAR=top %.0f%%  NEAR=%.0f-%.0f%%)",
                new_top, new_bottom,
                FAR_ZONE_END * 100, NEAR_ZONE_START * 100, NEAR_ZONE_END * 100
            )

    def _calc_offset(self, mask: np.ndarray, frame_w: int, roi_h: int) -> PathResult:
        M_full = cv2.moments(mask)
        area   = M_full["m00"]

        if area < MIN_GREEN_AREA:
            return PathResult(found=False, area=area)

        far_end  = int(roi_h * FAR_ZONE_END)
        far_mask = mask[:far_end, :]
        M_far    = cv2.moments(far_mask)

        if M_far["m00"] >= MIN_GREEN_AREA * 0.25:
            cx = int(M_far["m10"] / M_far["m00"])
            cy = int(M_far["m01"] / M_far["m00"])
        else:
            cx = int(M_full["m10"] / area)
            cy = int(M_full["m01"] / area)

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

    def _calc_bend(self, mask: np.ndarray, result: PathResult,
                   frame_w: int, roi_h: int):
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

        # Vertical distance between zone centroids
        near_cy_roi = near_start + int(M_near["m01"] / near_area)
        far_cy_roi  = int(M_far["m01"] / far_area)
        dy = max(abs(near_cy_roi - far_cy_roi), 1)

        # Bend angle & direction
        dx = float(far_cx - near_cx)
        angle_deg = math.degrees(math.atan2(abs(dx), dy))
        result.bend_angle_deg = angle_deg
        result.bend_direction = "right" if dx > 0 else ("left" if dx < 0 else "none")

        # Speed factor
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

