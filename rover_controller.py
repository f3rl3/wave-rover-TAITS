"""
Wave Rover HTTP-Controller
===========================
Sends drive commands to the Wave Rover via its REST API.

Wave Rover API format (Waveshare UGV / Wave Rover):
  POST http://192.168.4.1/js
  Body: {"T": 1, "L": <left_speed>, "R": <right_speed>}

  T=1  → Motor control
  L    → Left side: -1.0 (backward) to 1.0 (forward)
  R    → Right side: -1.0 (backward) to 1.0 (forward)
"""

import json
import queue
import threading
import time
import logging
import requests
from config import ROVER_URL, HTTP_TIMEOUT

logger = logging.getLogger(__name__)

class RoverController:
    """Controls the Wave Rover via HTTP."""

    CMD_DRIVE = 1   # Motor control command type
    CMD_STOP  = 0   # Stop command type (some firmware versions)

    # Rover firmware stops motors when no command arrives (safety timeout).
    # We therefore resend at least every KEEPALIVE_S seconds, even with
    # identical values - otherwise the rover only turns briefly and then stops.
    KEEPALIVE_S = 0.15

    def __init__(self):
        self._last_l = 0.0
        self._last_r = 0.0
        self._last_send_t = 0.0
        self._connected = False
        self._session = requests.Session()
        self._session.headers.update({"Content-Type": "application/json"})

        # HTTP runs in a background thread so the main loop is never
        # blocked by network latency.
        # maxsize=1: only the latest command counts, older ones are discarded.
        self._cmd_queue: queue.Queue = queue.Queue(maxsize=1)
        self._sender = threading.Thread(
            target=self._sender_loop, name="RoverHTTP", daemon=True
        )
        self._sender.start()

    # ── Connection ─────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Tests the connection to the rover.
        Returns True if reachable, otherwise False.
        """
        try:
            # Short test ping with stop command (synchronous - we need immediate feedback)
            self._send_http({"T": self.CMD_DRIVE, "L": 0, "R": 0})
            self._connected = True
            logger.info("[OK] Rover connected at %s", ROVER_URL)
            return True
        except Exception as exc:
            logger.error("[ERR] Rover not reachable: %s", exc)
            self._connected = False
            return False

    # ── Drive commands ────────────────────────────────────────────────────────

    def drive(self, left: float, right: float) -> bool:
        """
        Sets both motor sides.

        Args:
            left:  Speed of left side  (-1.0 ... 1.0)
            right: Speed of right side (-1.0 ... 1.0)

        Returns:
            True if the command was sent successfully.
        """
        left  = self._clamp(left)
        right = self._clamp(right)

        now = time.time()
        same_values   = abs(left - self._last_l) < 0.001 and abs(right - self._last_r) < 0.001
        within_keepalive = (now - self._last_send_t) < self.KEEPALIVE_S

        # Skip only if values are equal AND keepalive interval has not yet expired.
        # The keepalive ensures that the rover firmware does not stop the motors.
        if same_values and within_keepalive:
            return True

        self._enqueue({"T": self.CMD_DRIVE, "L": round(left, 3), "R": round(right, 3)})
        self._last_l = left
        self._last_r = right
        self._last_send_t = now
        return True

    def forward(self, speed: float = 0.4) -> bool:
        """Drive straight ahead."""
        return self.drive(speed, speed)

    def stop(self) -> bool:
        """Stop immediately."""
        self._last_l = 999  # Force resend
        self._last_r = 999
        return self.drive(0.0, 0.0)

    def turn_in_place(self, speed: float = 0.3, direction: str = "left") -> bool:
        """
        Rotate in place (one wheel forward, other backward).

        Args:
            speed:     Rotation speed (0.0 - 1.0)
            direction: "left" or "right"
        """
        speed = abs(self._clamp(speed))
        if direction == "left":
            return self.drive(-speed, speed)
        else:
            return self.drive(speed, -speed)

    def steer(self, base_speed: float, offset_normalized: float,
              turn_max: float = 0.35, kp: float = 0.55, kd: float = 0.10,
              prev_error: float = 0.0) -> bool:
        """
        Steers the rover based on the error offset (PD controller).

        Args:
            base_speed:         Base speed (> 0 = forward)
            offset_normalized:  Normalized offset (-1.0 = far left, +1.0 = far right)
            turn_max:           Maximum steering difference between left/right
            kp:                 Proportional coefficient  (for offset -1...+1: ~0.4-0.7)
            kd:                 Differential coefficient  (dampens overshoot)
            prev_error:         Offset of the last frame (for D component)

        Returns:
            True if command was sent.
        """
        error      = offset_normalized
        derivative = error - prev_error
        correction = kp * error + kd * derivative
        correction = self._clamp(correction, -turn_max, turn_max)

        left  = base_speed - correction
        right = base_speed + correction

        # ── Backward protection ────────────────────────────────────────────────
        # When driving forward (base_speed > 0) no wheel may run backward.
        # Without this safeguard a wheel can go negative when base_speed
        # is heavily reduced by speed_factor (e.g. 0.12 - 0.35 = -0.23).
        # The inner wheel brakes to 0 instead - the rover still turns.
        if base_speed > 0:
            left  = max(0.0, left)
            right = max(0.0, right)

        # Normalize: no side above 1.0
        max_val = max(abs(left), abs(right), 1.0)
        left  /= max_val
        right /= max_val

        return self.drive(left, right)

    # ── Internal ─────────────────────────────────────────────────────────────

    def _enqueue(self, payload: dict):
        """Places command in the queue. If the queue is full, the old command is discarded."""
        try:
            self._cmd_queue.put_nowait(payload)
        except queue.Full:
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._cmd_queue.put_nowait(payload)
            except queue.Full:
                pass

    def _sender_loop(self):
        """Background thread: waits for commands and sends them via HTTP."""
        while True:
            try:
                payload = self._cmd_queue.get(timeout=1.0)
                self._send_http(payload)
            except queue.Empty:
                pass

    def _send_http(self, payload: dict) -> bool:
        """Blocking HTTP POST. Only called directly from connect() and _sender_loop()."""
        try:
            resp = self._session.post(
                ROVER_URL,
                data=json.dumps(payload),
                timeout=HTTP_TIMEOUT
            )
            return resp.status_code < 400
        except requests.exceptions.Timeout:
            logger.debug("HTTP timeout - rover busy?")
            return False
        except requests.exceptions.ConnectionError as exc:
            logger.warning("Connection error: %s", exc)
            self._connected = False
            return False
        except Exception as exc:
            logger.warning("Unknown error when sending: %s", exc)
            return False

    @staticmethod
    def _clamp(value: float, min_val: float = -1.0, max_val: float = 1.0) -> float:
        return max(min_val, min(max_val, value))

    def __del__(self):
        try:
            self._send_http({"T": self.CMD_DRIVE, "L": 0, "R": 0})
        except Exception:
            pass
