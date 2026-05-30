"""
Wave Rover HTTP-Controller
===========================
Sendet Fahrbefehle an den Wave Rover über seine REST-API.

Wave Rover API-Format (Waveshare UGV / Wave Rover):
  POST http://192.168.4.1/js
  Body: {"T": 1, "L": <left_speed>, "R": <right_speed>}

  T=1  → Motorsteuerung
  L    → Linke Seite: -1.0 (zurück) bis 1.0 (vorwärts)
  R    → Rechte Seite: -1.0 (zurück) bis 1.0 (vorwärts)
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
    """Steuert den Wave Rover über HTTP."""

    CMD_DRIVE = 1   # Motorsteuerungs-Befehlstyp
    CMD_STOP  = 0   # Stop-Befehlstyp (manche Firmware-Versionen)

    # Rover-Firmware stoppt Motoren wenn kein Befehl kommt (Sicherheits-Timeout).
    # Wir senden deshalb mindestens alle KEEPALIVE_S Sekunden neu, auch bei
    # identischen Werten – sonst dreht der Rover nur kurz und stoppt dann.
    KEEPALIVE_S = 0.15

    def __init__(self):
        self._last_l = 0.0
        self._last_r = 0.0
        self._last_send_t = 0.0
        self._connected = False
        self._session = requests.Session()
        self._session.headers.update({"Content-Type": "application/json"})

        # HTTP läuft in einem Hintergrund-Thread, damit der Main-Loop nie
        # durch Netzwerklatenz blockiert wird.
        # maxsize=1: nur der neueste Befehl zählt, ältere werden verworfen.
        self._cmd_queue: queue.Queue = queue.Queue(maxsize=1)
        self._sender = threading.Thread(
            target=self._sender_loop, name="RoverHTTP", daemon=True
        )
        self._sender.start()

    # ── Verbindung ─────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Testet die Verbindung zum Rover.
        Gibt True zurück wenn erreichbar, sonst False.
        """
        try:
            # Kurzer Test-Ping mit Stop-Befehl (synchron – wir brauchen sofortiges Feedback)
            self._send_http({"T": self.CMD_DRIVE, "L": 0, "R": 0})
            self._connected = True
            logger.info("✅ Rover verbunden unter %s", ROVER_URL)
            return True
        except Exception as exc:
            logger.error("❌ Rover nicht erreichbar: %s", exc)
            self._connected = False
            return False

    # ── Fahrbefehle ────────────────────────────────────────────────────────

    def drive(self, left: float, right: float) -> bool:
        """
        Setzt beide Motorseiten.

        Args:
            left:  Geschwindigkeit linke Seite  (-1.0 … 1.0)
            right: Geschwindigkeit rechte Seite (-1.0 … 1.0)

        Returns:
            True wenn der Befehl erfolgreich gesendet wurde.
        """
        left  = self._clamp(left)
        right = self._clamp(right)

        now = time.time()
        same_values   = abs(left - self._last_l) < 0.001 and abs(right - self._last_r) < 0.001
        within_keepalive = (now - self._last_send_t) < self.KEEPALIVE_S

        # Überspringen nur wenn Werte gleich UND Keepalive-Intervall noch nicht abgelaufen.
        # Das Keepalive stellt sicher, dass die Rover-Firmware die Motoren nicht stoppt.
        if same_values and within_keepalive:
            return True

        self._enqueue({"T": self.CMD_DRIVE, "L": round(left, 3), "R": round(right, 3)})
        self._last_l = left
        self._last_r = right
        self._last_send_t = now
        return True

    def forward(self, speed: float = 0.4) -> bool:
        """Geradeaus fahren."""
        return self.drive(speed, speed)

    def stop(self) -> bool:
        """Sofort stoppen."""
        self._last_l = 999  # Zwinge erneutes Senden
        self._last_r = 999
        return self.drive(0.0, 0.0)

    def turn_in_place(self, speed: float = 0.3, direction: str = "left") -> bool:
        """
        Auf der Stelle drehen (ein Rad vorwärts, anderes rückwärts).

        Args:
            speed:     Rotationsgeschwindigkeit (0.0 – 1.0)
            direction: "left" oder "right"
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
        Lenkt den Rover basierend auf dem Fehler-Offset (PD-Regler).

        Args:
            base_speed:         Grundgeschwindigkeit (> 0 = vorwärts)
            offset_normalized:  Normalisierter Offset (-1.0 = ganz links, +1.0 = ganz rechts)
            turn_max:           Maximale Lenkdifferenz zwischen links/rechts
            kp:                 Proportional-Koeffizient  (für Offset -1…+1: ~0.4–0.7)
            kd:                 Differenzial-Koeffizient  (dämpft Überschwingen)
            prev_error:         Offset des letzten Frames (für D-Anteil)

        Returns:
            True wenn Befehl gesendet wurde.
        """
        error      = offset_normalized
        derivative = error - prev_error
        correction = kp * error + kd * derivative
        correction = self._clamp(correction, -turn_max, turn_max)

        left  = base_speed - correction
        right = base_speed + correction

        # ── Rückwärtsschutz ────────────────────────────────────────────────
        # Bei Vorwärtsfahrt (base_speed > 0) darf kein Rad rückwärts laufen.
        # Ohne diese Absicherung kann ein Rad negativ werden, wenn base_speed
        # durch speed_factor stark reduziert wurde (z.B. 0.12 – 0.35 = -0.23).
        # Das innere Rad bremst stattdessen auf 0 – der Rover dreht trotzdem.
        if base_speed > 0:
            left  = max(0.0, left)
            right = max(0.0, right)

        # Normalisieren: keine Seite über 1.0
        max_val = max(abs(left), abs(right), 1.0)
        left  /= max_val
        right /= max_val

        return self.drive(left, right)

    # ── Intern ─────────────────────────────────────────────────────────────

    def _enqueue(self, payload: dict):
        """Legt Befehl in die Queue. Bei voller Queue wird der alte Befehl verworfen."""
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
        """Hintergrund-Thread: wartet auf Befehle und sendet sie per HTTP."""
        while True:
            try:
                payload = self._cmd_queue.get(timeout=1.0)
                self._send_http(payload)
            except queue.Empty:
                pass

    def _send_http(self, payload: dict) -> bool:
        """Blockierender HTTP-POST. Nur direkt von connect() und _sender_loop() aufgerufen."""
        try:
            resp = self._session.post(
                ROVER_URL,
                data=json.dumps(payload),
                timeout=HTTP_TIMEOUT
            )
            return resp.status_code < 400
        except requests.exceptions.Timeout:
            logger.debug("HTTP Timeout – Rover beschäftigt?")
            return False
        except requests.exceptions.ConnectionError as exc:
            logger.warning("Verbindungsfehler: %s", exc)
            self._connected = False
            return False
        except Exception as exc:
            logger.warning("Unbekannter Fehler beim Senden: %s", exc)
            return False

    @staticmethod
    def _clamp(value: float, min_val: float = -1.0, max_val: float = 1.0) -> float:
        return max(min_val, min(max_val, value))

    def __del__(self):
        try:
            self._send_http({"T": self.CMD_DRIVE, "L": 0, "R": 0})
        except Exception:
            pass
