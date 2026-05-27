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
import time
import logging
import requests
from config import ROVER_URL, HTTP_TIMEOUT

logger = logging.getLogger(__name__)


class RoverController:
    """Steuert den Wave Rover über HTTP."""

    CMD_DRIVE = 1   # Motorsteuerungs-Befehlstyp
    CMD_STOP  = 0   # Stop-Befehlstyp (manche Firmware-Versionen)

    def __init__(self):
        self._last_l = 0.0
        self._last_r = 0.0
        self._connected = False
        self._session = requests.Session()
        self._session.headers.update({"Content-Type": "application/json"})

    # ── Verbindung ─────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Testet die Verbindung zum Rover.
        Gibt True zurück wenn erreichbar, sonst False.
        """
        try:
            # Kurzer Test-Ping mit Stop-Befehl
            self._send({"T": self.CMD_DRIVE, "L": 0, "R": 0})
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

        # Gleiche Werte nicht erneut senden → reduziert Netzwerklast
        if abs(left - self._last_l) < 0.01 and abs(right - self._last_r) < 0.01:
            return True

        success = self._send({"T": self.CMD_DRIVE, "L": round(left, 3), "R": round(right, 3)})
        if success:
            self._last_l = left
            self._last_r = right
        return success

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
              turn_max: float = 0.35, kp: float = 0.0025, kd: float = 0.0005,
              prev_error: float = 0.0) -> bool:
        """
        Lenkt den Rover basierend auf dem Fehler-Offset.

        Args:
            base_speed:         Grundgeschwindigkeit
            offset_normalized:  Normalisierter Offset (-1.0 = ganz links, +1.0 = ganz rechts)
            turn_max:           Maximale Lenkdifferenz
            kp:                 Proportional-Koeffizient
            kd:                 Differenzial-Koeffizient
            prev_error:         Vorheriger Fehler (für D-Anteil)

        Returns:
            True wenn Befehl gesendet wurde.
        """
        error = offset_normalized
        derivative = error - prev_error
        correction = kp * error + kd * derivative
        correction = self._clamp(correction, -turn_max, turn_max)

        left  = base_speed - correction
        right = base_speed + correction

        # Normalisieren damit keine Seite über 1.0 geht
        max_val = max(abs(left), abs(right), 1.0)
        left  /= max_val
        right /= max_val

        return self.drive(left, right)

    # ── Intern ─────────────────────────────────────────────────────────────

    def _send(self, payload: dict) -> bool:
        """Sendet JSON-Payload an den Rover. Gibt True bei Erfolg zurück."""
        try:
            resp = self._session.post(
                ROVER_URL,
                data=json.dumps(payload),
                timeout=HTTP_TIMEOUT
            )
            # Manche Rover-Firmwares antworten mit 200, manche mit anderen Codes
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
        """Stoppe den Rover beim Beenden des Objekts."""
        try:
            self.stop()
        except Exception:
            pass
