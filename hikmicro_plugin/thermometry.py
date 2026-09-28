"""Polling Hikvision ISAPI thermometry into a temperature reading.

The thermal camera reports per-region radiometric temperatures over ISAPI. We
poll it at a fixed rate and forward the hottest reading in the scene as a
``sensor_msgs/Temperature``.
"""

import json
import re
import threading
import time
from typing import Any, Callable, List, Optional


def _log(message: str) -> None:
    """Log through rclpy when it is available, else print."""
    try:
        from rclpy.logging import get_logger

        get_logger("hikmicro_thermometry").info(message)
    except Exception:
        print(f"[hikmicro_thermometry] {message}", flush=True)


#: Temperature-named fields that are configured limits, not live readings.
_LIMIT_KEY_PARTS = ("alarm", "alert", "threshold", "warning", "limit", "tolerance")
#: Matches ``<...Temperature>-12.3</...>`` (namespace prefix optional) in XML.
_XML_TEMP = re.compile(
    r"<(?:\w+:)?(\w*[Tt]emperature)>\s*(-?\d+(?:\.\d+)?)\s*</", re.IGNORECASE
)


def _is_reading_key(key: str) -> bool:
    """Whether a temperature-named field is a live reading (not a limit)."""
    low = key.lower()
    return "temperature" in low and not any(p in low for p in _LIMIT_KEY_PARTS)


def _json_temps(obj: Any) -> List[float]:
    """Every reading temperature under a parsed JSON object, recursively."""
    temps: List[float] = []
    if isinstance(obj, dict):
        for key, value in obj.items():
            if isinstance(value, bool):
                continue  # JSON bools are ints in Python; never a temperature
            if isinstance(value, (int, float)) and _is_reading_key(key):
                temps.append(float(value))
            else:
                temps.extend(_json_temps(value))
    elif isinstance(obj, list):
        for item in obj:
            temps.extend(_json_temps(item))
    return temps


def parse_max_temperature(body: str) -> Optional[float]:
    """The hottest reading temperature in an ISAPI thermometry response.

    Accepts JSON or XML; returns ``None`` when the body carries no usable
    reading (empty, unparseable, or only configuration limits).
    """
    body = (body or "").strip()
    if not body:
        return None
    try:
        temps = _json_temps(json.loads(body))
    except (ValueError, TypeError):
        temps = []
    if not temps:
        temps = [
            float(match.group(2))
            for match in _XML_TEMP.finditer(body)
            if _is_reading_key(match.group(1))
        ]
    return max(temps) if temps else None


def reading_to_temperature_msg(payload: Any, frame_id: str, msg_type):
    """Convert a ``(temp_c, timestamp)`` pair into ``sensor_msgs/Temperature``."""
    if not isinstance(payload, tuple) or len(payload) != 2:
        return None
    temperature, stamp = payload
    if temperature is None:
        return None
    msg = msg_type()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    msg.temperature = float(temperature)
    msg.variance = 0.0
    return msg


class ThermometryPoller:
    """Poll a thermometry read function on a background thread.

    Delivers ``(temperature_c, timestamp)`` pairs to the callback; a read that
    returns ``None`` (or raises) is skipped.

    :param read_fn: Zero-arg callable returning the current max temperature in
        degrees Celsius, or ``None`` when there is no reading.
    :param poll_hz: Reads per second.
    """

    def __init__(
        self, read_fn: Callable[[], Optional[float]], poll_hz: float = 1.0
    ) -> None:
        self._read_fn = read_fn
        self.poll_hz = poll_hz
        self._callback: Optional[Callable[[Any], None]] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.readings = 0
        self.failures = 0

    # -- SdkCallbackTransport interface -------------------------------------
    def register(self, callback: Callable[[Any], None]) -> "ThermometryPoller":
        """Start polling and deliver readings to ``callback``."""
        self._callback = callback
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="thermometry-poll", daemon=True
        )
        self._thread.start()
        return self

    def unregister(self, _handle: Any = None) -> None:
        """Stop polling. Safe to call when never registered."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._callback = None

    # -- internals ----------------------------------------------------------
    def _run(self) -> None:
        interval = 1.0 / self.poll_hz if self.poll_hz else 1.0
        while not self._stop.is_set():
            try:
                temperature = self._read_fn()
            except Exception as exc:  # keep polling through transient errors
                self.failures += 1
                if self.failures <= 1:
                    _log(f"thermometry read failed: {exc} -- will keep polling")
                temperature = None
            if temperature is not None:
                self.readings += 1
                callback = self._callback
                if callback is not None:
                    callback((temperature, time.time()))
            self._stop.wait(interval)


__all__ = ["parse_max_temperature", "reading_to_temperature_msg", "ThermometryPoller"]
