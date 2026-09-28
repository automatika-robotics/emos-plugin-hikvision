"""Wire-format helpers for the Hikvision ISAPI API.

Continuous / momentary speeds are signed integers in ``-100..100`` (0 stops that
axis). Absolute positions use the ISAPI ``AbsoluteHigh`` space: ``azimuth``
0..3600 (0.1 deg), ``elevation`` in 0.1-deg units, ``absoluteZoom`` an integer.
The absolute *angle* mapping is model-dependent.
"""

from typing import Optional

_ABS_NS = 'version="2.0" xmlns="http://www.isapi.org/ver20/XMLSchema"'


def _clamp(value, low: int, high: int) -> int:
    return max(low, min(high, round(value)))


def continuous_xml(pan=0, tilt=0, zoom=0) -> str:
    """A continuous-move body; each axis a signed speed -100..100 (0 = stop)."""
    return (
        f"<PTZData><pan>{_clamp(pan, -100, 100)}</pan>"
        f"<tilt>{_clamp(tilt, -100, 100)}</tilt>"
        f"<zoom>{_clamp(zoom, -100, 100)}</zoom></PTZData>"
    )


def stop_xml() -> str:
    """A continuous body with every axis zero. Stops all motion."""
    return continuous_xml(0, 0, 0)


def momentary_xml(pan=0, tilt=0, zoom=0, duration_ms=500) -> str:
    """A momentary move: like continuous, but auto-stops after ``duration_ms``."""
    return (
        f"<PTZData><pan>{_clamp(pan, -100, 100)}</pan>"
        f"<tilt>{_clamp(tilt, -100, 100)}</tilt>"
        f"<zoom>{_clamp(zoom, -100, 100)}</zoom>"
        f"<Momentary><duration>{max(0, int(duration_ms))}</duration></Momentary>"
        f"</PTZData>"
    )


def absolute_xml(pan_deg, tilt_deg, zoom: Optional[float] = None) -> str:
    """An absolute-position body from pan/tilt in degrees (+ optional zoom).

    ``azimuth`` is pan wrapped to 0..360 deg in 0.1-deg units; ``elevation`` is
    tilt in 0.1-deg units.
    """
    # TODO: Confirm the elevation sign convention on the unit.
    azimuth = _clamp((pan_deg % 360) * 10, 0, 3600)
    elevation = _clamp(tilt_deg * 10, -900, 900)
    zoom_tag = "" if zoom is None else f"<absoluteZoom>{int(zoom)}</absoluteZoom>"
    return (
        f"<PTZData {_ABS_NS}><AbsoluteHigh>"
        f"<elevation>{elevation}</elevation>"
        f"<azimuth>{azimuth}</azimuth>{zoom_tag}"
        f"</AbsoluteHigh></PTZData>"
    )


def preset_xml(preset_id: int, name: Optional[str] = None) -> str:
    """A body to store the current position as preset ``preset_id``."""
    label = name or f"Preset {int(preset_id)}"
    return (
        f"<PTZPreset {_ABS_NS}><enabled>true</enabled>"
        f"<id>{int(preset_id)}</id><presetName>{label}</presetName></PTZPreset>"
    )


__all__ = [
    "continuous_xml",
    "stop_xml",
    "momentary_xml",
    "absolute_xml",
    "preset_xml",
]
