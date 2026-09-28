"""Sugarcoat sensor plugin for HIKMICRO / Hikvision bi-spectrum PTZ cameras.

Two plugin classes:

- ``HikvisionPtzCamera``: a Hikvision network PTZ camera: a visible / optical
  zoom camera on a pan-tilt positioning system, streamed over RTSP and driven
  over the Hikvision ISAPI HTTP API. Covers any Hikvision PTZ camera.
- ``HikmicroBispectrum``: adds the co-mounted thermal stream, radiometric
  thermometry and over-temperature events, for the HM-TD / DS-2TD bi-spectrum
  positioning systems.

Both are ``SensorPlugin``s: they attach to a recipe alongside the robot plugin
and are placed with a recipe ``Mount``.
"""

__all__ = ["HikvisionPtzCamera", "HikmicroBispectrum"]


def __getattr__(name):
    # PEP 562 lazy attribute: only pull in plugin.py when a plugin class is
    # actually requested.
    if name in __all__:
        from . import plugin

        return getattr(plugin, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
