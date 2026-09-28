"""HIKMICRO / Hikvision bi-spectrum PTZ camera SensorPlugin."""

import os
from typing import Any, Callable, Dict, Optional, Tuple

from ros_sugar.core.action import Action
from ros_sugar.core.event import Event
from ros_sugar.robot import (
    ActionRegistry,
    EventRegistry,
    Feedback,
    PluginMetadata,
    SdkCallbackTransport,
    SensorPlugin,
    create_supported_type,
    plugin_action,
)
from ros_sugar.supported_types import Image
from sensor_msgs.msg import Temperature as ROSTemperature

from .codecs import absolute_xml, momentary_xml, preset_xml, stop_xml
from .isapi import IsapiClient, IsapiError
from .rtsp import RtspCamera, decode_frame
from .thermometry import (
    ThermometryPoller,
    parse_max_temperature,
    reading_to_temperature_msg,
)

#: ``sensor_msgs/Temperature`` as a SupportedType, for the thermometry feedback.
Temperature = create_supported_type(ROSTemperature)


class HikvisionPtzCamera(SensorPlugin):
    """A Hikvision network PTZ camera (visible stream over RTSP + PTZ over ISAPI).

    Defaults target a factory-fresh Hikvision unit (IP ``192.168.1.64``). These
    cameras ship **inactive**: the unit must be activated and given a password
    before any RTSP/ISAPI request succeeds.

    Per-deployment knobs (address, ports, channels, stream rate) can be passed
    to the constructor or set as class attributes on a subclass; the class
    attributes are the defaults. Credentials are the exception.
    """

    # --- network endpoints ---------------------------------------------------
    #: Camera IP on the robot's internal LAN (Hikvision factory default).
    CAMERA_IP = "192.168.1.64"
    #: ISAPI HTTP port (PTZ + thermometry).
    HTTP_PORT = 80
    RTSP_PORT = 554
    #: Credentials. Resolved host side.
    USERNAME = "admin"
    PASSWORD = ""
    USERNAME_ENV = "HIKMICRO_USER"
    PASSWORD_ENV = "HIKMICRO_PASS"

    # --- RTSP video ----------------------------------------------------------
    #: Hikvision channel id = camera_no * 100 + stream_no; optical = camera 1.
    VISIBLE_CHANNEL = 101  # optical main stream (102 = sub-stream)
    #: Frames-per-second forwarded onto the feedback bus (0 = every frame).
    VISIBLE_MAX_FPS = 15.0

    # --- PTZ (ISAPI, channel 1; shared by both optics) -----------------------
    HAS_PTZ = True
    PTZ_CHANNEL = 1
    #: Default continuous-move speeds; ISAPI range is -100..100. The camera does
    #: pan 360 continuous, tilt -90..+90, up to ~90 deg/s pan, ~40 deg/s tilt.
    PAN_SPEED = 30
    TILT_SPEED = 30
    ZOOM_SPEED = 30
    #: Duration (ms) of a directional nudge (a momentary move).
    MOMENTARY_MS = 500

    # --- capabilities this base class does NOT expose ------------------------
    HAS_THERMAL = False
    HAS_THERMOMETRY = False

    #: Frame the visible stream is expressed in, appended to the sensor's own frame_id
    VISIBLE_FRAME_SUFFIX = "visible_optical"

    def __init__(
        self,
        camera_ip: Optional[str] = None,
        http_port: Optional[int] = None,
        rtsp_port: Optional[int] = None,
        visible_channel: Optional[int] = None,
        visible_max_fps: Optional[float] = None,
        ptz_channel: Optional[int] = None,
    ):
        # Constructor arguments override the class defaults; None keeps them.
        for attr, value in (
            ("CAMERA_IP", camera_ip),
            ("HTTP_PORT", http_port),
            ("RTSP_PORT", rtsp_port),
            ("VISIBLE_CHANNEL", visible_channel),
            ("VISIBLE_MAX_FPS", visible_max_fps),
            ("PTZ_CHANNEL", ptz_channel),
        ):
            if value is not None:
                setattr(self, attr, value)

        self.metadata = PluginMetadata(
            name="Hikvision PTZ Camera",
            vendor="Hikvision",
            version="1.0",
            description=(
                "A Hikvision network PTZ camera: a visible, optical-zoom camera "
                "on a pan-tilt positioning system. Streams over RTSP and is "
                "aimed (pan / tilt / zoom, presets) over the Hikvision ISAPI "
                "HTTP API. Mounts on a robot or in the environment as an EMOS "
                "sensor."
            ),
        )
        # --- visible RTSP video ---------------------------------------------
        user, password = self._credentials()
        self._visible_camera = RtspCamera(
            url=self._rtsp_url(self.VISIBLE_CHANNEL, user, password),
            max_fps=self.VISIBLE_MAX_FPS,
        )
        visible = SdkCallbackTransport(
            "visible",
            register_fn=self._visible_camera.register,
            unregister_fn=self._visible_camera.unregister,
        )
        self.transports = {"visible": visible}
        self.feedbacks = {
            "visible_image": Feedback(
                key="visible_image",
                msg_type=Image,
                transport=visible,
                decoder=lambda payload: decode_frame(
                    payload, self._frame(self.VISIBLE_FRAME_SUFFIX)
                ),
                rate_hz=self.VISIBLE_MAX_FPS or None,
                description="Visible / optical camera, decoded in-plugin from RTSP",
            ),
        }
        # --- ISAPI HTTP client (PTZ and thermometry share one) --------------
        if self.HAS_PTZ or self.HAS_THERMOMETRY:
            self._isapi = IsapiClient(
                base_url=f"http://{self.CAMERA_IP}:{self.HTTP_PORT}",
                username=user,
                password=password,
            )
        # --- PTZ over ISAPI (channel 1; both optics move together) ----------
        if self.HAS_PTZ:
            self.actions = ActionRegistry(
                {
                    "look_at": self._make_look_at,
                    "pan_left": self._make_pan_left,
                    "pan_right": self._make_pan_right,
                    "tilt_up": self._make_tilt_up,
                    "tilt_down": self._make_tilt_down,
                    "zoom_in": self._make_zoom_in,
                    "zoom_out": self._make_zoom_out,
                    "stop_ptz": self._make_stop_ptz,
                    "goto_home": self._make_goto_home,
                    "goto_preset": self._make_goto_preset,
                    "set_preset": self._make_set_preset,
                }
            )

    # --- helpers -------------------------------------------------------------
    def _credentials(self):
        """Username / password, preferring the environment over class defaults."""
        return (
            os.environ.get(self.USERNAME_ENV) or self.USERNAME,
            os.environ.get(self.PASSWORD_ENV) or self.PASSWORD,
        )

    def _rtsp_url(self, channel: int, user: str, password: str) -> str:
        """Hikvision RTSP URL for a channel (id = camera_no * 100 + stream_no)."""
        auth = f"{user}:{password}@" if user else ""
        return (
            f"rtsp://{auth}{self.CAMERA_IP}:{self.RTSP_PORT}"
            f"/Streaming/Channels/{channel}"
        )

    def _frame(self, suffix: str) -> str:
        """Per-channel optical frame, namespaced by this sensor's id."""
        return f"{self.id}_{suffix}"

    # --- PTZ actions (ISAPI PUTs to /PTZCtrl/channels/<n>/...) ---------------
    def _ptz_path(self, verb: str) -> str:
        return f"/ISAPI/PTZCtrl/channels/{self.PTZ_CHANNEL}/{verb}"

    def _put(self, name: str, path: str, xml: str, moves: bool) -> Tuple[bool, str]:
        """PUT to the camera

        A success means the camera accepted the command: a move returns before
        the head arrives.
        """
        try:
            self._isapi.put_xml(path, xml)
        except IsapiError as e:
            return False, f"{name} failed: {e}"
        message = f"{name}: the camera accepted PUT {path}"
        if moves:
            message += "; the head may still be moving"
        return True, message

    @staticmethod
    def _ptz_action(
        name: str,
        send: Callable[..., Tuple[bool, str]],
        kwargs: Optional[Dict[str, Any]] = None,
        **action_kwargs,
    ) -> Action:
        """The Action for one PTZ command.

        ``send``'s arguments go in ``kwargs`` so a recipe can pass topic values;
        ``action_kwargs`` (``max_retries=``, ``on_fail=`` ...) go to the Action.
        """
        # An Action is named after its method unless told otherwise.
        send.__name__ = name
        return Action(method=send, kwargs=kwargs, **action_kwargs)

    def _momentary(self, name: str, pan=0, tilt=0, zoom=0, **action_kwargs) -> Action:
        """A brief move that auto-stops. Directional nudges."""

        def send() -> Tuple[bool, str]:
            xml = momentary_xml(pan, tilt, zoom, self.MOMENTARY_MS)
            return self._put(name, self._ptz_path("momentary"), xml, moves=True)

        return self._ptz_action(name, send, **action_kwargs)

    @plugin_action(
        description={
            "name": "look_at",
            "description": (
                "Aim the camera to an absolute pan and tilt (in degrees), with "
                "an optional zoom."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "pan_deg": {
                        "type": "number",
                        "description": "Absolute pan in degrees (0-360).",
                    },
                    "tilt_deg": {
                        "type": "number",
                        "description": "Absolute tilt in degrees (-90 down .. +90 up).",
                    },
                    "zoom": {
                        "type": "number",
                        "description": "Optional absolute zoom level.",
                    },
                },
                "required": ["pan_deg", "tilt_deg"],
            },
        }
    )
    def _make_look_at(
        self, pan_deg=0.0, tilt_deg=0.0, zoom=None, **action_kwargs
    ) -> Action:
        """Aim at an absolute pan / tilt in degrees; values may be topic references."""

        # A topic argument that has not published yet stops the call.
        def send(pan_deg, tilt_deg, zoom=None) -> Tuple[bool, str]:
            xml = absolute_xml(pan_deg, tilt_deg, zoom)
            return self._put("look_at", self._ptz_path("absolute"), xml, moves=True)

        return self._ptz_action(
            "look_at",
            send,
            kwargs={"pan_deg": pan_deg, "tilt_deg": tilt_deg, "zoom": zoom},
            **action_kwargs,
        )

    @plugin_action(description="Nudge the camera left (a brief pan).")
    def _make_pan_left(self, **action_kwargs) -> Action:
        """Nudge the camera left."""
        return self._momentary("pan_left", pan=-self.PAN_SPEED, **action_kwargs)

    @plugin_action(description="Nudge the camera right (a brief pan).")
    def _make_pan_right(self, **action_kwargs) -> Action:
        """Nudge the camera right."""
        return self._momentary("pan_right", pan=self.PAN_SPEED, **action_kwargs)

    @plugin_action(description="Nudge the camera up (a brief tilt).")
    def _make_tilt_up(self, **action_kwargs) -> Action:
        """Nudge the camera up."""
        return self._momentary("tilt_up", tilt=self.TILT_SPEED, **action_kwargs)

    @plugin_action(description="Nudge the camera down (a brief tilt).")
    def _make_tilt_down(self, **action_kwargs) -> Action:
        """Nudge the camera down."""
        return self._momentary("tilt_down", tilt=-self.TILT_SPEED, **action_kwargs)

    @plugin_action(description="Zoom the camera in (a brief zoom).")
    def _make_zoom_in(self, **action_kwargs) -> Action:
        """Zoom in a step."""
        return self._momentary("zoom_in", zoom=self.ZOOM_SPEED, **action_kwargs)

    @plugin_action(description="Zoom the camera out (a brief zoom).")
    def _make_zoom_out(self, **action_kwargs) -> Action:
        """Zoom out a step."""
        return self._momentary("zoom_out", zoom=-self.ZOOM_SPEED, **action_kwargs)

    @plugin_action(description="Stop all PTZ motion immediately.")
    def _make_stop_ptz(self, **action_kwargs) -> Action:
        """Stop all PTZ motion."""

        def send() -> Tuple[bool, str]:
            path = self._ptz_path("continuous")
            return self._put("stop_ptz", path, stop_xml(), moves=False)

        return self._ptz_action("stop_ptz", send, **action_kwargs)

    @plugin_action(description="Return the camera to its home position.")
    def _make_goto_home(self, **action_kwargs) -> Action:
        """Return the camera to its home position."""

        def send() -> Tuple[bool, str]:
            path = self._ptz_path("homeposition/goto")
            return self._put("goto_home", path, "", moves=True)

        return self._ptz_action("goto_home", send, **action_kwargs)

    @plugin_action(
        description={
            "name": "goto_preset",
            "description": "Recall a stored PTZ preset by number.",
            "parameters": {
                "type": "object",
                "properties": {
                    "preset": {"type": "integer", "description": "Preset number."}
                },
                "required": ["preset"],
            },
        }
    )
    def _make_goto_preset(self, preset=1, **action_kwargs) -> Action:
        """Recall a stored preset; ``preset`` may be a topic reference."""

        def send(preset) -> Tuple[bool, str]:
            path = self._ptz_path(f"presets/{int(preset)}/goto")
            return self._put("goto_preset", path, "", moves=True)

        return self._ptz_action(
            "goto_preset", send, kwargs={"preset": preset}, **action_kwargs
        )

    @plugin_action(
        description={
            "name": "set_preset",
            "description": "Store the current camera position as a PTZ preset.",
            "parameters": {
                "type": "object",
                "properties": {
                    "preset": {
                        "type": "integer",
                        "description": "Preset number to store.",
                    }
                },
                "required": ["preset"],
            },
        }
    )
    def _make_set_preset(self, preset=1, **action_kwargs) -> Action:
        """Store the current position as a preset; ``preset`` may be a topic reference."""

        def send(preset) -> Tuple[bool, str]:
            path = self._ptz_path(f"presets/{int(preset)}")
            return self._put("set_preset", path, preset_xml(int(preset)), moves=False)

        return self._ptz_action(
            "set_preset", send, kwargs={"preset": preset}, **action_kwargs
        )


class HikmicroBispectrum(HikvisionPtzCamera):
    """A HIKMICRO bi-spectrum PTZ positioning system (visible + thermal).

    Adds the co-mounted thermal stream and radiometric thermometry to the base
    Hikvision PTZ camera. Both optics move together on the single PTZ channel.
    """

    HAS_THERMAL = True
    HAS_THERMOMETRY = True

    # --- thermal RTSP + thermometry (thermal = camera 2) ---------------------
    THERMAL_CHANNEL = 201  # thermal main stream (202 = sub-stream)
    THERMAL_MAX_FPS = 15.0
    #: ISAPI thermometry addresses the thermal sensor as metering channel 2.
    THERMAL_METERING_CHANNEL = 2
    #: ISAPI path for one thermometry read; ``{chan}`` is the metering channel.
    #: Model specific: confirm on the unit at bring-up.
    THERMOMETRY_PATH = "/ISAPI/Thermal/channels/{chan}/thermometry/1/rules"
    #: Radiometric temperature poll rate (Hz) and default over-temp threshold.
    THERMOMETRY_POLL_HZ = 1.0
    OVER_TEMPERATURE_C = 60.0
    THERMAL_FRAME_SUFFIX = "thermal_optical"

    def __init__(
        self,
        thermal_channel: Optional[int] = None,
        thermal_max_fps: Optional[float] = None,
        thermal_metering_channel: Optional[int] = None,
        thermometry_path: Optional[str] = None,
        thermometry_poll_hz: Optional[float] = None,
        over_temperature_c: Optional[float] = None,
        camera_ip: Optional[str] = None,
        http_port: Optional[int] = None,
        rtsp_port: Optional[int] = None,
        visible_channel: Optional[int] = None,
        visible_max_fps: Optional[float] = None,
        ptz_channel: Optional[int] = None,
    ):
        super().__init__(
            camera_ip=camera_ip,
            http_port=http_port,
            rtsp_port=rtsp_port,
            visible_channel=visible_channel,
            visible_max_fps=visible_max_fps,
            ptz_channel=ptz_channel,
        )
        for attr, value in (
            ("THERMAL_CHANNEL", thermal_channel),
            ("THERMAL_MAX_FPS", thermal_max_fps),
            ("THERMAL_METERING_CHANNEL", thermal_metering_channel),
            ("THERMOMETRY_PATH", thermometry_path),
            ("THERMOMETRY_POLL_HZ", thermometry_poll_hz),
            ("OVER_TEMPERATURE_C", over_temperature_c),
        ):
            if value is not None:
                setattr(self, attr, value)

        self.metadata = PluginMetadata(
            name="HIKMICRO Bi-spectrum PTZ",
            vendor="HIKMICRO",
            version="1.0",
            description=(
                "A HIKMICRO thermal + optical bi-spectrum PTZ positioning "
                "system (HM-TD5537T class): a visible 32x-zoom camera and a "
                "384x288 radiometric thermal camera co-mounted on a pan-tilt "
                "head. Streams both channels over RTSP, aims over ISAPI, and "
                "reports per-region temperatures for over-temperature events."
            ),
        )
        # --- thermal RTSP video (thermal = camera 2, channel 201) -----------
        # The color-mapped visualization stream; temperatures come from ISAPI.
        if self.HAS_THERMAL:
            user, password = self._credentials()
            self._thermal_camera = RtspCamera(
                url=self._rtsp_url(self.THERMAL_CHANNEL, user, password),
                max_fps=self.THERMAL_MAX_FPS,
            )
            thermal = SdkCallbackTransport(
                "thermal",
                register_fn=self._thermal_camera.register,
                unregister_fn=self._thermal_camera.unregister,
            )
            self.transports["thermal"] = thermal
            self.feedbacks["thermal_image"] = Feedback(
                key="thermal_image",
                msg_type=Image,
                transport=thermal,
                decoder=lambda payload: decode_frame(
                    payload, self._frame(self.THERMAL_FRAME_SUFFIX)
                ),
                rate_hz=self.THERMAL_MAX_FPS or None,
                description=(
                    "Thermal / infrared camera (color-mapped), decoded "
                    "in-plugin from RTSP"
                ),
            )
        # --- thermometry (radiometric temperature over ISAPI) --------------
        # The hottest point in the scene; the over_temperature event watches it.
        if self.HAS_THERMOMETRY:
            self._thermometry = ThermometryPoller(
                read_fn=self._read_max_temperature,
                poll_hz=self.THERMOMETRY_POLL_HZ,
            )
            thermometry = SdkCallbackTransport(
                "thermometry",
                register_fn=self._thermometry.register,
                unregister_fn=self._thermometry.unregister,
            )
            self.transports["thermometry"] = thermometry
            self.feedbacks["thermal_temperature"] = Feedback(
                key="thermal_temperature",
                msg_type=Temperature,
                transport=thermometry,
                decoder=lambda payload: reading_to_temperature_msg(
                    payload, self._frame(self.THERMAL_FRAME_SUFFIX), ROSTemperature
                ),
                rate_hz=self.THERMOMETRY_POLL_HZ,
                description=(
                    "Hottest radiometric temperature in the thermal scene "
                    "(deg C), polled from ISAPI thermometry"
                ),
            )
            self.events = EventRegistry(
                {"over_temperature": self._make_over_temperature_event}
            )

    # --- thermometry --------------------------------------------------------
    def _read_max_temperature(self) -> Optional[float]:
        """One ISAPI thermometry read -> hottest temperature in deg C (or None)."""
        path = self.THERMOMETRY_PATH.format(chan=self.THERMAL_METERING_CHANNEL)
        resp = self._isapi.get(path, params={"format": "json"})
        return parse_max_temperature(resp.text)

    def _make_over_temperature_event(self, threshold: Optional[float] = None) -> Event:
        """Fire when the hottest point in the thermal scene exceeds a threshold.

        :param threshold: Over-temperature limit in degrees Celsius; defaults to
            ``OVER_TEMPERATURE_C``.
        """
        limit = self.OVER_TEMPERATURE_C if threshold is None else threshold
        temperature = self.feedbacks["thermal_temperature"].as_topic()
        return Event(
            event_condition=temperature.msg.temperature > limit, on_change=True
        )
