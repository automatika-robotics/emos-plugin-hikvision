"""Tests for the hikmicro_plugin camera SensorPlugin.

Introspection-level: construct the plugins and check the surface they expose.
No camera hardware or ISAPI server is needed -- ``RtspCamera`` never opens a
stream until its transport is opened by a plugin host.
"""

import inspect
import shutil

import pytest
from geometry_msgs.msg import Point
from ros_sugar.core.action import Action
from ros_sugar.io.topic import Topic
from std_msgs.msg import Float32

from hikmicro_plugin import HikmicroBispectrum, HikvisionPtzCamera
from hikmicro_plugin.isapi import IsapiError
from hikmicro_plugin.thermometry import (
    parse_max_temperature,
    reading_to_temperature_msg,
)


def test_visible_feedback_is_exposed():
    cam = HikvisionPtzCamera()
    assert cam.role.value == "sensor"
    assert "visible_image" in cam.feedbacks
    fb = cam.feedbacks["visible_image"]
    assert fb.msg_type.__name__ == "Image"
    assert fb.transport.kind == "SdkCallbackTransport"
    assert fb.rate_hz == 15.0


def test_rtsp_url_visible_channel():
    cam = HikvisionPtzCamera()
    assert cam._rtsp_url(cam.VISIBLE_CHANNEL, "admin", "pw") == (
        "rtsp://admin:pw@192.168.1.64:554/Streaming/Channels/101"
    )
    # no user -> no auth prefix
    assert cam._rtsp_url(101, "", "") == (
        "rtsp://192.168.1.64:554/Streaming/Channels/101"
    )


def test_visible_frame_is_id_namespaced():
    cam = HikvisionPtzCamera(id="front_cam")
    assert cam._frame(cam.VISIBLE_FRAME_SUFFIX) == "front_cam_visible_optical"


def test_credentials_prefer_environment(monkeypatch):
    monkeypatch.setenv("HIKMICRO_USER", "op")
    monkeypatch.setenv("HIKMICRO_PASS", "s3cret")
    assert HikvisionPtzCamera()._credentials() == ("op", "s3cret")


def test_bispectrum_inherits_visible_and_flags_thermal():
    bi = HikmicroBispectrum()
    assert "visible_image" in bi.feedbacks
    assert bi.HAS_THERMAL and bi.HAS_THERMOMETRY
    assert bi.THERMAL_CHANNEL == 201


def test_bispectrum_exposes_thermal_stream():
    bi = HikmicroBispectrum(id="head_cam")
    # both optics stream; PTZ is shared, so no second action set
    assert {"visible_image", "thermal_image"} <= set(bi.feedbacks)
    assert {"visible", "thermal"} <= set(bi.transports)
    fb = bi.feedbacks["thermal_image"]
    assert fb.msg_type.__name__ == "Image"
    assert fb.transport.kind == "SdkCallbackTransport"
    assert fb.rate_hz == bi.THERMAL_MAX_FPS
    # thermal rides camera 2 -> channel 201, in its own optical frame
    assert bi._rtsp_url(bi.THERMAL_CHANNEL, "admin", "pw").endswith(
        "/Streaming/Channels/201"
    )
    assert bi._frame(bi.THERMAL_FRAME_SUFFIX) == "head_cam_thermal_optical"


def test_base_camera_has_no_thermal_stream():
    cam = HikvisionPtzCamera()
    assert "thermal_image" not in cam.feedbacks
    assert "thermal" not in cam.transports


def test_override_via_subclass():
    class MyCam(HikvisionPtzCamera):
        CAMERA_IP = "10.0.0.5"
        VISIBLE_MAX_FPS = 30.0

    cam = MyCam()
    assert cam._rtsp_url(101, "admin", "pw").startswith(
        "rtsp://admin:pw@10.0.0.5:554"
    )
    assert cam.feedbacks["visible_image"].rate_hz == 30.0


# --- PTZ actions -----------------------------------------------------------


class _RecordingIsapi:
    """Stand-in for IsapiClient that records (path, xml) instead of doing HTTP."""

    def __init__(self):
        self.calls = []

    def put_xml(self, path, xml):
        self.calls.append((path, xml))


def _ptz_cam():
    cam = HikvisionPtzCamera()
    cam._isapi = _RecordingIsapi()  # no network in tests
    return cam


def test_ptz_actions_registered():
    names = set(HikvisionPtzCamera().actions.names())
    assert names == {
        "look_at",
        "pan_left",
        "pan_right",
        "tilt_up",
        "tilt_down",
        "zoom_in",
        "zoom_out",
        "stop_ptz",
        "goto_home",
        "goto_preset",
        "set_preset",
    }


def test_ptz_look_at_hits_absolute_endpoint():
    cam = _ptz_cam()
    succeeded, message = cam.actions.look_at(90, 30, zoom=50)()  # build, then invoke
    assert succeeded, message
    path, xml = cam._isapi.calls[-1]
    assert path == "/ISAPI/PTZCtrl/channels/1/absolute"
    assert "<azimuth>900</azimuth>" in xml  # 90 deg -> 900 (0.1-deg units)
    assert "<elevation>300</elevation>" in xml  # 30 deg -> 300
    assert "<absoluteZoom>50</absoluteZoom>" in xml


def test_ptz_nudge_uses_momentary():
    cam = _ptz_cam()
    cam.actions.pan_right()()
    path, xml = cam._isapi.calls[-1]
    assert path == "/ISAPI/PTZCtrl/channels/1/momentary"
    assert f"<pan>{cam.PAN_SPEED}</pan>" in xml
    assert f"<duration>{cam.MOMENTARY_MS}</duration>" in xml
    cam.actions.pan_left()()  # opposite sign
    assert f"<pan>-{cam.PAN_SPEED}</pan>" in cam._isapi.calls[-1][1]


def test_ptz_stop_zeroes_continuous():
    cam = _ptz_cam()
    cam.actions.stop_ptz()()
    path, xml = cam._isapi.calls[-1]
    assert path == "/ISAPI/PTZCtrl/channels/1/continuous"
    assert "<pan>0</pan><tilt>0</tilt><zoom>0</zoom>" in xml


def test_ptz_presets_and_home():
    cam = _ptz_cam()
    cam.actions.goto_preset(3)()
    assert cam._isapi.calls[-1][0] == "/ISAPI/PTZCtrl/channels/1/presets/3/goto"
    cam.actions.set_preset(5)()
    path, xml = cam._isapi.calls[-1]
    assert path == "/ISAPI/PTZCtrl/channels/1/presets/5"
    assert "<id>5</id>" in xml
    cam.actions.goto_home()()
    assert cam._isapi.calls[-1][0] == "/ISAPI/PTZCtrl/channels/1/homeposition/goto"


def test_ptz_action_reports_camera_errors():
    """A refused PUT is reported through the action's (success, message)
    result rather than raised at the caller."""

    class _RefusingIsapi:
        def put_xml(self, path, xml):
            raise IsapiError(f"PUT {path} -> HTTP 401: unauthorized")

    cam = HikvisionPtzCamera()
    cam._isapi = _RefusingIsapi()
    succeeded, message = cam.actions.goto_home()()
    assert succeeded is False
    assert "HTTP 401" in message
    assert "goto_home" in message


def test_ptz_actions_are_named_after_their_keys():
    """Every action is named after its registry key, which is what a Routine's
    cursor and the Monitor's logs report."""
    cam = _ptz_cam()
    for name in cam.actions.names():
        assert getattr(cam.actions, name)().action_name == name


def test_ptz_action_factories_forward_action_kwargs():
    """Keyword arguments given to an action factory reach the Action it builds,
    parametric factories included."""
    cam = _ptz_cam()
    assert cam.actions.stop_ptz(description="Halt").description == "Halt"
    action = cam.actions.goto_preset(2, description="Patrol stop")
    assert action.description == "Patrol stop"
    assert action.action_name == "goto_preset"


def test_look_at_reads_topic_values_when_it_fires():
    """Pan and tilt may come from a topic; they are read from the message that
    fires the action, not when the action is built."""
    cam = _ptz_cam()
    target = Topic(name="target", msg_type="Point")
    action = cam.actions.look_at(target.msg.x, target.msg.y)
    succeeded, message = action(topics={"target": Point(x=90.0, y=30.0)})
    assert succeeded, message
    path, xml = cam._isapi.calls[-1]
    assert path == "/ISAPI/PTZCtrl/channels/1/absolute"
    assert "<azimuth>900</azimuth>" in xml
    assert "<elevation>300</elevation>" in xml


def test_look_at_does_not_move_before_its_topic_has_data():
    """With no message on the topic yet, the camera must not be aimed at the
    zero position in its place."""
    cam = _ptz_cam()
    target = Topic(name="target", msg_type="Point")
    cam.actions.look_at(target.msg.x, target.msg.y)(topics={})
    assert cam._isapi.calls == []


def test_goto_preset_reads_topic_value_when_it_fires():
    cam = _ptz_cam()
    preset = Topic(name="preset_number", msg_type="Float32")
    action = cam.actions.goto_preset(preset.msg.data)
    succeeded, message = action(topics={"preset_number": Float32(data=4.0)})
    assert succeeded, message
    assert cam._isapi.calls[-1][0] == "/ISAPI/PTZCtrl/channels/1/presets/4/goto"


@pytest.mark.skipif(
    "max_retries" not in inspect.signature(Action.__init__).parameters,
    reason="this Sugarcoat's Action does not support retries",
)
def test_ptz_action_retries_a_refused_command():
    """The camera answers every PUT, so a refusal is real and worth retrying:
    retries given to the factory re-send the command."""

    class _BusyOnceIsapi(_RecordingIsapi):
        def put_xml(self, path, xml):
            if not getattr(self, "refused", False):
                self.refused = True
                raise IsapiError(f"PUT {path} -> HTTP 503: busy")
            super().put_xml(path, xml)

    cam = HikvisionPtzCamera()
    cam._isapi = _BusyOnceIsapi()
    succeeded, message = cam.actions.goto_home(max_retries=1)()
    assert succeeded, message
    assert cam._isapi.calls[-1][0] == "/ISAPI/PTZCtrl/channels/1/homeposition/goto"


def test_ptz_channel_is_overridable():
    class Ch2Cam(HikvisionPtzCamera):
        PTZ_CHANNEL = 2

    cam = Ch2Cam()
    cam._isapi = _RecordingIsapi()
    cam.actions.stop_ptz()()
    assert cam._isapi.calls[-1][0] == "/ISAPI/PTZCtrl/channels/2/continuous"


def test_no_ptz_actions_when_disabled():
    class NoPtz(HikvisionPtzCamera):
        HAS_PTZ = False

    assert len(NoPtz().actions.names()) == 0


def test_ptz_tool_descriptions_present():
    tools = HikvisionPtzCamera().actions.tool_descriptions()
    names = {t["function"]["name"] for t in tools}
    assert "look_at" in names and "goto_preset" in names


# --- thermometry -----------------------------------------------------------


def test_parse_max_temperature_json_picks_hottest():
    body = """
    {"ThermometryRuleList": [
        {"id": 1, "maxTemperature": 41.5, "minTemperature": 22.0},
        {"id": 2, "maxTemperature": 58.9, "minTemperature": 30.1}
    ]}
    """
    assert parse_max_temperature(body) == 58.9


def test_parse_max_temperature_ignores_configured_limits():
    # alarm/threshold fields are configuration, not readings -- must be skipped
    body = (
        '{"currentTemperature": 47.0, '
        '"alarmTemperature": 90.0, "thresholdTemperature": 80.0}'
    )
    assert parse_max_temperature(body) == 47.0


def test_parse_max_temperature_xml_fallback():
    body = (
        "<ThermometryRule><maxTemperature>63.2</maxTemperature>"
        "<alertTemperature>85.0</alertTemperature></ThermometryRule>"
    )
    assert parse_max_temperature(body) == 63.2


def test_parse_max_temperature_none_when_no_reading():
    assert parse_max_temperature("") is None
    assert parse_max_temperature("not json or xml") is None
    assert parse_max_temperature('{"alarmTemperature": 90.0}') is None


def test_reading_to_temperature_msg():
    from sensor_msgs.msg import Temperature as ROSTemperature

    msg = reading_to_temperature_msg(
        (57.25, 1000.5), "head_thermal_optical", ROSTemperature
    )
    assert msg is not None
    assert msg.temperature == 57.25
    assert msg.header.frame_id == "head_thermal_optical"
    assert msg.header.stamp.sec == 1000
    # bad / empty payloads decode to nothing (ignored packet)
    assert reading_to_temperature_msg((None, 1.0), "f", ROSTemperature) is None
    assert reading_to_temperature_msg("garbage", "f", ROSTemperature) is None


class _RecordingThermalIsapi:
    """Stand-in IsapiClient that returns canned thermometry bodies for GET."""

    def __init__(self, body):
        self._body = body
        self.gets = []

    def get(self, path, params=None):
        self.gets.append((path, params))
        return type("Resp", (), {"text": self._body})()


def test_thermometry_feedback_exposed():
    bi = HikmicroBispectrum(id="head")
    assert "thermal_temperature" in bi.feedbacks
    assert "thermometry" in bi.transports
    fb = bi.feedbacks["thermal_temperature"]
    assert fb.msg_type.__name__ == "Temperature"
    assert fb.transport.kind == "SdkCallbackTransport"
    assert fb.rate_hz == bi.THERMOMETRY_POLL_HZ


def test_over_temperature_event_registered():
    from ros_sugar.core.event import Event

    bi = HikmicroBispectrum()
    assert "over_temperature" in bi.events
    assert isinstance(bi.events.over_temperature(), Event)  # default threshold
    assert isinstance(bi.events.over_temperature(75.0), Event)


def test_read_max_temperature_hits_isapi_channel_2():
    bi = HikmicroBispectrum()
    bi._isapi = _RecordingThermalIsapi('{"maxTemperature": 44.4}')
    assert bi._read_max_temperature() == 44.4
    path, params = bi._isapi.gets[-1]
    assert path == "/ISAPI/Thermal/channels/2/thermometry/1/rules"
    assert params == {"format": "json"}


def test_base_has_no_thermometry():
    cam = HikvisionPtzCamera()
    assert "thermal_temperature" not in cam.feedbacks
    assert "thermometry" not in cam.transports
    assert cam.events.names() == []


# --- constructor overrides -------------------------------------------------


def test_init_params_override_class_defaults():
    cam = HikvisionPtzCamera(
        camera_ip="10.0.0.9", visible_channel=102, visible_max_fps=30.0, ptz_channel=2
    )
    assert cam._rtsp_url(cam.VISIBLE_CHANNEL, "u", "p") == (
        "rtsp://u:p@10.0.0.9:554/Streaming/Channels/102"
    )
    assert cam.feedbacks["visible_image"].rate_hz == 30.0
    assert cam._ptz_path("continuous") == "/ISAPI/PTZCtrl/channels/2/continuous"


def test_bispectrum_init_params_forward_and_extend():
    bi = HikmicroBispectrum(
        camera_ip="10.0.0.9",  # forwarded to the base
        thermal_channel=202,
        thermal_metering_channel=3,
        thermometry_path="/ISAPI/Thermal/channels/{chan}/thermometry/2/rules",
    )
    # base knob forwarded through super().__init__
    assert bi.CAMERA_IP == "10.0.0.9"
    assert bi._rtsp_url(bi.THERMAL_CHANNEL, "u", "p").endswith("/Channels/202")
    bi._isapi = _RecordingThermalIsapi('{"maxTemperature": 51.0}')
    assert bi._read_max_temperature() == 51.0
    assert bi._isapi.gets[-1][0] == "/ISAPI/Thermal/channels/3/thermometry/2/rules"


def test_to_spec_roundtrips_without_credentials(monkeypatch):
    import json

    monkeypatch.setenv("HIKMICRO_USER", "op")
    monkeypatch.setenv("HIKMICRO_PASS", "s3cret")
    bi = HikmicroBispectrum(id="head", camera_ip="10.0.0.9", thermal_channel=202)
    spec = bi.to_spec()
    # spec must be JSON-serializable and must NOT leak the credentials
    blob = json.dumps(spec)
    assert "s3cret" not in blob and "password" not in blob.lower()
    clone = HikmicroBispectrum.from_spec(spec)
    assert clone.CAMERA_IP == "10.0.0.9" and clone.THERMAL_CHANNEL == 202
    assert clone.id == "head"


# --- the ffmpeg reader and the frame decoder --------------------------------


def test_rtsp_reader_decodes_everything_and_emits_max_fps():
    """The throttle is ffmpeg's fps filter after the decoder, since H.265 needs
    every frame decoded; TCP so the server paces the stream."""
    from hikmicro_plugin.rtsp import RtspCamera

    command = RtspCamera("rtsp://h/x", max_fps=15)._command()
    assert command[0] == "ffmpeg"
    assert command[command.index("-rtsp_transport") + 1] == "tcp"
    assert command[command.index("-vf") + 1] == "fps=15"
    assert command[-5:] == ["-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1"]
    assert "-vf" not in RtspCamera("rtsp://h/x", max_fps=0)._command()


class _FakeDecoder:
    """Stands in for the ffmpeg child: raw frames on stdout, then EOF."""

    def __init__(self, frames: bytes):
        import io

        self.stdout = io.BytesIO(frames)
        self.stderr = io.BytesIO(b"")

    def kill(self):
        pass

    def wait(self, timeout=None):
        return 0


def test_rtsp_reader_cuts_the_pipe_into_frames_and_reconnects_on_eof():
    import time

    import numpy as np

    from hikmicro_plugin.rtsp import RtspCamera

    width, height = 4, 3
    first = np.full((height, width, 3), 7, dtype=np.uint8)
    second = np.zeros((height, width, 3), dtype=np.uint8)
    second[1, 2] = (1, 2, 3)
    camera = RtspCamera("rtsp://h/x", max_fps=0, reconnect_s=0.01)
    spawned = []
    camera._probe = lambda: (width, height)
    camera._spawn = lambda: spawned.append(
        _FakeDecoder(first.tobytes() + second.tobytes())
    ) or spawned[-1]
    got = []
    camera.register(got.append)
    deadline = time.time() + 3.0
    while (len(got) < 2 or len(spawned) < 2) and time.time() < deadline:
        time.sleep(0.01)
    camera.unregister()

    assert len(got) >= 2
    (frame_a, stamp_a), (frame_b, stamp_b) = got[:2]
    assert frame_a.shape == (height, width, 3) and frame_a[0, 0].tolist() == [7, 7, 7]
    assert frame_b[1, 2].tolist() == [1, 2, 3]
    assert stamp_b >= stamp_a
    assert camera.delivered >= 2 and camera.failures >= 1 and len(spawned) >= 2


def test_rtsp_reader_idles_while_the_camera_is_unreachable():
    import time

    from hikmicro_plugin.rtsp import RtspCamera

    camera = RtspCamera("rtsp://h/x", reconnect_s=0.01)
    probes = []
    camera._probe = lambda: probes.append(1) and None
    camera._spawn = lambda: (_ for _ in ()).throw(AssertionError("spawned"))
    camera.register(lambda payload: None)
    time.sleep(0.1)
    camera.unregister()

    assert len(probes) >= 2 and camera.failures == len(probes)


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg not installed")
def test_rtsp_reader_reads_real_ffmpeg_output(tmp_path):
    """The real decoder against a synthetic clip: the pipe protocol, the frame
    size from the probe and the BGR layout all line up."""
    import subprocess
    import time

    from hikmicro_plugin.rtsp import RtspCamera

    clip = tmp_path / "clip.mp4"
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-f", "lavfi", "-i",
         "color=c=blue:size=64x48:rate=30", "-t", "1", "-pix_fmt", "yuv420p",
         str(clip)],
        check=True,
    )
    camera = RtspCamera(str(clip), max_fps=0)
    got = []
    camera.register(got.append)
    deadline = time.time() + 10.0
    while len(got) < 5 and time.time() < deadline:
        time.sleep(0.02)
    camera.unregister()

    assert len(got) >= 5
    frame, _ = got[0]
    assert frame.shape == (48, 64, 3)
    blue, green, red = (int(v) for v in frame[24, 32])
    assert blue > 200 and red < 60 and green < 60


def test_frame_decodes_to_a_complete_bgr8_image_on_the_fast_path():
    import array

    import numpy as np

    from hikmicro_plugin.rtsp import decode_frame

    frame = np.zeros((4, 6, 3), dtype=np.uint8)
    frame[0, 0] = (1, 2, 3)
    msg = decode_frame((frame, 1234.5), "head_cam_visible_optical")
    assert (msg.height, msg.width, msg.step, msg.encoding) == (4, 6, 18, "bgr8")
    assert isinstance(msg.data, array.array) and bytes(msg.data[:3]) == bytes((1, 2, 3))
    assert msg.header.frame_id == "head_cam_visible_optical"
    assert msg.header.stamp.sec == 1234
    assert decode_frame(None, "f") is None
    assert decode_frame((np.zeros((4, 4)), 1.0), "f") is None
