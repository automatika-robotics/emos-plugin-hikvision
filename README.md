# hikmicro_plugin

An EMOS sensor plugin for Hikvision network PTZ cameras, with the thermal stream and radiometric thermometry of the HIKMICRO bi-spectrum models on top. The camera attaches to a recipe next to the robot plugin and is placed with a `Mount`, on the robot or fixed in the environment. Video is read in the plugin itself over RTSP and put on Sugarcoat's feedback bus, the head is aimed over Hikvision's ISAPI HTTP API, and every PTZ command is an action a recipe or Cortex can call.

## Two classes

`HikvisionPtzCamera` covers any Hikvision PTZ camera: the visible stream and pan, tilt, zoom, home and presets.

`HikmicroBispectrum` is for the bi-spectrum positioning systems (HM-TD and DS-2TD series) and adds the thermal stream, a temperature reading and an over-temperature event. Both optics sit on the same head, so there is one set of PTZ actions.

## Installing

```bash
emos plugin install emos-plugin-hikvision
```

The plugin decodes video with ffmpeg, which the install brings in. Credentials come from the environment and never from a recipe: set `HIKMICRO_USER` and `HIKMICRO_PASS` where the recipe runs.

## In a recipe

```python
from agents.ros import Launcher
from ros_sugar.robot import Mount
from hikmicro_plugin import HikmicroBispectrum

camera = HikmicroBispectrum(id="inspection_cam", camera_ip="192.168.1.50")

launcher = Launcher(robot_plugin=my_robot)
launcher.add_plugin(camera, mount=Mount(parent=my_robot, xyz=(0.15, 0.0, 0.35)))

# A hot spot turns the camera to preset 1
launcher.on(camera.events.over_temperature(60.0), camera.actions.goto_preset(1))
launcher.bringup()
```

A component reads the camera through a topic that names the plugin's feedback key and the camera:

```python
from agents.ros import Topic

image = Topic(name="visible_image", msg_type="Image", use_plugin=camera.id)
```

The keys are `visible_image`, and for a bi-spectrum unit `thermal_image` and `thermal_temperature`. The thermal stream is the camera's colour-mapped picture; the temperatures come from ISAPI thermometry, polled once a second, as the hottest point in the scene.

The defaults describe a factory-fresh unit: IP `192.168.1.64`, HTTP on port 80, RTSP on 554, the visible main stream on channel 101 and the thermal one on 201. Pass what differs in your setup to the constructor, or subclass and override the class attributes:

```python
class MyCam(HikmicroBispectrum):
    CAMERA_IP = "192.168.1.50"
    VISIBLE_MAX_FPS = 30.0
```

`recipes/01_camera_smoke_test.py` runs the camera on its own, with no robot, and is the quickest way to check a unit.

## Actions

`look_at(pan_deg, tilt_deg, zoom)` aims the head at an absolute position. `pan_left`, `pan_right`, `tilt_up`, `tilt_down`, `zoom_in` and `zoom_out` are short nudges that stop on their own. `stop_ptz` halts any motion, `goto_home` returns to the home position, and `goto_preset(preset)` and `set_preset(preset)` recall and store presets. Each reports `(success, message)`, where success means the camera accepted the command. A move returns before the head arrives.

The arguments of `look_at`, `goto_preset` and `set_preset` can be topic values, read from the message that fires the action:

```python
from ros_sugar.core import Event
from ros_sugar.io import Topic

# Aim wherever a tracker points, in degrees (x = pan, y = tilt)
aim = Topic(name="camera_aim", msg_type="Point")
launcher.on(Event(aim), camera.actions.look_at(aim.msg.x, aim.msg.y))
```

Keyword arguments given to an action factory go to the `Action` it builds, so a camera action is configured like any other, for instance to retry a command the camera refused:

```python
camera.actions.goto_preset(1, max_retries=2, retry_delay=1.0)
```

Retries suit the absolute commands, which land in the same place however often they are sent. The nudges are relative, so a nudge the camera carried out before its reply was lost would be repeated.

Every action carries a tool description, so Cortex can aim the camera from a task like "look at the door and tell me if it is open".

## Bringing up a unit

Hikvision cameras ship inactive. Activate the unit and set a password with SADP, the web interface or iVMS-4200, then give it a static address on the robot's subnet, since nothing answers over RTSP or ISAPI before that. Check the channel numbers with `GET /ISAPI/Streaming/channels`, as they vary between models, and pass `visible_channel` and `thermal_channel` if they differ from 101 and 201. The thermometry path varies too, and `thermometry_path` overrides it. The camera takes 24 V DC at about 18 W, not PoE, over a 10/100 link.

## Tests

```bash
python3 -m pytest test
```

The tests build the plugins and check what they expose. No camera is needed.

## License

MIT, Automatika Robotics
