#!/usr/bin/env python3
"""Smoke test: the camera plugin on its own, with no robot.

    RTSP (in-plugin ffmpeg decode) -> 'visible_image' feedback
        -> MotionDetector -> /motion   (std_msgs/Bool)

The camera runs as a fixed environment sensor: ``Launcher(robot_plugin=...)``
is optional and a ``Mount`` parent may be a plain frame name. MotionDetector
is the lightest Image consumer in EmbodiedAgents (frame differencing in
OpenCV, no model, no download), so if this recipe fails, the fault is in the
camera path.

Before running it:

  * Credentials come from the environment:
        export HIKMICRO_USER=admin
        export HIKMICRO_PASS='...'
  * Channel numbers are model specific. Confirm them on the unit:
        curl -s --digest -u "$HIKMICRO_USER:$HIKMICRO_PASS" \\
             http://$HIKMICRO_IP/ISAPI/Streaming/channels
    101 is the visible main stream, 201 the thermal one. Pass
    visible_channel= / thermal_channel= below if this unit differs.

Reading the result:

  The plugin logs its first frame:

      [hikmicro_rtsp] first frame from rtsp://...: 1920x1080, forwarding at up to 5.0 fps

  In a second shell on the same host:

      ros2 topic echo /motion        # Bool at ~5 Hz

  /motion ticks on every frame, so a steady `data: false` in a still room
  means frames are arriving. Wave at the camera and it flips to true. There is
  no ROS topic for the image itself: frames reach the component over
  Sugarcoat's feedback bus, so `ros2 topic hz` has nothing to show.

Environment:

  HIKMICRO_IP       camera address        (default 10.21.31.100)
  HIKMICRO_USER     ISAPI/RTSP user       (required)
  HIKMICRO_PASS     ISAPI/RTSP password   (required)
  HIKMICRO_THERMAL  set to 1 for a bi-spectrum unit (adds thermal + thermometry)
"""

import os

from agents.components import MotionDetector
from agents.config import MotionDetectorConfig
from agents.ros import Launcher, Topic
from ros_sugar.robot import Mount

from hikmicro_plugin import HikmicroBispectrum, HikvisionPtzCamera

CAMERA_IP = os.environ.get("HIKMICRO_IP", "10.21.31.100")
#: The bi-spectrum class opens a second RTSP stream and polls a thermometry
#: path a plain PTZ camera does not have, so it is only for bi-spectrum units.
THERMAL = os.environ.get("HIKMICRO_THERMAL", "0") == "1"


def main() -> None:
    if not os.environ.get("HIKMICRO_PASS"):
        raise SystemExit(
            "Set HIKMICRO_USER and HIKMICRO_PASS first -- the plugin reads "
            "credentials from the environment and never from source."
        )

    camera_cls = HikmicroBispectrum if THERMAL else HikvisionPtzCamera
    camera = camera_cls(
        id="inspection_cam",
        camera_ip=CAMERA_IP,
        # Enough to prove the path; raw 1080p at 30 fps is ~180 MB/s on the bus
        visible_max_fps=5.0,
    )

    # The name is the plugin's feedback key, not a topic path. use_plugin names
    # the camera because a bi-spectrum unit has two Image feedbacks.
    image = Topic(name="visible_image", msg_type="Image", use_plugin=camera.id)
    motion = Topic(name="motion", msg_type="Bool")

    detector = MotionDetector(
        inputs=[image],
        outputs=[motion],
        config=MotionDetectorConfig(
            motion_estimation_func="frame_difference",
            image_scale=0.5,
        ),
        # Runs on every frame
        trigger=image,
        component_name="hikmicro_motion",
    )

    launcher = Launcher()
    # No robot, so the mount parent is a bare frame name
    launcher.add_plugin(camera, mount=Mount(parent="world", xyz=(0.0, 0.0, 1.0)))
    launcher.add_pkg(
        components=[detector],
        # One process while bringing up, so a crash gives a traceback
        multiprocessing=False,
        package_name="automatika_embodied_agents",
    )

    if THERMAL:
        # The ISAPI control path in the same run: a hot spot aims at preset 1
        launcher.on(
            camera.events.over_temperature(60.0),
            camera.actions.goto_preset(1),
        )

    print(__doc__)
    launcher.bringup()


if __name__ == "__main__":
    main()
