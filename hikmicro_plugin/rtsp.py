"""RTSP frames read straight into the plugin, with no ROS node in between.

Frames go onto the plugin's feedback bus like any other telemetry.
"""

import shutil
import subprocess
import threading
import time
from typing import Any, Callable, List, Optional, Tuple

import numpy as np
from ros_sugar.io.supported_types import Image


def _log(message: str) -> None:
    """Log through rclpy when it is available, else print."""
    try:
        from rclpy.logging import get_logger

        get_logger("hikmicro_rtsp").info(message)
    except Exception:
        print(f"[hikmicro_rtsp] {message}", flush=True)


class RtspCamera:
    """One RTSP stream, decoded by ffmpeg and read in a background thread.

    Each frame reaches the callbacks as a ``(frame, timestamp)`` pair, stamped
    as it came off the decoder. ``register`` / ``unregister`` fit
    ``SdkCallbackTransport``.

    :param url: RTSP URL, credentials included if the server needs them.
    :param max_fps: Frames per second forwarded (ffmpeg still decodes every
        frame). ``0`` forwards them all.
    :param reconnect_s: Seconds to wait before reopening a stream that ended.
    :param transport: RTSP transport, ``tcp`` by default.
    """

    FFMPEG = "ffmpeg"
    FFPROBE = "ffprobe"
    #: Seconds ffprobe gets to answer before the stream counts as unreachable.
    PROBE_TIMEOUT_S = 15.0

    def __init__(
        self,
        url: str,
        max_fps: float = 5.0,
        reconnect_s: float = 2.0,
        transport: str = "tcp",
    ) -> None:
        self.url = url
        self.max_fps = max_fps
        self.reconnect_s = reconnect_s
        self.transport = transport

        #: Several feedbacks can ride one stream.
        self._callbacks: list = []
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._process: Optional[subprocess.Popen] = None
        #: Frames forwarded and stream re-opens.
        self.delivered = 0
        self.failures = 0
        self._logged_first = False

    # -- SdkCallbackTransport interface -------------------------------------
    def register(self, callback: Callable[[Any], None]) -> Callable:
        """Deliver frames to ``callback``; start reading on the first one.

        Returns the callback as its own handle, so ``unregister`` can drop just
        that subscriber and leave the stream running for the others.
        """
        self._callbacks.append(callback)
        if self._thread is not None:
            return callback
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=f"rtsp-{self.url.rsplit('/', 1)[-1]}",
            daemon=True,
        )
        self._thread.start()
        return callback

    def unregister(self, handle: Any = None) -> None:
        """Drop one subscriber; stop reading once none are left.

        Safe to call when never registered.
        """
        if handle is not None and handle in self._callbacks:
            self._callbacks.remove(handle)
            if self._callbacks:
                return
        self._stop.set()
        # Ending the decoder is what unblocks a reader waiting on its pipe
        self._end_process()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._callbacks = []

    # -- internals ----------------------------------------------------------
    def _live_input_args(self) -> List[str]:
        """Options for a live RTSP input."""
        if self.url.startswith("rtsp"):
            return ["-rtsp_transport", self.transport, "-fflags", "nobuffer"]
        return []

    def _command(self) -> List[str]:
        """The decoder command line."""
        command = [
            self.FFMPEG,
            "-nostdin",
            "-loglevel",
            "error",
            *self._live_input_args(),
            "-flags",
            "low_delay",
            "-i",
            self.url,
        ]
        if self.max_fps:
            command += ["-vf", f"fps={self.max_fps:g}"]
        return command + ["-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1"]

    def _probe(self) -> Optional[Tuple[int, int]]:
        """Width and height of the stream, or None when it cannot be reached.

        The raw pipe can only be cut into frames once the size is known.
        """
        command = [
            self.FFPROBE,
            "-v",
            "error",
            *self._live_input_args()[:2],
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height",
            "-of",
            "csv=p=0",
            self.url,
        ]
        # A failed probe is retried; its stderr goes to the log
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
                timeout=self.PROBE_TIMEOUT_S,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            _log(f"cannot probe {self.url}: {exc}")
            return None
        parts = result.stdout.strip().split(",")
        if result.returncode != 0 or len(parts) < 2:
            detail = result.stderr.strip() or f"exit {result.returncode}"
            _log(f"cannot probe {self.url}: {detail}")
            return None
        return int(parts[0]), int(parts[1])

    def _spawn(self) -> subprocess.Popen:
        process = subprocess.Popen(
            self._command(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        # ffmpeg's complaints go to the log
        threading.Thread(
            target=self._relay_errors,
            args=(process,),
            name=f"rtsp-errors-{self.url.rsplit('/', 1)[-1]}",
            daemon=True,
        ).start()
        return process

    def _relay_errors(self, process: subprocess.Popen) -> None:
        for line in iter(process.stderr.readline, b""):
            text = line.decode(errors="replace").strip()
            if text:
                _log(f"{self.url}: {text}")

    def _end_process(self) -> None:
        process, self._process = self._process, None
        if process is None:
            return
        process.kill()
        try:
            process.wait(timeout=2.0)
        except Exception:
            pass

    def _run(self) -> None:
        if shutil.which(self.FFMPEG) is None or shutil.which(self.FFPROBE) is None:
            _log(
                f"cannot read {self.url}: ffmpeg and ffprobe must be installed "
                "(apt install ffmpeg)"
            )
            return
        width = height = frame_bytes = 0
        try:
            while not self._stop.is_set():
                if self._process is None:
                    size = self._probe()
                    if size is None:
                        self.failures += 1
                        if self.failures <= 1:
                            _log(
                                f"cannot open {self.url} -- retrying every "
                                f"{self.reconnect_s:.0f}s"
                            )
                        # Idle rather than spin on an unreachable camera
                        self._stop.wait(self.reconnect_s)
                        continue
                    width, height = size
                    frame_bytes = width * height * 3
                    self._process = self._spawn()

                data = self._process.stdout.read(frame_bytes)
                if len(data) < frame_bytes:
                    # The stream ended or the decoder died; its stderr said why
                    self._end_process()
                    self.failures += 1
                    self._stop.wait(self.reconnect_s)
                    continue
                frame = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)

                if not self._logged_first:
                    self._logged_first = True
                    _log(
                        f"first frame from {self.url}: {width}x{height}, "
                        f"forwarding at up to {self.max_fps} fps"
                    )

                self.delivered += 1
                payload = (frame, time.time())
                # Snapshot: a subscriber may unregister from its own callback.
                for callback in list(self._callbacks):
                    callback(payload)
        finally:
            self._end_process()


def _unpack(payload: Any) -> Tuple[Optional[np.ndarray], float]:
    """The ``(frame, stamp)`` pair the reader delivers, or ``(None, 0)`` for
    anything else."""
    if not isinstance(payload, tuple) or len(payload) != 2:
        return None, 0.0
    frame, stamp = payload
    if not isinstance(frame, np.ndarray) or frame.ndim != 3:
        return None, 0.0
    return frame, float(stamp)


def decode_frame(payload: Any, frame_id: str) -> Optional[Any]:
    """Feedback decoder. A frame off the decoder into a ``sensor_msgs/Image``."""
    frame, stamp = _unpack(payload)
    if frame is None:
        return None
    return Image.convert(frame, encoding="bgr8", stamp=stamp, frame_id=frame_id)


__all__ = ["RtspCamera", "decode_frame"]
