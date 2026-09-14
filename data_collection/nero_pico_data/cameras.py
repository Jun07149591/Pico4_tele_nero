from collections import deque
from contextlib import ExitStack
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class Frame:
    rgb: np.ndarray
    monotonic: float
    device_timestamp_ms: float
    sequence: int


def discover():
    import pyrealsense2 as rs
    return [{"name": d.get_info(rs.camera_info.name), "serial": d.get_info(rs.camera_info.serial_number),
             "firmware": d.get_info(rs.camera_info.firmware_version),
             "usb": d.get_info(rs.camera_info.usb_type_descriptor)} for d in rs.context().query_devices()]


class Camera:
    def __init__(self, settings, width, height):
        self.settings, self.width, self.height = settings, width, height
        self.history = deque(maxlen=12)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.worker = None
        self.error = None
        self.device = None

    def __enter__(self):
        if self.settings["backend"] == "realsense":
            import pyrealsense2 as rs
            self.device = rs.pipeline()
            config = rs.config()
            config.enable_device(self.settings["serial"])
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.settings["fps"])
            self.device.start(config)
        else:
            self.device = cv2.VideoCapture(self.settings["device"], cv2.CAP_V4L2)
            if not self.device.isOpened():
                self.device.release()
                raise RuntimeError(f"cannot open camera {self.settings['device']}")
            self.device.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.device.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.device.set(cv2.CAP_PROP_FPS, self.settings["fps"])
            self.device.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        def run():
            sequence = 0
            try:
                while not self.stop.is_set():
                    if self.settings["backend"] == "realsense":
                        frame = self.device.wait_for_frames(timeout_ms=1000).get_color_frame()
                        if not frame:
                            raise RuntimeError("camera did not return color")
                        received = time.monotonic()
                        rgb = np.asanyarray(frame.get_data()).copy()
                        stamp, sequence = frame.get_timestamp(), frame.get_frame_number()
                    else:
                        ok, bgr = self.device.read()
                        received = time.monotonic()
                        if not ok:
                            raise RuntimeError("camera frame read failed")
                        rgb, stamp, sequence = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), -1., sequence + 1
                    if rgb.shape != (self.height, self.width, 3):
                        raise RuntimeError(f"camera size {rgb.shape} does not match configured size")
                    rgb.setflags(write=False)
                    with self.lock:
                        self.history.append(Frame(rgb, received, stamp, sequence))
            except Exception as exc:
                self.error = str(exc)

        self.worker = threading.Thread(target=run, name="data-camera", daemon=True)
        self.worker.start()
        return self

    def latest_before(self, timestamp):
        with self.lock:
            return next((f for f in reversed(self.history) if f.monotonic <= timestamp), None)

    def snapshot(self):
        with self.lock:
            return list(self.history)

    def __exit__(self, *_args):
        self.stop.set()
        if self.settings["backend"] == "opencv":
            self.device.release()
        self.worker.join(timeout=2.)
        if self.settings["backend"] == "realsense":
            self.device.stop()
        if self.worker.is_alive():
            raise RuntimeError("camera worker did not stop")


class CameraRig:
    def __init__(self, config, *, allow_unavailable=False):
        self.config = config
        self.allow_unavailable = allow_unavailable
        self.stack = ExitStack()
        self.cameras = {}

    def __enter__(self):
        try:
            for role, settings in self.config["cameras"].items():
                camera = Camera(settings, self.config["image_width"], self.config["image_height"])
                try:
                    self.cameras[role] = self.stack.enter_context(camera)
                except (RuntimeError, OSError) as exc:
                    if not self.allow_unavailable:
                        raise
                    unavailable = Camera(settings, self.config["image_width"], self.config["image_height"])
                    unavailable.error = str(exc)
                    self.cameras[role] = unavailable
        except Exception:
            self.stack.close()
            raise
        return self

    def __exit__(self, *_args):
        self.stack.close()
