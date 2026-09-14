"""Explicit test source. Does not open CAN, VR, cameras, or robot SDKs."""

import math
import threading
import time

import cv2
import numpy as np

from .cameras import Camera, Frame
from .receiver import TelemetryReceiver
from .schema import arm_order


class SyntheticSource(TelemetryReceiver):
    def __init__(self, config):
        super().__init__("/unused-synthetic-socket")
        self.config = config
        self.cameras = {role: Camera({}, config["image_width"], config["image_height"])
                        for role in config["cameras"]}

    def __enter__(self):
        def run():
            sequence = 0
            started = time.monotonic()
            width, height = self.config["image_width"], self.config["image_height"]
            while not self.stop.is_set():
                now = time.monotonic()
                phase = now - started
                sequence += 1
                arms = {}
                for name in arm_order(self.config["mode"]):
                    q = [.15 * math.sin(phase + i * .2) for i in range(7)]
                    arms[name] = {"observed_monotonic": now, "state_joints_rad": q, "state_gripper_m": .04 + .01 * math.sin(phase),
                                  "feedback_monotonic": now, "gripper_feedback_monotonic": now,
                                  "action_joints_rad": [v + .008 for v in q], "action_gripper_m": .04 + .01 * math.sin(phase),
                                  "last_command_monotonic": now, "target_batches_sent": sequence,
                                  "output_state": "ACTIVE", "ready": True, "home_state": "complete",
                                  "input_healthy": True, "clutch_held": True, "clutch_epoch": 1, "gripper_enabled": True}
                packet = {"schema_version": 1, "session_id": "synthetic", "sequence": sequence,
                          "monotonic": now, "wall_time_ns": time.time_ns(), "configurations": {}, "arms": arms}
                with self.lock:
                    self.history.append(packet)
                for index, (role, camera) in enumerate(self.cameras.items()):
                    rgb = np.full((height, width, 3), (224, 231, 229), np.uint8)
                    for x in range(0, width, 40):
                        cv2.line(rgb, (x, 0), (x, height), (196, 204, 201), 1)
                    for y in range(0, height, 40):
                        cv2.line(rgb, (0, y), (width, y), (196, 204, 201), 1)
                    center = (int(width * (.5 + .22 * math.sin(phase + index))), int(height * .55))
                    cv2.rectangle(rgb, (width // 5, height // 3), (width * 4 // 5, height * 4 // 5), (107, 140, 130), 2)
                    cv2.circle(rgb, center, max(6, width // 20), (196, 78, 62), -1)
                    cv2.putText(rgb, "SYNTHETIC / " + role, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, .65, (32, 44, 40), 2)
                    cv2.putText(rgb, f"{sequence:06d}", (16, height - 20), cv2.FONT_HERSHEY_SIMPLEX, .65, (32, 44, 40), 2)
                    rgb.setflags(write=False)
                    with camera.lock:
                        camera.history.append(Frame(rgb, now, phase * 1000, sequence))
                self.stop.wait(1 / 60)

        self.worker = threading.Thread(target=run, name="synthetic-source", daemon=True)
        self.worker.start()
        return self

    def __exit__(self, *_args):
        self.stop.set()
        self.worker.join()
