import json
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np

from nero_pico_teleop.telemetry import TelemetryPublisher


def output():
    return SimpleNamespace(config={"arm": {"can_channel": "can0", "controller_hand": "right"}, "tcp_offset_m": [0, 0, .1]},
        lock=threading.Lock(), input_source=None, last_feedback_snapshot={"joints_rad": [.1] * 7, "group_timestamps_s": [time.time()] * 4},
        gripper_feedback_timestamp_s=time.time(), gripper_measured=.025, last_target=np.full(7, .2),
        gripper_target=.03, gripper_last_sent=time.monotonic(), gripper_frames=1, last_sent=time.monotonic(), targets_sent=3,
        state="ACTIVE", teleop_ready=True, home_state="complete", gripper=object())


class TelemetryTests(unittest.TestCase):
    def test_snapshot_distinguishes_feedback_and_sent_command(self):
        arm = output()
        packet = TelemetryPublisher({"right_arm": arm}, "/unused").snapshot()
        self.assertEqual(packet["arms"]["right_arm"]["state_joints_rad"], [.1] * 7)
        self.assertEqual(packet["arms"]["right_arm"]["action_joints_rad"], [.2] * 7)
        self.assertEqual(packet["arms"]["right_arm"]["state_gripper_m"], .025)
        self.assertEqual(packet["arms"]["right_arm"]["action_gripper_m"], .03)
        self.assertFalse(packet["arms"]["right_arm"]["input_healthy"])
        self.assertLess(abs(packet["monotonic"] - packet["arms"]["right_arm"]["feedback_monotonic"]), .1)
        json.dumps(packet, allow_nan=False)

    def test_busy_robot_or_input_lock_skips_without_blocking(self):
        arm = output()
        publisher = TelemetryPublisher({"right_arm": arm}, "/unused")
        with arm.lock:
            started = time.monotonic()
            self.assertIsNone(publisher.snapshot())
            self.assertLess(time.monotonic() - started, .02)
        source = SimpleNamespace(lock=threading.Lock(), snapshot=lambda: self.fail("busy input must not be read"))
        arm.input_source = source
        with source.lock:
            self.assertIsNone(publisher.snapshot())
        self.assertTrue(arm.lock.acquire(blocking=False))
        arm.lock.release()

    def test_absent_receiver_is_optional_then_receiver_can_connect(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "collector.sock")
            with TelemetryPublisher({"right_arm": output()}, path, hz=100) as publisher:
                time.sleep(.04)
                self.assertTrue(publisher.worker.is_alive())
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as receiver:
                    receiver.bind(path)
                    receiver.settimeout(1.)
                    first = json.loads(receiver.recv(65536))
                    second = json.loads(receiver.recv(65536))
                    self.assertEqual(first["session_id"], second["session_id"])
                    self.assertGreater(second["sequence"], first["sequence"])
            self.assertFalse(publisher.worker.is_alive())


if __name__ == "__main__":
    unittest.main()
