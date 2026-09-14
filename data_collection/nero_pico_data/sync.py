import math

import numpy as np

from .schema import arm_order, vector
from .eva_alignment import hold, interpolate, nearest_index


class SampleInvalid(ValueError):
    pass


def check_packet(packet, timestamp, config):
    if packet is None or not 0 <= timestamp - packet["monotonic"] <= config["max_robot_age_s"]:
        raise SampleInvalid("telemetry missing or stale")
    if set(packet["arms"]) != set(arm_order(config["mode"])):
        raise SampleInvalid("teleop arm mode differs from capture mode")
    for name in arm_order(config["mode"]):
        arm = packet["arms"][name]
        if arm["home_state"] == "returning":
            raise SampleInvalid(f"{name}: returning home")
        if not arm["ready"]:
            raise SampleInvalid(f"{name}: teleop not ready")
        if arm["output_state"] not in ("ACTIVE", "HOLDING"):
            raise SampleInvalid(f"{name}: output {arm['output_state']}")
        if not arm["input_healthy"]:
            raise SampleInvalid(f"{name}: input lost")
        if not arm["gripper_enabled"]:
            raise SampleInvalid(f"{name}: gripper unavailable")
        for key in ("feedback_monotonic", "gripper_feedback_monotonic"):
            stamp = arm[key]
            if stamp is None or not math.isfinite(stamp) or not 0 <= timestamp - stamp <= config["max_robot_age_s"]:
                raise SampleInvalid(f"{name}: {key} missing/stale")
        sent = arm["last_command_monotonic"]
        if sent is None or not math.isfinite(sent) or sent > packet["monotonic"]:
            raise SampleInvalid(f"{name}: invalid command timestamp")
    try:
        vector(packet, config["mode"], "state")
        vector(packet, config["mode"], "action")
    except (KeyError, TypeError, ValueError) as exc:
        raise SampleInvalid(str(exc)) from exc


class Synchronizer:
    def __init__(self, config, receiver, cameras):
        self.config, self.receiver, self.cameras = config, receiver, cameras
        self.session_id = None
        self.camera_sequences = {}
        self.previous_timestamp = None

    def sample(self, timestamp):
        config = self.config
        observation = self.receiver.latest_before(timestamp)
        check_packet(observation, timestamp, config)
        history = self.receiver.snapshot()
        start = self.previous_timestamp if self.previous_timestamp is not None else timestamp
        packets = [observation, *[p for p in history if start < p["monotonic"] <= timestamp + .06]]
        for packet in packets:
            check_packet(packet, packet["monotonic"], config)
            if packet["session_id"] != observation["session_id"]:
                raise SampleInvalid("teleop restarted during episode")
        if self.session_id is not None and observation["session_id"] != self.session_id:
            raise SampleInvalid("teleop session changed")
        state = []
        try:
            for name in arm_order(config["mode"]):
                nearby = [p["arms"][name] for p in history
                          if p["session_id"] == observation["session_id"]
                          and abs(p["monotonic"] - timestamp) <= config["max_robot_age_s"] + .06]
                joints = {a["feedback_monotonic"]: a["state_joints_rad"] for a in nearby}
                grippers = {a["gripper_feedback_monotonic"]: a["state_gripper_m"] for a in nearby}
                joint_times, gripper_times = sorted(joints), sorted(grippers)
                state.extend(interpolate(joint_times, [joints[t] for t in joint_times], timestamp, config["max_robot_age_s"]))
                state.append(hold(gripper_times, [grippers[t] for t in gripper_times], timestamp, config["max_robot_age_s"]))
        except (ValueError, TypeError, KeyError) as exc:
            raise SampleInvalid(str(exc)) from exc
        if not np.isfinite(state).all():
            raise SampleInvalid("nonfinite interpolated state")
        frames = {}
        for role, camera in self.cameras.items():
            if camera.error:
                raise SampleInvalid(f"camera {role}: {camera.error}")
            history_frames = camera.snapshot()
            if not history_frames:
                raise SampleInvalid(f"camera {role} missing")
            frame = history_frames[nearest_index([f.monotonic for f in history_frames], timestamp)]
            if abs(timestamp - frame.monotonic) > config["max_camera_age_s"]:
                raise SampleInvalid(f"camera {role} missing/stale")
            previous_sequence = self.camera_sequences.get(role)
            if previous_sequence is not None and frame.sequence <= previous_sequence:
                # At equal camera/capture rates, arrival jitter can make the
                # previous image nearest again. Use an unused frame within one tick.
                unused = [f for f in history_frames if f.sequence > previous_sequence]
                if not unused:
                    raise SampleInvalid(f"camera {role} repeated a frame")
                frame = unused[nearest_index([f.monotonic for f in unused], timestamp)]
                if abs(timestamp - frame.monotonic) > min(config["max_camera_age_s"], 1 / config["fps"]):
                    raise SampleInvalid(f"camera {role} repeated a frame")
            frames[role] = frame
        stamps = [frame.monotonic for frame in frames.values()]
        if max(stamps) - min(stamps) > config["max_camera_skew_s"]:
            raise SampleInvalid("camera time skew too large")
        self.session_id = observation["session_id"]
        self.previous_timestamp = timestamp
        self.camera_sequences = {role: frame.sequence for role, frame in frames.items()}
        return {"monotonic": timestamp, "action_monotonic": timestamp,
                "wall_time_ns": observation["wall_time_ns"] + round((timestamp - observation["monotonic"]) * 1e9),
                "state": np.asarray(state, dtype=np.float32),
                "action": vector(observation, config["mode"], "action"), "images": frames,
                "diagnostics": {"observation": observation,
                                "camera_skew_s": max(stamps) - min(stamps),
                                "camera_offsets_s": {r: f.monotonic - timestamp for r, f in frames.items()}}}
