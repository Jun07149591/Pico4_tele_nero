import copy
import json
import math
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/capture.local.json"
if not DEFAULT_CONFIG.is_file():
    DEFAULT_CONFIG = ROOT / "config/capture.json"
CAMERA_ROLES = ("front", "left_wrist", "right_wrist")


def default_socket():
    return Path(f"/tmp/nero_pico_data_{os.getuid()}.sock")


def arm_order(mode):
    if mode not in ("single", "dual"):
        raise ValueError("mode must be single or dual")
    return ["right_arm"] if mode == "single" else ["left_arm", "right_arm"]


def vector_names(mode):
    return [name for arm in arm_order(mode)
            for name in [*(f"{arm}.joint_{i + 1}_rad" for i in range(7)), f"{arm}.gripper_width_m"]]


def capture_fps_limit(config):
    return min(60, *(camera["fps"] for camera in config["cameras"].values()))


def validate_capture_fps(fps, config):
    maximum = capture_fps_limit(config)
    if type(fps) is not int or not 1 <= fps <= maximum:
        raise ValueError(f"capture fps must be an integer between 1 and {maximum}")
    return fps


def load_config(path=DEFAULT_CONFIG, mode=None):
    config = json.loads(Path(path).read_text())
    if mode is not None:
        config["mode"] = mode
    arm_order(config["mode"])
    if config.get("schema_version") != 1:
        raise ValueError("unsupported capture schema")
    for name, lower, upper in (("fps", 1, 60), ("image_width", 32, 1920), ("image_height", 32, 1080),
            ("min_episode_frames", 2, 10000), ("max_episode_seconds", 1, 3600),
            ("jpeg_quality", 50, 100), ("writer_queue_frames", 2, 1000)):
        value = config[name]
        if type(value) is not int or not lower <= value <= upper:
            raise ValueError(f"invalid {name}")
    for name in ("max_robot_age_s", "max_camera_age_s", "max_camera_skew_s",
                 "max_schedule_lateness_s", "minimum_free_gb"):
        if type(config[name]) not in (float, int) or not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f"invalid {name}")
    cameras = config["cameras"]
    if not isinstance(cameras, dict) or "front" not in cameras or set(cameras) - set(CAMERA_ROLES):
        raise ValueError("cameras require front; optional roles are left_wrist and right_wrist")
    devices = []
    for name, camera in cameras.items():
        if camera.get("backend") not in ("realsense", "opencv"):
            raise ValueError(f"unsupported camera backend: {name}")
        if type(camera.get("fps")) is not int or not config["fps"] <= camera["fps"] <= 60:
            raise ValueError(f"camera {name} fps must be at least collection fps")
        identifier = camera.get("serial") if camera["backend"] == "realsense" else camera.get("device")
        if identifier is None or str(identifier).startswith("SET_"):
            raise ValueError(f"configure camera {name} serial/device after running cameras")
        devices.append((camera["backend"], str(identifier)))
    if len(set(devices)) != len(devices):
        raise ValueError("one camera cannot fill two camera roles")
    return config


def manifest(config, synthetic=False):
    return {"schema_version": 1, "robot_type": f"nero_{config['mode']}",
            "mode": config["mode"], "arm_order": arm_order(config["mode"]),
            "vector_names": vector_names(config["mode"]), "fps": config["fps"],
            "action_type": "absolute_joint_position_rad_and_gripper_width_m",
            "action_alignment": "last_published_successful_command_at_sample_tick",
            "observation_alignment": "interpolated_joint_feedback_causal_gripper_nearest_camera",
            "camera_clock": "host_receive_monotonic_not_hardware_synchronized",
            "synthetic": synthetic, "config": copy.deepcopy(config)}


def vector(packet, mode, kind):
    result = []
    for arm in arm_order(mode):
        state = packet["arms"][arm]
        q = np.asarray(state[f"{kind}_joints_rad"], dtype=np.float64)
        width = state[f"{kind}_gripper_m"]
        if q.shape != (7,) or not np.isfinite(q).all() or width is None or not math.isfinite(width):
            raise ValueError(f"missing/nonfinite {kind} for {arm}")
        if not 0 <= width <= .100:
            raise ValueError(f"gripper outside 0-100 mm: {arm}")
        result.extend([*q, width])
    return np.asarray(result, dtype=np.float32)
