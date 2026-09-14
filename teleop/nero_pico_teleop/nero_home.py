"""Saved seven-joint home pose and bounded joint-space return steps."""

import json
import math

import numpy as np
import pinocchio as pin


def load_home_pose(path, config):
    from .nero_relative_core import NeroKinematics, vector

    saved = json.loads(path.read_text())
    if saved.get("schema_version") != 1 or type(saved.get("on_startup")) is not bool:
        raise ValueError("invalid home pose schema or on_startup flag")
    arm = saved["arms"][config["arm_id"]]
    for name in ("can_channel", "controller_hand"):
        if arm[name] != config["arm"][name]:
            raise ValueError(f"home pose {name} differs from {config['arm_id']} configuration")
    model = NeroKinematics(config)
    joints = model.quantize_command(np.deg2rad(vector(arm["joints_deg"], 7)))
    settings = {key: value for key, value in saved.items() if key != "arms"}
    bounds = {"joint_speed_deg_s": 120., "tcp_speed_m_s": .4,
              "angular_speed_deg_s": 60., "timeout_s": 120.,
              "arrival_joint_tolerance_deg": 1., "arrival_tcp_tolerance_m": .005,
              "arrival_angular_tolerance_deg": 1., "arrival_stable_s": 2.}
    for key, upper in bounds.items():
        value = settings[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f"invalid home pose setting: {key}")
    # FK metadata detects a changed TCP/model rather than moving to a stale pose.
    pose = model.pose(joints)
    if (not np.allclose(pose.translation, vector(arm["tcp_position_m"], 3), atol=1e-6, rtol=0)
            or not np.allclose(pose.rotation, arm["tcp_rotation"], atol=1e-6, rtol=0)):
        raise ValueError("saved home pose does not match the current model/TCP")
    return {**settings, "joints_rad": joints.tolist(), "file": str(path.resolve())}


def home_step(model, previous, goal, dt, config):
    """A joint-space segment; check FK speed after CAN millidegree rounding."""
    settings = config["home_pose"]
    dt = min(max(0., dt), config["max_step_interval_s"])
    if dt == 0:
        return previous.copy()
    joint_step = math.radians(min(config["max_joint_step_deg"],
        min(settings["joint_speed_deg_s"], config["max_joint_speed_deg_s"]) * dt))
    tcp_step = min(settings["tcp_speed_m_s"], config["max_tcp_speed_m_s"]) * dt
    angular_step = math.radians(min(settings["angular_speed_deg_s"],
        config.get("max_angular_speed_deg_s", settings["angular_speed_deg_s"])) * dt)
    delta = goal - previous
    fraction = min(1., joint_step / max(float(np.max(np.abs(delta))), 1e-12))
    before = model.pose(previous)
    for _ in range(24):
        candidate = model.quantize_command(previous + fraction * delta)
        after = model.pose(candidate)
        if (np.max(np.abs(candidate - previous)) <= joint_step + 1e-10
                and np.linalg.norm(after.translation - before.translation) <= tcp_step + 1e-10
                and np.linalg.norm(pin.log3(before.rotation.T @ after.rotation)) <= angular_step + 1e-10):
            return candidate
        fraction *= .5
    return previous.copy()
