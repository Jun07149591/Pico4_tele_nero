"""XRoboToolkit frame convention and Xnero's fixed shoulder transforms."""

import numpy as np
import pinocchio as pin


# XRoboToolkit-Teleop-Sample-Python/utils/geometry.py: forward, left, up.
HEADSET_TO_BODY = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
XNERO_ROOT_TO_BODY = np.diag([-1., -1., 1.])


def requires_head(config):
    return config.get("mapping_mode") in ("head_relative_tcp", "head_relative_body")


def body_from_arm(config, arm_id=None):
    """T_body_arm; the inverse maps body targets into a single-arm IK model."""
    mount = config["humanoid_mount"] if arm_id is None else config["humanoid_mounts"][arm_id]
    # Xnero's bracket has -X forward, -Y left, +Z up.
    root_to_body = np.asarray(config.get("root_to_body_rotation", XNERO_ROOT_TO_BODY), dtype=float)
    rotation = root_to_body @ pin.rpy.rpyToMatrix(np.asarray(mount["rpy_rad"], dtype=float))
    translation = root_to_body @ np.asarray(mount["xyz_m"], dtype=float)
    return pin.SE3(rotation, translation)


def heading_rotation(head_rotation):
    """Freeze yaw at clutch; looking down does not tilt the robot's up axis."""
    forward = head_rotation @ np.array([0., 0., -1.])
    forward[1] = 0.
    norm = np.linalg.norm(forward)
    if norm < .1:
        raise ValueError("headset_heading_undefined")
    forward /= norm
    up = np.array([0., 1., 0.])
    right = np.cross(forward, up)
    return np.column_stack((right, up, -forward))


def head_to_arm(config, head_rotation, arm_id=None):
    """Map tracking-frame displacements and spatial rotation axes into arm base."""
    return body_from_arm(config, arm_id).rotation.T @ HEADSET_TO_BODY @ heading_rotation(head_rotation).T
