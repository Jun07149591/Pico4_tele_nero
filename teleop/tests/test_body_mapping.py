import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from nero_pico_teleop.cli import demo_events
from nero_pico_teleop.humanoid_frames import body_from_arm, head_to_arm
from nero_pico_teleop.nero_relative_core import NeroKinematics, RelativeTeleop, load_config
from nero_pico_teleop.nero_stream import stream_configs
from nero_pico_teleop.paths import PROJECT_ROOT


CONFIG = PROJECT_ROOT / "config/nero_humanoid_config.json"
SEED = np.deg2rad([0, 45, -60, 60, 0, 20, 0])
# Active matrices in Maniskill_xnero/examples/webvr_teleop.py, not its stale README.
EXPECTED = {
    "left_arm": np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]]),
    "right_arm": np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]),
}
HEAD_AXES = ([0., 0., -1.], [0., 1., 0.], [1., 0., 0.])
BODY_AXES = ([1., 0., 0.], [0., 0., 1.], [0., -1., 0.])


class BodyMappingTests(unittest.TestCase):
    def setUp(self):
        self.config, _ = stream_configs(load_config(CONFIG), session_limits=False,
            tcp_speed_mm_s=400., joint_speed_deg_s=120., angular_speed_deg_s=60.)

    def core(self, side, head, controller, seed=SEED):
        config = copy.deepcopy(self.config)
        config["humanoid_mount"] = config["humanoid_mounts"][side]
        model = NeroKinematics(config)
        # Isolate requested frame mapping from reachability and solver residuals.
        model.solve = lambda *args: {"ok": True, "joints_rad": seed.tolist()}
        core = RelativeTeleop(config, seed, model)
        event = copy.deepcopy(next(demo_events(config)))
        event["tracking"]["Head"]["pose"][3:] = Rotation.from_matrix(head).as_quat().tolist()
        event["tracking"]["Controller"]["right"]["pose"][3:] = Rotation.from_matrix(controller).as_quat().tolist()
        self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
        return core, event

    @staticmethod
    def advance(event):
        event["received_monotonic"] += .02
        event["processed_monotonic"] += .02
        event["tracking"]["timeStampNs"] += 20_000_000

    def test_both_shoulders_match_reference_source_matrices(self):
        for side, expected in EXPECTED.items():
            mapping = head_to_arm(self.config, np.eye(3), side)
            np.testing.assert_allclose(mapping, expected, atol=8e-6)
            np.testing.assert_allclose(mapping.T @ mapping, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(mapping), 1.)

    def test_six_directions_ignore_wrist_orientation_and_head_tilt(self):
        for side in EXPECTED:
            for yaw in (-1.2, 0., 2.1):
                heading = Rotation.from_rotvec([0., yaw, 0.]).as_matrix()
                head = heading @ Rotation.from_euler("XZ", [.6, -.35]).as_matrix()
                for wrist in (np.eye(3), Rotation.from_euler("xyz", [.5, -.4, .8]).as_matrix()):
                    for seed in (SEED, SEED + np.deg2rad([0, 0, 0, 0, 0, 30, 40])):
                        for source, body_axis in zip(HEAD_AXES, BODY_AXES):
                            for sign in (-1, 1):
                                core, event = self.core(side, head, wrist, seed)
                                origin = core.target.copy()
                                pose = event["tracking"]["Controller"]["right"]["pose"]
                                pose[:3] = (np.asarray(pose[:3]) + heading @ source * sign * .001).tolist()
                                self.advance(event)
                                self.assertEqual(core.process_pose(event)["state"], "FOLLOW")
                                actual_body_delta = body_from_arm(core.config).rotation @ (core.target - origin)
                                np.testing.assert_allclose(actual_body_delta, np.asarray(body_axis) * sign * .001,
                                                           atol=1e-12)

    def test_spatial_rotation_axes_and_order_for_both_shoulders(self):
        controller = Rotation.from_euler("xyz", [.7, -.5, .4]).as_matrix()
        for side in EXPECTED:
            for yaw in (-1.2, 1.4):
                heading = Rotation.from_rotvec([0., yaw, 0.]).as_matrix()
                head = heading @ Rotation.from_euler("XZ", [.5, .3]).as_matrix()
                for source, body_axis in zip(HEAD_AXES, BODY_AXES):
                    for sign in (-1, 1):
                        core, event = self.core(side, head, controller)
                        initial = core.rotation.copy()
                        delta = Rotation.from_rotvec(np.asarray(source) * sign * .008).as_matrix()
                        current = heading @ delta @ heading.T @ controller
                        event["tracking"]["Controller"]["right"]["pose"][3:] = Rotation.from_matrix(current).as_quat().tolist()
                        self.advance(event)
                        core.process_pose(event)
                        mount = body_from_arm(core.config).rotation
                        actual = mount @ core.rotation @ initial.T @ mount.T
                        expected = Rotation.from_rotvec(np.asarray(body_axis) * sign * .008).as_matrix()
                        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_combined_rotation_quaternion_sign_and_repeated_pose_do_not_drift(self):
        heading = Rotation.from_rotvec([0., .9, 0.]).as_matrix()
        head = heading @ Rotation.from_euler("XZ", [.4, -.2]).as_matrix()
        controller = Rotation.from_euler("xyz", [.5, .7, -.8]).as_matrix()
        for side in EXPECTED:
            core, event = self.core(side, head, controller)
            initial = core.rotation.copy()
            delta = Rotation.from_rotvec([0., .006, 0.]).as_matrix() @ Rotation.from_rotvec([0., 0., -.004]).as_matrix()
            current = heading @ delta @ heading.T @ controller
            quat = Rotation.from_matrix(current).as_quat()
            expected = Rotation.from_rotvec([0., 0., .006]).as_matrix() @ Rotation.from_rotvec([.004, 0., 0.]).as_matrix()
            for sign in (1, -1) * 10:
                event["tracking"]["Controller"]["right"]["pose"][3:] = (sign * quat).tolist()
                self.advance(event)
                core.process_pose(event)
                mount = body_from_arm(core.config).rotation
                np.testing.assert_allclose(mount @ core.rotation @ initial.T @ mount.T, expected, atol=1e-12)

    def test_head_turn_during_grip_and_regrip_have_no_position_jump(self):
        core, event = self.core("right_arm", np.eye(3), np.eye(3))
        initial = core.target.copy()
        first_mapping = core.head_tracking_to_base.copy()
        head = Rotation.from_rotvec([0., .9, 0.]).as_matrix()
        event["tracking"]["Head"]["pose"][3:] = Rotation.from_matrix(head).as_quat().tolist()
        self.advance(event)
        core.process_pose(event)
        np.testing.assert_allclose(core.head_tracking_to_base, first_mapping)
        np.testing.assert_allclose(core.target, initial)
        core.hold("grip_released")
        self.advance(event)
        self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
        np.testing.assert_allclose(core.target, initial)
        self.assertFalse(np.allclose(core.head_tracking_to_base, first_mapping))
        pose = event["tracking"]["Controller"]["right"]["pose"]
        pose[:3] = (np.asarray(pose[:3]) + head @ np.array([0., 0., -.001])).tolist()
        self.advance(event)
        core.process_pose(event)
        np.testing.assert_allclose(body_from_arm(core.config).rotation @ (core.target - initial),
                                   [.001, 0., 0.], atol=1e-12)

    def test_undefined_heading_does_not_create_a_target(self):
        core, event = self.core("right_arm", np.eye(3), np.eye(3))
        initial = core.target.copy()
        core.hold("grip_released")
        event["tracking"]["Head"]["pose"][3:] = Rotation.from_rotvec([math.pi / 2, 0., 0.]).as_quat().tolist()
        self.advance(event)
        result = core.process_pose(event)
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(result["reason"], "headset_heading_undefined")
        self.assertIsNone(core.anchor)
        np.testing.assert_allclose(core.target, initial)

    def test_reflected_body_frame_is_rejected_at_config_load(self):
        config = json.loads(CONFIG.read_text())
        frames = json.loads((CONFIG.parent / config["humanoid_frames"]).read_text())
        frames["root_to_body_rotation"] = [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "humanoid_frames.json").write_text(json.dumps(frames))
            (path / "control.json").write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "handedness"):
                load_config(path / "control.json")
