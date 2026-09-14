import copy
from contextlib import ExitStack
import json
import math
from pathlib import Path
import queue
import socket
import threading
import time
import tempfile
import unittest
from unittest.mock import patch
import uuid

import can
import numpy as np
import pinocchio as pin

from nero_pico_teleop.cli import demo_events
from nero_pico_teleop import deploy
from nero_pico_teleop.humanoid_frames import body_from_arm, head_to_arm, heading_rotation
from nero_pico_teleop.nero_differential_ik import NeroDifferentialIK
from nero_pico_teleop.nero_joint_output import VirtualNeroJointOutput
from nero_pico_teleop.nero_placo_ik import ProcessPlacoIK
from nero_pico_teleop.nero_relative_core import NeroKinematics, load_config
from nero_pico_teleop.nero_stream import NeroStreamOutput, StreamInput, StreamTargets, stream_configs
from nero_pico_teleop.paths import PROJECT_ROOT, use_agx_sdk
from nero_pico_teleop.pico_input_guard import InputGuard

CONFIG = PROJECT_ROOT / "config/nero_humanoid_config.json"
SEED = np.deg2rad([0, 45, -60, 60, 0, 20, 0])
# First zero-progress sample above 100 mm error in the 20260912T160334 log.
LOG_SEED = np.array([.061732295643039434, 1.4781542500990377, .12334241823843928,
                    .03424335992412875, 1.8265394220896258, .066793750473823, .6773622826989993])
LOG_STALL = np.array([.05302310267558773, 1.4886436789035236, .12480849481011451,
                     -.016161748873467493, 1.8253002383207095, .10969394348784361, .6817605124140249])
LOG_DISPLACEMENT = np.array([100.39120058719564, -22.43458146903407, 28.518221455329396]) / 1000


def configs(holding=False):
    config = load_config(CONFIG)
    config["mapping_mode"] = "head_relative_body"
    return stream_configs(config, translation_only=holding, session_limits=False,
                          tcp_speed_mm_s=200., joint_speed_deg_s=60., angular_speed_deg_s=30.)


class HumanoidMappingTests(unittest.TestCase):
    def test_default_startup_selects_humanoid_profile(self):
        import sys
        with tempfile.TemporaryDirectory() as directory:
            args = ["start_robot", "--profile", str(PROJECT_ROOT / "config/deployment.json"), "--confirm-clearance"]
            with patch.object(deploy, "PROJECT_ROOT", Path(directory)), patch.object(sys, "argv", args), \
                    patch("nero_pico_teleop.preflight.run_checks", return_value={"technical_checks_passed": True, "errors": []}), \
                    patch("nero_pico_teleop.firmware.query_firmware", return_value={"sent_frames": [], "blocked_frames": [], "error": None}), \
                    patch("nero_pico_teleop.nero_stream.run_physical_stream", return_value=0) as run:
                self.assertEqual(deploy.main(), 0)
                self.assertEqual(run.call_args.args[0]["mapping_mode"], "head_relative_body")
                config, _ = stream_configs(run.call_args.args[0], **{k: v for k, v in run.call_args.kwargs.items()
                    if k not in ("clearance_confirmed", "enable_joints")})
                self.assertEqual(config["ik_solver"], "placo")
                self.assertEqual(config["joint_command_mode"], "move_js")
                self.assertEqual(config["orientation_mode"], "relative_link7")
                self.assertEqual(config["translation_scale"], 1.)
                self.assertFalse(config["feedback_gating_enabled"])
                self.assertEqual(config["max_tcp_speed_m_s"], .4)
                self.assertEqual(config["max_joint_speed_deg_s"], 120.)
                self.assertEqual(config["max_angular_speed_deg_s"], 60.)
                self.assertEqual(config["max_joint_step_deg"], 2.)

    def test_speed_validation_rejects_invalid_values_and_preserves_legacy_bounds(self):
        config = load_config(CONFIG)
        for option, maximum in (("tcp_speed_mm_s", 400.), ("joint_speed_deg_s", 120.),
                                ("angular_speed_deg_s", 60.)):
            for value in (0., maximum + 1., float("nan"), float("inf"), True):
                with self.subTest(option=option, value=value):
                    with self.assertRaisesRegex(ValueError, option):
                        stream_configs(config, **{option: value})
            legacy = {**config, "runtime_profile": "legacy"}
            with self.assertRaisesRegex(ValueError, option):
                stream_configs(legacy, **{option: maximum})

    def test_body_directions_for_both_mounts_and_head_yaws(self):
        config, _ = configs()
        for roll in (-1.5708, 1.5708):
            config["humanoid_mount"]["rpy_rad"][0] = roll
            mount = body_from_arm(config)
            for yaw in (-1.2, 0., 2.1):
                head = pin.exp3(np.array([0., yaw, 0.])) @ pin.exp3(np.array([.4, 0., 0.]))
                mapping = head_to_arm(config, head)
                heading = heading_rotation(head)
                for source, body in (([0, 0, -1], [1, 0, 0]), ([0, 1, 0], [0, 0, 1]), ([1, 0, 0], [0, -1, 0])):
                    np.testing.assert_allclose(mount.rotation @ mapping @ heading @ source, body, atol=1e-12)

    def test_regrip_updates_heading_but_not_body_axes_with_wrist_rotation(self):
        config, _ = configs()
        model = NeroKinematics(config)
        model.solve = lambda *args: {"ok": True, "joints_rad": SEED.tolist()}
        core = StreamTargets(config, SEED, model)
        event = copy.deepcopy(next(demo_events(config)))
        core.process_pose(event)
        first = core.head_tracking_to_base.copy()
        head = pin.exp3(np.array([0., .8, 0.]))
        event["tracking"]["Head"]["pose"][3:] = pin.Quaternion(head).coeffs().tolist()
        event["received_monotonic"] += .04
        event["processed_monotonic"] += .04
        core.process_pose(event)
        np.testing.assert_allclose(core.head_tracking_to_base, first)
        core.hold("release")
        q = SEED.copy(); q[6] += .1
        core.sync_command(q)
        origin = core.target.copy()
        core.process_pose(event)
        np.testing.assert_allclose(core.target, origin)
        np.testing.assert_allclose(core.head_tracking_to_base, head_to_arm(config, head))

    def test_hold_mode_has_no_inherited_180_degree_allowance(self):
        target, output = configs(holding=True)
        for config in (target, output):
            self.assertNotIn("max_rotation_session_deg", config)
            self.assertEqual(config["orientation_mode"], "hold_initial_link7")
        self.assertEqual(configs()[0]["max_rotation_session_deg"], 180.)


class PlacoTests(unittest.TestCase):
    def test_default_tcp_mapping_with_placo(self):
        config, _ = configs()
        config["mapping_mode"] = "head_relative_tcp"
        with ProcessPlacoIK(config) as solver:
            initial = solver.pose(SEED)
            for source, axis in (([0, 0, -1], [1, 0, 0]), ([0, 1, 0], [0, 1, 0]), ([1, 0, 0], [0, 0, 1])):
                for sign in (-1, 1):
                    core = StreamTargets(config, SEED, solver)
                    event = copy.deepcopy(next(demo_events(config)))
                    core.process_pose(event)
                    pose = event["tracking"]["Controller"]["right"]["pose"]
                    pose[:3] = (np.asarray(pose[:3]) + np.asarray(source) * sign * .01).tolist()
                    for _ in range(100):
                        event["received_monotonic"] += 1 / 75
                        event["processed_monotonic"] += 1 / 75
                        core.process_pose(event)
                        core.sync_command(solver.quantize_command(core.q))
                    expected = initial.translation + initial.rotation @ (np.asarray(axis) * sign * .01)
                    self.assertLess(np.linalg.norm(core.target - expected), .00015)

    def test_current_position_takeover_encodes_js_without_enabling_other_arm(self):
        _, output_config = configs()
        model = NeroKinematics(output_config)
        with ExitStack() as stack:
            stack.enter_context(patch.object(socket, "socket", side_effect=AssertionError("physical socket opened")))
            use_agx_sdk()
            from pyAgxArm import AgxArmFactory, create_agx_arm_config
            channel = "takeover_test_" + uuid.uuid4().hex
            receiver = can.Bus(interface="virtual", channel=channel)
            stack.callback(receiver.shutdown)
            arm = AgxArmFactory.create_arm(create_agx_arm_config(
                robot="nero", firmeware_version="v120", interface="virtual", channel=channel, auto_connect=False))
            arm.connect(); stack.callback(arm.disconnect)
            stack.enter_context(patch("nero_pico_teleop.nero_joint_output.robot_snapshot", side_effect=lambda *a, **kw: {
                "joints_rad": SEED.tolist(), "joint_output_blockers": [], "group_timestamps_s": [time.time()] * 4}))
            output = stack.enter_context(NeroStreamOutput(arm, output_config, model, clearance_confirmed=True))
            stack.enter_context(patch.object(output, "_startup_feedback", return_value=({"joints_enabled": [True] * 7}, SEED.copy())))
            # Test the actual command transaction; physical acknowledgements
            # are deliberately not simulated as evidence of hardware readiness.
            stack.enter_context(patch.object(output, "_wait_takeover_ready"))
            output.takeover()
            frames = []
            while (frame := receiver.recv(timeout=.01)) is not None:
                frames.append(frame)
            modes = [frame for frame in frames if frame.arbitration_id == 0x151]
            self.assertEqual(len(modes), 1)
            self.assertEqual((modes[0].data[1], modes[0].data[3]), (1, 0xAD))
            self.assertEqual(len(frames), 13)
            self.assertTrue(all(frame.arbitration_id in (0x151, 0x155, 0x156, 0x157, 0x170) for frame in frames))
            np.testing.assert_allclose(output.last_target, SEED)

    def test_recorded_orientation_boundary_stall(self):
        config, _ = configs(holding=True)
        old_config = {**config, "ik_priority": "translation", "max_joint_step_deg": 1.}
        old = NeroDifferentialIK(old_config)
        origin, stalled = old.pose(LOG_SEED), old.pose(LOG_STALL)
        direction = origin.translation + LOG_DISPLACEMENT - stalled.translation
        goal = stalled.translation + direction / np.linalg.norm(direction) * .02
        errors = []
        with ProcessPlacoIK(config) as new:
            for solver in (old, new):
                q = LOG_STALL.copy()
                for _ in range(200):
                    pose = solver.pose(q)
                    delta = goal - pose.translation
                    target = pose.translation + delta * min(1., .2 / 60 / max(np.linalg.norm(delta), 1e-12))
                    result = solver.solve(target, origin.rotation, q, LOG_SEED, 1 / 60)
                    q = solver.quantize_command(result["joints_rad"])
                errors.append(np.linalg.norm(solver.pose(q).translation - goal))
            self.assertGreater(errors[0], .019)
            self.assertLess(errors[1], .0001)
            self.assertLess(np.linalg.norm(pin.log3(new.pose(q).rotation.T @ origin.rotation)), math.radians(.3))
        self.assertIsNone(new.process)

    def test_pose_rotation_and_position_converge(self):
        config, _ = configs()
        with ProcessPlacoIK(config) as solver:
            start = solver.pose(SEED)
            for axis in np.eye(3):
                q = SEED.copy()
                rotation = pin.exp3(axis * math.radians(10.)) @ start.rotation
                for _ in range(150):
                    pose = solver.pose(q)
                    dr = pin.log3(pose.rotation.T @ rotation)
                    bounded = pose.rotation @ pin.exp3(dr * min(1., math.radians(30.) / 75 / max(np.linalg.norm(dr), 1e-12)))
                    result = solver.solve(start.translation, bounded, q, SEED, 1 / 75)
                    q = solver.quantize_command(result["joints_rad"])
                self.assertLess(np.linalg.norm(solver.pose(q).translation - start.translation), .0002)
                self.assertLess(np.linalg.norm(pin.log3(solver.pose(q).rotation.T @ rotation)), math.radians(.1))

    def test_both_body_mounts_rotate_about_expected_axes_through_placo(self):
        config, _ = stream_configs(load_config(CONFIG), session_limits=False,
            tcp_speed_mm_s=400., joint_speed_deg_s=120., angular_speed_deg_s=60.)
        heading = pin.exp3(np.array([0., .9, 0.]))
        head = heading @ pin.exp3(np.array([.4, 0., 0.]))
        controller = pin.exp3(np.array([.5, -.6, .3]))
        with ProcessPlacoIK(config) as solver:
            initial = solver.pose(SEED)
            for side in ("left_arm", "right_arm"):
                side_config = copy.deepcopy(config)
                side_config["humanoid_mount"] = side_config["humanoid_mounts"][side]
                mount = body_from_arm(side_config).rotation
                for source, body_axis in (([0, 0, -1], [1, 0, 0]), ([0, 1, 0], [0, 0, 1]),
                                          ([1, 0, 0], [0, -1, 0])):
                    for sign in (-1, 1):
                        core = StreamTargets(side_config, SEED, solver)
                        event = copy.deepcopy(next(demo_events(side_config)))
                        event["tracking"]["Head"]["pose"][3:] = pin.Quaternion(head).coeffs().tolist()
                        pose = event["tracking"]["Controller"]["right"]["pose"]
                        pose[3:] = pin.Quaternion(controller).coeffs().tolist()
                        core.process_pose(event)
                        delta = pin.exp3(np.asarray(source) * sign * math.radians(5.))
                        pose[3:] = pin.Quaternion(heading @ delta @ heading.T @ controller).coeffs().tolist()
                        for _ in range(80):
                            event["received_monotonic"] += 1 / 75
                            event["processed_monotonic"] += 1 / 75
                            core.process_pose(event)
                            core.sync_command(solver.quantize_command(core.q))
                        actual = solver.pose(core.q)
                        expected_body = pin.exp3(np.asarray(body_axis) * sign * math.radians(5.))
                        actual_body = mount @ actual.rotation @ initial.rotation.T @ mount.T
                        self.assertLess(np.linalg.norm(pin.log3(expected_body.T @ actual_body)), math.radians(.1))
                        self.assertLess(np.linalg.norm(actual.translation - initial.translation), .0002)

    def test_faster_placo_six_directions_through_virtual_js_output(self):
        config, output_config = stream_configs(load_config(CONFIG), session_limits=False,
            tcp_speed_mm_s=400., joint_speed_deg_s=120., angular_speed_deg_s=60.)
        with ExitStack() as stack:
            solver = stack.enter_context(ProcessPlacoIK(config))
            stack.enter_context(patch.object(socket, "socket", side_effect=AssertionError("physical socket opened")))
            use_agx_sdk()
            from pyAgxArm import AgxArmFactory, create_agx_arm_config
            channel = "humanoid_test_" + uuid.uuid4().hex
            receiver = can.Bus(interface="virtual", channel=channel)
            stack.callback(receiver.shutdown)
            arm = AgxArmFactory.create_arm(create_agx_arm_config(
                robot="nero", firmeware_version="v120", interface="virtual", channel=channel, auto_connect=False))
            arm.connect(); stack.callback(arm.disconnect)
            arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.JS)
            mode = receiver.recv(timeout=.1)
            self.assertEqual((mode.arbitration_id, mode.data[1], mode.data[3]), (0x151, 1, 0xAD))
            now = [10.]
            measured = [SEED.copy()]
            stack.enter_context(patch("nero_pico_teleop.nero_joint_output.robot_snapshot",
                                      side_effect=lambda *a, **kw: {"joints_rad": measured[0], "joint_output_blockers": []}))
            output = stack.enter_context(VirtualNeroJointOutput(arm, output_config, solver, clock=lambda: now[0]))
            output.activate()
            peak_tcp_speed, peak_joint_speed = 0., 0.
            for axis in range(3):
                for sign in (-1, 1):
                    core = StreamTargets(config, output.last_target, solver)
                    event = copy.deepcopy(next(demo_events(config)))
                    event["received_monotonic"] = event["processed_monotonic"] = now[0]
                    core.process_pose(event)
                    event["tracking"]["Controller"]["right"]["pose"][axis] += sign * .03
                    for _ in range(120):
                        # Recorded scheduling is about 70 Hz, below nominal 75 Hz.
                        now[0] += 1 / 70
                        event["received_monotonic"] = event["processed_monotonic"] = now[0]
                        event["tracking"]["timeStampNs"] += 14_285_714
                        result = core.process_pose(event)
                        self.assertIn(result["state"], ("FOLLOW", "LIMITED"))
                        previous_q = output.last_target.copy()
                        previous_pose = solver.pose(previous_q)
                        output.send_target(core.q, now[0])
                        peak_tcp_speed = max(peak_tcp_speed, np.linalg.norm(
                            solver.pose(output.last_target).translation - previous_pose.translation) * 70)
                        peak_joint_speed = max(peak_joint_speed, np.max(
                            np.abs(np.rad2deg(output.last_target - previous_q))) * 70)
                        measured[0] = output.last_target.copy()
                        core.sync_command(output.last_target)
                        frames = [receiver.recv(timeout=.1) for _ in range(4)]
                        self.assertEqual([f.arbitration_id for f in frames], [0x155, 0x156, 0x157, 0x170])
                    self.assertLess(np.linalg.norm(core.target - core.requested_tcp), .00015)
            self.assertIsNone(receiver.recv(timeout=.01))
            self.assertEqual(output.frames_sent, 6 * 120 * 4)
            self.assertGreater(peak_tcp_speed, .3)
            self.assertLessEqual(peak_tcp_speed, output_config["max_tcp_speed_m_s"])
            self.assertGreater(peak_joint_speed, 60.)
            self.assertLessEqual(peak_joint_speed, output_config["max_joint_speed_deg_s"])

    def test_faster_orientation_converges_without_the_old_angular_cap(self):
        config, _ = stream_configs(load_config(CONFIG), session_limits=False,
            tcp_speed_mm_s=400., joint_speed_deg_s=120., angular_speed_deg_s=60.)
        with ProcessPlacoIK(config) as solver:
            core = StreamTargets(config, SEED, solver)
            event = copy.deepcopy(next(demo_events(config)))
            core.process_pose(event)
            initial = solver.pose(SEED)
            pose = event["tracking"]["Controller"]["right"]["pose"]
            reference = pin.Quaternion(np.asarray(pose[3:])).matrix()
            pose[3:] = pin.Quaternion(pin.exp3(np.array([0., math.radians(20.), 0.])) @ reference).coeffs().tolist()
            peak_angular_speed = 0.
            for _ in range(100):
                previous = solver.pose(core.q)
                event["received_monotonic"] += 1 / 70
                event["processed_monotonic"] += 1 / 70
                core.process_pose(event)
                core.sync_command(solver.quantize_command(core.q))
                actual = solver.pose(core.q)
                speed = math.degrees(np.linalg.norm(pin.log3(previous.rotation.T @ actual.rotation))) * 70
                peak_angular_speed = max(peak_angular_speed, speed)
                self.assertLessEqual(speed, 61.)
            self.assertGreater(peak_angular_speed, 45.)
            self.assertAlmostEqual(math.degrees(np.linalg.norm(pin.log3(initial.rotation.T @ actual.rotation))),
                                   20., delta=.1)
            self.assertLess(np.linalg.norm(actual.translation - initial.translation), .0005)


class InputRecoveryTests(unittest.TestCase):
    def test_dropout_and_head_loss_require_fresh_release_then_new_grip(self):
        config, _ = configs()
        events = queue.Queue()
        class Pico:
            def next(self):
                try:
                    return events.get(timeout=.01)
                except queue.Empty:
                    return {"kind": "tick", "received_monotonic": time.monotonic()}
        guard = InputGuard(stale_s=.2, neutral_s=0., require_head_pose=True)
        with StreamInput(Pico(), guard, recover_input=True) as source:
            counter = [0]
            def push(grip, head=True):
                counter[0] += 1
                event = copy.deepcopy(next(demo_events(config)))
                event["received_monotonic"] = event["processed_monotonic"] = time.monotonic()
                event["tracking"]["timeStampNs"] = 1_000_000_000 + counter[0] * 10_000_000
                event["tracking"]["Controller"]["right"]["grip"] = grip
                if not head:
                    event["tracking"]["Head"]["status"] = 0
                frames = source.snapshot()["frames"]
                events.put(event)
                deadline = time.monotonic() + 1.
                while source.snapshot()["frames"] == frames and time.monotonic() < deadline:
                    time.sleep(.001)
                self.assertGreater(source.snapshot()["frames"], frames)
            push(0.); push(1.)
            self.assertTrue(source.permits(source.epoch))
            epoch = source.epoch
            source.suspend("tracking_stale")
            self.assertFalse(source.permits(epoch))
            push(1.)
            self.assertFalse(source.held)
            push(0.); push(1.)
            self.assertTrue(source.permits(source.epoch))
            self.assertGreater(source.epoch, epoch)
            push(1., head=False)
            self.assertIsNone(source.terminal_reason)
            self.assertFalse(source.permits(source.epoch))
            push(1.); self.assertFalse(source.held)
            push(0.); push(1.)
            self.assertTrue(source.permits(source.epoch))

    def test_stale_input_holds_and_keeps_watchdog_feedback_observation(self):
        source = StreamInput(None, InputGuard(), recover_input=True)
        source.held = True
        source.guard.accepted = True
        source.guard.last_valid_received = 1.
        class Output(NeroStreamOutput):
            def __init__(self):
                self.home_goal = None
            def hold_motion(self):
                self.state = "HOLDING"
            def _duration_elapsed(self):
                return False
            def _update_hold_observation(self):
                self.observations += 1
        output = Output()
        output.lock = threading.RLock()
        output.state, output.input_source, output.input_seen = "ACTIVE", source, 1.
        output.clock = lambda: 2.
        output.config = {"max_gap_s": .2, "recover_input": True}
        output.observations = 0
        NeroStreamOutput.tick(output)
        self.assertEqual(output.state, "HOLDING")
        self.assertIsNone(output.input_seen)
        self.assertFalse(source.held)
        self.assertEqual(output.observations, 1)


if __name__ == "__main__":
    unittest.main()
