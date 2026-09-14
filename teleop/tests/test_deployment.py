import copy
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
import uuid

import can
import numpy as np
import pinocchio as pin

from nero_pico_teleop.cli import demo_events
from nero_pico_teleop import deploy
from nero_pico_teleop.deploy import require_site_ready
from nero_pico_teleop.nero_differential_ik import NeroDifferentialIK
from nero_pico_teleop.nero_ik_process import ProcessNeroIK
from nero_pico_teleop.nero_joint_output import VirtualNeroJointOutput
from nero_pico_teleop.nero_relative_core import NeroKinematics, RelativeTeleop, load_config
from nero_pico_teleop.nero_stream import NeroStreamOutput, InputInterrupted, StreamTargets, stream_configs
from nero_pico_teleop.paths import DEFAULT_CONFIG, PROJECT_ROOT, use_agx_sdk
from nero_pico_teleop.pico_input_guard import InputGuard

SEED = np.deg2rad([0, 45, -60, 60, 0, 20, 0])
RECORDED_SEED = np.deg2rad([.161, 95.506, 2.457, -7.729, 97.669, .434, 42.748])
RECORDED_STALL = np.deg2rad([-3.786, 72.173, 2.492, 22.271, 97.826, -5.6, 46.845])


class StartupModeTests(unittest.TestCase):
    def run_startup(self, flags, *, profile_arms=None, left_unavailable=False):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            root = Path(directory)
            settings = json.loads((PROJECT_ROOT / "config/deployment.json").read_text())
            settings["control_config"] = str(PROJECT_ROOT / "config/nero_humanoid_config.json")
            if profile_arms is not None:
                settings["enabled_arms"] = profile_arms
            profile = root / "deployment.json"
            profile.write_text(json.dumps(settings))
            stack.enter_context(patch.object(deploy, "PROJECT_ROOT", root))
            stack.enter_context(patch("nero_pico_teleop.cli.PROJECT_ROOT", root))
            stack.enter_context(patch.object(sys, "argv", ["start_robot", "--profile", str(profile),
                                                         "--confirm-clearance", "--enable-joints", *flags]))
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("physical socket opened")))
            stack.enter_context(redirect_stdout(io.StringIO()))
            stack.enter_context(redirect_stderr(io.StringIO()))

            def check(_path, *, arm_id, **_kwargs):
                ok = not (left_unavailable and arm_id == "left_arm")
                return {"technical_checks_passed": ok, "errors": [] if ok else ["can1 unavailable"]}

            checks = stack.enter_context(patch("nero_pico_teleop.preflight.run_checks", side_effect=check))
            firmware = stack.enter_context(patch("nero_pico_teleop.firmware.query_firmware", return_value={
                "sent_frames": [], "blocked_frames": [], "error": None}))
            single = stack.enter_context(patch("nero_pico_teleop.nero_stream.run_physical_stream", return_value=0))
            dual = stack.enter_context(patch("nero_pico_teleop.nero_stream.run_physical_dual_stream", return_value=0))
            failure = None
            try:
                self.assertEqual(deploy.main(), 0)
            except (ValueError, SystemExit) as exc:
                failure = exc
            reports = [json.loads(path.read_text()) for path in (root / "logs").glob("startup_*.json")]
            return checks, firmware, single, dual, reports, failure

    def test_single_overrides_dual_profile_and_ignores_unavailable_left_arm(self):
        checks, firmware, single, dual, reports, failure = self.run_startup(
            ["--single"], profile_arms=["right_arm", "left_arm"], left_unavailable=True)
        self.assertIsNone(failure)
        self.assertEqual([call.kwargs["arm_id"] for call in checks.call_args_list], ["right_arm"])
        firmware.assert_called_once_with("can0")
        dual.assert_not_called()
        self.assertEqual(single.call_args.args[0]["arm_id"], "right_arm")
        self.assertEqual(single.call_args.args[0]["arm"]["controller_hand"], "right")
        self.assertTrue(single.call_args.args[0]["home_pose"]["on_startup"])
        self.assertTrue(single.call_args.kwargs["gripper"])
        self.assertEqual(single.call_args.kwargs["tcp_speed_mm_s"], 400.)
        self.assertEqual(reports[0]["teleop_mode"], "single")
        self.assertEqual(reports[0]["enabled_arms"], ["right_arm"])

    def test_dual_overrides_single_profile_and_launches_both_arms(self):
        checks, firmware, single, dual, reports, failure = self.run_startup(["--dual"])
        self.assertIsNone(failure)
        self.assertEqual([call.kwargs["arm_id"] for call in checks.call_args_list], ["right_arm", "left_arm"])
        self.assertEqual([call.args[0] for call in firmware.call_args_list], ["can0", "can1"])
        single.assert_not_called()
        self.assertEqual(set(dual.call_args.args[0]), {"right_arm", "left_arm"})
        self.assertEqual(reports[0]["teleop_mode"], "dual")

    def test_default_mode_is_right_arm_only(self):
        checks, _, single, dual, reports, failure = self.run_startup([], left_unavailable=True)
        self.assertIsNone(failure)
        self.assertEqual(checks.call_count, 1)
        self.assertEqual(single.call_count, 1)
        dual.assert_not_called()
        self.assertEqual(reports[0]["teleop_mode"], "single")

    def test_optional_telemetry_socket_forwards_through_single_and_dual_startup(self):
        for mode in ("--single", "--dual"):
            with self.subTest(mode=mode):
                _, _, single, dual, _, failure = self.run_startup([mode, "--telemetry-socket", "/tmp/nero_capture_test.sock"])
                self.assertIsNone(failure)
                runner = single if mode == "--single" else dual
                self.assertEqual(runner.call_args.kwargs["telemetry_socket"], Path("/tmp/nero_capture_test.sock"))

    def test_dual_reports_left_failure_without_starting_either_arm(self):
        _, firmware, single, dual, reports, failure = self.run_startup(["--dual"], left_unavailable=True)
        self.assertIsInstance(failure, ValueError)
        self.assertIn("can1 unavailable", str(failure))
        firmware.assert_not_called()
        single.assert_not_called()
        dual.assert_not_called()
        self.assertFalse(reports[0]["technical_checks_passed"])

    def test_conflicting_modes_are_rejected_before_hardware_checks(self):
        checks, firmware, single, dual, reports, failure = self.run_startup(["--single", "--dual"])
        self.assertIsInstance(failure, SystemExit)
        self.assertEqual(failure.code, 2)
        for call in (checks, firmware, single, dual):
            call.assert_not_called()
        self.assertEqual(reports, [])


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(DEFAULT_CONFIG)
        # Legacy mapping assertions below exercise the original fixed
        # tracking-to-base path; the deployment profile uses head_relative_tcp.
        self.config["mapping_mode"] = "fixed_tracking_to_base"
        self.model = NeroKinematics(self.config)

    def test_measured_directions_map_to_side_mount_axes_once(self):
        report = json.loads((PROJECT_ROOT / "tests/fixtures/legacy_directions.json").read_text())
        source = np.array(report["orthogonalized_source_basis_columns_up_outward_left"])
        np.testing.assert_allclose(np.array(self.config["tracking_to_base"]) @ source,
                                   [[1, 0, 0], [0, 0, 1], [0, 1, 0]], atol=1e-12)

    def test_regrip_tilt_does_not_change_world_translation(self):
        targets = []
        for rotation in (np.eye(3), pin.exp3(np.array([.7, -.3, .2]))):
            core = RelativeTeleop(self.config, SEED, ik=self.model)
            core.ik.solve = lambda *a: {"ok": True, "joints_rad": SEED.tolist()}
            event = copy.deepcopy(next(demo_events(self.config)))
            pose = event["tracking"]["Controller"]["right"]["pose"]
            pose[3:] = pin.Quaternion(rotation).coeffs().tolist()
            self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
            event["received_monotonic"] += .04
            event["processed_monotonic"] += .04
            pose[1] += .001
            result = core.process_pose(event)
            targets.append(np.array(result["tcp_target_base_m"]) - core.origin)
        np.testing.assert_allclose(targets[0], targets[1], atol=1e-12)
        np.testing.assert_allclose(targets[0], np.array(self.config["tracking_to_base"])[:, 1] * .0001, atol=1e-12)

    def test_spatial_rotation_uses_current_times_reference_inverse(self):
        config, _ = stream_configs(self.config, angular_speed_deg_s=30.)
        core = RelativeTeleop(config, SEED, ik=self.model)
        core.ik.solve = lambda *a: {"ok": True, "joints_rad": SEED.tolist()}
        event = copy.deepcopy(next(demo_events(config)))
        reference = pin.exp3(np.array([.4, -.3, .1]))
        pose = event["tracking"]["Controller"]["right"]["pose"]
        pose[3:] = pin.Quaternion(reference).coeffs().tolist()
        core.process_pose(event)
        spatial_delta = pin.exp3(np.array([0., .001, 0.]))
        pose[3:] = pin.Quaternion(spatial_delta @ reference).coeffs().tolist()
        event["received_monotonic"] += .04
        event["processed_monotonic"] += .04
        core.process_pose(event)
        mapping = np.array(config["tracking_to_base"])
        np.testing.assert_allclose(core.rotation, mapping @ spatial_delta @ mapping.T @ core.origin_rotation, atol=1e-10)

    def test_model_limits_round_inward_on_can_grid(self):
        for q in (self.model.model.lowerPositionLimit, self.model.model.upperPositionLimit):
            rounded = self.model.quantize_command(q)
            self.model.validate_joints(rounded)
            np.testing.assert_allclose(np.rad2deg(rounded) * 1000, np.round(np.rad2deg(rounded) * 1000), atol=1e-8)

    def test_one_cm_hand_requests_two_cm_robot_with_speed_limit(self):
        config, _ = stream_configs(self.config, scale=2., radius_mm=100.,
                                   tcp_speed_mm_s=20., translation_only=True)
        self.model.solve = lambda *args: {"ok": True, "joints_rad": SEED.tolist()}
        for axis in np.eye(3):
            core = StreamTargets(config, SEED, ik=self.model)
            event = copy.deepcopy(next(demo_events(config)))
            self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
            pose = event["tracking"]["Controller"]["right"]["pose"]
            pose[:3] = (np.array(pose[:3]) + np.array(config["tracking_to_base"]).T @ (axis * .01)).tolist()
            event["received_monotonic"] += .04
            event["processed_monotonic"] += .04
            self.assertEqual(core.process_pose(event)["state"], "FOLLOW")
            np.testing.assert_allclose(core.requested_tcp - core.origin, axis * .02, atol=1e-12)
            np.testing.assert_allclose(core.target - core.origin, axis * .0008, atol=1e-12)

    def test_double_scale_clips_to_session_radius_after_regrip(self):
        config, _ = stream_configs(self.config, scale=2., radius_mm=100.,
                                   tcp_speed_mm_s=20., translation_only=True)
        self.model.solve = lambda *args: {"ok": True, "joints_rad": SEED.tolist()}
        core = StreamTargets(config, SEED, ik=self.model)
        event = copy.deepcopy(next(demo_events(config)))
        core.process_pose(event)
        increment = np.array(config["tracking_to_base"]).T @ np.array([0., .01, 0.])
        for _ in range(7):
            pose = event["tracking"]["Controller"]["right"]["pose"]
            pose[:3] = (np.array(pose[:3]) + increment).tolist()
            event["received_monotonic"] += .04
            event["processed_monotonic"] += .04
            core.process_pose(event)
        np.testing.assert_allclose(core.unclipped_requested_tcp - core.origin, [0, .14, 0], atol=1e-12)
        np.testing.assert_allclose(core.requested_tcp - core.origin, [0, .1, 0], atol=1e-12)
        core.hold("grip_released")
        core.process_pose(event)
        pose[:3] = (np.array(pose[:3]) + increment * 7).tolist()
        event["received_monotonic"] += .04
        event["processed_monotonic"] += .04
        core.process_pose(event)
        np.testing.assert_allclose(core.requested_tcp - core.origin, [0, .1, 0], atol=1e-12)

    def test_invalid_stream_scales_are_rejected(self):
        for scale in (0., 2.01, float("inf"), float("nan")):
            with self.subTest(scale=scale), self.assertRaisesRegex(ValueError, "scale must be between"):
                stream_configs(self.config, scale=scale)

    def test_no_session_limits_keeps_full_target_and_speed_limit_after_regrip(self):
        config, output = stream_configs(self.config, scale=2., radius_mm=None,
                                        tcp_speed_mm_s=20., translation_only=True, session_limits=False)
        for settings in (config, output):
            self.assertIsNone(settings["max_displacement_m"])
            self.assertIsNone(settings["max_joint_session_deg"])
        self.model.solve = lambda *args: {"ok": True, "joints_rad": SEED.tolist()}
        core = StreamTargets(config, SEED, ik=self.model)
        event = copy.deepcopy(next(demo_events(config)))
        core.process_pose(event)
        increment = np.array(config["tracking_to_base"]).T @ np.array([0., .01, 0.])
        for _ in range(7):
            pose = event["tracking"]["Controller"]["right"]["pose"]
            pose[:3] = (np.array(pose[:3]) + increment).tolist()
            event["received_monotonic"] += .04
            event["processed_monotonic"] += .04
            self.assertEqual(core.process_pose(event)["state"], "FOLLOW")
        np.testing.assert_allclose(core.requested_tcp - core.origin, [0, .14, 0], atol=1e-12)
        self.assertFalse(core.translation_status(np.rad2deg(SEED))["workspace_clipped"])
        core.hold("grip_released")
        core.process_pose(event)
        previous_target = core.target.copy()
        pose[:3] = (np.array(pose[:3]) + increment).tolist()
        event["received_monotonic"] += .04
        event["processed_monotonic"] += .04
        core.process_pose(event)
        np.testing.assert_allclose(core.requested_tcp - previous_target, [0, .02, 0], atol=1e-12)
        np.testing.assert_allclose(core.target - previous_target, [0, .0008, 0], atol=1e-12)

    def test_recorded_j4_stall_progresses_only_after_session_limits_disabled(self):
        for enabled in (True, False):
            with self.subTest(session_limits=enabled):
                config, _ = stream_configs(self.config, radius_mm=100., tcp_speed_mm_s=20.,
                                           translation_only=True, session_limits=enabled)
                model = NeroDifferentialIK(config)
                origin = model.pose(RECORDED_SEED)
                goal = origin.translation + np.array([40.78762817681858, 44.82471442677466, 65.45602467076847]) / 1000
                q = RECORDED_STALL.copy()
                for _ in range(120):
                    pose = model.pose(q)
                    delta = goal - pose.translation
                    distance = np.linalg.norm(delta)
                    target = pose.translation + delta * min(1., .02 / 60 / distance) if distance else pose.translation
                    result = model.solve(target, origin.rotation, q, RECORDED_SEED, 1 / 60)
                    q = model.quantize_command(result["joints_rad"])
                if enabled:
                    np.testing.assert_allclose(q, RECORDED_STALL, atol=1e-12)
                else:
                    self.assertGreater(np.rad2deg(q[3] - RECORDED_SEED[3]), 30.)
                    self.assertLess(np.linalg.norm(model.pose(q).translation - goal), .0005)
                    self.assertLess(np.linalg.norm(pin.log3(origin.rotation.T @ model.pose(q).rotation)), np.deg2rad(.3))

    def test_invalid_session_limit_settings_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "session_limits must be a boolean"):
            stream_configs(self.config, session_limits="false")
        with self.assertRaisesRegex(ValueError, "radius_mm must be between"):
            stream_configs(self.config, radius_mm=None, session_limits=True)

    def test_deployment_forwards_disabled_limits_through_cli_without_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "deployment.json"
            profile.write_text(json.dumps({
                "control_config": str(DEFAULT_CONFIG), "duration_s": 120,
                "translation_only": True, "gripper": False, "scale": 2.,
                "session_limits": False, "radius_mm": None, "tcp_speed_mm_s": 20.,
                "angular_speed_deg_s": 5., "speed_percent": 5,
            }))
            args = ["start_robot", "--profile", str(profile), "--confirm-clearance", "--enable-joints"]
            with patch.object(deploy, "PROJECT_ROOT", Path(directory)), \
                    patch.object(sys, "argv", args), \
                    patch("socket.socket", side_effect=AssertionError("startup test opened a physical socket")), \
                    patch("nero_pico_teleop.preflight.run_checks", return_value={
                        "technical_checks_passed": True, "errors": []}), \
                    patch("nero_pico_teleop.firmware.query_firmware", return_value={
                        "sent_frames": [], "blocked_frames": [], "error": None}), \
                    patch("nero_pico_teleop.nero_stream.run_physical_stream", return_value=0) as run:
                self.assertEqual(deploy.main(), 0)
                self.assertFalse(run.call_args.kwargs["session_limits"])
                self.assertEqual(run.call_args.kwargs["scale"], 2.)
                self.assertTrue(run.call_args.kwargs["translation_only"])
                self.assertTrue(run.call_args.kwargs["enable_joints"])

    def test_unverified_zero_rejects_physical_startup(self):
        self.config["arm"]["connection_verification"]["physical_zero_alignment_verified"] = False
        with self.assertRaisesRegex(ValueError, "zero remains unverified"):
            require_site_ready(self.config)

    def test_differential_worker_runs_and_closes(self):
        config, _ = stream_configs(self.config, translation_only=True, session_limits=False)
        with ProcessNeroIK(config) as worker:
            pose = worker.pose(SEED)
            result = worker.solve(pose.translation + [0, .00001, 0], pose.rotation, SEED, SEED, .04)
            self.assertTrue(result["ok"], result)
            self.assertEqual(len(result["joints_rad"]), 7)


class HeadMappingTests(unittest.TestCase):
    def setUp(self):
        self.config, _ = stream_configs(load_config(DEFAULT_CONFIG), scale=1., tcp_speed_mm_s=200.,
                                        joint_speed_deg_s=60., translation_only=True, session_limits=False)
        self.assertEqual(self.config["mapping_mode"], "head_relative_tcp")
        self.model = NeroKinematics(self.config)
        self.model.solve = Mock(return_value={"ok": True, "joints_rad": SEED.tolist()})

    def core_and_event(self, seed=SEED, head_rotation=np.eye(3)):
        self.model.solve.return_value = {"ok": True, "joints_rad": seed.tolist()}
        core = StreamTargets(self.config, seed, ik=self.model)
        event = copy.deepcopy(next(demo_events(self.config)))
        event["tracking"]["Head"]["pose"][3:] = pin.Quaternion(head_rotation).coeffs().tolist()
        self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
        return core, event

    @staticmethod
    def advance(event, delta=(0., 0., 0.)):
        pose = event["tracking"]["Controller"]["right"]["pose"]
        pose[:3] = (np.asarray(pose[:3]) + delta).tolist()
        event["received_monotonic"] += .04
        event["processed_monotonic"] += .04
        event["tracking"]["timeStampNs"] += 40_000_000

    def test_six_directions_in_tcp_frame_with_rotated_head_and_robot(self):
        directions = (([0, 0, -1], [1, 0, 0]), ([0, 1, 0], [0, 1, 0]), ([1, 0, 0], [0, 0, 1]))
        rotations = (np.eye(3), pin.exp3(np.array([0., np.pi / 2, 0.])), pin.exp3(np.array([.4, -.7, .2])))
        for seed in (SEED, RECORDED_STALL):
            for head_rotation in rotations:
                for source, tcp_axis in directions:
                    for sign in (-1, 1):
                        with self.subTest(seed=seed.tolist(), source=source, sign=sign, head=head_rotation.tolist()):
                            core, event = self.core_and_event(seed, head_rotation)
                            self.advance(event, head_rotation @ np.asarray(source) * sign * .01)
                            result = core.process_pose(event)
                            self.assertEqual(result["state"], "FOLLOW")
                            in_tcp = self.model.pose(seed).rotation.T @ (core.requested_tcp - core.origin)
                            np.testing.assert_allclose(in_tcp, np.asarray(tcp_axis) * sign * .01, atol=1e-12)
                            self.assertLessEqual(np.linalg.norm(core.target - core.origin), .2 * .04 + 1e-12)

    def test_head_turn_does_not_move_target_or_change_axes_until_regrip(self):
        core, event = self.core_and_event()
        self.advance(event, [0, 0, -.002])
        core.process_pose(event)
        before = core.target.copy()
        head_rotation = pin.exp3(np.array([.2, .8, -.3]))
        event["tracking"]["Head"]["pose"][3:] = pin.Quaternion(head_rotation).coeffs().tolist()
        event["tracking"]["Controller"]["right"]["pose"][3:] = pin.Quaternion(pin.exp3(np.array([.7, .1, .4]))).coeffs().tolist()
        self.advance(event)
        core.process_pose(event)
        np.testing.assert_allclose(core.target, before, atol=1e-12)
        self.advance(event, [0, 0, -.002])
        core.process_pose(event)
        np.testing.assert_allclose(core.anchor_rotation.T @ (core.requested_tcp - core.origin), [.004, 0, 0], atol=1e-12)
        core.hold("grip_released")
        core.sync_command(RECORDED_STALL)
        self.model.solve.return_value = {"ok": True, "joints_rad": RECORDED_STALL.tolist()}
        before = core.target.copy()
        self.advance(event)
        self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
        np.testing.assert_array_equal(core.target, before)
        self.advance(event, head_rotation @ [0, 0, -.002])
        core.process_pose(event)
        np.testing.assert_allclose(self.model.pose(RECORDED_STALL).rotation.T @ (core.requested_tcp - before),
                                   [.002, 0, 0], atol=1e-12)

    def test_invalid_head_holds_without_legacy_fallback(self):
        bad_heads = (None, {}, {"status": 0, "pose": [0., 1.6, 0., 0., 0., 0., 1.]},
                     {"status": 3, "pose": [0., 0., 0., 0., 0., 0., 1.]},
                     {"status": 3, "pose": [0., 1.6, 0., 0., 0., 0., float("nan")]})
        for bad in bad_heads:
            for anchored in (False, True):
                with self.subTest(head=bad, anchored=anchored):
                    core, event = self.core_and_event()
                    if not anchored:
                        core.hold("new_clutch")
                    self.model.solve.reset_mock()
                    before = core.target.copy()
                    event["tracking"]["Head"] = bad
                    self.advance(event, [0, 0, -.01])
                    result = core.process_pose(event)
                    self.assertEqual((result["state"], result["reason"]), ("HOLD", "head_pose_invalid"))
                    self.assertIsNone(core.head_tracking_to_base)
                    np.testing.assert_array_equal(core.target, before)
                    self.model.solve.assert_not_called()

    def test_head_loss_revokes_input_and_requires_release(self):
        guard = InputGuard(stale_s=.2, require_head_pose=True)
        frames = iter(demo_events(self.config))
        for _ in range(11):
            event = next(frames)
            guard.step(event["tracking"], event["received_monotonic"], event["processed_monotonic"], event["device_id"])
        self.assertTrue(guard.accepted)
        event = next(frames)
        event["tracking"]["Head"]["status"] = 0
        result = guard.step(event["tracking"], event["received_monotonic"], event["processed_monotonic"], event["device_id"])
        self.assertFalse(result["input_held"])
        self.assertEqual(result["reason"], "head_pose_invalid")
        event = next(frames)
        result = guard.step(event["tracking"], event["received_monotonic"], event["processed_monotonic"], event["device_id"])
        self.assertEqual(result["state"], "WAIT_RELEASE")

    def test_relative_rotation_uses_same_clutch_mapping(self):
        self.config["orientation_mode"] = "relative_link7"
        self.config.update(rotation_scale=1., max_rotation_session_deg=30., max_angular_speed_deg_s=30.)
        core, event = self.core_and_event(head_rotation=pin.exp3(np.array([.4, .5, -.2])))
        spatial_delta = pin.exp3(np.array([0., .001, 0.]))
        event["tracking"]["Controller"]["right"]["pose"][3:] = pin.Quaternion(spatial_delta).coeffs().tolist()
        self.advance(event)
        core.process_pose(event)
        expected = core.head_tracking_to_base @ spatial_delta @ core.head_tracking_to_base.T @ core.anchor_rotation
        np.testing.assert_allclose(core.rotation, expected, atol=1e-12)


class VirtualOutputTests(unittest.TestCase):
    def setUp(self):
        self.guard = patch.object(socket, "socket", side_effect=AssertionError("test opened a physical socket"))
        self.guard.start()
        self.addCleanup(self.guard.stop)
        use_agx_sdk()
        from pyAgxArm import AgxArmFactory, create_agx_arm_config
        channel = "deployment_test_" + uuid.uuid4().hex
        self.receiver = can.Bus(interface="virtual", channel=channel)
        self.addCleanup(self.receiver.shutdown)
        self.arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot="nero", firmeware_version="v120", channel=channel, interface="virtual", auto_connect=False))
        self.arm.connect()
        self.addCleanup(self.arm.disconnect)
        self.config = load_config(DEFAULT_CONFIG)
        self.model = NeroKinematics(self.config)
        self.now = 10.
        self.snapshot = patch("nero_pico_teleop.nero_joint_output.robot_snapshot", return_value={
            "joints_rad": SEED.tolist(), "joint_output_blockers": []})
        self.snapshot.start()
        self.addCleanup(self.snapshot.stop)
        self.output = VirtualNeroJointOutput(self.arm, self.config, self.model, clock=lambda: self.now)
        self.addCleanup(self.output.__exit__)
        self.output.activate()

    def test_sdk_sends_four_frames_for_seven_joints(self):
        self.assertIsNone(self.receiver.recv(timeout=.01))
        target = SEED.copy()
        target[6] += np.deg2rad(.005)
        self.now += .04
        self.output.send_target(target, self.now)
        frames = [self.receiver.recv(timeout=.1) for _ in range(4)]
        self.assertEqual([f.arbitration_id for f in frames], [0x155, 0x156, 0x157, 0x170])
        values = [int.from_bytes(f.data[i:i+4], "big", signed=True)
                  for f in frames for i in ((0,) if f.arbitration_id == 0x170 else (0, 4))]
        np.testing.assert_array_equal(values, np.rint(np.rad2deg(target) * 1000).astype(int))
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_head_mapping_reaches_tcp_axes_through_ik_and_sdk_virtual_can(self):
        config, self.output.config = stream_configs(self.config, scale=1., tcp_speed_mm_s=200.,
                                                    joint_speed_deg_s=60., translation_only=True,
                                                    session_limits=False)
        core = StreamTargets(config, SEED, ik=NeroDifferentialIK(config))
        event = copy.deepcopy(next(demo_events(config)))
        head_rotation = pin.exp3(np.array([.3, -.8, .1]))
        event["tracking"]["Head"]["pose"][3:] = pin.Quaternion(head_rotation).coeffs().tolist()
        directions = (([0, 0, -1], [1, 0, 0]), ([0, 1, 0], [0, 1, 0]), ([1, 0, 0], [0, 0, 1]))
        for source, tcp_axis in directions:
            for sign in (-1, 1):
                core.hold("grip_released")
                core.sync_command(self.output.last_target)
                start = self.model.pose(self.output.last_target)
                event["received_monotonic"] = event["processed_monotonic"] = self.now
                self.assertEqual(core.process_pose(event)["state"], "ANCHOR")
                pose = event["tracking"]["Controller"]["right"]["pose"]
                pose[:3] = (np.asarray(pose[:3]) + head_rotation @ np.asarray(source) * sign * .003).tolist()
                for _ in range(6):
                    self.now += .04
                    event["received_monotonic"] = event["processed_monotonic"] = self.now
                    event["tracking"]["timeStampNs"] += 40_000_000
                    result = core.process_pose(event)
                    self.assertIn(result["state"], ("FOLLOW", "LIMITED"))
                    with patch("nero_pico_teleop.nero_joint_output.robot_snapshot", return_value={
                            "joints_rad": self.output.last_target.tolist(), "joint_output_blockers": []}):
                        self.assertTrue(self.output.send_target(core.q, self.now))
                    core.sync_command(self.output.last_target)
                    frames = [self.receiver.recv(timeout=.1) for _ in range(4)]
                    self.assertEqual([f.arbitration_id for f in frames], [0x155, 0x156, 0x157, 0x170])
                actual_tcp_delta = start.rotation.T @ (self.model.pose(self.output.last_target).translation - start.translation)
                np.testing.assert_allclose(actual_tcp_delta, np.asarray(tcp_axis) * sign * .003, atol=.0001)
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_unexpected_mode_write_is_blocked(self):
        with self.assertRaisesRegex(RuntimeError, "authorized joint batch"):
            self.arm.set_motion_mode("j")
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_stale_target_sends_nothing(self):
        self.now += .04
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.output.send_target(SEED, self.now - 1.)
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_disabled_session_limits_allow_commands_beyond_old_joint_and_tcp_bounds(self):
        _, self.output.config = stream_configs(self.config, radius_mm=None, tcp_speed_mm_s=20.,
                                               translation_only=True, session_limits=False)
        # Offline IK solutions from the recorded seed exceed each former boundary.
        cases = (
            np.deg2rad([-5.686, 65.581, 2.76, 42.142, 98.133, -18.507, 49.909]),
            np.deg2rad([-1.502, 78.441, 2.431, 2.683, 97.867, 7.158, 42.466]),
        )
        self.assertGreater(np.max(np.abs(np.rad2deg(cases[0] - RECORDED_SEED))), 30.)
        self.assertGreater(np.linalg.norm(self.model.pose(cases[1]).translation
                                          - self.model.pose(RECORDED_SEED).translation), .1)
        for current in cases:
            self.output.seed = RECORDED_SEED.copy()
            self.output.last_target = current.copy()
            self.now += .04
            with patch("nero_pico_teleop.nero_joint_output.robot_snapshot", return_value={
                    "joints_rad": current.tolist(), "joint_output_blockers": []}):
                self.assertTrue(self.output.send_target(current, self.now))
            frames = [self.receiver.recv(timeout=.1) for _ in range(4)]
            self.assertEqual([frame.arbitration_id for frame in frames], [0x155, 0x156, 0x157, 0x170])
            self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_disabled_session_limits_still_reject_joint_speed_violation(self):
        _, self.output.config = stream_configs(self.config, session_limits=False)
        target = SEED.copy()
        target[0] += np.deg2rad(2.)
        self.now += .04
        with self.assertRaisesRegex(RuntimeError, "joint command step/speed limit"):
            self.output.send_target(target, self.now)
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_disabled_session_limits_still_reject_model_joint_limit_violation(self):
        _, self.output.config = stream_configs(self.config, session_limits=False)
        target = SEED.copy()
        target[0] = self.model.model.upperPositionLimit[0] + .01
        self.now += .04
        with self.assertRaisesRegex(RuntimeError, "exceeds official Nero model limits"):
            self.output.send_target(target, self.now)
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_released_clutch_rejects_batch_before_sdk_call(self):
        fake = type("Stream", (), {})()
        fake.state, fake.motion_epoch = "ACTIVE", 1
        fake.input_source = type("Source", (), {"lock": threading.RLock(), "permits": lambda *_: False})()
        fake._duration_elapsed = lambda: False
        with self.assertRaisesRegex(InputInterrupted, "clutch"):
            NeroStreamOutput._batch(fake, SEED)
        self.assertIsNone(self.receiver.recv(timeout=.01))


if __name__ == "__main__":
    unittest.main()
