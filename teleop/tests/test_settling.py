from contextlib import ExitStack
import math
import socket
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

import can
import numpy as np

from nero_pico_teleop.nero_relative_core import NeroKinematics, load_config
from nero_pico_teleop.nero_stream import NeroStreamOutput, run_physical_stream, run_stream_session, stream_configs
from nero_pico_teleop.paths import PROJECT_ROOT, use_agx_sdk


# Current-position takeover and final feedback from 20260912T165728_752219Z.
TARGET = np.deg2rad([6.083, 85.215, 5.714, 2.780, 105.994, 1.575, 36.615])
MEASURED = np.deg2rad([6.083, 85.215, 5.718, 2.798, 106.019, 1.575, 36.876])


class VirtualStreamCase(unittest.TestCase):
    feedback_gating = True

    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(socket, "socket", side_effect=AssertionError("physical socket opened")))
        use_agx_sdk()
        from pyAgxArm import AgxArmFactory, create_agx_arm_config
        channel = "settling_test_" + uuid.uuid4().hex
        self.receiver = can.Bus(interface="virtual", channel=channel)
        self.stack.callback(self.receiver.shutdown)
        arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot="nero", firmeware_version="v120", interface="virtual", channel=channel, auto_connect=False))
        arm.connect()
        self.stack.callback(arm.disconnect)
        config = load_config(PROJECT_ROOT / "config/nero_humanoid_config.json")
        self.targets_config, output_config = stream_configs(config, session_limits=False)
        if self.feedback_gating:
            # Exercise the optional legacy observer independently of defaults.
            output_config["feedback_gating_enabled"] = True
        self.now = 10.
        self.measured = TARGET.copy()
        self.blockers = []
        self.stack.enter_context(patch("nero_pico_teleop.nero_joint_output.robot_snapshot", side_effect=self.snapshot))
        self.output = self.stack.enter_context(NeroStreamOutput(
            arm, output_config, NeroKinematics(output_config), clearance_confirmed=True, clock=lambda: self.now))
        self.output.activate()
        self.measured = MEASURED.copy()
        self.stack.enter_context(patch("nero_pico_teleop.nero_pilot.time",
                                      SimpleNamespace(sleep=self.advance, time=time.time)))

    def snapshot(self, *_args, **_kwargs):
        return {"joints_rad": self.measured.tolist(), "joint_output_blockers": self.blockers,
                "group_timestamps_s": [time.time()] * 4, "status": {"motion_status": 0}}

    def advance(self, seconds):
        self.now += seconds

    def run_session(self, neutral):
        records = []
        with patch.object(self.output, "takeover"), patch.object(self.output, "start_watchdog"), \
                patch("nero_pico_teleop.nero_placo_ik.ProcessPlacoIK") as solver, \
                patch("nero_pico_teleop.nero_stream.wait_neutral", side_effect=neutral):
            solver.return_value.__enter__.return_value.solve_timeout_s = .15
            result = run_stream_session(self.output, None, self.targets_config, 1., lambda: False, records.append)
        return result, records


class SettlingTests(VirtualStreamCase):
    def test_recorded_static_offset_settles_in_js_without_changing_target(self):
        for state in ("ACTIVE", "FROZEN"):
            self.output.state = state
            result = self.output.observe_settled()
            self.assertTrue(result["feedback_settled"], result)
            self.assertAlmostEqual(result["settling_errors"]["joint_error_deg"], .261)
            self.assertAlmostEqual(result["settling_errors"]["tcp_error_mm"], .769024866138101)
            self.assertGreaterEqual(result["stable_for_s"], .3)
            self.assertFalse(result["physical_stop_confirmed"])
            np.testing.assert_allclose(self.output.last_target, TARGET)
        self.assertEqual(self.output.frames_sent, 0)
        self.assertIsNone(self.receiver.recv(timeout=.01))

    def test_legacy_stream_still_requires_precise_arrival(self):
        self.output.config["joint_command_mode"] = "move_j"
        result = self.output.observe_settled(timeout=.5)
        self.assertFalse(result["feedback_settled"])
        self.assertEqual(result["settling_limits"]["joint_error_deg"], .05)
        self.assertEqual(result["settling_blockers"], ["joint_error_deg"])
        self.measured = TARGET.copy()
        self.assertTrue(self.output.observe_settled()["feedback_settled"])

    def test_small_feedback_noise_can_complete_stability_window(self):
        original = self.snapshot

        def dither(*args, **kwargs):
            self.measured = MEASURED.copy()
            self.measured[6] += math.radians(.003 * math.sin(self.now * 100))
            return original(*args, **kwargs)

        with patch("nero_pico_teleop.nero_joint_output.robot_snapshot", side_effect=dither):
            self.assertTrue(self.output.observe_settled()["feedback_settled"])

    def test_moving_feedback_does_not_settle_even_with_small_target_error(self):
        original = self.snapshot

        def moving(*args, **kwargs):
            self.measured = MEASURED.copy()
            self.measured[6] += math.radians(.1 * math.sin(self.now * 40))
            return original(*args, **kwargs)

        with patch("nero_pico_teleop.nero_joint_output.robot_snapshot", side_effect=moving):
            result = self.output.observe_settled(timeout=.8)
        self.assertFalse(result["feedback_settled"])
        self.assertTrue(result["settling_timed_out"])
        self.assertEqual(result["settling_blockers"], [])
        self.assertLess(result["stable_for_s"], .3)

    def test_joint_tcp_and_orientation_residuals_are_independently_bounded(self):
        # These poses each exceed exactly one settling tolerance with real FK.
        cases = {
            "joint_error_deg": [.047, .462, .467, -.513, -.299, -.144, -.270],
            "tcp_error_mm": [.004, -.364, .145, .252, -.056, -.301, -.039],
            "angular_error_deg": [.007, -.299, -.187, .421, .202, .465, -.239],
        }
        for name, delta in cases.items():
            with self.subTest(criterion=name):
                self.measured = TARGET + np.deg2rad(delta)
                result = self.output.observe_settled(timeout=.5)
                self.assertFalse(result["feedback_settled"])
                self.assertEqual(result["settling_blockers"], [name])
                self.assertEqual(result["settling_reason"], "target_error")
                self.assertIsNone(self.output.lag_since)

    def test_regrip_requires_same_bounds_and_a_new_stability_window(self):
        self.output.hold_motion()
        with self.assertRaisesRegex(RuntimeError, "settled feedback"):
            self.output.begin_motion(self.now)
        self.advance(.31)
        self.output.tick()
        self.assertTrue(self.output.status()["regrip_feedback_ready"])
        self.assertTrue(self.output.status()["hold_feedback_observation"]["feedback_settled"])
        self.output.begin_motion(self.now)
        self.assertEqual(self.output.state, "ACTIVE")
        np.testing.assert_allclose(self.output.last_target, TARGET)
        self.output.hold_motion()
        self.output.tick()
        self.assertFalse(self.output.hold_settled)
        self.measured = TARGET + np.deg2rad([0, 0, 0, 0, 0, 0, .65])
        self.advance(.31)
        self.output.tick()
        self.assertFalse(self.output.hold_settled)
        self.assertEqual(self.output.hold_observation["settling_reason"], "target_error")

    def test_following_recovery_still_blocks_settling(self):
        self.output.tcp_following_recovery_since = self.now
        result = self.output.observe_settled(timeout=.5)
        self.assertFalse(result["feedback_settled"])
        self.assertEqual(result["settling_blockers"], ["following_recovery"])
        self.output.tcp_following_recovery_since = None
        self.assertTrue(self.output.observe_settled()["feedback_settled"])

    def test_feedback_failure_cannot_be_reported_as_settled(self):
        self.blockers = ["stale joint feedback"]
        with self.assertRaisesRegex(RuntimeError, "stale joint feedback"):
            self.output.observe_settled()
        self.assertEqual(self.output.state, "FAULT")

    def test_cancellation_does_not_wait_for_timeout(self):
        started = self.now
        result = self.output.observe_settled(stopping=lambda: True)
        self.assertFalse(result["feedback_settled"])
        self.assertEqual(result["settling_reason"], "observation_cancelled")
        self.assertEqual(started, self.now)

    def test_startup_timeout_reports_errors_and_preserves_exit_diagnostics(self):
        self.measured[6] = TARGET[6] + math.radians(.65)
        result, records = self.run_session(lambda *_args, **_kwargs: None)
        observation = next(r for r in records if r["event"] == "takeover_feedback_observation")
        self.assertFalse(observation["feedback_settled"])
        self.assertIn("joint_error_deg", observation["settling_blockers"])
        self.assertIn("current-position takeover did not settle: target_error", result["error"])
        self.assertIn("joint_error_deg", result["feedback_observation"]["settling_blockers"])
        self.assertEqual(result["grip_count"], 0)

    def test_startup_success_is_not_reused_when_exit_feedback_fails(self):
        def neutral(*_args, **kwargs):
            if kwargs.get("phase") == "after_takeover":
                self.blockers = ["stale joint feedback"]
                raise RuntimeError("input setup failed after takeover")

        result, records = self.run_session(neutral)
        observation = next(r for r in records if r["event"] == "takeover_feedback_observation")
        self.assertTrue(observation["feedback_settled"])
        self.assertFalse(result["feedback_observation"]["feedback_settled"])
        self.assertEqual(result["feedback_observation"]["settling_reason"], "exit_observation_failed")
        self.assertEqual(result["output_state"], "FAULT")

    def test_fault_after_exit_observation_invalidates_settled_status(self):
        def neutral(*_args, **kwargs):
            if kwargs.get("phase") == "after_takeover":
                raise RuntimeError("input setup failed after takeover")

        def fault_before_watchdog_stops():
            self.output.state = "FAULT"
            self.output.fault_reason = "feedback failed during exit"

        with patch.object(self.output, "stop_watchdog", side_effect=fault_before_watchdog_stops):
            result, _ = self.run_session(neutral)
        self.assertFalse(result["feedback_observation"]["feedback_settled"])
        self.assertEqual(result["feedback_observation"]["settling_reason"], "output_fault")
        self.assertEqual(result["error"], "feedback failed during exit")


class ReferenceStreamTests(VirtualStreamCase):
    feedback_gating = False

    def source(self):
        self.output.input_source = SimpleNamespace(lock=threading.RLock(), permits=lambda epoch: epoch == 1)
        self.output.motion_epoch = 1

    def boundary_feedback(self, sign=1):
        self.output.last_target = np.deg2rad([17.175, sign * 99.694, 50.569, -43.796, 54.212, 22.247, 65.028])
        self.measured = self.output.last_target.copy()
        self.measured[1] = math.radians(sign * 99.699)

    def test_recorded_boundary_feedback_remains_raw_and_can_move_inward(self):
        self.source()
        for sign in (-1, 1):
            self.boundary_feedback(sign)
            self.advance(.02)
            observed = self.output._feedback()
            np.testing.assert_array_equal(observed, self.measured)
            status = self.output.status()
            self.assertAlmostEqual(status["last_measured_joints_deg"][1], sign * 99.699)
            self.assertAlmostEqual(status["measured_joint_limit_excess_deg"][1], .004343647237,
                                   places=8)
            command = self.output.last_target.copy()
            command[1] -= math.radians(sign * .01)
            self.assertTrue(self.output.send_target(command, self.now))
            self.assertEqual(self.output.state, "ACTIVE")
            np.testing.assert_allclose(self.output.last_target, command)
        self.assertEqual(self.output.frames_sent, 8)

    def test_boundary_regrip_clamps_only_command_not_measured_feedback(self):
        self.boundary_feedback()
        self.output.hold_motion()
        self.source()
        self.output.begin_stream_motion({"epoch": 1, "event": {"received_monotonic": self.now}})
        self.assertAlmostEqual(math.degrees(self.output.last_target[1]), 99.694)
        self.assertAlmostEqual(self.output.status()["last_measured_joints_deg"][1], 99.699)
        for _ in range(2):
            frames = [self.receiver.recv(timeout=.1) for _ in range(4)]
            self.assertEqual(int.from_bytes(frames[0].data[4:8], "big", signed=True), 99694)

    def test_boundary_takeover_sends_only_in_limit_seed(self):
        self.boundary_feedback()
        def snapshot(*_args, **_kwargs):
            return {**self.snapshot(), "joints_enabled": [True] * 7, "joint_faults": [[]] * 7,
                    "status_timestamp_s": time.time(), "status": {
                        "ctrl_mode": 1, "mode_feedback": 1, "arm_status": 0,
                        "err_code": 0, "teach_status": 0, "motion_status": 1}}

        self.output.state, self.output.last_target = "PREPARED", None
        with patch("nero_pico_teleop.nero_pilot.robot_snapshot", side_effect=snapshot):
            self.output.takeover()
        self.assertEqual(self.output.state, "ACTIVE")
        self.assertAlmostEqual(math.degrees(self.output.seed[1]), 99.694)
        self.assertAlmostEqual(self.output.last_measured[1], 99.699)
        frames = [self.receiver.recv(timeout=.1) for _ in range(13)]
        j2 = [int.from_bytes(f.data[4:8], "big", signed=True) for f in frames if f.arbitration_id == 0x155]
        self.assertEqual(j2, [99694] * 3)

    def test_outside_model_target_is_still_rejected(self):
        self.boundary_feedback()
        self.source()
        self.advance(.02)
        with self.assertRaisesRegex(ValueError, "exceeds official Nero model limits"):
            self.output.send_target(self.measured.copy(), self.now)
        self.assertEqual(self.output.frames_sent, 0)

    def test_all_feedback_boundaries_project_commands_inward(self):
        model = self.output.ik
        for index in range(7):
            for sign, boundary in ((-1, model.model.lowerPositionLimit), (1, model.model.upperPositionLimit)):
                measured = TARGET.copy()
                measured[index] = boundary[index] + sign * math.radians(.01)
                original = measured.copy()
                command = model.command_from_feedback(measured)
                model.validate_joints(command)
                np.testing.assert_array_equal(measured, original)
                expected = model.command_lower[index] if sign == -1 else model.command_upper[index]
                self.assertAlmostEqual(command[index], expected)

    def test_large_seed_correction_is_not_sent_automatically(self):
        self.measured[1] = self.output.ik.model.upperPositionLimit[1] + math.radians(2.)
        self.output.state, self.output.last_target = "PREPARED", None
        with patch.object(self.output, "_startup_feedback", return_value=({"joints_enabled": [True] * 7}, self.measured)):
            with self.assertRaisesRegex(ValueError, "feedback cannot seed.*joint step limit"):
                self.output.takeover()
        self.assertEqual(self.output.frames_sent, 0)

    def test_nonfinite_feedback_is_still_rejected(self):
        self.measured[1] = float("nan")
        with self.assertRaisesRegex(RuntimeError, "expected 7 finite numbers"):
            self.output._feedback()
        self.assertEqual(self.output.state, "FAULT")
        self.assertEqual(self.output.frames_sent, 0)

    def test_startup_and_exit_only_observe_feedback_without_waiting(self):
        def neutral(*_args, **kwargs):
            if kwargs.get("phase") == "after_takeover":
                raise RuntimeError("reached input setup")

        started = self.now
        result, records = self.run_session(neutral)
        self.assertFalse(self.output.feedback_gating_enabled)
        self.assertEqual(result["error"], "reached input setup")
        for observed in (next(r for r in records if r["event"] == "takeover_feedback_observation"),
                         result["feedback_observation"]):
            self.assertFalse(observed["settling_required"])
            self.assertFalse(observed["feedback_settled"])
            self.assertEqual(observed["settling_reason"], "not_required")
        self.assertEqual(self.now, started)

    def test_takeover_accepts_fresh_mode_ack_without_arrival_or_stable_wait(self):
        def snapshot(*args, **kwargs):
            return {**self.snapshot(), "joints_enabled": [True] * 7, "joint_faults": [[]] * 7,
                    "status_timestamp_s": time.time(), "status": {
                        "ctrl_mode": 1, "mode_feedback": 1, "arm_status": 0,
                        "err_code": 0, "teach_status": 0, "motion_status": 1}}

        self.output.state, self.output.last_target = "PREPARED", None
        started = self.now
        with patch("nero_pico_teleop.nero_pilot.robot_snapshot", side_effect=snapshot):
            self.output._startup_feedback(TARGET, require_arrived=False)
            self.output.takeover()
        self.assertEqual(self.output.state, "ACTIVE")
        self.assertLess(self.now - started, .1)
        self.assertEqual(self.output.frames_sent, 13)
        np.testing.assert_allclose(self.output.last_target, MEASURED)

    def test_continuous_targets_send_despite_large_measured_lag(self):
        self.source()
        for index in range(1, 121):
            target = TARGET.copy()
            target[6] += math.radians(index * .1)
            self.advance(.02)
            self.assertTrue(self.output.send_target(target, self.now))
            np.testing.assert_allclose(self.output.last_target, target, atol=1e-12)
        self.output._feedback()
        status = self.output.status()
        self.assertGreater(status["joint_following_error_deg"], 3.)
        self.assertGreater(status["tcp_following_error_mm"], 15.)
        self.assertGreater(status["angular_following_error_deg"], 8.)
        self.assertFalse(status["waiting_for_feedback"])
        self.assertFalse(status["measured_lag_wait"])
        self.assertEqual(status["lead_step_fraction"], 1.)
        self.assertEqual(status["lead_limited_targets"], 0)
        self.assertEqual(self.output.frames_sent, 480)
        self.assertFalse(any(e["event"] in ("following_wait", "stream_lag_wait", "output_fault")
                             for e in self.output.events))
        for _ in range(120):
            self.assertEqual([self.receiver.recv(timeout=.1).arbitration_id for _ in range(4)],
                             [0x155, 0x156, 0x157, 0x170])

    def test_regrip_immediately_reanchors_to_measured_position(self):
        self.output.last_target = TARGET.copy()
        self.output.last_target[6] += math.radians(12.)
        self.output.hold_motion()
        self.output._update_hold_observation()
        self.assertFalse(self.output.hold_settled)
        self.assertTrue(self.output.regrip_ready)
        self.assertTrue(self.output.status()["regrip_feedback_ready"])
        self.source()
        started = self.now
        self.output.begin_stream_motion({"epoch": 1, "event": {"received_monotonic": self.now}})
        self.assertEqual(self.now, started)
        np.testing.assert_allclose(self.output.last_target, MEASURED)
        self.assertEqual(self.output.state, "ACTIVE")
        self.assertEqual(self.output.frames_sent, 8)
        frames = [self.receiver.recv(timeout=.1) for _ in range(8)][4:]
        values = [int.from_bytes(frame.data[i:i + 4], "big", signed=True)
                  for frame in frames for i in ((0,) if frame.arbitration_id == 0x170 else (0, 4))]
        np.testing.assert_array_equal(values, np.rint(np.rad2deg(MEASURED) * 1000).astype(int))

    def test_feedback_driver_fault_still_blocks_ungated_stream(self):
        self.blockers = ["joint_driver_fault"]
        with self.assertRaisesRegex(RuntimeError, "joint_driver_fault"):
            self.output.observe_settled()
        self.assertEqual(self.output.state, "FAULT")
        self.assertEqual(self.output.frames_sent, 0)

    def test_stale_input_still_rejects_regrip(self):
        self.output.hold_motion()
        self.source()
        before = self.output.frames_sent
        with self.assertRaisesRegex(RuntimeError, "input is stale"):
            self.output.begin_stream_motion({"epoch": 1, "event": {"received_monotonic": self.now - 1.}})
        self.assertEqual(self.output.frames_sent, before)

    def test_excessive_command_step_still_fails(self):
        self.source()
        self.advance(.1)
        target = TARGET.copy()
        target[6] += math.radians(.9)
        with self.assertRaisesRegex(RuntimeError, "joint command step/speed limit"):
            self.output.send_target(target, self.now)
        self.assertEqual(self.output.frames_sent, 0)

    def test_legacy_profile_retains_feedback_gating(self):
        _, config = stream_configs(load_config(PROJECT_ROOT / "config/nero_relative_config.json"))
        self.assertTrue(config["feedback_gating_enabled"])

    def test_success_exit_does_not_require_a_settled_claim(self):
        config = load_config(PROJECT_ROOT / "config/nero_humanoid_config.json")
        summary = {"error": None, "grip_count": 1, "output_state": "FROZEN",
                   "feedback_observation": {"feedback_settled": False, "settling_required": False}}
        with tempfile.TemporaryDirectory() as directory, \
                patch("nero_pico_teleop.nero_stream.StreamConnection"), \
                patch("nero_pico_teleop.nero_stream.PicoEvents"), \
                patch("nero_pico_teleop.nero_stream.run_stream_session", return_value=summary), patch("builtins.print"):
            result = run_physical_stream(config, Path(directory) / "run.jsonl", 1., lambda: False,
                                         clearance_confirmed=True, session_limits=False)
        self.assertEqual(result, 0)


if __name__ == "__main__":
    unittest.main()
