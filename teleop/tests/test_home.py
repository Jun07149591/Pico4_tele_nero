import copy
from contextlib import ExitStack
import json
import math
from pathlib import Path
import socket
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

import can
import numpy as np
import pinocchio as pin

from nero_pico_teleop.cli import demo_events
from nero_pico_teleop.nero_home import home_step, load_home_pose
from nero_pico_teleop.nero_relative_core import NeroKinematics, load_config
from nero_pico_teleop.nero_stream import NeroStreamOutput, StreamInput, run_stream_session, stream_configs
from nero_pico_teleop.paths import PROJECT_ROOT, use_agx_sdk
from nero_pico_teleop.pico_input_guard import InputGuard


CONFIG = PROJECT_ROOT / "config/nero_humanoid_config.json"
HOME = PROJECT_ROOT / "config/home_poses.json"
CAPTURED = {
    "right_arm": [2.646, 83.261, -90.751, -16.052, -.77, -5.139, -72.295],
    "left_arm": [3.676, 87.142, -95.026, 28.062, 1.843, .364, 64.113],
}


def config_for(arm_id):
    return stream_configs(load_config(CONFIG, arm_id=arm_id), session_limits=False,
                          tcp_speed_mm_s=400., joint_speed_deg_s=120., angular_speed_deg_s=60.)


def feed(source, config, now, **buttons):
    event = copy.deepcopy(next(demo_events(config)))
    event["received_monotonic"] = event["processed_monotonic"] = now
    event["tracking"]["timeStampNs"] = round(now * 1e9)
    event["tracking"]["Controller"]["left"] = copy.deepcopy(event["tracking"]["Controller"]["right"])
    for controller in event["tracking"]["Controller"].values():
        controller.update(grip=0., trigger=0., primaryButton=False, secondaryButton=False, menuButton=False)
    event["tracking"]["Controller"][source.hand].update(buttons)
    with source.lock:
        state, _ = source._tracking_step(event, now)
        source.event = event
        source.frames += 1
        if source.held != state["input_held"]:
            source.epoch += 1
            source.held = state["input_held"]
        if source.recovery_reason and state["state"] == "READY_IDLE":
            source.recovery_reason = None
    return event


class HomePoseTests(unittest.TestCase):
    def test_fixed_live_capture_for_each_can_and_model(self):
        for arm_id, channel in (("right_arm", "can0"), ("left_arm", "can1")):
            config = load_config(CONFIG, arm_id=arm_id)
            np.testing.assert_allclose(np.rad2deg(config["home_pose"]["joints_rad"]), CAPTURED[arm_id])
            self.assertEqual(config["arm"]["can_channel"], channel)
            self.assertTrue(config["home_pose"]["on_startup"])
            self.assertEqual(config["home_pose"]["can_commands_sent"], 0)

    def test_wrong_arm_binding_model_and_invalid_settings_rejected(self):
        config = load_config(CONFIG)
        original = json.loads(HOME.read_text())
        changes = [lambda s: s["arms"]["right_arm"].update(can_channel="can1"),
                   lambda s: s["arms"]["right_arm"].update(controller_hand="left"),
                   lambda s: s["arms"]["right_arm"].update(tcp_position_m=[0., 0., 0.]),
                   lambda s: s["arms"]["right_arm"].update(joints_deg=[999.] * 7),
                   lambda s: s.update(timeout_s=0.), lambda s: s.update(on_startup="true"),
                   lambda s: s.update(joint_speed_deg_s=float("nan"))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "home.json"
            for change in changes:
                saved = copy.deepcopy(original)
                change(saved)
                path.write_text(json.dumps(saved))
                with self.assertRaises(ValueError):
                    load_home_pose(path, config)

    def test_quantized_steps_converge_with_joint_tcp_and_rotation_speed_bounds(self):
        for arm_id in CAPTURED:
            _, config = config_for(arm_id)
            home = config["home_pose"]
            model = NeroKinematics(config)
            goal = np.asarray(config["home_pose"]["joints_rad"])
            current = model.quantize_command(goal + np.deg2rad([12., -8., 7., -6., 3., 4., 5.]))
            for index in range(5000):
                dt = (.014, .021, .08)[index % 3]
                candidate = home_step(model, current, goal, dt, config)
                before, after = model.pose(current), model.pose(candidate)
                elapsed = min(dt, config["max_step_interval_s"])
                self.assertLessEqual(np.max(np.abs(np.rad2deg(candidate - current))),
                                     home["joint_speed_deg_s"] * elapsed + 1e-8)
                self.assertLessEqual(np.linalg.norm(after.translation - before.translation),
                                     home["tcp_speed_m_s"] * elapsed + 1e-10)
                self.assertLessEqual(np.linalg.norm(pin.log3(before.rotation.T @ after.rotation)),
                                     math.radians(home["angular_speed_deg_s"]) * elapsed + 1e-10)
                self.assertTrue(np.all(candidate >= model.model.lowerPositionLimit))
                self.assertTrue(np.all(candidate <= model.model.upperPositionLimit))
                current = candidate
                if np.array_equal(current, goal):
                    break
            else:
                self.fail(f"home steps did not converge: {arm_id}")
            np.testing.assert_array_equal(home_step(model, goal, goal, 0., config), goal)

    def test_gripper_speed_config_and_legacy_default(self):
        config = load_config(CONFIG)
        self.assertEqual(stream_configs(config)[1]["gripper_speed_m_s"], .06)
        config.pop("gripper_speed_m_s")
        self.assertEqual(stream_configs(config)[1]["gripper_speed_m_s"], .02)
        for invalid in (0., -.01, .101, float("nan"), float("inf"), True, "0.06"):
            with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, "gripper_speed_m_s"):
                stream_configs({**config, "gripper_speed_m_s": invalid})

    def test_legacy_secondary_button_still_pauses(self):
        config, _ = config_for("right_arm")
        source = StreamInput(None, InputGuard(neutral_s=0.))
        feed(source, config, 1.)
        feed(source, config, 1.02, secondaryButton=True)
        self.assertTrue(source.guard.paused)
        self.assertEqual(source.guard.home_requests, 0)

    def test_input_worker_routes_secondary_button_to_selected_hand(self):
        config, _ = config_for("right_arm")
        frames = []
        for index, pressed_hand in enumerate((None, "right", "right", None)):
            event = copy.deepcopy(next(demo_events(config)))
            controller = event["tracking"]["Controller"]["right"]
            controller.update(grip=0., trigger=0., primaryButton=False, secondaryButton=False, menuButton=False)
            event["tracking"]["Controller"]["left"] = copy.deepcopy(controller)
            if pressed_hand:
                event["tracking"]["Controller"][pressed_hand]["secondaryButton"] = True
            event["tracking"]["timeStampNs"] = 1_000_000_000 + index * 10_000_000
            frames.append(event)
        for hand, expected in (("right", 1), ("left", 0)):
            pending = iter(copy.deepcopy(frames) + [{"kind": "disconnect"}])

            def next_event():
                event = next(pending)
                event["received_monotonic"] = time.monotonic()
                return event

            source = StreamInput(SimpleNamespace(next=next_event),
                                 InputGuard(neutral_s=0., hand=hand, secondary_action="home"), hand=hand)
            source._run()
            self.assertEqual(source.guard.home_requests, expected)
            self.assertEqual(source.frames, len(frames))
            self.assertFalse(source.guard.paused)


class HomeOutputTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(socket, "socket", side_effect=AssertionError("physical socket opened")))
        self.now = 10.
        self.outputs, self.sources, self.receivers, self.feedback, self.targets = {}, {}, {}, {}, {}
        self.stack.enter_context(patch("nero_pico_teleop.nero_joint_output.robot_snapshot", side_effect=self.snapshot))
        self.stack.enter_context(patch("nero_pico_teleop.nero_stream.time", SimpleNamespace(
            monotonic=lambda: self.now, sleep=self.sleep, time=time.time)))
        use_agx_sdk()
        from pyAgxArm import AgxArmFactory, create_agx_arm_config
        for arm_id in CAPTURED:
            target_config, config = config_for(arm_id)
            self.targets[arm_id] = target_config
            channel = "home_" + uuid.uuid4().hex
            receiver = can.Bus(interface="virtual", channel=channel)
            self.stack.callback(receiver.shutdown)
            arm = AgxArmFactory.create_arm(create_agx_arm_config(
                robot="nero", firmeware_version="v120", interface="virtual", channel=channel, auto_connect=False))
            arm.connect()
            self.stack.callback(arm.disconnect)
            self.feedback[channel] = np.asarray(config["home_pose"]["joints_rad"]) + np.deg2rad([2., -3., 1., 2., 1., 1., -2.])
            output = self.stack.enter_context(NeroStreamOutput(
                arm, config, NeroKinematics(config), clearance_confirmed=True, clock=lambda: self.now))
            output.activate()
            source = StreamInput(None, InputGuard(stale_s=config["max_gap_s"], neutral_s=0.,
                                                 secondary_action="home", hand=config["arm"]["controller_hand"]),
                                 recover_input=True, hand=config["arm"]["controller_hand"])
            output.input_source = source
            self.outputs[arm_id], self.sources[arm_id], self.receivers[arm_id] = output, source, receiver
        self.push()

    def snapshot(self, arm, *_args, **_kwargs):
        channel = arm._config["comm"]["can"]["channel"]
        return {"joints_rad": self.feedback[channel].tolist(), "joint_output_blockers": [],
                "group_timestamps_s": [time.time()] * 4, "status": {"motion_status": 0}}

    def push(self, arm_id=None, **buttons):
        self.now += .02
        for name, source in self.sources.items():
            feed(source, self.targets[name], self.now, **(buttons if name == arm_id else {}))

    def follow(self, arm_id):
        output = self.outputs[arm_id]
        channel = output.arm._config["comm"]["can"]["channel"]
        self.feedback[channel] = output.last_target.copy()

    def sleep(self, seconds):
        self.now += seconds
        self.push()
        for arm_id in self.outputs:
            self.follow(arm_id)

    def request(self, arm_id="right_arm"):
        self.push(arm_id, secondaryButton=True)
        source, output = self.sources[arm_id], self.outputs[arm_id]
        self.assertTrue(output.begin_home("secondary_button", source.home_request_token))
        return output, source

    def finish_home(self, arm_id):
        output = self.outputs[arm_id]
        for _ in range(2000):
            self.push()
            self.follow(arm_id)
            output.step_home()
            if output.home_goal is None:
                break
        self.assertEqual(output.home_state, "complete")

    def test_each_button_returns_only_its_arm_to_same_fixed_goal_on_virtual_can(self):
        original = HOME.read_bytes()
        for arm_id in CAPTURED:
            other_id = "left_arm" if arm_id == "right_arm" else "right_arm"
            other = self.outputs[other_id]
            other_count = other.frames_sent
            other_target = other.last_target.copy()
            output, _ = self.request(arm_id)
            self.finish_home(arm_id)
            np.testing.assert_allclose(np.rad2deg(output.last_target), CAPTURED[arm_id])
            self.assertEqual(output.state, "HOLDING")
            self.assertEqual(other.frames_sent, other_count)
            np.testing.assert_array_equal(other.last_target, other_target)
            frames = []
            while (frame := self.receivers[arm_id].recv(timeout=.001)) is not None:
                frames.append(frame)
            self.assertGreater(len(frames), 4)
            self.assertEqual([f.arbitration_id for f in frames], [0x155, 0x156, 0x157, 0x170] * (len(frames) // 4))
            expected = [output.arm._parser.pack(msg) for msg in output.arm._deal_move_j_msgs(output.last_target.tolist())]
            self.assertEqual([bytes(f.data) for f in frames[-4:]], [bytes(f.data) for f in expected])
            self.assertEqual(output.gripper_frames, 0)
            # A new clutch/reference never overwrites the recorded destination.
            output.last_target = output.ik.quantize_command(output.last_target + np.deg2rad([1., 0., 0., 0., 0., 0., 0.]))
            output.seed = output.last_target.copy()
            self.follow(arm_id)
            self.push()
            self.request(arm_id)
            self.finish_home(arm_id)
            np.testing.assert_allclose(np.rad2deg(output.last_target), CAPTURED[arm_id])
        self.assertEqual(HOME.read_bytes(), original)

    def test_button_hold_is_one_request_and_initial_held_button_does_not_trigger(self):
        output, source = self.request()
        token = source.home_request_token
        for _ in range(5):
            self.push("right_arm", secondaryButton=True)
            self.assertEqual(source.guard.home_requests, 1)
            self.assertTrue(source.permits_home(token))
        fresh = StreamInput(None, InputGuard(neutral_s=0., secondary_action="home"))
        feed(fresh, self.targets["right_arm"], self.now, secondaryButton=True)
        self.assertEqual(fresh.guard.home_requests, 0)
        self.assertFalse(fresh.guard.paused)

    def test_grip_or_trigger_pulse_cancels_even_if_released_before_next_send(self):
        for button in ("grip", "trigger"):
            output, source = self.request()
            target = output.last_target.copy()
            self.push("right_arm", **{button: 1.})
            self.push()
            output.step_home()
            self.assertEqual(output.home_state, "cancelled")
            cancelled = [event for event in output.events if event["event"] == "home_return_cancelled"][-1]
            self.assertEqual(cancelled["reason"], f"{button}_pressed")
            diagnostic = cancelled["input_diagnostic"]
            self.assertEqual(diagnostic["controls"][button], 0.)
            self.assertEqual(diagnostic["last_interruption"]["controls"][button], 1.)
            self.assertTrue(source.guard.needs_release)
            np.testing.assert_array_equal(output.last_target, target)
            count = output.frames_sent
            self.push()
            output.step_home()
            self.assertEqual(output.frames_sent, count)

    def test_second_button_press_cancels_and_is_not_queued(self):
        output, source = self.request()
        self.push()
        self.push("right_arm", secondaryButton=True)
        second_token = source.home_request_token
        output.step_home()
        self.assertEqual(output.home_state, "cancelled")
        cancelled = [event for event in output.events if event["event"] == "home_return_cancelled"][-1]
        self.assertEqual(cancelled["reason"], "home_button_pressed")
        self.assertFalse(source.permits_home(second_token))
        self.assertIsNone(output.home_goal)

    def test_queued_button_does_not_survive_dropout_or_grip(self):
        for interruption in ("disconnect", "grip"):
            self.push("right_arm", secondaryButton=True)
            source, output = self.sources["right_arm"], self.outputs["right_arm"]
            token = source.home_request_token
            if interruption == "disconnect":
                source.suspend("disconnect")
            else:
                self.push("right_arm", grip=1.)
            self.push()
            self.assertFalse(output.begin_home("secondary_button", token))
            self.assertEqual(output.frames_sent, 0)

    def test_active_return_cannot_resume_after_recovered_dropout(self):
        output, source = self.request()
        source.suspend("tracking_stale")
        self.push()
        self.assertIsNone(source.recovery_reason)
        output.step_home()
        self.assertEqual(output.home_state, "cancelled")
        self.assertEqual(output.state, "HOLDING")
        cancelled = [event for event in output.events if event["event"] == "home_return_cancelled"][-1]
        self.assertEqual(cancelled["reason"], "tracking_stale")

    def test_stale_sample_does_not_advance_and_no_cached_sample_repeats(self):
        output, _ = self.request()
        output.step_home()
        target = output.last_target.copy()
        count = output.frames_sent
        self.now += .1
        output.step_home()
        self.assertEqual(output.frames_sent, count)
        self.now += .11
        output.step_home()
        self.assertEqual(output.home_state, "cancelled")
        cancelled = [event for event in output.events if event["event"] == "home_return_cancelled"][-1]
        self.assertEqual(cancelled["reason"], "tracking_stale")
        self.assertGreater(cancelled["input_diagnostic"]["last_valid_input_age_ms"], 200.)
        np.testing.assert_array_equal(output.last_target, target)

    def test_input_expiring_during_feedback_wait_recovers_without_terminating(self):
        for arm_id, state in (("right_arm", "ACTIVE"), ("left_arm", "HOLDING")):
            output, source = self.outputs[arm_id], self.sources[arm_id]
            self.push(arm_id, grip=1. if state == "ACTIVE" else 0., axisY=0.)
            output.state = state
            output.input_seen = source.guard.last_valid_received
            output.motion_epoch = source.epoch if state == "ACTIVE" else None
            count = output.frames_sent
            target = output.last_target.copy()
            self.now += .19
            original_feedback = output._feedback

            def delayed_feedback():
                self.now += .02
                return original_feedback()

            with patch.object(output, "_feedback", side_effect=delayed_feedback):
                output.tick()
            self.assertEqual(output.state, "HOLDING")
            self.assertIsNone(output.freeze_reason)
            self.assertIsNone(output.input_seen)
            self.assertEqual(source.recovery_reason, "tracking_stale")
            self.assertEqual(output.frames_sent, count + (4 if state == "ACTIVE" else 0))
            np.testing.assert_array_equal(output.last_target, target)
            self.assertFalse(source.held)
            self.push(arm_id, axisY=0.)
            self.push(arm_id, axisY=0.)
            output.tick()
            self.assertIsNone(source.recovery_reason)
            self.assertFalse(source.held)
            self.push(arm_id, grip=1., axisY=0.)
            output.begin_stream_motion(source.snapshot())
            self.assertEqual(output.state, "ACTIVE")

    def test_new_input_during_feedback_replaces_expired_watchdog_timestamp(self):
        output, source = self.outputs["right_arm"], self.sources["right_arm"]
        self.push("right_arm", grip=1.)
        output.input_seen = source.guard.last_valid_received
        output.motion_epoch = source.epoch
        self.now += .17
        original_feedback = output._feedback

        def feedback_with_new_input():
            self.push("right_arm", grip=1.)
            self.now += .02
            return original_feedback()

        with patch.object(output, "_feedback", side_effect=feedback_with_new_input):
            output.tick()
        self.assertEqual(output.state, "ACTIVE")
        self.assertIsNone(output.freeze_reason)
        self.assertIsNone(source.recovery_reason)
        self.assertEqual(output.input_seen, source.guard.last_valid_received)
        self.assertTrue(source.permits(output.motion_epoch))

    def test_nonrecovering_profile_still_freezes_when_input_expires_during_feedback(self):
        output, source = self.outputs["right_arm"], self.sources["right_arm"]
        output.config["recover_input"] = False
        output.input_seen = source.guard.last_valid_received
        self.now += .19
        original_feedback = output._feedback

        def delayed_feedback():
            self.now += .02
            return original_feedback()

        with patch.object(output, "_feedback", side_effect=delayed_feedback):
            output.tick()
        self.assertEqual(output.state, "FROZEN")
        self.assertEqual(output.freeze_reason, "input_watchdog_timeout")

    def test_grip_held_at_button_press_is_rejected_and_not_delayed_until_release(self):
        source, output = self.sources["right_arm"], self.outputs["right_arm"]
        self.push("right_arm", grip=1., secondaryButton=True)
        self.assertIsNone(source.home_request_token)
        self.push()
        self.assertFalse(output.begin_home("secondary_button", source.home_request_token))
        self.assertEqual(output.frames_sent, 0)

    def test_arrival_requires_measured_pose_then_new_grip(self):
        output, source = self.request()
        for _ in range(2000):
            self.push()
            output.step_home()
            if np.array_equal(output.last_target, output.home_goal):
                break
        self.assertIsNotNone(output.home_goal)
        self.assertEqual(output.home_state, "returning")
        self.assertEqual(output.home_progress["phase"], "waiting_for_arrival")
        self.assertGreater(output.home_progress["tcp_error_mm"], output.home_progress["arrival_tcp_tolerance_mm"])
        progress = [event for event in output.events if event["event"] == "home_return_progress"]
        self.assertTrue(progress)
        for previous, current in zip(progress, progress[1:]):
            self.assertGreaterEqual(current["monotonic"] - previous["monotonic"], 1.)
        self.finish_home("right_arm")
        self.assertTrue(source.guard.needs_release)
        self.assertFalse(source.held)
        self.push()
        self.push("right_arm", grip=1.)
        self.assertTrue(source.permits(source.epoch))
        output.begin_stream_motion(source.snapshot())
        self.assertEqual(output.state, "ACTIVE")

    def test_invalid_tracking_diagnostic_does_not_hide_guard_failure(self):
        output, source = self.request()
        event = copy.deepcopy(source.event)
        event["tracking"]["Controller"] = None
        with source.lock:
            state, _ = source._tracking_step(event, self.now)
            source.event = event
        self.assertEqual(state["state"], "INVALID")
        output.step_home()
        cancelled = [event for event in output.events if event["event"] == "home_return_cancelled"][-1]
        self.assertEqual(cancelled["reason"], state["reason"])
        self.assertEqual(cancelled["input_diagnostic"]["input_state"], "INVALID")
        json.dumps(cancelled, allow_nan=False)

    def attach_gripper(self, arm_id="right_arm"):
        from pyAgxArm.protocols.can_protocol.msgs.effector.agx_gripper.default import ArmMsgFeedbackGripper
        output = self.outputs[arm_id]
        output.gripper = output.arm.init_effector(output.arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
        message = ArmMsgFeedbackGripper(value=.03)
        message.status_code = 64
        self.stack.enter_context(patch.object(output.gripper, "get_gripper_status",
                                 side_effect=lambda: SimpleNamespace(timestamp=time.time(), msg=message)))
        output._gripper_feedback()
        output.teleop_ready = True
        output.home_state = "complete"
        output.state = "HOLDING"
        self.push(arm_id, axisY=0.)
        return output, self.sources[arm_id], message

    def test_joystick_alone_sends_only_gripper_frames_up_closes_down_opens(self):
        for arm_id in CAPTURED:
            output, source, feedback = self.attach_gripper(arm_id)
            joints = output.last_target.copy()
            for axis, expected_sign in ((1., -1), (-1., 1)):
                self.push(arm_id, axisY=axis)
                previous_input = output.gripper_last_input
                dt = min(output.config["max_step_interval_s"],
                         self.now - previous_input if previous_input is not None else 1. / output.config["control_hz"])
                output.send_gripper(axis, self.now, source.gripper_generation)
                frame = self.receivers[arm_id].recv(timeout=.1)
                self.assertIsNotNone(frame)
                self.assertEqual(frame.arbitration_id, 0x159)
                width = int.from_bytes(frame.data[:4], "big", signed=True) / 1e6
                self.assertGreater((width - feedback.value) * expected_sign, 0)
                self.assertAlmostEqual(abs(width - feedback.value), output.config["gripper_speed_m_s"] * dt, places=6)
                self.assertGreater(abs(width - feedback.value), .02 * dt)
                feedback.value = width
                self.assertFalse(source.held)
                self.assertEqual(output.state, "HOLDING")
            self.assertEqual(output.targets_sent, 0)
            np.testing.assert_array_equal(output.last_target, joints)
            count = output.gripper_frames
            output.send_gripper(-1., self.now, source.gripper_generation)
            self.assertEqual(output.gripper_frames, count)
            self.push(arm_id, axisY=0.)
            output.send_gripper(0., self.now, source.gripper_generation)
            self.assertEqual(output.gripper_frames, count)

    def test_gripper_dropout_home_and_stop_require_valid_new_joystick_input(self):
        output, source, _ = self.attach_gripper()
        self.push("right_arm", axisY=1.)
        token = source.gripper_generation
        source.suspend("tracking_stale")
        self.push("right_arm", axisY=1.)
        output.send_gripper(1., self.now, token)
        output.send_gripper(1., self.now, source.gripper_generation)
        self.assertEqual(output.gripper_frames, 0)
        self.push("right_arm", axisY=0.)
        self.push("right_arm", axisY=1.)
        output.send_gripper(1., self.now, source.gripper_generation)
        self.assertEqual(output.gripper_frames, 1)
        self.push("right_arm", axisY=0.)
        self.assertTrue(output.begin_home("startup"))
        self.push("right_arm", axisY=1.)
        output.send_gripper(1., self.now, source.gripper_generation)
        self.assertEqual(output.gripper_frames, 1)
        output.cancel_home("test_cancel")
        self.push("right_arm", axisY=1.)
        output.send_gripper(1., self.now, source.gripper_generation)
        self.assertEqual(output.gripper_frames, 1)
        self.push("right_arm", axisY=0.)
        self.push("right_arm", axisY=1.)
        self.now += .21
        output.send_gripper(1., self.now, source.gripper_generation)
        self.assertEqual(output.gripper_frames, 1)

    def test_session_services_joystick_with_grip_released(self):
        output, source, _ = self.attach_gripper()
        output.state = "ACTIVE"
        self.targets["right_arm"]["home_pose"]["on_startup"] = False
        output.ik.solve_timeout_s = .15
        stopped = [False]

        def emit(record):
            if record["event"] == "teleop_ready":
                output.hold_motion()
                self.push("right_arm", axisY=1.)
            elif record["event"] == "teleop_sample" and record["gripper_frames_sent"]:
                self.assertEqual(record["output_state"], "HOLDING")
                self.assertEqual(record["right_grip"], 0.)
                stopped[0] = True

        with patch.object(output, "takeover"), patch.object(output, "start_watchdog"), \
                patch("nero_pico_teleop.nero_placo_ik.ProcessPlacoIK") as solver, \
                patch("nero_pico_teleop.nero_stream.wait_neutral", return_value=source.guard), \
                patch("nero_pico_teleop.nero_stream.StreamInput") as input_class:
            solver.return_value.__enter__.return_value = output.ik
            input_class.return_value.__enter__.return_value = source
            result = run_stream_session(output, None, self.targets["right_arm"], 1., lambda: stopped[0], emit)
        self.assertIsNone(result["error"])
        self.assertTrue(stopped[0])
        self.assertEqual(result["grip_count"], 0)

    def test_timeout_and_feedback_fault_never_report_home_complete(self):
        output, _ = self.request()
        output.home_started = self.now - output.config["home_pose"]["timeout_s"]
        output.step_home()
        self.assertEqual(output.home_state, "cancelled")
        self.push()
        self.request()
        with patch("nero_pico_teleop.nero_joint_output.robot_snapshot", return_value={
                "joint_output_blockers": ["stale feedback"]}):
            with self.assertRaisesRegex(RuntimeError, "stale feedback"):
                output.step_home()
        self.assertIsNone(output.home_goal)
        self.assertEqual(output.state, "FAULT")

    def test_deadline_interrupts_without_advancing_target(self):
        output, _ = self.request()
        target = output.last_target.copy()
        output.deadline = self.now
        output.step_home()
        self.assertEqual(output.state, "FROZEN")
        self.assertEqual(output.freeze_reason, "duration_elapsed")
        np.testing.assert_array_equal(output.last_target, target)

    def test_real_session_reaches_saved_home_before_ready(self):
        output, source = self.outputs["right_arm"], self.sources["right_arm"]
        records, stopped = [], [False]
        model = output.ik
        model.solve_timeout_s = .15

        def emit(record):
            records.append(record)
            if record["event"] == "teleop_ready":
                self.assertEqual(output.home_state, "complete")
                np.testing.assert_allclose(np.rad2deg(output.last_target), CAPTURED["right_arm"])
                stopped[0] = True

        with patch.object(output, "takeover"), patch.object(output, "start_watchdog"), \
                patch("nero_pico_teleop.nero_placo_ik.ProcessPlacoIK") as solver, \
                patch("nero_pico_teleop.nero_stream.wait_neutral", return_value=source.guard), \
                patch("nero_pico_teleop.nero_stream.StreamInput") as input_class:
            solver.return_value.__enter__.return_value = model
            input_class.return_value.__enter__.return_value = source
            result = run_stream_session(output, None, self.targets["right_arm"], 60., lambda: stopped[0], emit)
        self.assertIsNone(result["error"])
        events = [r["event"] for r in records]
        self.assertLess(events.index("home_return_complete"), events.index("teleop_ready"))
        self.assertEqual(result["grip_count"], 0)
        self.assertEqual(self.outputs["left_arm"].frames_sent, 0)

    def test_startup_accepts_recorded_static_home_residual_without_changing_goal(self):
        output, source = self.outputs["right_arm"], self.sources["right_arm"]
        channel = output.arm._config["comm"]["can"]["channel"]
        measured = np.deg2rad([2.605, 83.248, -90.746, -15.762, -.567, -5.171, -72.027])
        self.feedback[channel] = measured.copy()
        output.last_target = output.ik.quantize_command(measured)
        output.seed = output.last_target.copy()
        output.ik.solve_timeout_s = .15
        records, stopped = [], [False]

        def sleep_with_static_feedback(seconds):
            self.now += seconds
            self.push()

        def emit(record):
            records.append(record)
            if record["event"] == "teleop_ready":
                self.assertEqual(output.home_state, "complete")
                stopped[0] = True

        with patch.object(output, "takeover"), patch.object(output, "start_watchdog"), \
                patch("nero_pico_teleop.nero_stream.time.sleep", side_effect=sleep_with_static_feedback), \
                patch("nero_pico_teleop.nero_placo_ik.ProcessPlacoIK") as solver, \
                patch("nero_pico_teleop.nero_stream.wait_neutral", return_value=source.guard), \
                patch("nero_pico_teleop.nero_stream.StreamInput") as input_class:
            solver.return_value.__enter__.return_value = output.ik
            input_class.return_value.__enter__.return_value = source
            result = run_stream_session(output, None, self.targets["right_arm"], 60., lambda: stopped[0], emit)
        self.assertIsNone(result["error"])
        completed = next(record for record in records if record["event"] == "home_return_complete")
        self.assertAlmostEqual(completed["tcp_error_mm"], 2.9191904127828234)
        self.assertAlmostEqual(completed["angular_error_deg"], .6236465807863303)
        self.assertGreaterEqual(self.now - output.home_stable_since, output.config["home_pose"]["arrival_stable_s"])
        self.assertTrue(stopped[0])
        np.testing.assert_array_equal(self.feedback[channel], measured)
        np.testing.assert_allclose(np.rad2deg(output.last_target), CAPTURED["right_arm"])
        self.assertEqual(self.outputs["left_arm"].frames_sent, 0)

    def test_session_dispatches_secondary_button_return_after_ready(self):
        output, source = self.outputs["right_arm"], self.sources["right_arm"]
        self.targets["right_arm"]["home_pose"]["on_startup"] = False
        records, stopped = [], [False]
        output.ik.solve_timeout_s = .15

        def emit(record):
            records.append(record)
            if record["event"] == "teleop_ready":
                self.push("right_arm", secondaryButton=True)
            elif record["event"] == "home_return_complete":
                stopped[0] = True

        with patch.object(output, "takeover"), patch.object(output, "start_watchdog"), \
                patch("nero_pico_teleop.nero_placo_ik.ProcessPlacoIK") as solver, \
                patch("nero_pico_teleop.nero_stream.wait_neutral", return_value=source.guard), \
                patch("nero_pico_teleop.nero_stream.StreamInput") as input_class:
            solver.return_value.__enter__.return_value = output.ik
            input_class.return_value.__enter__.return_value = source
            result = run_stream_session(output, None, self.targets["right_arm"], 60., lambda: stopped[0], emit)
        self.assertIsNone(result["error"])
        started = [r for r in records if r["event"] == "home_return_started"]
        self.assertEqual([r["reason"] for r in started], ["secondary_button"])
        np.testing.assert_allclose(np.rad2deg(output.last_target), CAPTURED["right_arm"])

    def test_cancelled_startup_never_announces_ready(self):
        output, source = self.outputs["right_arm"], self.sources["right_arm"]
        output.ik.solve_timeout_s = .15
        records = []
        with patch.object(output, "takeover"), patch.object(output, "start_watchdog"), \
                patch("nero_pico_teleop.nero_placo_ik.ProcessPlacoIK") as solver, \
                patch("nero_pico_teleop.nero_stream.wait_neutral", return_value=source.guard), \
                patch("nero_pico_teleop.nero_stream.StreamInput") as input_class, \
                patch("nero_pico_teleop.nero_stream.time.sleep", side_effect=lambda _: self.push("right_arm", grip=1.)):
            solver.return_value.__enter__.return_value = output.ik
            input_class.return_value.__enter__.return_value = source
            result = run_stream_session(output, None, self.targets["right_arm"], 60., lambda: False, records.append)
        self.assertIn("startup home return did not complete", result["error"])
        self.assertNotIn("teleop_ready", [r["event"] for r in records])
        self.assertEqual(output.home_state, "cancelled")


if __name__ == "__main__":
    unittest.main()
