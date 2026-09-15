"""PICO relative pose streaming through Nero V120 joint-position APIs."""

from collections import deque
from contextlib import ExitStack
import copy
import json
import math
import queue
from pathlib import Path
import threading
import time

import numpy as np
import pinocchio as pin

from .nero_ik_process import ProcessNeroIK
from .nero_joint_output import VirtualNeroJointOutput
from .nero_pilot import NeroPilotOutput, PilotConnection, wait_neutral
from .nero_preview_io import PicoEvents
from .nero_relative_core import NeroKinematics, RelativeTeleop
from .pico_input_guard import finite_number, valid_pose
from .paths import pico_sdk_path


def stream_configs(config, *, scale=1., radius_mm=50., tcp_speed_mm_s=50.,
                   angular_speed_deg_s=30., speed_percent=20, translation_only=False,
                   gripper=False, session_limits=True, joint_speed_deg_s=20.):
    if type(session_limits) is not bool:
        raise ValueError("session_limits must be a boolean")
    reference_profile = config.get("runtime_profile") == "moonbot_xnero"
    limits = [("scale", scale, .05, 2.),
              ("tcp_speed_mm_s", tcp_speed_mm_s, 1., 400. if reference_profile else 200.),
              ("joint_speed_deg_s", joint_speed_deg_s, 1., 120. if reference_profile else 60.),
              ("angular_speed_deg_s", angular_speed_deg_s, 1., 60. if reference_profile else 30.)]
    if session_limits or radius_mm is not None:
        limits.append(("radius_mm", radius_mm, 2., 150.))
    for name, value, lower, upper in limits:
        if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"{name} must be between {lower:g} and {upper:g}")
    if type(speed_percent) is not int or not 1 <= speed_percent <= 100:
        raise ValueError("speed_percent must be an integer from 1 to 100")
    gripper_speed = config.get("gripper_speed_m_s", .020)
    if not finite_number(gripper_speed) or not 0 < gripper_speed <= .1:
        raise ValueError("gripper_speed_m_s must be between 0 (exclusive) and 0.1")
    targets = copy.deepcopy(config)
    if translation_only:
        targets.pop("max_rotation_session_deg", None)
        targets.pop("max_angular_speed_deg_s", None)
    targets.update(pilot_profile="stream", translation_scale=scale,
                   feedback_gating_enabled=not reference_profile,
                   session_limits_enabled=session_limits,
                   max_displacement_m=radius_mm / 1000 if session_limits else None,
                   max_tcp_speed_m_s=tcp_speed_mm_s / 1000,
                   max_joint_speed_deg_s=joint_speed_deg_s, max_joint_step_deg=1.,
                   max_joint_session_deg=30. if session_limits else None, position_tolerance_m=.0005,
                   orientation_tolerance_deg=.3, control_hz=60., max_step_interval_s=.05,
                   latest_sample_control=True, max_target_age_s=.05, ik_solver="differential",
                   ik_priority="translation", translation_priority_slack_m=.00005,
                   max_gap_s=.2, ik_execution="process", rotation_scale=1.,
                   orientation_mode="hold_initial_link7" if translation_only else "relative_link7",
                   gripper_mode="joystick_velocity" if gripper else "hold",
                   firmware_speed_percent=speed_percent)
    if not translation_only:
        targets.update(max_rotation_session_deg=30., max_angular_speed_deg_s=angular_speed_deg_s)
    output = copy.deepcopy(targets)
    output.update(max_displacement_m=(radius_mm + .5) / 1000 if session_limits else None,
                  max_tcp_speed_m_s=(tcp_speed_mm_s + 1.) / 1000,
                  max_joint_speed_deg_s=joint_speed_deg_s + 1., max_joint_step_deg=1.001,
                  max_joint_session_deg=30.001 if session_limits else None, orientation_tolerance_deg=.4,
                  feedback_coherence_wait_s=.02, max_output_gap_s=2.,
                  joint_feedback_fault_deg=3., tcp_feedback_fault_m=.015,
                  angular_feedback_fault_deg=8., max_joint_command_lead_deg=1.,
                  max_tcp_command_lead_m=.005, max_angular_command_lead_deg=3.,
                  following_wait_timeout_s=1.5, gripper_max_width_m=.100,
                  gripper_speed_m_s=gripper_speed, gripper_force_n=1., gripper_axis_deadzone=.12)
    if not translation_only:
        output.update(max_rotation_session_deg=30.1, max_angular_speed_deg_s=angular_speed_deg_s + 1.)
    if reference_profile:
        for settings in (targets, output):
            settings.update(ik_solver="placo", ik_priority="pose", control_hz=75.,
                            joint_command_mode="move_js", recover_input=True)
        # Allow a 60 Hz send cadence without reducing the requested joint speed.
        targets["max_joint_step_deg"] = max(.8, joint_speed_deg_s / 60.)
        output["max_joint_step_deg"] = targets["max_joint_step_deg"] + .001
        if not translation_only and not session_limits:
            targets["max_rotation_session_deg"] = 180.
            output["max_rotation_session_deg"] = 180.
    return targets, output


class TranslationSpeed:
    """Diagnostic speed from a >=50 ms position difference, never a control input."""

    def __init__(self, max_gap_s):
        self.max_gap_s = max_gap_s
        self.reference = self.last_stamp = self.speed = None

    def update(self, position, stamp):
        if self.last_stamp is None or not 0 <= stamp - self.last_stamp <= self.max_gap_s:
            self.reference = None
            self.speed = None
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        position = np.asarray(position).copy()
        if self.reference is not None:
            previous, previous_stamp = self.reference
            dt = stamp - previous_stamp
            if dt < .05:
                return
            self.speed = float(np.linalg.norm(position - previous) * 1000 / dt)
        self.reference = position, stamp

    def value(self, now):
        if self.last_stamp is None or not 0 <= now - self.last_stamp <= self.max_gap_s:
            return None
        return self.speed


class StreamInput:
    """Validate every input frame; retain only the latest pose and clutch epoch."""

    recoverable_reasons = frozenset((
        "tracking_stale", "tracking_gap_requires_release", "callback_too_old_or_future",
        "head_pose_invalid", "right_pose_invalid", "left_pose_invalid", "app_focus_invalid", "controller_mode_required",
        "position_jump_requires_release", "orientation_jump_requires_release", "disconnect", "overflow"))

    def __init__(self, pico, guard, *, recover_input=False, hand="right"):
        self.pico, self.guard = pico, guard
        self.recover_input = recover_input
        if hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        self.hand = hand
        self.recovery_reason = None
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.event = None
        self.epoch = 0
        self.home_generation = 0
        self.home_request_token = None
        self.home_interruption = None
        self.gripper_generation = 0
        self.gripper_neutral_required = True
        self.held = False
        self.terminal_reason = None
        self.frames = 0
        self.events = deque(maxlen=64)

    def __enter__(self):
        self.worker = threading.Thread(target=self._run, name="nero-stream-input", daemon=True)
        self.worker.start()
        return self

    def _tracking_step(self, event, now):
        tracking = event["tracking"]
        controllers = tracking.get("Controller") if isinstance(tracking, dict) else None
        controller = controllers.get(self.hand) if isinstance(controllers, dict) else None
        controller = controller if isinstance(controller, dict) else {}
        previous = self.guard.last_pose
        previous_source = self.guard.last_source
        previous_received = self.guard.last_received
        current = valid_pose(controller.get("pose"))
        position_jump_mm = orientation_jump_deg = None
        if previous is not None and current is not None:
            position_jump_mm = math.dist(current[:3], previous[:3]) * 1000
            a, b = np.asarray(previous[3:]), np.asarray(current[3:])
            dot = abs(float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b))))
            orientation_jump_deg = math.degrees(2 * math.acos(min(1., dot)))
        jumped = (position_jump_mm is not None and position_jump_mm > 50.
                  or orientation_jump_deg is not None and orientation_jump_deg > 45.)
        neutral = (all(finite_number(controller.get(name)) and 0 <= controller[name] <= limit
                       for name, limit in (("grip", .2), ("trigger", .1)))
                   and all(controller.get(name) is False
                           for name in ("primaryButton", "secondaryButton", "menuButton")))
        idle_reanchor = (jumped and neutral and not self.held and not self.guard.accepted
                         and not self.guard.paused and not self.guard.reference_requests
                         and self.guard.state in ("READY_IDLE", "WAIT_RELEASE"))
        if idle_reanchor:
            # No clutch is authorized. Start a new neutral interval while keeping
            # identity, focus, timestamp and freshness validation in InputGuard.
            self.guard.last_pose = None
            self.guard.needs_release = True
            self.guard.neutral_since = None
        home_requests = self.guard.home_requests
        state = self.guard.step(tracking, event["received_monotonic"], now, event["device_id"])
        if (not idle_reanchor and orientation_jump_deg is not None and orientation_jump_deg > 45.
                and state["state"] not in ("INVALID", "DISCONNECTED", "PAUSED")
                and not self.guard.reference_requests):
            state = self.guard.invalidate("orientation_jump_requires_release")
        home_neutral = self._home_buttons_released(controller)
        if (not home_neutral or idle_reanchor
                or state["state"] in ("INVALID", "DISCONNECTED", "PAUSED")):
            # Latch interruptions even if tracking recovers before the sender wakes.
            self.home_generation += 1
            reason = (state["reason"] if state["state"] in ("INVALID", "DISCONNECTED", "PAUSED")
                      else "pose_jump_while_released" if idle_reanchor else self._home_button_blocker(controller))
            self.home_interruption = {"reason": reason, **self._home_input_detail(event)}
        if self.guard.home_requests != home_requests:
            self.home_request_token = (self.home_token() if home_neutral
                                       and self.recovery_reason is None else None)
            if home_neutral:
                self.home_interruption = {"reason": "home_button_pressed", **self._home_input_detail(event)}
        if (idle_reanchor or state["state"] in ("INVALID", "DISCONNECTED", "PAUSED")
                or self.guard.reference_requests or self.guard.home_requests != home_requests):
            self.require_gripper_neutral()
        elif (state["state"] in ("READY_IDLE", "INPUT_HELD")
              and finite_number(controller.get("axisY")) and abs(controller["axisY"]) <= .12):
            self.gripper_neutral_required = False
        detail = {"previous_pose": previous, "current_pose": current,
                  "position_jump_mm": position_jump_mm, "orientation_jump_deg": orientation_jump_deg,
                  f"{self.hand}_grip": controller.get("grip"), f"{self.hand}_trigger": controller.get("trigger"),
                  "input_held_before": self.held, "input_received_monotonic": event["received_monotonic"],
                  "previous_received_monotonic": previous_received,
                  "source_timestamp_ns": tracking.get("timeStampNs") if isinstance(tracking, dict) else None,
                  "previous_source_timestamp_ns": previous_source,
                  "monotonic": now}
        if idle_reanchor and state["state"] in ("WAIT_RELEASE", "READY_IDLE"):
            self.events.append({"event": "idle_pose_reanchored", "reason": "pose_jump_while_released",
                                "neutral_required_ms": self.guard.neutral_s * 1000, **detail})
        return state, detail

    def _run(self):
        try:
            while not self.stop_event.is_set():
                event = self.pico.next()
                now = time.monotonic()
                with self.lock:
                    detail = {"input_event_kind": event["kind"], "monotonic": now}
                    if event["kind"] == "tracking":
                        state, detail = self._tracking_step(event, now)
                        self.frames += 1
                        self.event = event
                    elif event["kind"] == "tick":
                        state = self.guard.tick(now)
                    else:
                        state = self.guard.invalidate(event["kind"], disconnected=event["kind"] == "disconnect")
                    held = state["input_held"]
                    if held != self.held:
                        self.epoch += 1
                        self.held = held
                    if state["state"] in ("INVALID", "DISCONNECTED", "PAUSED") or self.guard.reference_requests:
                        self.terminal_reason = state["reason"] if not self.guard.reference_requests else "reference_button_pressed"
                        if (self.recover_input and self.terminal_reason in self.recoverable_reasons
                                and not self.guard.reference_requests):
                            self.suspend(self.terminal_reason)
                            self.terminal_reason = None
                        else:
                            self.events.append({"event": "stream_input_stopped", "reason": self.terminal_reason,
                                                "tracking_frames": self.frames, **detail})
                            return
                    elif self.recovery_reason is not None and state["state"] == "READY_IDLE":
                        self.events.append({"event": "stream_input_recovered", "previous_reason": self.recovery_reason,
                                            "resume_requires_new_grip": True, "tracking_frames": self.frames})
                        self.recovery_reason = None
        except Exception as exc:
            with self.lock:
                self.terminal_reason = f"input_worker_error: {exc}"
                self.held = False
                self.epoch += 1

    def suspend(self, reason):
        with self.lock:
            if self.recovery_reason is None:
                self.events.append({"event": "stream_input_suspended", "reason": reason,
                                    "resume_requires_release": True, "tracking_frames": self.frames})
            self.recovery_reason = reason
            self.home_generation += 1
            self.home_interruption = {"reason": reason, **self._home_input_detail()}
            self.require_gripper_neutral()
            self.guard.invalidate(reason, disconnected=reason == "disconnect")
            if self.held:
                self.epoch += 1
            self.held = False

    def permits(self, epoch):
        with self.lock:
            received = self.guard.last_valid_received
            return (not self.stop_event.is_set() and self.terminal_reason is None and self.held
                    and self.recovery_reason is None and epoch == self.epoch and received is not None
                    and 0 <= time.monotonic() - received <= self.guard.stale_s)

    def require_release(self):
        with self.lock:
            self.guard.needs_release = True
            self.guard.neutral_since = None
            self.guard.accepted = self.held = False
            self.epoch += 1
            self.home_generation += 1
            self.home_interruption = {"reason": "fresh_release_required", **self._home_input_detail()}
            self.require_gripper_neutral()

    def require_gripper_neutral(self):
        with self.lock:
            self.gripper_generation += 1
            self.gripper_neutral_required = True

    def permits_gripper(self, generation):
        with self.lock:
            received = self.guard.last_valid_received
            return (not self.stop_event.is_set() and self.terminal_reason is None
                    and self.recovery_reason is None and not self.gripper_neutral_required
                    and generation == self.gripper_generation
                    and self.guard.state in ("READY_IDLE", "INPUT_HELD")
                    and not self.guard.needs_release and not self.guard.paused
                    and not self.guard.reference_requests and received is not None
                    and 0 <= time.monotonic() - received <= self.guard.stale_s)

    def home_token(self):
        with self.lock:
            return self.home_generation, self.guard.home_requests

    @staticmethod
    def _home_button_blocker(controller):
        for key, limit in (("grip", .5), ("trigger", .1)):
            value = controller.get(key)
            if not finite_number(value) or value < 0:
                return f"{key}_invalid"
            if value > limit:
                return f"{key}_pressed"
        for key in ("primaryButton", "menuButton"):
            if controller.get(key) is not False:
                return f"{key}_pressed_or_invalid"
        return None

    @classmethod
    def _home_buttons_released(cls, controller):
        return cls._home_button_blocker(controller) is None

    def _home_input_detail(self, event=None):
        event = self.event if event is None else event
        tracking = event.get("tracking") if event else None
        controllers = tracking.get("Controller") if isinstance(tracking, dict) else None
        controller = controllers.get(self.hand) if isinstance(controllers, dict) else None
        controller = controller if isinstance(controller, dict) else {}
        received = self.guard.last_valid_received
        return {"monotonic": time.monotonic(), "controller_hand": self.hand,
                "input_state": self.guard.state, "input_reason": self.guard.reason,
                "last_valid_input_age_ms": None if received is None else (time.monotonic() - received) * 1000,
                "home_token": self.home_token(),
                "controls": {key: controller.get(key) for key in
                             ("grip", "trigger", "primaryButton", "secondaryButton", "menuButton")}}

    def _home_blocker(self, token):
        if self.stop_event.is_set():
            return "input_worker_stopped"
        if self.terminal_reason or self.recovery_reason:
            return self.terminal_reason or self.recovery_reason
        if token != self.home_token():
            return self.home_interruption["reason"] if self.home_interruption else "home_authorization_changed"
        received = self.guard.last_valid_received
        if received is None:
            return "waiting_for_tracking"
        if not 0 <= time.monotonic() - received <= self.guard.stale_s:
            return "tracking_stale"
        if (self.guard.state not in ("WAIT_RELEASE", "READY_IDLE")
                or self.guard.paused or self.guard.reference_requests):
            return self.guard.reason
        if self.event is None:
            return "waiting_for_tracking"
        return self._home_button_blocker(self.event["tracking"]["Controller"].get(self.hand, {}))

    def permits_home(self, token):
        with self.lock:
            return self._home_blocker(token) is None

    def home_diagnostic(self, token):
        with self.lock:
            return {"reason": self._home_blocker(token), **self._home_input_detail(),
                    "authorized_home_token": token, "last_interruption": self.home_interruption}

    def snapshot(self):
        with self.lock:
            return {"event": self.event, "guard": copy.copy(self.guard), "epoch": self.epoch,
                    "held": self.held, "terminal_reason": self.terminal_reason, "frames": self.frames,
                    "home_request_token": self.home_request_token,
                    "gripper_generation": self.gripper_generation,
                    "recovery_reason": self.recovery_reason}

    def drain_events(self):
        with self.lock:
            records = list(self.events)
            self.events.clear()
            return records

    def __exit__(self, *_args):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=1.)
            if self.worker.is_alive():
                raise RuntimeError("stream input worker did not exit")


class InputInterrupted(RuntimeError):
    """A computed target lost its clutch authorization before sending."""


class TargetObsolete(RuntimeError):
    """Discard a delayed result while retaining the current clutch reference."""


class StreamTargets(RelativeTeleop):
    def __init__(self, *args, **kwargs):
        self.requested_tcp = self.unclipped_requested_tcp = None
        self.controller_displacement_mm = None
        super().__init__(*args, **kwargs)
        self.controller_speed = TranslationSpeed(self.config["max_gap_s"])

    def hold(self, reason):
        self.requested_tcp = self.unclipped_requested_tcp = None
        self.controller_displacement_mm = None
        self.controller_speed = TranslationSpeed(self.config["max_gap_s"])
        return super().hold(reason)

    def process_pose(self, event):
        hand = self.config["arm"].get("controller_hand", "right")
        pose = valid_pose(event["tracking"]["Controller"][hand]["pose"])
        self.controller_speed.update(pose[:3], event["tracking"]["timeStampNs"] / 1e9)
        if self.anchor is not None:
            self.controller_displacement_mm = ((np.asarray(pose[:3]) - self.anchor[:3]) * 1000).tolist()
        return super().process_pose(event)

    def limit_requested_target(self, requested):
        delta = np.asarray(requested) - self.origin
        distance = np.linalg.norm(delta)
        limit = self.config["max_displacement_m"]
        result = np.asarray(requested).copy()
        if limit is not None:
            result = self.origin + delta * min(1., limit / distance) if distance else self.origin.copy()
            while np.linalg.norm(result - self.origin) > limit:
                result = np.nextafter(result, self.origin)
        self.unclipped_requested_tcp = np.asarray(requested).copy()
        self.requested_tcp = result.copy()
        return result

    def translation_status(self, measured_joints_deg):
        speed = self.controller_speed.speed
        speeds = {"controller_speed_mm_s": speed,
                  "mapped_controller_speed_mm_s": (speed * self.config["translation_scale"]
                                                    if speed is not None else None)}
        if self.requested_tcp is None:
            return {**speeds, "controller_displacement_mm": self.controller_displacement_mm,
                    "requested_tcp_displacement_mm": None, "commanded_tcp_displacement_mm": None,
                    "requested_to_command_mm": None, "requested_to_measured_mm": None,
                    "translation_error_base_mm": None, "workspace_clipped": False,
                    "workspace_excess_mm": 0.}
        commanded = self.ik.pose(self.q).translation
        measured = (self.ik.pose(np.deg2rad(measured_joints_deg)).translation
                    if measured_joints_deg is not None else None)
        excess = float(np.linalg.norm(self.unclipped_requested_tcp - self.requested_tcp) * 1000)
        return {**speeds, "controller_displacement_mm": self.controller_displacement_mm,
                "requested_tcp_displacement_mm": ((self.requested_tcp - self.origin) * 1000).tolist(),
                "commanded_tcp_displacement_mm": ((commanded - self.origin) * 1000).tolist(),
                "requested_to_command_mm": float(np.linalg.norm(self.requested_tcp - commanded) * 1000),
                "requested_to_measured_mm": (float(np.linalg.norm(self.requested_tcp - measured) * 1000)
                                             if measured is not None else None),
                "translation_error_base_mm": ((self.requested_tcp - measured) * 1000).tolist() if measured is not None else None,
                "workspace_clipped": excess > 1e-6, "workspace_excess_mm": excess}

    def sync_command(self, joints):
        self.q = joints.copy()
        pose = self.ik.pose(self.q)
        self.target = pose.translation.copy()
        if self.config["orientation_mode"] == "relative_link7":
            self.rotation = pose.rotation.copy()


class NeroStreamOutput(NeroPilotOutput):
    def __init__(self, *args, **kwargs):
        self.input_source = None
        self.teleop_ready = False
        self.motion_epoch = None
        self.home_goal = self.home_authorization = None
        self.home_started = self.home_stable_since = self.home_last_input = None
        self.home_last_progress = None
        self.home_progress = None
        self.home_state = "idle"
        self.deadline = None
        self.pending_sampled_at = None
        self.deferred_reason = None
        self.obsolete_targets = 0
        self.input_to_send_ms = None
        self.lead_step_fraction = None
        self.lead_limited_targets = 0
        self.send_times = deque(maxlen=120)
        self.lag_since = None
        self.joint_error_deg = self.angular_error_deg = None
        self.gripper = None
        self.gripper_target = self.gripper_measured = self.gripper_last_sent = None
        self.gripper_feedback_timestamp_s = None
        self.gripper_picked_up = False
        self.gripper_pickup_error = None
        self.gripper_frames = 0
        self.gripper_last_input = None
        self.gripper_last_command = 0.
        self.gripper_sending = False
        super().__init__(*args, **kwargs)
        self.commanded_speed = TranslationSpeed(self.config["max_gap_s"])
        self.measured_speed = TranslationSpeed(self.config["max_gap_s"])
        if self.config["gripper_mode"] == "joystick_velocity":
            if self.config["arm"].get("tool") != "agx_gripper":
                raise ValueError("joystick control requires the configured AGX gripper")
            self.gripper = self.arm.init_effector(self.arm.OPTIONS.EFFECTOR.AGX_GRIPPER)

    def _duration_elapsed(self):
        return self.deadline is not None and self.clock() >= self.deadline

    @property
    def allowed_send_states(self):
        return NeroPilotOutput.allowed_send_states + (("HOLDING",) if self.gripper_sending else ())

    def _batch(self, target):
        if self.state == "ACTIVE" and getattr(self, "home_goal", None) is not None:
            with self.input_source.lock:
                if (self.cancel_requested() or self._duration_elapsed()
                        or not self.input_source.permits_home(self.home_authorization)):
                    raise InputInterrupted("home return authorization expired")
                super()._batch(target)
                self.commanded_speed.update(self.ik.pose(self.last_target).translation, self.last_sent)
                return
        if self.state == "ACTIVE" and self.motion_epoch is not None:
            # Hold the input lock only for the four CAN frames, not IK or pacing.
            with self.input_source.lock:
                if self._duration_elapsed():
                    raise InputInterrupted("duration elapsed before joint batch")
                if not self.input_source.permits(self.motion_epoch):
                    raise InputInterrupted("clutch changed before joint batch")
                self._check_target_age()
                super()._batch(target)
                self.commanded_speed.update(self.ik.pose(self.last_target).translation, self.last_sent)
                if self.pending_sampled_at is not None:
                    self.send_times.append(self.last_sent)
                    self.input_to_send_ms = (self.last_sent - self.pending_sampled_at) * 1000
                    if self.lead_step_fraction is not None and self.lead_step_fraction < 1.:
                        self.lead_limited_targets += 1
                return
        result = super()._batch(target)
        self.commanded_speed.update(self.ik.pose(self.last_target).translation, self.last_sent)
        return result

    def _feedback(self):
        measured = VirtualNeroJointOutput._feedback(self)
        self.last_measured = np.rad2deg(measured).tolist()
        pose = self.ik.pose(measured)
        self.measured_speed.update(pose.translation, min(self.last_feedback_snapshot["group_timestamps_s"]))
        if self.seed is not None:
            self.last_measured_tcp_delta = ((pose.translation - self.ik.pose(self.seed).translation) * 1000).tolist()
        if self.last_target is not None:
            command = self.ik.pose(self.last_target)
            self.joint_error_deg = float(np.max(np.abs(np.rad2deg(measured - self.last_target))))
            self.tcp_following_error_mm = float(np.linalg.norm(pose.translation - command.translation) * 1000)
            self.angular_error_deg = math.degrees(float(np.linalg.norm(pin.log3(command.rotation.T @ pose.rotation))))
            if self.feedback_gating_enabled:
                self._check_following_feedback()
        if (self.feedback_gating_enabled and self.following_wait_since is not None
                and self.clock() - self.following_wait_since >= self.config["following_wait_timeout_s"]):
            self._fault("stream target lead remained outside its budget for 1.5 seconds")
        if self.gripper is not None:
            try:
                self._gripper_feedback()
            except ValueError as exc:
                self._fault(exc)
        return measured

    def _check_following_feedback(self):
        if (self.tcp_following_error_mm > self.config["tcp_feedback_fault_m"] * 1000
                or self.angular_error_deg > self.config["angular_feedback_fault_deg"]):
            self._fault(f"stream following hard limit: TCP={self.tcp_following_error_mm:.3f} mm, "
                        f"orientation={self.angular_error_deg:.3f} degree")
        errors = (self.joint_error_deg, self.tcp_following_error_mm, self.angular_error_deg)
        limits = (self.config["max_joint_command_lead_deg"], self.config["max_tcp_command_lead_m"] * 1000,
                  self.config["max_angular_command_lead_deg"])
        now = self.clock()
        if self.lag_since is not None:
            if now - self.lag_since >= self.config["following_wait_timeout_s"]:
                self._fault("stream measured feedback did not recover within 1.5 seconds")
            if all(error <= limit * .5 for error, limit in zip(errors, limits)):
                self.events.append({"event": "stream_lag_recovered", "wait_ms": (now - self.lag_since) * 1000})
                self.lag_since = None
        elif any(error > limit for error, limit in zip(errors, limits)):
            self.lag_since = now
            self.events.append({"event": "stream_lag_wait", "joint_error_deg": errors[0],
                                "tcp_error_mm": errors[1], "angular_error_deg": errors[2]})

    def _target_lead(self, target, measured, actual):
        pose = self.ik.pose(target)
        return (float(np.max(np.abs(np.rad2deg(target - measured)))),
                float(np.linalg.norm(pose.translation - actual.translation) * 1000),
                math.degrees(float(np.linalg.norm(pin.log3(actual.rotation.T @ pose.rotation)))))

    def _lead_within_budget(self, lead):
        return (lead[0] <= self.config["max_joint_command_lead_deg"]
                and lead[1] <= self.config["max_tcp_command_lead_m"] * 1000
                and lead[2] <= self.config["max_angular_command_lead_deg"])

    def _prepare_target(self, target, measured):
        self.lead_step_fraction = 1.
        if not self.feedback_gating_enabled:
            return self.ik.quantize_command(target)
        if self.lag_since is not None:
            return target
        actual = self.ik.pose(measured)
        rounded = self.ik.quantize_command(target)
        if self._lead_within_budget(self._target_lead(rounded, measured, actual)):
            return rounded
        self.lead_step_fraction = 0.
        if not self._lead_within_budget(self._target_lead(self.last_target, measured, actual)):
            return rounded

        # Search only along the current increment; every trial includes CAN
        # quantization and nonlinear TCP/orientation lead. Final checks still run.
        lower, upper = 0., 1.
        bounded = self.last_target.copy()
        for _ in range(10):
            fraction = (lower + upper) / 2
            candidate = self.last_target + fraction * (rounded - self.last_target)
            candidate = self.ik.quantize_command(candidate)
            if self._lead_within_budget(self._target_lead(candidate, measured, actual)):
                lower, bounded = fraction, candidate
            else:
                upper = fraction
        if np.array_equal(bounded, self.last_target):
            return rounded
        self.lead_step_fraction = lower
        return bounded

    def _feedback_allows_target(self, target, measured):
        lead = self._target_lead(target, measured, self.ik.pose(measured))
        self.candidate_joint_lead_deg, self.candidate_tcp_lead_mm, _ = lead
        if not self.feedback_gating_enabled:
            return True
        blocked = self.lag_since is not None or not self._lead_within_budget(lead)
        if blocked:
            wait_reason = "stream_measured_lag" if self.lag_since is not None else "stream_lead_budget"
            self.deferred_reason = wait_reason
            if self.following_wait_since is None:
                self.following_wait_since = self.clock()
            if self.following_wait_reason != wait_reason:
                self.events.append({"event": "following_wait", "reason": wait_reason})
            self.following_wait_reason = wait_reason
            return False
        if self.following_wait_since is not None:
            self.events.append({"event": "following_resumed", "wait_ms": (self.clock() - self.following_wait_since) * 1000})
        self.following_wait_since = self.following_wait_reason = None
        return True

    def _check_target_age(self):
        if (self.pending_sampled_at is not None
                and self.clock() - self.pending_sampled_at > self.config["max_target_age_s"]):
            raise TargetObsolete("target_age_budget")

    def send_target(self, target, sampled_at):
        try:
            with self.lock:
                self.pending_sampled_at = sampled_at
                self.deferred_reason = None
                self._check_target_age()
            return self._send_current_target(target, sampled_at)
        except TargetObsolete:
            with self.lock:
                self.obsolete_targets += 1
                self.deferred_reason = "target_age_budget"
            return False
        finally:
            with self.lock:
                self.pending_sampled_at = None

    def _send_current_target(self, target, sampled_at):
        with self.lock:
            if self.state != "ACTIVE" or not self.input_source.permits(self.motion_epoch):
                raise InputInterrupted("clutch changed while solving")
            rounded = self.ik.quantize_command(target)
            pose, previous = self.ik.pose(rounded), self.ik.pose(self.last_target)
            interval = max(1. / self.config["control_hz"],
                           float(np.max(np.abs(rounded - self.last_target))) / math.radians(self.config["max_joint_speed_deg_s"]),
                           float(np.linalg.norm(pose.translation - previous.translation)) / self.config["max_tcp_speed_m_s"])
            if "max_angular_speed_deg_s" in self.config:
                interval = max(interval, float(np.linalg.norm(pin.log3(previous.rotation.T @ pose.rotation)))
                               / math.radians(self.config["max_angular_speed_deg_s"]))
            if interval >= self.config["max_gap_s"]:
                self._fault("stream target cannot meet speed limits before its input expires")
            due = self.last_sent + interval
        while self.clock() < due:
            if self.cancel_requested() or not self.input_source.permits(self.motion_epoch):
                raise InputInterrupted("clutch changed while pacing")
            self.tick()
            self._check_target_age()
            time.sleep(min(.005, max(0., due - self.clock())))
        with self.lock:
            if self.state != "ACTIVE" or not self.input_source.permits(self.motion_epoch):
                raise InputInterrupted("clutch changed before sending")
            self._check_target_age()
            return VirtualNeroJointOutput.send_target(self, rounded, sampled_at)

    def begin_stream_motion(self, sample):
        with self.lock:
            if getattr(self, "home_goal", None) is not None:
                raise InputInterrupted("home return is still active")
            if not self.input_source.permits(sample["epoch"]):
                raise InputInterrupted("clutch changed before anchoring")
            super().begin_motion(sample["event"]["received_monotonic"])
            self.motion_epoch = sample["epoch"]
            if not self.feedback_gating_enabled:
                # Re-anchor the new clutch at measured joints, including when
                # the previous command was still ahead at release.
                self._batch(self.ik.command_from_feedback(self._feedback()))
            self.gripper_picked_up = False
            self.gripper_pickup_error = None
            self.gripper_last_sent = self.clock()
            self.send_times.clear()
            self.input_to_send_ms = None
            self.lead_step_fraction = None
            self.deferred_reason = None

    def begin_home(self, reason, token=None):
        with self.lock:
            if self.state not in ("ACTIVE", "HOLDING"):
                raise InputInterrupted("output unavailable for home return")
            goal = self.ik.quantize_command(self.config["home_pose"]["joints_rad"])
            with self.input_source.lock:
                if reason == "startup":
                    token = self.input_source.home_token()
                if not self.input_source.permits_home(token):
                    return False
                self._feedback()
                self.home_goal, self.home_authorization = goal, token
                self.input_source.require_gripper_neutral()
                self.home_started = self.clock()
                self.home_stable_since = self.home_last_input = None
                self.home_last_progress = self.home_progress = None
                self.home_state = "returning"
                self.motion_epoch = None
                self.gripper_picked_up = False
                self.state = "ACTIVE"
                self.events.append({"event": "home_return_started", "reason": reason,
                                    "goal_joints_deg": np.rad2deg(goal).tolist(),
                                    "gripper_commanded": False})
                return True

    def _home_cancel_record(self, reason):
        diagnostic = self.input_source.home_diagnostic(self.home_authorization)
        if reason in ("input_changed", "input_changed_or_stale"):
            reason = diagnostic["reason"] or reason
        return {"event": "home_return_cancelled", "reason": reason,
                "input_diagnostic": diagnostic, "last_progress": self.home_progress,
                "physical_stop_confirmed": False}

    def _fault(self, reason):
        if self.home_goal is not None:
            self.events.append(self._home_cancel_record(str(reason)))
            self.home_goal = self.home_authorization = None
            self.home_state = "cancelled"
        super()._fault(reason)

    def cancel_home(self, reason):
        with self.lock:
            if self.home_goal is None:
                return
            record = self._home_cancel_record(reason)
            self.home_goal = self.home_authorization = None
            self.home_state = "cancelled"
            self.hold_motion()
            self.input_source.require_release()
            self.events.append(record)

    def step_home(self):
        from .nero_home import home_step
        with self.lock:
            if self.home_goal is None:
                return
            self.tick()
            if self.home_goal is None:
                return
            settings = self.config["home_pose"]
            if self.clock() - self.home_started >= settings["timeout_s"]:
                self.cancel_home("home_return_timeout")
                return
            sample = self.input_source.snapshot()
            received = sample["guard"].last_valid_received
            if received != self.home_last_input and self.clock() - self.last_sent >= 1. / self.config["control_hz"]:
                candidate = home_step(self.ik, self.last_target, self.home_goal,
                                      self.clock() - self.last_sent, self.config)
                try:
                    self._batch(candidate)
                except InputInterrupted:
                    self.cancel_home("input_changed")
                    return
                self.home_last_input = received
            measured = self._feedback()
            actual, goal = self.ik.pose(measured), self.ik.pose(self.home_goal)
            errors = {"joint_error_deg": float(np.max(np.abs(np.rad2deg(measured - self.home_goal)))),
                      "tcp_error_mm": float(np.linalg.norm(actual.translation - goal.translation) * 1000),
                      "angular_error_deg": math.degrees(float(np.linalg.norm(pin.log3(goal.rotation.T @ actual.rotation))))}
            arrived = (np.array_equal(self.last_target, self.home_goal)
                       and errors["joint_error_deg"] <= settings["arrival_joint_tolerance_deg"]
                       and errors["tcp_error_mm"] <= settings["arrival_tcp_tolerance_m"] * 1000
                       and errors["angular_error_deg"] <= settings["arrival_angular_tolerance_deg"])
            if not arrived:
                self.home_stable_since = None
            elif self.home_stable_since is None:
                self.home_stable_since = self.clock()
            elif self.clock() - self.home_stable_since >= settings["arrival_stable_s"]:
                self.home_goal = self.home_authorization = None
                self.home_state = "complete"
                self.hold_motion()
                self.seed = self.last_target.copy()
                self.input_source.require_release()
                self.events.append({"event": "home_return_complete", **errors,
                                    "requires_new_grip": True, "physical_stop_confirmed": False})
            if self.home_goal is not None:
                self.home_progress = {"monotonic": self.clock(), "elapsed_s": self.clock() - self.home_started,
                                      "phase": "waiting_for_arrival" if np.array_equal(self.last_target, self.home_goal)
                                               else "moving", **errors,
                                      "arrival_joint_tolerance_deg": settings["arrival_joint_tolerance_deg"],
                                      "arrival_tcp_tolerance_mm": settings["arrival_tcp_tolerance_m"] * 1000,
                                      "arrival_angular_tolerance_deg": settings["arrival_angular_tolerance_deg"]}
                if self.home_last_progress is None or self.clock() - self.home_last_progress >= 1.:
                    self.events.append({"event": "home_return_progress", **self.home_progress})
                    self.home_last_progress = self.clock()

    def freeze(self, reason="requested_stop"):
        with self.lock:
            if self.home_goal is not None:
                self.events.append(self._home_cancel_record(reason))
                self.home_goal = self.home_authorization = None
                self.home_state = "cancelled"
            super().freeze(reason)

    def _settling_allowed(self):
        return self.lag_since is None and super()._settling_allowed()

    def _settling_errors_and_limits(self, measured):
        if self.config.get("joint_command_mode") != "move_js":
            return super()._settling_errors_and_limits(measured)
        # JS can hold with a small static servo offset. Bound it in both joint
        # and task space; the shared observer still requires temporal stability.
        return {"joint_error_deg": (self.joint_error_deg, .5),
                "tcp_error_mm": (self.tcp_following_error_mm, 2.),
                "angular_error_deg": (self.angular_error_deg, .5)}

    def _check_input_watchdog(self):
        if self.input_source is None or self.home_goal is not None:
            return super()._check_input_watchdog()
        if self.state not in ("ACTIVE", "HOLDING"):
            return
        # Feedback reads can cross the input deadline or overlap a new frame.
        # Use the same latest-input policy before and after reading feedback.
        sample = self.input_source.snapshot()
        received = sample["guard"].last_valid_received
        if sample["terminal_reason"] is not None:
            self.freeze(sample["terminal_reason"])
            return
        if received is None or not 0 <= self.clock() - received <= self.config["max_gap_s"]:
            if self.config.get("recover_input", False):
                self.input_source.suspend(sample.get("recovery_reason") or "tracking_stale")
                if self.state == "ACTIVE":
                    self.hold_motion()
                self.input_seen = None
                self.gripper_picked_up = False
                return
            self.events.append({"event": "stream_input_timeout", "tracking_frames": sample["frames"],
                                "last_valid_input_age_ms": None if received is None else (self.clock() - received) * 1000,
                                "input_state": sample["guard"].state, "monotonic": self.clock()})
            self.freeze("input_watchdog_timeout")
            return
        self.input_seen = received
        if self.state == "ACTIVE" and self.motion_epoch is not None and not self.input_source.permits(self.motion_epoch):
            self.hold_motion()
            self.gripper_picked_up = False

    def tick(self):
        with self.lock:
            if self.state in ("ACTIVE", "HOLDING") and self._duration_elapsed():
                self.freeze("duration_elapsed")
                return
            if getattr(self, "home_goal", None) is not None:
                sample = self.input_source.snapshot()
                if self.cancel_requested() or sample["terminal_reason"]:
                    self.freeze(sample["terminal_reason"] or "operator_interrupt")
                    return
                if not self.input_source.permits_home(self.home_authorization):
                    self.cancel_home("input_changed_or_stale")
                    return
                self.input_seen = sample["guard"].last_valid_received
                super().tick()
                return
            if self.input_source is not None and self.state in ("ACTIVE", "HOLDING"):
                self._check_input_watchdog()
                if self.state not in ("ACTIVE", "HOLDING"):
                    return
            super().tick()

    def _gripper_feedback(self):
        feedback = self.gripper.get_gripper_status()
        if feedback is None or not 0 <= time.time() - feedback.timestamp <= self.config["max_gap_s"]:
            raise ValueError("fresh AGX gripper feedback is required for --gripper")
        message = copy.deepcopy(feedback.msg)
        flags = message.foc_status
        if (message.mode != "width" or not flags.driver_enable_status
                or any(getattr(flags, name) for name in ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                                                        "driver_overheating", "sensor_status", "driver_error_status"))):
            raise ValueError("AGX gripper must already be enabled in width mode without driver faults")
        if not math.isfinite(message.value) or not 0 <= message.value <= self.config["gripper_max_width_m"]:
            raise ValueError("AGX gripper width is outside the configured 0-100 mm range")
        self.gripper_measured = float(message.value)
        self.gripper_feedback_timestamp_s = feedback.timestamp
        if self.gripper_target is None:
            self.gripper_target = self.gripper_measured
        return self.gripper_measured

    @staticmethod
    def gripper_axis_command(axis_y, deadzone=.12):
        """Return a normalized velocity command: up closes, down opens."""
        if not finite_number(axis_y):
            return None
        value = float(np.clip(axis_y, -1., 1.))
        if abs(value) <= deadzone:
            return 0.
        elif value > 0:
            value = (value - deadzone) / (1. - deadzone)
        else:
            value = (value + deadzone) / (1. - deadzone)
        return -value

    def send_gripper(self, axis_y, sampled_at, generation):
        if self.gripper is None:
            return
        with self.lock:
            if (not self.teleop_ready or self.home_goal is not None
                    or self.state not in ("ACTIVE", "HOLDING")
                    or not self.input_source.permits_gripper(generation)):
                self.gripper_picked_up = False
                self.gripper_last_command = 0.
                return
            if not 0 <= self.clock() - sampled_at <= self.config["max_target_age_s"]:
                return
            if self.gripper_last_input is not None and sampled_at <= self.gripper_last_input:
                return
            try:
                width = self._gripper_feedback()
            except ValueError as exc:
                self._fault(exc)
            command = self.gripper_axis_command(axis_y, self.config["gripper_axis_deadzone"])
            previous_input, self.gripper_last_input = self.gripper_last_input, sampled_at
            if command is None:
                self.input_source.require_gripper_neutral()
                self.gripper_picked_up = False
                self.gripper_last_command = 0.
                return
            base_target = (width if not self.gripper_picked_up or command * self.gripper_last_command <= 0
                           else self.gripper_target)
            self.gripper_picked_up = True
            self.gripper_last_command = command
            if command == 0:
                return
            dt = min(self.config["max_step_interval_s"],
                     sampled_at - previous_input if previous_input is not None else 1. / self.config["control_hz"])
            step = max(0., dt) * self.config["gripper_speed_m_s"] * command
            target = round(float(np.clip(base_target + step, 0., self.config["gripper_max_width_m"])), 6)
            if abs(target - self.gripper_target) < 1e-6:
                return
            from pyAgxArm.protocols.can_protocol.msgs.effector.agx_gripper.default import ArmMsgGripperCtrl
            force = self.config["gripper_force_n"]
            message = ArmMsgGripperCtrl(value=round(target * 1e6), force=round(force * 1e3), status_code=1)
            frame = self.gripper._parser.pack(message)
            with self.input_source.lock:
                if (self._duration_elapsed() or self.cancel_requested()
                        or not self.input_source.permits_gripper(generation)):
                    self.gripper_picked_up = False
                    return
                if not 0 <= self.clock() - sampled_at <= self.config["max_target_age_s"]:
                    return
                self.expected = deque([(frame.arbitration_id, bytes(frame.data))])
                self.gripper_sending = True
                try:
                    self.gripper.move_gripper_m(value=target, force=force)
                    if self.expected:
                        self._fault("SDK omitted gripper command")
                except Exception as exc:
                    self._fault(f"gripper send failed: {exc}")
                finally:
                    self.expected.clear()
                    self.gripper_sending = False
            self.gripper_target, self.gripper_last_sent = target, self.clock()
            self.gripper_frames += 1

    def status(self):
        with self.lock:
            status = super().status()
            if self.last_target is not None:
                margins = np.rad2deg(np.minimum(self.last_target - self.ik.command_lower,
                                               self.ik.command_upper - self.last_target))
                closest = int(np.argmin(margins))
                status.update(nearest_joint_limit=closest + 1,
                              nearest_joint_limit_margin_deg=float(margins[closest]))
            instantaneous = self.config.get("joint_command_mode") == "move_js"
            status.update(control_api="move_js" if instantaneous else "move_j",
                          motion_mode="JS" if instantaneous else "J", mit_mode=instantaneous,
                          firmware_speed_applies=not instantaneous,
                          feedback_gating_enabled=self.feedback_gating_enabled,
                          session_limits_enabled=self.config["session_limits_enabled"],
                          command_hz=((len(self.send_times) - 1) / (self.send_times[-1] - self.send_times[0])
                                      if len(self.send_times) > 1 and self.state == "ACTIVE" else None),
                          input_to_send_ms=self.input_to_send_ms, obsolete_targets=self.obsolete_targets,
                          lead_step_fraction=self.lead_step_fraction,
                          lead_limited_targets=self.lead_limited_targets,
                          deferred_reason=self.deferred_reason,
                          firmware_speed_percent=self.config["firmware_speed_percent"],
                          commanded_tcp_speed_mm_s=self.commanded_speed.value(self.clock()),
                          measured_tcp_speed_mm_s=self.measured_speed.value(time.time()),
                          joint_following_error_deg=self.joint_error_deg,
                          angular_following_error_deg=self.angular_error_deg,
                          measured_lag_wait=self.lag_since is not None,
                          gripper_enabled=self.gripper is not None,
                          gripper_picked_up=self.gripper_picked_up,
                          gripper_target_mm=self.gripper_target * 1000 if self.gripper_target is not None else None,
                          gripper_measured_mm=self.gripper_measured * 1000 if self.gripper_measured is not None else None,
                          gripper_requires_grip=False,
                          gripper_frames_sent=self.gripper_frames)
            status.update(home_state=self.home_state,
                          home_goal_joints_deg=(np.rad2deg(self.home_goal).tolist()
                                                if self.home_goal is not None else None))
            if self.lag_since is not None and self.state == "ACTIVE":
                status.update(waiting_for_feedback=True, following_wait_reason="stream_measured_lag")
            status["real_motion_allowed"] = (status["real_motion_allowed"] and self.input_source is not None
                                             and (self.input_source.permits_home(self.home_authorization)
                                                  if self.home_goal is not None else self.input_source.permits(self.motion_epoch))
                                             and not self._duration_elapsed()
                                             and not status["waiting_for_feedback"])
            return status


class StreamConnection(PilotConnection):
    output_class = NeroStreamOutput


class PicoFanout:
    """Broadcast one XRoboToolkit event stream to independent arm sessions."""

    def __init__(self, pico, hands=("right", "left")):
        self.pico = pico
        self.queues = {hand: queue.Queue(maxsize=256) for hand in hands}
        self.stop_event = threading.Event()
        self.worker = None

    def __enter__(self):
        self.worker = threading.Thread(target=self._run, name="nero-pico-fanout", daemon=True)
        self.worker.start()
        return self

    def _run(self):
        while not self.stop_event.is_set():
            event = self.pico.next()
            for target in self.queues.values():
                try:
                    target.put_nowait(event)
                except queue.Full:
                    try:
                        target.get_nowait()
                        target.put_nowait(event)
                    except queue.Empty:
                        pass

    def channel(self, hand):
        target = self.queues[hand]
        class Channel:
            def next(self_nonlocal):
                try:
                    return target.get(timeout=.05)
                except queue.Empty:
                    return {"kind": "tick", "received_monotonic": time.monotonic(),
                            "processed_monotonic": time.monotonic()}
        return Channel()

    def __exit__(self, *_args):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=1.)


def run_physical_dual_stream(configs, output_path, duration, stopping, *, clearance_confirmed,
                             enable_joints=False, telemetry_socket=None, **options):
    """Run two independent arm sessions while consuming one PICO event stream."""
    if set(configs) != {"right_arm", "left_arm"}:
        raise ValueError("dual stream requires right_arm and left_arm configurations")
    targets = {}
    outputs = {}
    for arm_id, config in configs.items():
        targets[arm_id], output_config = stream_configs(config, **options)
        outputs[arm_id] = output_config
    output_path.parent.mkdir(parents=True, exist_ok=True)
    emit_lock = threading.RLock()
    with output_path.open("x", encoding="utf-8") as stream:
        def emit(record, arm_id=None):
            payload = {"logged_monotonic": time.monotonic(), **record}
            if arm_id is not None:
                payload["arm_id"] = arm_id
            with emit_lock:
                stream.write(json.dumps(payload) + "\n")
                stream.flush()
                if payload.get("event") != "can_tx":
                    print(json.dumps(payload), flush=True)
        emit({"event": "teleop_start", "arms": list(configs),
              "clearance_confirmed": clearance_confirmed})
        with ExitStack() as stack:
            pico = stack.enter_context(PicoEvents(pico_sdk_path()))
            fanout = stack.enter_context(PicoFanout(pico))
            for arm_id, output_config in outputs.items():
                outputs[arm_id] = stack.enter_context(StreamConnection(
                    output_config, NeroKinematics(output_config), clearance_confirmed=clearance_confirmed))
            summaries = {}
            threads = []
            if telemetry_socket is not None:
                from .telemetry import TelemetryPublisher
                stack.enter_context(TelemetryPublisher(outputs, telemetry_socket))
                emit({"event": "telemetry_enabled", "socket": str(telemetry_socket)})

            def worker(arm_id):
                try:
                    summaries[arm_id] = run_stream_session(
                        outputs[arm_id], fanout.channel(outputs[arm_id].config["arm"].get("controller_hand", arm_id.split("_")[0])),
                        targets[arm_id], duration, stopping, lambda record: emit(record, arm_id),
                        enable_joints=enable_joints)
                except Exception as exc:
                    summaries[arm_id] = {"event": "teleop_complete", "reason": "teleop_error", "error": str(exc),
                                         "output_state": outputs[arm_id].state, "grip_count": 0}
            for arm_id in configs:
                thread = threading.Thread(target=worker, args=(arm_id,), name=f"nero-{arm_id}", daemon=True)
                threads.append(thread)
                thread.start()
            for thread in threads:
                thread.join()
        result = {"event": "teleop_complete", "arms": summaries,
                  "error": next((v.get("error") for v in summaries.values() if v.get("error")), None),
                  "output_state": "FROZEN" if all(v.get("output_state") == "FROZEN" for v in summaries.values()) else "FAULT"}
        emit(result)
    return 0 if result["error"] is None and result["output_state"] == "FROZEN" else 4


def run_home_return(output, source, reason, stopping, emit, token=None):
    # The stream thread starts after wait_neutral; allow it to receive its first frame.
    first_frame_deadline = time.monotonic() + output.config["max_gap_s"]
    while source.snapshot()["event"] is None and not stopping() and time.monotonic() < first_frame_deadline:
        output.tick()
        time.sleep(.005)
    if stopping() or not output.begin_home(reason, token):
        emit({"event": "home_return_rejected", "reason": "release_grip_trigger_and_check_tracking"})
        return False
    while output.home_goal is not None:
        if stopping():
            output.freeze("operator_interrupt")
        else:
            output.step_home()
        for record in source.drain_events():
            emit(record)
        while output.events:
            emit(output.events.popleft())
        if output.home_goal is not None:
            time.sleep(.005)
    return output.home_state == "complete"


def run_stream_session(output, pico, targets_config, duration, stopping, emit, *, enable_joints=False):
    reason, error = "duration_elapsed", None
    grip_count = 0
    source = None
    settled = {"feedback_settled": False, "physical_stop_confirmed": False}
    contexts = ExitStack()
    try:
        emit({"event": "waiting_for_neutral", "can_frames_sent": 0})
        from .humanoid_frames import body_from_arm, requires_head
        require_head_pose = requires_head(targets_config)
        hand = targets_config["arm"].get("controller_hand", "right")
        wait_neutral(pico, stopping, timeout=30., emit=emit, require_head_pose=require_head_pose, hand=hand)
        solver_class = ProcessNeroIK
        if targets_config.get("ik_solver") == "placo":
            from .nero_placo_ik import ProcessPlacoIK
            solver_class = ProcessPlacoIK
        ik = contexts.enter_context(solver_class(targets_config, stopping=lambda: stopping() or output.state in ("FAULT", "FROZEN", "CLOSED")))
        emit({"event": "ik_worker_ready", "ik_execution": "process", "ik_solver": targets_config["ik_solver"],
              "solve_timeout_ms": ik.solve_timeout_s * 1000})
        if output.gripper is not None:
            output._gripper_feedback()
        output.takeover(enable_joints=enable_joints, stopping=stopping)
        while output.events:
            emit(output.events.popleft())
        output.start_watchdog()
        settled = output.observe_settled(stopping=stopping)
        emit({"event": "takeover_feedback_observation", **settled})
        if output.feedback_gating_enabled and not settled["feedback_settled"]:
            raise RuntimeError(f"current-position takeover did not settle: {settled['settling_reason']}; "
                               "see takeover_feedback_observation for errors and limits")
        guard = wait_neutral(pico, stopping, monitor=output.tick, emit=emit, phase="after_takeover",
                             require_head_pose=require_head_pose, hand=hand)
        home_settings = targets_config.get("home_pose")
        if home_settings:
            guard.secondary_action = "home"
        source = contexts.enter_context(StreamInput(pico, guard, recover_input=targets_config.get("recover_input", False), hand=hand))
        output.input_source = source
        deadline = time.monotonic() + duration
        output.deadline = deadline
        if home_settings and home_settings["on_startup"]:
            if not run_home_return(output, source, "startup", stopping, emit):
                raise RuntimeError("startup home return did not complete; teleop not started")
        targets = StreamTargets(targets_config, output.last_target, ik)
        handled_home_requests = source.snapshot()["guard"].home_requests
        output.teleop_ready = True
        emit({"event": "teleop_ready", "orientation_mode": targets_config["orientation_mode"],
              "home_pose_file": home_settings["file"] if home_settings else None,
              "secondary_button_action": "return_saved_home" if home_settings else "pause",
              "mapping_mode": targets_config.get("mapping_mode", "legacy_reference_mapping"),
              "mapping_status": targets_config["mapping_status"],
              "mapping_reference": ("headset_heading_and_robot_body_at_grip"
                  if targets_config.get("mapping_mode") == "head_relative_body" else
                  "headset_and_tcp_at_grip" if require_head_pose else "configured_tracking_mapping"),
              "head_to_tcp_axes": (targets_config.get("head_to_tcp_axes")
                                   if targets_config.get("mapping_mode") == "head_relative_tcp" else None),
              "body_from_arm_transform": (body_from_arm(targets_config).homogeneous.tolist()
                  if targets_config.get("mapping_mode") == "head_relative_body" else None),
              "ik_priority": (targets_config["ik_priority"]
                              if targets_config["orientation_mode"] == "relative_link7" else "pose"),
              "translation_priority_slack_mm": targets_config["translation_priority_slack_m"] * 1000,
              "ik_solver": targets_config["ik_solver"], "max_target_age_ms": targets_config["max_target_age_s"] * 1000,
              "max_step_interval_ms": targets_config["max_step_interval_s"] * 1000,
              "joint_speed_deg_s": targets_config["max_joint_speed_deg_s"],
              "joint_command_lower_deg": (ik.command_lower_mdeg / 1000).tolist(),
              "joint_command_upper_deg": (ik.command_upper_mdeg / 1000).tolist(),
              "control_hz": targets_config["control_hz"], "translation_scale": targets_config["translation_scale"],
              "translation_tracking": "scaled_position", "catch_up_while_grip_held": True,
              "tcp_radius_mm": (targets_config["max_displacement_m"] * 1000
                                if targets_config["max_displacement_m"] is not None else None),
              "joint_session_limit_deg": targets_config["max_joint_session_deg"],
              "tcp_speed_mm_s": targets_config["max_tcp_speed_m_s"] * 1000,
              "angular_speed_deg_s": targets_config.get("max_angular_speed_deg_s", 0.),
              "joint_lead_limit_deg": output.config["max_joint_command_lead_deg"] if output.feedback_gating_enabled else None,
              "tcp_lead_limit_mm": output.config["max_tcp_command_lead_m"] * 1000 if output.feedback_gating_enabled else None,
              "following_timeout_ms": output.config["following_wait_timeout_s"] * 1000 if output.feedback_gating_enabled else None,
              "reversal_requires_arrival": False, "require_post_command_feedback": False,
              "ik_execution": "process", **output.status()})
        next_cycle = time.monotonic()
        anchored_epoch = None
        last_sample_received = None
        regrip_announced = False
        while not stopping() and time.monotonic() < deadline:
            for record in source.drain_events():
                emit(record)
            output.tick()
            while output.events:
                emit(output.events.popleft())
            sample = source.snapshot()
            if output.state not in ("ACTIVE", "HOLDING"):
                reason = output.fault_reason or output.freeze_reason or output.state
                break
            if sample["terminal_reason"] is not None:
                reason = sample["terminal_reason"]
                break
            if home_settings and sample["guard"].home_requests != handled_home_requests:
                run_home_return(output, source, "secondary_button", stopping, emit,
                                sample["home_request_token"])
                handled_home_requests = source.snapshot()["guard"].home_requests
                targets = StreamTargets(targets_config, output.last_target, ik)
                anchored_epoch = last_sample_received = None
                regrip_announced = False
                continue
            cycle_remaining = next_cycle - time.monotonic()
            if cycle_remaining > 0:
                time.sleep(min(.005, cycle_remaining))
                continue
            next_cycle = time.monotonic() + 1. / targets_config["control_hz"]
            event = sample["event"]
            if event is None:
                continue
            targets.guard = sample["guard"]
            if output.gripper is not None:
                output.send_gripper(event["tracking"]["Controller"][hand].get("axisY"),
                                    event["received_monotonic"], sample["gripper_generation"])
            if not sample["held"]:
                targets.hold("grip_released")
                anchored_epoch = None
                if (output.regrip_ready
                        and sample["guard"].state == "READY_IDLE" and not regrip_announced):
                    emit({"event": "teleop_regrip_ready", **output.status()})
                    regrip_announced = True
                controller = event["tracking"]["Controller"][hand]
                emit({"event": "teleop_sample", "target_state": "HOLD", "reason": sample["guard"].reason,
                      "input_state": sample["guard"].state, f"{hand}_grip": controller["grip"],
                      f"{hand}_trigger": controller["trigger"], f"{hand}_axis_y": controller.get("axisY"),
                      **output.status()})
                continue
            if event["received_monotonic"] == last_sample_received:
                continue
            last_sample_received = event["received_monotonic"]
            try:
                if anchored_epoch != sample["epoch"]:
                    if output.state == "HOLDING" and not output.regrip_ready:
                        # A grip before settle must be released before it can authorize motion.
                        with source.lock:
                            source.guard.needs_release = True
                            source.guard.neutral_since = None
                            source.guard.accepted = source.held = False
                            source.epoch += 1
                        continue
                    output.begin_stream_motion(sample)
                    targets.hold("new_clutch")
                    targets.sync_command(output.last_target)
                    anchored_epoch = sample["epoch"]
                    regrip_announced = False
                    grip_count += 1
                    emit({"event": "teleop_grip", "grip_count": grip_count, **output.status()})
                event = {**event, "processed_monotonic": time.monotonic()}
                result = targets.process_pose(event)
                if result["state"] == "HOLD":
                    reason = result["reason"]
                    break
                if stopping() or time.monotonic() >= deadline:
                    reason = "operator_interrupt" if stopping() else "duration_elapsed"
                    break
                sent = None
                if result["state"] in ("FOLLOW", "LIMITED"):
                    sent = output.send_target(targets.q, event["received_monotonic"])
                    targets.sync_command(output.last_target)
                    if sent is False:
                        result = {**result, "candidate_state": result["state"], "candidate_reason": result["reason"],
                                  "state": "WAIT_SAMPLE" if output.deferred_reason == "target_age_budget" else "WAIT_FEEDBACK",
                                  "reason": output.deferred_reason or "stream_lead_budget"}
                    else:
                        if output.lead_step_fraction is not None and output.lead_step_fraction < 1.:
                            result = {**result, "candidate_state": result["state"], "candidate_reason": result["reason"],
                                      "state": "LIMITED", "reason": "stream_lead_limited"}
                output_status = output.status()
                emit({"event": "teleop_sample", "target_state": result["state"], "reason": result["reason"],
                      "ik_ms": (result.get("ik") or {}).get("solve_ms"),
                      "joint_limit_active": (result.get("ik") or {}).get("joint_limit_active", []),
                      "command_sent": sent,
                      "input_received_monotonic": event["received_monotonic"], "clutch_epoch": sample["epoch"],
                      f"{hand}_axis_y": event["tracking"]["Controller"][hand].get("axisY"),
                      "input": event, "target_calculation": result, **output_status,
                      **targets.translation_status(output_status["last_measured_joints_deg"])})
            except InputInterrupted:
                targets.sync_command(output.last_target)
                targets.hold("clutch_changed")
                anchored_epoch = None
            while output.events:
                emit(output.events.popleft())
        if stopping():
            reason = "operator_interrupt"
    except Exception as exc:
        if stopping() or output.state == "FROZEN":
            reason, error = output.freeze_reason or "operator_interrupt", None
        else:
            reason, error = "teleop_error", str(exc)
    finally:
        # A startup observation must never stand in for feedback after exit.
        settled = {"feedback_settled": False, "settling_reason": "exit_feedback_not_observed",
                   "physical_stop_confirmed": False}
        try:
            output.freeze(reason)
            if output.state == "FROZEN":
                settled = output.observe_settled()
        except Exception as exc:
            error = error or str(exc)
            settled = {"feedback_settled": False, "settling_reason": "exit_observation_failed",
                       "observation_error": str(exc), "physical_stop_confirmed": False}
        try:
            output.stop_watchdog()
        except Exception as exc:
            error = error or str(exc)
        try:
            contexts.close()
        except Exception as exc:
            error = error or str(exc)
        if source is not None:
            for record in source.drain_events():
                emit(record)
        if output.state == "FAULT":
            reason, error = "teleop_error", output.fault_reason or error
            if settled["feedback_settled"]:
                settled = {**settled, "feedback_settled": False, "settling_reason": "output_fault",
                           "observation_error": output.fault_reason}
        while output.events:
            emit(output.events.popleft())
        summary = {"event": "teleop_complete", "reason": reason, "error": error, "grip_count": grip_count,
                   "feedback_observation": settled, "motor_disable_sent": False,
                   "electronic_estop_sent": False, **output.status()}
        emit(summary)
    return summary


def run_physical_stream(config, output_path, duration, stopping, *, clearance_confirmed,
                        enable_joints=False, telemetry_socket=None, **options):
    targets_config, output_config = stream_configs(config, **options)
    with output_path.open("x", encoding="utf-8") as stream:
        last_print = 0.

        def emit(record):
            nonlocal last_print
            record = {"logged_monotonic": time.monotonic(), **record}
            stream.write(json.dumps(record) + "\n")
            if record["event"] == "can_tx":
                return
            if record["event"] == "teleop_sample":
                if time.monotonic() - last_print < .5:
                    return
                last_print = time.monotonic()
                record = {key: value for key, value in record.items() if key not in ("input", "target_calculation")}
            elif record["event"] == "teleop_start":
                record = {key: value for key, value in record.items() if key not in ("target_config", "output_config")}
            print(json.dumps(record), flush=True)
            stream.flush()

        emit({"event": "teleop_start", "output": str(output_path), "target_config": targets_config,
              "output_config": output_config, "clearance_confirmed": clearance_confirmed})
        with ExitStack() as stack:
            output = stack.enter_context(StreamConnection(output_config, NeroKinematics(output_config), clearance_confirmed=clearance_confirmed))
            pico = stack.enter_context(PicoEvents(pico_sdk_path()))
            if telemetry_socket is not None:
                from .telemetry import TelemetryPublisher
                stack.enter_context(TelemetryPublisher({config["arm_id"]: output}, telemetry_socket))
                emit({"event": "telemetry_enabled", "socket": str(telemetry_socket)})
            summary = run_stream_session(output, pico, targets_config, duration, stopping, emit, enable_joints=enable_joints)
        completed = (summary["error"] is None and summary["grip_count"] and summary["output_state"] == "FROZEN"
                     and (not output_config["feedback_gating_enabled"] or summary["feedback_observation"]["feedback_settled"]))
        return 0 if completed else 4
