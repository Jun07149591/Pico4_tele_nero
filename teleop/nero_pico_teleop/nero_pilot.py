"""Bounded first-motion commissioning. Only run physically after clearance confirmation."""

from collections import deque
from contextlib import ExitStack
import copy
import fcntl
import math
from pathlib import Path
import re
import sys
import threading
import time

import numpy as np

from .nero_joint_output import VirtualNeroJointOutput
from .nero_preview_io import INACTIVE_TEACH_STATUSES, robot_snapshot
from .nero_relative_core import NeroIK, NeroKinematics, RelativeTeleop, apply_mapping_candidate
from .pico_input_guard import InputGuard, valid_pose
from .paths import pico_sdk_path, use_agx_sdk


def pilot_configs(config, profile="initial"):
    if profile not in ("initial", "axis-check", "yz-check", "translation-check", "teleop"):
        raise ValueError("unknown pilot profile")
    targets = copy.deepcopy(config)
    targets.update(pilot_profile=profile, translation_scale=.1, max_displacement_m=.0018,
                   max_tcp_speed_m_s=.00075, max_joint_speed_deg_s=1.5,
                   max_joint_step_deg=.075, max_joint_session_deg=1.5,
                   position_tolerance_m=.0001, orientation_tolerance_deg=.1,
                   control_hz=20., max_gap_s=.2)
    if profile != "initial":
        targets["max_joint_session_deg"] = 5.
    if profile == "teleop":
        targets.update(max_displacement_m=.009, max_tcp_speed_m_s=.004,
                       max_joint_speed_deg_s=3., max_joint_step_deg=.15,
                       max_joint_session_deg=10., max_step_interval_s=.05,
                       ik_execution="process")
    output = copy.deepcopy(targets)
    # Leave room for CAN millidegree quantization; enforce these after rounding.
    output.update(max_displacement_m=.002, max_tcp_speed_m_s=.001,
                  max_joint_speed_deg_s=2., max_joint_step_deg=.1,
                  max_joint_session_deg=2., orientation_tolerance_deg=.15,
                  feedback_coherence_wait_s=.02)
    if profile != "initial":
        output["max_joint_session_deg"] = 5.001
    if profile == "teleop":
        output.update(max_displacement_m=.010, max_tcp_speed_m_s=.005,
                      max_joint_speed_deg_s=4., max_joint_step_deg=.2,
                      max_joint_session_deg=10.001, max_output_gap_s=1.2,
                      max_joint_command_lead_deg=.2, max_tcp_command_lead_m=.0004,
                      following_wait_timeout_s=1., reversal_requires_arrival=True,
                      require_post_command_feedback=True,
                      tcp_following_pause_m=.0005, tcp_following_resume_m=.0003,
                      tcp_following_fault_m=.001, tcp_following_recovery_timeout_s=.2)
    return targets, output


class PilotTargets(RelativeTeleop):
    def limit_requested_target(self, requested):
        result = self.origin.copy()
        limit = self.config["max_displacement_m"]
        if self.config.get("pilot_profile") in ("translation-check", "teleop"):
            delta = np.asarray(requested) - self.origin
            distance = np.linalg.norm(delta)
            if not np.isfinite(distance):
                raise ValueError("nonfinite translation request")
            result += delta * min(1., limit / distance) if distance else delta
            # Addition to base coordinates can round a spherical endpoint out.
            while np.linalg.norm(result - self.origin) > limit:
                result = np.nextafter(result, self.origin)
            return result
        if self.config.get("pilot_profile") == "yz-check":
            delta = np.asarray(requested) - self.origin
            delta[0] = 0.0
            distance = np.linalg.norm(delta)
            if not np.isfinite(distance):
                raise ValueError("nonfinite translation request")
            result += delta * min(1., limit / distance) if distance else delta
            while np.linalg.norm(result - self.origin) > limit:
                result = np.nextafter(result, self.origin)
            return result
        result[0] += np.clip(requested[0] - self.origin[0], 0., limit)
        # Adding a tiny displacement to the base coordinate can round outward.
        # Keep the representable endpoint inside the existing strict boundary.
        if result[0] - self.origin[0] > limit:
            result[0] = np.nextafter(result[0], self.origin[0])
        return result


class NeroPilotOutput(VirtualNeroJointOutput):
    """Explicit commissioning output, with a single owner and a latched watchdog.

    Freeze means no new target progression plus one repeat of the last target.
    It does not guarantee firmware queue cancellation or a physical emergency stop.
    """

    allowed_send_states = ("PREPARING", "ACTIVE", "FREEZING")
    require_connected = False

    def __init__(self, arm, config, ik, *, clearance_confirmed, clock=time.monotonic):
        if clearance_confirmed is not True:
            raise ValueError("physical clearance must be confirmed before opening pilot output")
        self.lock = threading.RLock()
        self.events = deque(maxlen=4096)
        self.stop_worker = threading.Event()
        self.worker = None
        self.input_seen = None
        self.freeze_reason = None
        self.attempted_frames = 0
        self._selected_channel = config["arm"]["can_channel"]
        self.cancel_requested = lambda: False
        self.last_measured = self.last_measured_tcp_delta = None
        self.hold_reference = self.hold_stable_since = None
        self.hold_settled = False
        self.hold_observation = None
        self.following_wait_since = None
        self.following_wait_reason = None
        self.tcp_following_recovery_since = None
        self.tcp_following_error_mm = None
        self.last_command_sent_wall = None
        self.last_command_direction = np.zeros(7)
        self.candidate_joint_lead_deg = self.candidate_tcp_lead_mm = None
        super().__init__(arm, config, ik, clock)

    def validate_transport(self, config):
        transport = config.get("comm", {}).get("can", {})
        if (transport.get("interface") not in ("socketcan", "virtual")
                or config.get("robot") != "nero" or config.get("firmeware_version") != "v120"):
            raise ValueError("pilot requires Nero V120 on SocketCAN or virtual CAN")
        if transport["interface"] == "socketcan" and transport.get("channel") != self.configured_channel:
            raise ValueError("pilot channel differs from selected arm")

    @property
    def configured_channel(self):
        return getattr(self, "_selected_channel", "can0")

    def _fault(self, reason):
        if self.state != "FAULT":
            feedback_detail = {}
            if self.last_feedback_snapshot is not None:
                stamps = self.last_feedback_snapshot["group_timestamps_s"]
                feedback_detail = {
                    "feedback_snapshot_source": "last_valid_snapshot",
                    "motion_status": self.last_feedback_snapshot["status"]["motion_status"],
                    "joint_feedback_group_span_ms": (max(stamps) - min(stamps)) * 1000,
                    "joint_feedback_oldest_age_ms": (time.time() - min(stamps)) * 1000,
                }
            self.events.append({"event": "output_fault", "reason": str(reason),
                                "monotonic": self.clock(),
                                "last_target_deg": np.rad2deg(self.last_target).tolist() if self.last_target is not None else None,
                                "last_measured_joints_deg": self.last_measured,
                                "tcp_following_error_mm": self.tcp_following_error_mm,
                                "tcp_following_recovery_elapsed_ms": (
                                    (self.clock() - self.tcp_following_recovery_since) * 1000
                                    if self.tcp_following_recovery_since is not None else None),
                                **feedback_detail,
                                "physical_stop_confirmed": False})
        self.state = "FAULT"
        self.fault_reason = self.fault_reason or str(reason)
        raise RuntimeError(self.fault_reason)

    def _checked_send(self, frame, timeout=None):
        if self.state in ("PREPARING", "ACTIVE") and self.cancel_requested():
            self._fault("commissioning cancelled before CAN frame send")
        self.attempted_frames += 1
        try:
            super()._checked_send(frame, timeout)
        except Exception as exc:
            self.events.append({"event": "can_tx_error", "id": hex(frame.arbitration_id),
                                "data": bytes(frame.data).hex(), "reason": str(exc),
                                "monotonic": self.clock()})
            raise
        self.events.append({"event": "can_tx", "id": hex(frame.arbitration_id),
                            "data": bytes(frame.data).hex(), "monotonic": self.clock()})

    def _batch(self, target):
        previous = self.last_target
        super()._batch(target)
        self.last_command_sent_wall = time.time()
        if previous is not None:
            step = target - previous
            moving = np.abs(step) > math.radians(.005)
            # Repeats and rounding noise must not erase the last motion direction.
            self.last_command_direction[moving] = np.sign(step[moving])

    def status(self):
        with self.lock:
            physical = self.arm._config["comm"]["can"]["interface"] == "socketcan"
            recovering = self.tcp_following_recovery_since is not None
            wait_reason = "tcp_following_error_recovery" if recovering else self.following_wait_reason
            return {"pilot_profile": self.config.get("pilot_profile", "initial"),
                    "output_state": self.state, "fault_reason": self.fault_reason,
                    "freeze_reason": self.freeze_reason, "target_batches_sent": self.targets_sent,
                    "regrip_feedback_ready": self.regrip_ready,
                    "hold_feedback_observation": self.hold_observation if self.state == "HOLDING" else None,
                    "waiting_for_feedback": self.state == "ACTIVE" and (recovering or self.following_wait_since is not None),
                    "following_wait_reason": wait_reason if self.state == "ACTIVE" else None,
                    "tcp_following_error_mm": self.tcp_following_error_mm,
                    "tcp_following_recovery_elapsed_ms": (
                        (self.clock() - self.tcp_following_recovery_since) * 1000 if recovering else None),
                    "candidate_joint_lead_deg": self.candidate_joint_lead_deg,
                    "candidate_tcp_lead_mm": self.candidate_tcp_lead_mm,
                    "can_frames_sent": self.frames_sent if physical else 0,
                    "virtual_can_frames_sent": self.frames_sent if not physical else 0,
                    "frame_send_attempts": self.attempted_frames,
                    "last_measured_joints_deg": self.last_measured,
                    "measured_joint_limit_excess_deg": self.measured_joint_limit_excess_deg,
                    "last_measured_tcp_displacement_mm": self.last_measured_tcp_delta,
                    "physical_stop_confirmed": False, "read_only": False,
                    "real_motion_allowed": physical and self.state == "ACTIVE"}

    def _feedback(self):
        measured = super()._feedback()
        self.last_measured = np.rad2deg(measured).tolist()
        if self.seed is not None:
            self.last_measured_tcp_delta = ((self.ik.pose(measured).translation -
                                             self.ik.pose(self.seed).translation) * 1000).tolist()
        if self.last_target is not None:
            errors_deg = np.abs(np.rad2deg(measured - self.last_target))
            index = int(np.argmax(errors_deg))
            if errors_deg[index] > .25:
                self._fault(f"pilot joint following error exceeds 0.25 degree: J{index + 1}, "
                            f"measured={math.degrees(measured[index]):.3f}, "
                            f"target={math.degrees(self.last_target[index]):.3f}, "
                            f"error={errors_deg[index]:.3f} degree")
            delta = self.ik.pose(measured).translation - self.ik.pose(self.last_target).translation
            self._check_tcp_following_error(float(np.linalg.norm(delta)))
        if (self.following_wait_since is not None
                and self.clock() - self.following_wait_since >= self.config["following_wait_timeout_s"]):
            self._fault(f"pilot feedback did not catch up within 1 second: {self.following_wait_reason}")
        return measured

    def _check_tcp_following_error(self, error_m):
        self.tcp_following_error_mm = error_m * 1000
        if "tcp_following_recovery_timeout_s" not in self.config:
            if error_m > .0005:
                self._fault(f"pilot measured TCP is over 0.5 mm from last target: "
                            f"error={self.tcp_following_error_mm:.3f} mm")
            return
        hard_limit = self.config["tcp_following_fault_m"]
        if error_m > hard_limit:
            self._fault(f"pilot TCP following error exceeds {hard_limit * 1000:g} mm hard limit: "
                        f"error={self.tcp_following_error_mm:.3f} mm")
        now = self.clock()
        timeout = self.config["tcp_following_recovery_timeout_s"]
        resume_limit = self.config["tcp_following_resume_m"]
        if self.tcp_following_recovery_since is not None:
            elapsed = now - self.tcp_following_recovery_since
            # This deadline belongs to measured error, not candidate sends or grips.
            if elapsed >= timeout:
                self._fault(f"pilot TCP following error did not recover to {resume_limit * 1000:g} mm "
                            f"within {timeout * 1000:g} ms: error={self.tcp_following_error_mm:.3f} mm")
            if error_m <= resume_limit:
                self.events.append({"event": "tcp_following_recovered", "monotonic": now,
                                    "error_mm": self.tcp_following_error_mm, "wait_ms": elapsed * 1000})
                self.tcp_following_recovery_since = None
        elif error_m > self.config["tcp_following_pause_m"]:
            self.tcp_following_recovery_since = now
            self.events.append({"event": "tcp_following_wait", "monotonic": now,
                                "error_mm": self.tcp_following_error_mm,
                                "resume_error_mm": resume_limit * 1000,
                                "hard_error_mm": hard_limit * 1000,
                                "timeout_ms": timeout * 1000})

    def _previous_target_arrived(self, measured):
        feedback = self.last_feedback_snapshot
        if feedback is None or self.last_command_sent_wall is None:
            return False
        # Require a post-command arrival report and all four new joint groups.
        if (feedback["status"]["motion_status"] != 0
                or feedback["status_timestamp_s"] <= self.last_command_sent_wall
                or min(feedback["group_timestamps_s"]) <= self.last_command_sent_wall):
            return False
        return (np.max(np.abs(measured - self.last_target)) <= math.radians(.02)
                and np.linalg.norm(self.ik.pose(measured).translation -
                                   self.ik.pose(self.last_target).translation) <= .0001)

    def _feedback_allows_target(self, target, measured):
        if "following_wait_timeout_s" not in self.config:
            return True
        self.candidate_joint_lead_deg = float(np.max(np.abs(np.rad2deg(target - measured))))
        self.candidate_tcp_lead_mm = float(np.linalg.norm(
            self.ik.pose(target).translation - self.ik.pose(measured).translation) * 1000)
        step = target - self.last_target
        reversing = np.any((step * self.last_command_direction < 0)
                           & (np.abs(step) > math.radians(.005)))
        wait_reason = None
        if self.tcp_following_recovery_since is not None:
            wait_reason = "tcp_following_error_recovery"
        elif (self.config.get("require_post_command_feedback")
                and (self.last_feedback_snapshot is None or self.last_command_sent_wall is None
                     or min(self.last_feedback_snapshot["group_timestamps_s"]) <= self.last_command_sent_wall)):
            wait_reason = "waiting_for_post_command_feedback"
        elif (self.config.get("reversal_requires_arrival") and reversing
                and not self._previous_target_arrived(measured)):
            wait_reason = "previous_target_pending_before_reversal"
        elif (self.candidate_joint_lead_deg > self.config["max_joint_command_lead_deg"]
                or self.candidate_tcp_lead_mm > self.config["max_tcp_command_lead_m"] * 1000):
            wait_reason = "robot_feedback_catching_up"
        if wait_reason is not None:
            if self.following_wait_since is None:
                self.following_wait_since = self.clock()
            if wait_reason != self.following_wait_reason:
                self.events.append({"event": "following_wait", "monotonic": self.clock(),
                                    "reason": wait_reason,
                                    "candidate_joint_lead_deg": self.candidate_joint_lead_deg,
                                    "candidate_tcp_lead_mm": self.candidate_tcp_lead_mm})
            self.following_wait_reason = wait_reason
            return False
        if self.following_wait_since is not None:
            self.events.append({"event": "following_resumed", "monotonic": self.clock(),
                                "wait_ms": (self.clock() - self.following_wait_since) * 1000})
        self.following_wait_since = None
        self.following_wait_reason = None
        return True

    def _startup_feedback(self, initial=None, *, require_arrived=True):
        if not self.arm.is_connected() or self.arm._ctx.has_comm_error():
            raise RuntimeError("Nero connection fault before takeover completed")
        data = robot_snapshot(self.arm, self.config["max_gap_s"],
                              coherence_wait_s=self.config.get("feedback_coherence_wait_s", 0.))
        status = data["status"]
        if (status["ctrl_mode"] not in (1, 3) or status["arm_status"] not in (0, 6)
                or status["err_code"] or status["teach_status"] not in INACTIVE_TEACH_STATUSES
                or any(data["joint_faults"])):
            raise RuntimeError("startup requires normal/braked arm, no driver fault, CAN or Ethernet control")
        if status["motion_status"] not in (0, 1):
            raise RuntimeError("unknown Nero motion status during takeover")
        if require_arrived and status["motion_status"] != 0:
            raise RuntimeError("startup requires arrival at the previous robot target")
        q = self.ik.validate_feedback_joints(data["joints_rad"])
        if (self.feedback_gating_enabled and initial is not None
                and np.max(np.abs(q - initial)) > math.radians(.15)):
            raise RuntimeError("arm changed over 0.15 degree during takeover")
        return data, q

    def _wait_takeover_ready(self, initial, sent_wall_time, *, phase,
                             enable_joints=False, stopping=lambda: False):
        deadline = self.clock() + 2.
        pending = set()
        previous = None
        while True:
            if stopping():
                raise RuntimeError(f"takeover cancelled while waiting for {phase}")
            data, q = self._startup_feedback(initial, require_arrived=False)
            status = data["status"]
            new_status = data["status_timestamp_s"] > sent_wall_time
            mode_ready = status["ctrl_mode"] == 1 and status["mode_feedback"] == 1
            if new_status and mode_ready and enable_joints:
                for index, enabled in enumerate(data["joints_enabled"], 1):
                    if not enabled and index not in pending:
                        message = self.arm._MSG_MotorEnableDisableConfig(joint_index=index, enable_flag=2)
                        self._command([message], lambda i=index: self.arm.enable(i, timeout=0.))
                        pending.add(index)
            ready = (new_status and mode_ready and not data["joint_output_blockers"]
                     and (not self.feedback_gating_enabled or status["motion_status"] == 0))
            signature = (new_status, status["ctrl_mode"], status["mode_feedback"],
                         status["motion_status"], tuple(data["joint_output_blockers"]))
            if signature != previous:
                self.events.append({"event": "takeover_target_ready" if ready else "takeover_wait",
                                    "phase": phase, "motion_status": status["motion_status"],
                                    "arrival_required": self.feedback_gating_enabled,
                                    "status_after_command": new_status,
                                    "joint_output_blockers": data["joint_output_blockers"],
                                    "max_joint_drift_deg": float(np.max(np.abs(np.rad2deg(q - initial)))),
                                    "monotonic": self.clock()})
                previous = signature
            if self.clock() >= deadline:
                raise RuntimeError(f"takeover {phase} timed out: motion_status={status['motion_status']}, "
                                   f"status_after_command={new_status}, blockers={data['joint_output_blockers']}")
            if ready:
                return
            time.sleep(.01)

    def _command(self, messages, callback):
        packed = [self.arm._parser.pack(msg) for msg in messages]
        self.expected = deque((m.arbitration_id, bytes(m.data)) for m in packed)
        try:
            callback()
            if self.expected:
                self._fault("SDK omitted a commissioning frame")
        finally:
            self.expected.clear()

    def takeover(self, *, enable_joints=False, stopping=lambda: False):
        """Seed before switching mode, then enable only individually addressed arm joints."""
        with self.lock:
            if self.state != "PREPARED":
                raise RuntimeError("takeover can only run once")
            self.cancel_requested = stopping
            data, initial = self._startup_feedback(require_arrived=self.feedback_gating_enabled)
            initial = self.ik.command_from_feedback(initial)
            if not all(data["joints_enabled"]) and not enable_joints:
                raise RuntimeError("seven joints are not enabled; use official controls or explicit --enable-joints")
            self.seed = initial.copy()
            deadline = self.clock() + (.3 if self.feedback_gating_enabled else 0.)
            while self.clock() < deadline:
                if stopping():
                    raise RuntimeError("takeover cancelled before sending")
                _, q = self._startup_feedback(initial)
                if np.max(np.abs(q - initial)) > math.radians(.03):
                    raise RuntimeError("arm must be stationary before takeover")
                time.sleep(.01)
            self.state = "PREPARING"
            try:
                # Preload the current target; Ethernet firmware may ignore it.
                # Repeat immediately after the CAN/J mode request before enabling.
                self._batch(initial)
                mode = self.arm._msg_mode
                mode.ctrl_mode = 1
                mode.move_mode = 1
                mode.move_spd_rate_ctrl = self.config.get("firmware_speed_percent", 1)
                instantaneous = self.config.get("joint_command_mode") == "move_js"
                mode.mit_mode = 0xAD if instantaneous else 0
                requested_mode = self.arm.OPTIONS.MOTION_MODE.JS if instantaneous else self.arm.OPTIONS.MOTION_MODE.J
                self._command([mode], lambda: self.arm.set_motion_mode(requested_mode))
                self._batch(initial)
                self._wait_takeover_ready(initial, time.time(), phase="mode_and_target",
                                          enable_joints=enable_joints, stopping=stopping)
                self.state = "PREPARED"
                previous_target, previous_sent = self.last_target.copy(), self.last_sent
                self.last_target = None
                super().activate()
                # The fresh seed can differ slightly from the preloaded target.
                # Pace that correction too, using the previous actual batch time.
                joint_step = np.max(np.abs(self.last_target - previous_target))
                tcp_step = np.linalg.norm(self.ik.pose(self.last_target).translation -
                                          self.ik.pose(previous_target).translation)
                interval = max(1. / self.config["control_hz"],
                               joint_step / math.radians(self.config["max_joint_speed_deg_s"]),
                               tcp_step / self.config["max_tcp_speed_m_s"])
                if (joint_step > math.radians(self.config["max_joint_step_deg"]) + 1e-10
                        or interval >= self.config["max_gap_s"]):
                    raise RuntimeError("takeover seed correction exceeds output step or timing limits")
                while self.clock() < previous_sent + interval:
                    if stopping():
                        raise RuntimeError("takeover cancelled before seed correction")
                    self._startup_feedback(initial, require_arrived=False)
                    time.sleep(min(.005, max(0., previous_sent + interval - self.clock())))
                self._feedback()
                self._batch(self.last_target)
                self._wait_takeover_ready(initial, time.time(), phase="seed_target", stopping=stopping)
                self.events.append({"event": "takeover_complete", "seed_joints_rad": self.seed.tolist(),
                                    "speed_percent": mode.move_spd_rate_ctrl, "holding_physically_verified": False})
            except Exception as exc:
                self._fault(f"takeover incomplete; no automatic disable/reset/retry: {exc}")

    @property
    def regrip_ready(self):
        return self.state == "HOLDING" and (not self.feedback_gating_enabled or self.hold_settled)

    def begin_motion(self, received):
        with self.lock:
            if self.state not in ("ACTIVE", "HOLDING"):
                raise RuntimeError("pilot is not active")
            self._feedback()
            if self.state == "HOLDING" and self.feedback_gating_enabled:
                self._update_hold_observation()
                if not self.hold_settled:
                    raise RuntimeError("regrip requires settled feedback")
            self.observe_input(received)
            self.state = "ACTIVE"
            self.following_wait_since = None
            self.following_wait_reason = None
            self.hold_settled = False
            self.hold_observation = None
            self.last_sent = self.clock()
            self.last_input = received

    def hold_motion(self):
        """Hold a released grip; only this state can resume in the same session."""
        with self.lock:
            if self.state == "HOLDING":
                return
            if self.state != "ACTIVE":
                raise RuntimeError("cannot hold inactive pilot output")
            self._feedback()
            self.state = "FREEZING"
            self._batch(self.last_target)
            self.state = "HOLDING"
            self.following_wait_since = None
            self.following_wait_reason = None
            self.hold_reference = self.hold_stable_since = None
            self.hold_settled = False
            self.hold_observation = None
            self.events.append({"event": "target_held", "reason": "grip_released",
                                "last_target_rad": self.last_target.tolist(),
                                "monotonic": self.clock(), "physical_stop_confirmed": False})

    def _settling_allowed(self):
        return self.tcp_following_recovery_since is None

    def _settling_errors_and_limits(self, measured):
        return {"joint_error_deg": (float(np.max(np.abs(np.rad2deg(measured - self.last_target)))), .05)}

    def _observe_settling_sample(self, measured, reference, stable_since):
        checks = self._settling_errors_and_limits(measured)
        blockers = [name for name, (error, limit) in checks.items() if not math.isfinite(error) or error > limit]
        if not self._settling_allowed():
            blockers.append("following_recovery")
        change = None if reference is None else float(np.max(np.abs(np.rad2deg(measured - reference))))
        now = self.clock()
        if reference is None or blockers or change > .01:
            reference, stable_since = measured.copy(), now
        stable_s = now - stable_since
        settled = not blockers and stable_s >= .3
        reason = ("target_error" if any(name in checks for name in blockers) else
                  "following_recovery" if blockers else
                  "settled" if settled else
                  "feedback_moving" if change is not None and change > .01 else "observing_stability")
        observation = {"feedback_settled": settled, "settling_reason": reason,
                       "settling_blockers": blockers,
                       "settling_errors": {name: values[0] for name, values in checks.items()},
                       "settling_limits": {name: values[1] for name, values in checks.items()},
                       "joint_change_deg": change, "stability_tolerance_deg": .01,
                       "stable_for_s": stable_s, "required_stable_s": .3,
                       "observed_joints_rad": measured.tolist(), "physical_stop_confirmed": False}
        return observation, reference, stable_since

    def _update_hold_observation(self):
        measured = self._feedback()
        if not self.feedback_gating_enabled:
            self.hold_settled = False
            self.hold_observation = self._feedback_only_observation(measured)
            return
        self.hold_observation, self.hold_reference, self.hold_stable_since = self._observe_settling_sample(
            measured, self.hold_reference, self.hold_stable_since)
        self.hold_settled = self.hold_observation["feedback_settled"]

    def observe_input(self, received):
        with self.lock:
            now = self.clock()
            if not math.isfinite(received) or not 0 <= now - received <= self.config["max_gap_s"]:
                self._fault("pilot input is stale")
            self.input_seen = received

    def send_target(self, target, sampled_at):
        with self.lock:
            if self.state != "ACTIVE":
                raise RuntimeError(self.fault_reason or "pilot output is not active")
            try:
                target = self.ik.validate_joints(target)
            except ValueError as exc:
                self._fault(exc)
            rounded = self.ik.quantize_command(target)
            delta = self.ik.pose(rounded).translation - self.ik.pose(self.seed).translation
            if (self.config.get("pilot_profile") not in ("translation-check", "teleop", "yz-check")
                    and (delta[0] < -.00015 or np.linalg.norm(delta[1:]) > .0002)):
                self._fault("pilot target exceeds the +X translation corridor")
            if (self.config.get("pilot_profile") == "yz-check"
                    and abs(delta[0]) > .00015):
                self._fault("yz-check target contains forbidden base-X motion")
            joint_step = np.max(np.abs(rounded - self.last_target))
            tcp_step = np.linalg.norm(self.ik.pose(rounded).translation -
                                      self.ik.pose(self.last_target).translation)
            interval = max(1. / self.config["control_hz"],
                           joint_step / math.radians(self.config["max_joint_speed_deg_s"]),
                           tcp_step / self.config["max_tcp_speed_m_s"])
            if interval >= self.config["max_gap_s"]:
                self._fault("pilot target cannot meet speed limits before timeout")
            due = self.last_sent + interval
        # Solver duration varies. Pace the rounded command by actual send time,
        # releasing the lock while waiting so the watchdog can still freeze it.
        while self.clock() < due:
            if self.cancel_requested():
                raise RuntimeError("commissioning cancelled before target send")
            self.tick()
            with self.lock:
                if self.state != "ACTIVE":
                    raise RuntimeError(self.fault_reason or "pilot stopped while pacing target")
            time.sleep(min(.005, max(0., due - self.clock())))
        with self.lock:
            if self.state != "ACTIVE":
                raise RuntimeError(self.fault_reason or "pilot stopped before target send")
            return super().send_target(rounded, sampled_at)

    def freeze(self, reason="requested_stop"):
        with self.lock:
            if self.state in ("FROZEN", "FAULT", "CLOSED", "PREPARED"):
                return
            self.freeze_reason = reason
            if self.state == "HOLDING":
                self._feedback()
                self.state = "FREEZING"
                self._batch(self.last_target)
                self.state = "FROZEN"
            else:
                super().freeze()
            self.following_wait_since = None
            self.following_wait_reason = None
            self.events.append({"event": "target_frozen", "reason": reason,
                                "last_target_rad": self.last_target.tolist(),
                                "physical_stop_confirmed": False})

    def _check_input_watchdog(self):
        if self.state in ("ACTIVE", "HOLDING") and self.input_seen is not None:
            if self.clock() - self.input_seen > self.config["max_gap_s"]:
                self.freeze("input_watchdog_timeout")

    def tick(self):
        with self.lock:
            if self.state not in ("ACTIVE", "HOLDING", "FROZEN"):
                return
            if self.state == "HOLDING":
                self._update_hold_observation()
            else:
                self._feedback()
            self._check_input_watchdog()

    def start_watchdog(self):
        if self.worker is not None:
            raise RuntimeError("watchdog already started")

        def watch():
            while not self.stop_worker.wait(.02):
                try:
                    self.tick()
                except Exception as exc:
                    with self.lock:
                        if self.state != "FAULT":
                            try:
                                self._fault(f"watchdog failure: {exc}")
                            except RuntimeError:
                                pass
                    return

        self.worker = threading.Thread(target=watch, name="nero-pilot-watchdog", daemon=True)
        self.worker.start()

    def _feedback_only_observation(self, measured):
        return {"feedback_settled": False, "settling_required": False,
                "settling_reason": "not_required", "observed_joints_rad": measured.tolist(),
                "physical_stop_confirmed": False}

    def observe_settled(self, timeout=2., stopping=lambda: False):
        """Feedback-based observation, not certification of a physical stop."""
        if not self.feedback_gating_enabled:
            with self.lock:
                if self.state not in ("ACTIVE", "FROZEN"):
                    raise RuntimeError(f"output is {self.state}: {self.fault_reason}")
                return self._feedback_only_observation(self._feedback())
        deadline, stable_since, reference = self.clock() + timeout, None, None
        observation = {"feedback_settled": False, "settling_reason": "no_feedback_observed",
                       "physical_stop_confirmed": False}
        while self.clock() < deadline:
            with self.lock:
                if self.state not in ("ACTIVE", "FROZEN"):
                    raise RuntimeError(f"output is {self.state}: {self.fault_reason}")
                q = self._feedback()
                observation, reference, stable_since = self._observe_settling_sample(q, reference, stable_since)
                if observation["feedback_settled"]:
                    return observation
            if stopping():
                return {**observation, "settling_reason": "observation_cancelled", "feedback_settled": False}
            time.sleep(.02)
        return {**observation, "settling_timed_out": True}

    def stop_watchdog(self):
        self.stop_worker.set()
        if self.worker is not None:
            self.worker.join(timeout=1.)
            if self.worker.is_alive():
                raise RuntimeError("pilot watchdog did not finish")

    def __exit__(self, *_args):
        self.stop_watchdog()
        super().__exit__()


class PilotConnection:
    output_class = NeroPilotOutput

    def __init__(self, config, ik, *, clearance_confirmed):
        if clearance_confirmed is not True:
            raise ValueError("--confirm-clearance is required before connecting pilot output")
        self.config, self.ik = config, ik
        self.arm = self.output = self.lock_file = None

    def __enter__(self):
        arm_config = self.config["arm"]
        channel = arm_config["can_channel"]
        if self.config["arm_id"] not in ("right_arm", "left_arm") or not re.fullmatch(r"can[0-9]+", channel):
            raise ValueError("pilot output requires a configured left_arm or right_arm CAN channel")
        if arm_config["firmware"] != "NeroFW.V120":
            raise ValueError("first-motion pilot requires Nero V120")
        from .deploy import require_site_ready
        require_site_ready(self.config)
        try:
            self.lock_file = open(f"/tmp/pico_nero_{channel}.lock", "a")
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            use_agx_sdk()
            from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
            config = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.V120,
                                          interface="socketcan", channel=channel,
                                          bitrate=arm_config["bitrate"], auto_connect=False)
            self.arm = AgxArmFactory.create_arm(config)
            self.arm._ctx.init_comm()
            self.output = self.output_class(self.arm, self.config, self.ik, clearance_confirmed=True)
            # Install the instance send guard before starting any SDK threads.
            self.arm.connect()
            return self.output
        except Exception:
            self.__exit__()
            raise

    def __exit__(self, *_args):
        try:
            if self.output is not None:
                try:
                    self.output.freeze("connection_exit")
                finally:
                    self.output.stop_watchdog()
        finally:
            try:
                if self.arm is not None:
                    self.arm.disconnect()
            finally:
                try:
                    if self.output is not None:
                        self.output.__exit__()
                finally:
                    if self.lock_file is not None:
                        self.lock_file.close()


def wait_neutral(pico, stopping, timeout=8., monitor=lambda: None,
                 emit=lambda record: None, phase="before_takeover", *, require_head_pose=False,
                 hand="right"):
    guard = InputGuard(stale_s=.2, require_head_pose=require_head_pose, hand=hand)
    deadline = time.monotonic() + timeout
    frames = 0
    last_tracking = None
    last_received = None
    last_report = float("-inf")
    last_signature = None
    while not stopping() and time.monotonic() < deadline:
        monitor()
        event = pico.next()
        now = time.monotonic()
        if event["kind"] == "tracking":
            frames += 1
            last_tracking, last_received = event["tracking"], event["received_monotonic"]
            state = guard.step(event["tracking"], event["received_monotonic"], now, event["device_id"])
        elif event["kind"] == "tick":
            # An empty queue poll is not a disconnect. Let the 200 ms watchdog
            # decide freshness so normal packet spacing can satisfy neutral.
            state = guard.tick(now)
        else:
            state = guard.invalidate(event["kind"], disconnected=event["kind"] == "disconnect")
        signature = (state["state"], state["reason"])
        if signature != last_signature or now - last_report >= 1.:
            controllers = last_tracking.get("Controller") if isinstance(last_tracking, dict) else None
            controller = controllers.get(hand) if isinstance(controllers, dict) else None
            controller = controller if isinstance(controller, dict) else {}
            emit({"event": "input_wait", "phase": phase, "input_state": state["state"],
                  "reason": state["reason"], "tracking_frames": frames,
                  f"last_{hand}_grip": controller.get("grip"), f"last_{hand}_trigger": controller.get("trigger"),
                  f"last_{hand}_pose_valid": valid_pose(controller.get("pose")) is not None,
                  "last_tracking_age_ms": (now - last_received) * 1000 if last_received is not None else None,
                  "wait_remaining_s": round(max(0., deadline - now), 1)})
            last_report, last_signature = now, signature
        if event["kind"] == "tracking" and state["state"] == "READY_IDLE":
            return guard
    if stopping():
        raise RuntimeError(f"PICO neutral wait cancelled ({phase})")
    raise RuntimeError(f"fresh neutral PICO input not available ({phase}): "
                       f"tracking_frames={frames}, reason={guard.reason}; "
                       "release buttons and check WORKING+Send")


def run_pilot_session(output, pico, targets_config, duration, stopping, emit, *, enable_joints=False,
                      continuous_grip=False):
    """Optionally regrip after normal release; all other stops remain latched."""
    motion_started = False
    grip_count = hold_count = 0
    regrip_announced = False
    reason = "duration_elapsed"
    error = None
    settled = {"feedback_settled": False, "physical_stop_confirmed": False}
    teleop = None
    solver_context = ExitStack()
    secondary_error = None
    try:
        emit({"event": "waiting_for_neutral", "can_frames_sent": 0})
        wait_neutral(pico, stopping, timeout=30., emit=emit)
        # Solver construction happens before takeover, not while input can move an arm.
        if targets_config.get("ik_execution") == "process":
            from .nero_ik_process import ProcessNeroIK
            target_ik = solver_context.enter_context(ProcessNeroIK(
                targets_config, stopping=lambda: stopping() or output.state in ("FAULT", "FROZEN", "CLOSED")))
            emit({"event": "ik_worker_ready", "ik_execution": "process",
                  "solve_timeout_ms": target_ik.solve_timeout_s * 1000})
        else:
            target_ik = NeroIK(targets_config)
        output.takeover(enable_joints=enable_joints, stopping=stopping)
        output.start_watchdog()
        startup_settled = output.observe_settled(stopping=stopping)
        if not startup_settled["feedback_settled"]:
            raise RuntimeError("current-position takeover did not settle")
        emit({"event": "takeover_feedback_settled", **startup_settled})
        guard = wait_neutral(pico, stopping, monitor=output.tick, emit=emit, phase="after_takeover")
        teleop = PilotTargets(targets_config, output.seed, target_ik)
        teleop.guard = guard
        output.observe_input(guard.last_valid_received)
        profile = targets_config.get("pilot_profile")
        axes = ("base_XYZ" if profile in ("translation-check", "teleop")
                else "base_YZ" if profile == "yz-check" else "+base_X")
        emit({"event": "pilot_ready", "axis": axes,
              "ik_execution": targets_config.get("ik_execution", "in_process"),
              "max_tcp_target_radius_mm": targets_config["max_displacement_m"] * 1000,
              "max_tcp_command_radius_mm": output.config["max_displacement_m"] * 1000,
              "max_tcp_command_speed_mm_s": output.config["max_tcp_speed_m_s"] * 1000,
              "max_joint_command_speed_deg_s": output.config["max_joint_speed_deg_s"],
              "feedback_coherence_wait_ms": output.config.get("feedback_coherence_wait_s", 0.) * 1000,
              "feedback_pacing": "following_wait_timeout_s" in output.config,
              "reversal_requires_arrival": output.config.get("reversal_requires_arrival", False),
              "require_post_command_feedback": output.config.get("require_post_command_feedback", False),
              "tcp_following_pause_mm": output.config.get("tcp_following_pause_m", .0005) * 1000,
              "tcp_following_resume_mm": (output.config["tcp_following_resume_m"] * 1000
                                          if "tcp_following_resume_m" in output.config else None),
              "tcp_following_fault_mm": output.config.get("tcp_following_fault_m", .0005) * 1000,
              "tcp_following_recovery_timeout_ms": output.config.get("tcp_following_recovery_timeout_s", 0.) * 1000,
              "gripper_commanded": False,
              "max_joint_target_travel_deg": targets_config["max_joint_session_deg"],
              "max_joint_command_travel_deg": output.config["max_joint_session_deg"],
              "continuous_grip": continuous_grip,
              "seed_joints_deg": np.rad2deg(output.seed).tolist(), **output.status()})
        deadline = time.monotonic() + duration
        while not stopping() and time.monotonic() < deadline:
            state = output.status()
            if state["output_state"] not in ("ACTIVE", "HOLDING"):
                reason = state["fault_reason"] or state["freeze_reason"] or state["output_state"]
                break
            event = pico.next()
            event["processed_monotonic"] = time.monotonic()
            previous_target = teleop.target.copy()
            result = teleop.process(event)
            state = result["guard"]["state"]
            stop_reason = None
            if state in ("INVALID", "DISCONNECTED", "PAUSED"):
                stop_reason = result["reason"]
            elif continuous_grip and teleop.guard.reference_requests:
                stop_reason = "reference_reset_requested"
            elif event["kind"] == "tracking":
                output.observe_input(event["received_monotonic"])
            if stop_reason is not None:
                pass
            elif result["state"] == "ANCHOR":
                if motion_started and not continuous_grip:
                    raise RuntimeError("pilot cannot reanchor in the same run")
                with output.lock:
                    if output.state == "HOLDING":
                        output._update_hold_observation()
                    if output.state == "HOLDING" and not output.hold_settled:
                        teleop.guard.accepted = False
                        teleop.guard.needs_release = True
                        teleop.guard.neutral_since = None
                        teleop.guard.state = "WAIT_RELEASE"
                        teleop.guard.reason = "fresh_neutral_required"
                        result = teleop.hold("regrip_requires_settled_feedback_and_release")
                    else:
                        output.begin_motion(event["received_monotonic"])
                        if motion_started:
                            # Reanchor to the rounded command, retaining the original
                            # session seed, corridor and orientation constraints.
                            teleop.q = output.last_target.copy()
                            teleop.target = teleop.limit_requested_target(target_ik.pose(teleop.q).translation)
                            teleop.anchor_target = teleop.target.copy()
                            result = teleop.snapshot("ANCHOR", "held_command_and_controller_reference")
                        motion_started = True
                        grip_count += 1
                        emit({"event": "pilot_grip", "grip_count": grip_count,
                              "continuous_grip": continuous_grip, **output.status()})
            elif motion_started and result["state"] == "HOLD":
                if (continuous_grip and state in ("WAIT_RELEASE", "READY_IDLE")
                        and result["reason"] in ("fresh_neutral_required", "grip_released")):
                    if output.state == "ACTIVE":
                        output.hold_motion()
                        hold_count += 1
                        regrip_announced = False
                        emit({"event": "pilot_hold", "hold_count": hold_count,
                              "grip_count": grip_count, **output.status()})
                else:
                    stop_reason = result["reason"]
            elif result["state"] in ("FOLLOW", "LIMITED"):
                sent = output.send_target(teleop.q, event["received_monotonic"])
                teleop.q = output.last_target.copy()
                if sent is False:
                    teleop.target = previous_target
                    result = teleop.snapshot("WAIT_FEEDBACK", output.following_wait_reason or "robot_feedback_catching_up",
                                             deferred_tcp_target_base_m=result["tcp_target_base_m"])
            if (continuous_grip and not regrip_announced and stop_reason is None
                    and teleop.guard.state == "READY_IDLE" and output.status()["regrip_feedback_ready"]):
                emit({"event": "pilot_regrip_ready", "grip_count": grip_count, **output.status()})
                regrip_announced = True
            emit({"event": "pilot_sample", "input": event, "target_calculation": result,
                  "commanded_joints_deg": np.rad2deg(output.last_target).tolist(), **output.status()})
            if stop_reason is not None:
                reason = stop_reason
                break
        if stopping():
            reason = "operator_interrupt"
    except Exception as exc:
        if stopping():
            reason, error = "operator_interrupt", None
        else:
            reason, error = "pilot_error", str(exc)
    finally:
        try:
            output.freeze(reason)
            if output.state == "FROZEN":
                settled = output.observe_settled()
        except Exception as exc:
            error = error or str(exc)
        try:
            output.stop_watchdog()
        except Exception as exc:
            error = error or str(exc)
        try:
            solver_context.close()
        except Exception as exc:
            error = error or str(exc)
        if output.state == "FAULT":
            secondary_error = error if error != output.fault_reason else None
            reason, error = "pilot_error", output.fault_reason or error
            settled = {"feedback_settled": False, "physical_stop_confirmed": False}
        while output.events:
            emit(output.events.popleft())
        summary = {"event": "pilot_complete", "reason": reason, "error": error,
                   "secondary_error": secondary_error,
                   "continuous_grip": continuous_grip, "grip_count": grip_count, "hold_count": hold_count,
                   "grip_started": motion_started, "feedback_observation": settled,
                   "motor_disable_sent": False, "electronic_estop_sent": False,
                   "holding_behavior_physically_verified": False, **output.status()}
        emit(summary)
    return summary


def run_physical_pilot(config, output_path, duration, stopping, *, clearance_confirmed,
                       enable_joints, profile="initial", continuous_grip=False,
                       mapping_candidate=None):
    import json
    from .nero_preview_io import PicoEvents
    if mapping_candidate is not None:
        apply_mapping_candidate(config, mapping_candidate)
    if profile == "yz-check" and config.get("mapping_status") != "physically_verified_candidate":
        raise ValueError("yz-check requires an explicitly physically_verified_candidate mapping")
    targets_config, output_config = pilot_configs(config, profile)
    with output_path.open("x", encoding="utf-8") as stream:
        last_console = 0.
        previous = None

        def emit(record):
            nonlocal last_console, previous
            stream.write(json.dumps({"logged_monotonic": time.monotonic(), **record}) + "\n")
            event = record["event"]
            if event == "can_tx":
                return
            if event == "pilot_start":
                record = {k: record[k] for k in ("event", "output", "pilot_profile", "continuous_grip", "clearance_confirmed", "enable_joints_requested")}
            if event == "pilot_sample":
                result = record["target_calculation"]
                signature = (result["state"], result["reason"])
                if time.monotonic() - last_console < 1. and (signature == previous or result["state"] == "WAIT_SAMPLE"):
                    return
                if result["state"] != "WAIT_SAMPLE":
                    previous = signature
                record = {k: record[k] for k in ("event", "output_state", "can_frames_sent", "commanded_joints_deg",
                                                "last_measured_tcp_displacement_mm", "tcp_following_error_mm",
                                                "tcp_following_recovery_elapsed_ms")}
                record.update(target_state=result["state"], reason=result["reason"],
                              tcp_displacement_base_mm=result["tcp_displacement_base_mm"])
            print(json.dumps(record), flush=True)
            stream.flush()
            last_console = time.monotonic()

        emit({"event": "pilot_start", "output": str(output_path), "pilot_profile": profile,
              "continuous_grip": continuous_grip,
              "clearance_confirmed": clearance_confirmed,
              "enable_joints_requested": enable_joints, "target_config": targets_config,
              "output_config": output_config, "physical_stop_confirmed": False})
        with ExitStack() as stack:
            writer = stack.enter_context(PilotConnection(output_config, NeroKinematics(output_config),
                                                        clearance_confirmed=clearance_confirmed))
            pico = stack.enter_context(PicoEvents(pico_sdk_path()))
            summary = run_pilot_session(writer, pico, targets_config, duration, stopping, emit,
                                        enable_joints=enable_joints, continuous_grip=continuous_grip)
        return 0 if summary["grip_started"] and summary["error"] is None and summary["output_state"] == "FROZEN" and summary["feedback_observation"]["feedback_settled"] else 4
