"""Nero SDK joint-output prototype. Restricted to python-can virtual buses.

No enable, mode switch, gripper command, homing, reconnect or physical-stop API.
The caller must supply an already connected virtual Nero V120 in CAN/J mode.
This exercises the real SDK encoder and error path without a physical transport.
"""

from collections import deque
import math
import time

import numpy as np

from .nero_preview_io import robot_snapshot


class VirtualNeroJointOutput:
    allowed_send_states = ("ACTIVE", "FREEZING")
    require_connected = True

    def validate_transport(self, driver_config):
        transport = driver_config.get("comm", {}).get("can", {})
        if (transport.get("interface") != "virtual" or driver_config.get("robot") != "nero"
                or driver_config.get("firmeware_version") != "v120"):
            raise ValueError("output prototype requires virtual CAN and Nero V120; physical output is unavailable")

    def __init__(self, arm, config, ik, clock=time.monotonic):
        driver_config = arm._config
        self.validate_transport(driver_config)
        if config.get("joint_command_mode", "move_j") not in ("move_j", "move_js"):
            raise ValueError("unsupported joint command mode")
        self.arm, self.config, self.ik, self.clock = arm, config, ik, clock
        self.state = "PREPARED"
        self.fault_reason = None
        self.frames_sent = self.targets_sent = 0
        self.expected = deque()
        self.last_target = self.seed = None
        self.last_sent = self.last_input = None
        self.last_feedback_snapshot = None
        self.measured_joint_limit_excess_deg = None
        self.comm = arm._ctx.get_comm()
        if self.comm is None or (self.require_connected and not self.comm.is_connected()):
            raise ValueError("virtual arm must be connected")
        self.original_send = self.comm.send
        self.original_auto_mode = arm._auto_set_motion_mode_enabled
        self.comm.send = self._checked_send
        arm.set_auto_set_motion_mode_enabled(False)

    def __enter__(self):
        return self

    @property
    def feedback_gating_enabled(self):
        return self.config.get("feedback_gating_enabled", True)

    def __exit__(self, *_args):
        # Closing a transport does not stop an arm; this prototype is virtual only.
        self.comm.send = self.original_send
        self.arm.set_auto_set_motion_mode_enabled(self.original_auto_mode)
        self.state = "CLOSED"

    def _fault(self, reason):
        self.state, self.fault_reason = "FAULT", str(reason)
        raise RuntimeError(self.fault_reason)

    def _checked_send(self, frame, timeout=None):
        if self.state not in self.allowed_send_states or not self.expected:
            self._fault("CAN send outside an authorized joint batch")
        expected_id, expected_data = self.expected[0]
        if (frame.is_extended_id or frame.is_remote_frame or frame.is_error_frame
                or frame.arbitration_id != expected_id or bytes(frame.data) != expected_data):
            self._fault("unexpected CAN command in joint batch")
        try:
            self.original_send(frame, timeout=.02)
            # SDK swallows ENOBUFS/ENETDOWN. Inspect each frame before a later
            # successful send can overwrite comm.last_error.
            if self.comm.last_error is not None:
                raise RuntimeError(f"CAN transport reported: {self.comm.last_error}")
        except Exception as exc:
            self._fault(f"joint batch may be partially transmitted: {exc}")
        self.frames_sent += 1
        self.expected.popleft()

    def _feedback(self):
        if not self.arm.is_connected() or self.arm._ctx.has_comm_error():
            self._fault("Nero connection fault")
        try:
            result = robot_snapshot(self.arm, self.config["max_gap_s"],
                                    coherence_wait_s=self.config.get("feedback_coherence_wait_s", 0.))
            if result["joint_output_blockers"]:
                raise ValueError(", ".join(result["joint_output_blockers"]))
            measured = self.ik.validate_feedback_joints(result["joints_rad"])
            following_limit = self.config.get("joint_feedback_fault_deg", 1.)
            if (self.feedback_gating_enabled and self.last_target is not None
                    and np.max(np.abs(measured - self.last_target)) > math.radians(following_limit)):
                raise ValueError(f"joint following error exceeds {following_limit:g} degree")
            self.last_feedback_snapshot = result
            excess = np.maximum(0., np.maximum(self.ik.model.lowerPositionLimit - measured,
                                               measured - self.ik.model.upperPositionLimit))
            self.measured_joint_limit_excess_deg = np.rad2deg(excess).tolist()
            return measured
        except ValueError as exc:
            self._fault(exc)

    def activate(self):
        if self.state != "PREPARED":
            self._fault("output activation requires a new prepared session")
        measured = self._feedback()
        self.last_target = self.seed = self.ik.command_from_feedback(measured)
        self.last_sent = self.last_input = self.clock()
        self.state = "ACTIVE"

    def _batch(self, target):
        packed = [self.arm._parser.pack(msg) for msg in self.arm._deal_move_j_msgs(target.tolist())]
        self.expected = deque((frame.arbitration_id, bytes(frame.data)) for frame in packed)
        try:
            getattr(self.arm, self.config.get("joint_command_mode", "move_j"))(target.tolist())
            if self.expected:
                raise RuntimeError("SDK omitted a joint frame")
        except Exception as exc:
            self._fault(f"joint batch incomplete; no automatic retry: {exc}")
        finally:
            self.expected.clear()
        self.last_target = target.copy()
        self.last_sent = self.clock()
        self.targets_sent += 1

    def _target_interval(self, sampled_at):
        now = self.clock()
        if not math.isfinite(sampled_at) or not 0 <= now - sampled_at <= self.config["max_gap_s"]:
            self._fault("stale or invalid target timestamp")
        dt = now - self.last_sent
        # Feedback pacing can idle the sender while input must still stay fresh.
        if (not 0 < dt <= self.config.get("max_output_gap_s", self.config["max_gap_s"])
                or sampled_at <= self.last_input):
            self._fault("output gap or non-increasing input timestamp")
        return dt

    def _feedback_allows_target(self, target, measured):
        return True

    def _prepare_target(self, target, measured):
        """Allow a sender to bound a candidate before all final command checks."""
        return target

    def send_target(self, target, sampled_at):
        """Return False when feedback defers a valid target without sending frames."""
        if self.state != "ACTIVE":
            self._fault("joint output is not active")
        dt = self._target_interval(sampled_at)
        measured = self._feedback()
        try:
            target = self.ik.validate_joints(target)
            target = self.ik.validate_joints(self._prepare_target(target, measured))
            # Validate the millidegree values actually representable on CAN.
            target = self.ik.quantize_command(target)
            step = math.radians(min(self.config["max_joint_step_deg"], self.config["max_joint_speed_deg_s"] * dt))
            if np.max(np.abs(target - self.last_target)) > step + 1e-10:
                raise ValueError("joint command step/speed limit")
            joint_travel = self.config["max_joint_session_deg"]
            if joint_travel is not None and np.max(np.abs(target - self.seed)) > math.radians(joint_travel):
                raise ValueError("joint session travel limit")
            previous_pose, pose, origin = (self.ik.pose(q) for q in (self.last_target, target, self.seed))
            if np.linalg.norm(pose.translation - previous_pose.translation) > self.config["max_tcp_speed_m_s"] * dt + 1e-10:
                raise ValueError("quantized TCP command speed limit")
            radius = self.config["max_displacement_m"]
            if radius is not None and np.linalg.norm(pose.translation - origin.translation) > radius:
                raise ValueError("TCP session travel limit")
            import pinocchio as pin
            rotation_limit = self.config.get("max_rotation_session_deg", self.config["orientation_tolerance_deg"])
            if np.linalg.norm(pin.log3(origin.rotation.T @ pose.rotation)) > math.radians(rotation_limit):
                raise ValueError("quantized orientation session limit" if "max_rotation_session_deg" in self.config
                                 else "quantized orientation hold limit")
            if ("max_angular_speed_deg_s" in self.config
                    and np.linalg.norm(pin.log3(previous_pose.rotation.T @ pose.rotation))
                    > math.radians(self.config["max_angular_speed_deg_s"]) * dt + 1e-10):
                raise ValueError("quantized orientation speed limit")
        except ValueError as exc:
            self._fault(exc)
        allowed = self._feedback_allows_target(target, measured)
        # Feedback can wait for coherent groups. An expired candidate must not send.
        self._target_interval(sampled_at)
        if not allowed:
            return False
        self._batch(target)
        self.last_input = sampled_at
        return True

    def freeze(self):
        """Latch output and repeat its last target, not a verified physical stop."""
        if self.state == "FROZEN":
            return
        if self.state != "ACTIVE":
            self._fault("cannot freeze an inactive/faulted output")
        self._feedback()
        self.state = "FREEZING"
        self._batch(self.last_target)
        self.state = "FROZEN"

    def tick(self):
        """Caller-driven input watchdog; no independent real-time stop guarantee."""
        if self.state == "ACTIVE" and self.clock() - self.last_input > self.config["max_gap_s"]:
            self.freeze()
