#!/usr/bin/env python3
"""Relative targets and Nero IK. Pure computation; no robot transport."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import casadi as ca
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin

from .pico_input_guard import InputGuard, valid_head_pose, valid_pose
from .humanoid_frames import body_from_arm, head_to_arm, requires_head


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"expected {size} finite numbers")
    return result


def basis(value, allow_reflection=False):
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.isfinite(result).all():
        raise ValueError("expected a finite 3x3 matrix")
    determinant = np.linalg.det(result)
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-8):
        raise ValueError("basis must be orthonormal")
    if not np.isclose(abs(determinant) if allow_reflection else determinant, 1):
        raise ValueError("unexpected basis handedness")
    return result


def load_config(path, arm_id=None):
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    if config.get("motion_output_enabled") is not False:
        raise ValueError("this program only supports motion_output_enabled=false")
    if config.get("orientation_mode") not in ("hold_initial_link7", "relative_link7") or config.get("gripper_mode") != "hold":
        raise ValueError("unsupported orientation or gripper mode")
    for name in ("translation_scale", "max_displacement_m", "max_tcp_speed_m_s",
                 "max_joint_speed_deg_s", "max_joint_step_deg", "max_joint_session_deg",
                 "position_tolerance_m", "orientation_tolerance_deg", "max_gap_s", "control_hz"):
        value = config[name]
        if value is None and name in ("max_displacement_m", "max_joint_session_deg"):
            continue
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if config["orientation_mode"] == "relative_link7":
        for name in ("max_angular_speed_deg_s", "max_rotation_session_deg", "rotation_scale"):
            value = config.get(name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
    if not 1 / config["control_hz"] < config["max_gap_s"]:
        raise ValueError("control interval must be shorter than the input timeout")
    basis(config["pose_basis_change"], allow_reflection=True)
    basis(config["controller_reference_to_base"])
    if config.get("mapping_mode") == "fixed_tracking_to_base":
        basis(config["tracking_to_base"])
    if config.get("mapping_mode") == "head_relative_tcp":
        basis(config["head_to_tcp_axes"])
    selected_arm_id = arm_id or config["arm_id"]
    if "humanoid_frames" in config:
        frames = json.loads((path.parent / config["humanoid_frames"]).read_text())
        config["root_to_body_rotation"] = basis(frames["root_to_body_rotation"]).tolist()
        config["humanoid_mounts"] = frames["mounts"]
        for arm_id in ("left_arm", "right_arm"):
            mount = body_from_arm(config, arm_id)
            basis(mount.rotation)
            vector(mount.translation, 3)
        config["humanoid_mount"] = config["humanoid_mounts"][selected_arm_id]
    if config.get("mapping_mode") == "head_relative_body":
        mount = body_from_arm(config)
        basis(mount.rotation)
        vector(mount.translation, 3)
    robot_config = json.loads((path.parent / config["robot_config"]).read_text())
    if selected_arm_id not in robot_config["arms"]:
        raise ValueError(f"unknown arm configuration: {selected_arm_id}")
    arm = robot_config["arms"][selected_arm_id]
    if arm.get("enabled") is not True:
        raise ValueError(f"configured arm is disabled: {selected_arm_id}")
    expected_hand = "left" if selected_arm_id == "left_arm" else "right"
    if arm.get("controller_hand") != expected_hand:
        raise ValueError(f"{selected_arm_id} must use the {expected_hand} controller")
    tool = arm["tcp_point_preview"]
    if tool["parent_frame"] != "sdk_flange_link7":
        raise ValueError("TCP must be expressed in SDK link7")
    config["tcp_offset_m"] = vector(tool["point_in_parent_m"], 3).tolist()
    config["arm_id"] = selected_arm_id
    config["arm"] = arm
    config["urdf"] = str((path.parent / config["urdf"]).resolve())
    config["replay_seed_report"] = str((path.parent / config["replay_seed_report"]).resolve())
    if "home_poses" in config:
        from .nero_home import load_home_pose
        config["home_pose"] = load_home_pose(path.parent / config["home_poses"], config)
    return config


def apply_mapping_candidate(config, candidate_path):
    """Attach a direction-capture candidate without authorizing CAN output."""
    candidate_path = Path(candidate_path).resolve()
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    if (candidate.get("read_only") is not True
            or candidate.get("real_motion_allowed") is not False
            or candidate.get("active_configuration_updated") is not False
            or candidate.get("motion_targets_generated") is not False
            or candidate.get("mapping_status") != "offline_candidate_only"):
        raise ValueError("mapping candidate is not a read-only offline candidate")
    mapping = np.asarray(candidate.get("mapping_matrix_source_to_right_arm_base"), dtype=float)
    directions_path = Path(candidate.get("directions_report", ""))
    if not directions_path.is_absolute():
        directions_path = candidate_path.parent / directions_path
    directions = json.loads(directions_path.read_text(encoding="utf-8"))
    source_basis = np.asarray(
        directions.get("orthogonalized_source_basis_columns_up_outward_left"), dtype=float)
    basis(mapping)
    # PICO's measured source basis may be left-handed; the destination robot
    # base mapping remains a proper rotation and is checked above.
    basis(source_basis, allow_reflection=True)
    config["mapping_mode"] = "fixed_tracking_to_base"
    config["tracking_to_base"] = mapping.tolist()
    config["mapping_candidate_path"] = str(candidate_path)
    config["mapping_matrix_source_to_right_arm_base"] = mapping.tolist()
    config["mapping_source_basis"] = source_basis.tolist()
    config["mapping_status"] = candidate.get("mapping_status")
    return config


class NeroKinematics:
    """Joint validation and FK without constructing an optimization solver."""

    def __init__(self, config):
        self.config = config
        self.model = pin.buildModelFromUrdf(config["urdf"])
        if self.model.nq != 7 or list(self.model.names)[1:] != [f"joint{i}" for i in range(1, 8)]:
            raise ValueError("expected the official Nero seven-joint model")
        # CAN encodes integer millidegrees. Round each model boundary inward so
        # a valid floating-point IK result cannot round outside that boundary.
        self.command_lower_mdeg = np.ceil(np.rad2deg(self.model.lowerPositionLimit) * 1000).astype(np.int64)
        self.command_upper_mdeg = np.floor(np.rad2deg(self.model.upperPositionLimit) * 1000).astype(np.int64)
        self.command_lower_mdeg += (np.deg2rad(self.command_lower_mdeg / 1000) < self.model.lowerPositionLimit)
        self.command_upper_mdeg -= (np.deg2rad(self.command_upper_mdeg / 1000) > self.model.upperPositionLimit)
        self.command_lower = np.deg2rad(self.command_lower_mdeg / 1000)
        self.command_upper = np.deg2rad(self.command_upper_mdeg / 1000)
        if np.any(self.command_lower >= self.command_upper):
            raise ValueError("model joint limits contain no CAN command interval")
        link7 = self.model.getFrameId("link7")
        self.link7 = link7
        parent = self.model.frames[link7]
        self.tcp = self.model.addFrame(pin.Frame(
            "teleop_tcp", parent.parentJoint, link7,
            parent.placement * pin.SE3(np.eye(3), vector(config["tcp_offset_m"], 3)),
            pin.FrameType.OP_FRAME))
        self.data = self.model.createData()

    def validate_joints(self, q):
        q = vector(q, 7)
        outside = np.flatnonzero((q < self.model.lowerPositionLimit) | (q > self.model.upperPositionLimit))
        if outside.size:
            detail = "; ".join(
                f"J{i + 1}={math.degrees(q[i]):.9f} deg, "
                f"allowed=[{math.degrees(self.model.lowerPositionLimit[i]):.9f}, "
                f"{math.degrees(self.model.upperPositionLimit[i]):.9f}] deg" for i in outside)
            raise ValueError(f"joint value exceeds official Nero model limits: {detail}")
        return q

    def quantize_command(self, q):
        """Encode a valid model target on the CAN grid without expanding limits."""
        q = self.validate_joints(q)
        millidegrees = np.clip(np.rint(np.rad2deg(q) * 1000),
                               self.command_lower_mdeg, self.command_upper_mdeg)
        return self.validate_joints(np.deg2rad(millidegrees / 1000))

    def validate_feedback_joints(self, q):
        """Encoder readings are observations, not targets constrained by URDF."""
        return vector(q, 7)

    def command_from_feedback(self, q):
        measured = self.validate_feedback_joints(q)
        bounded = np.clip(measured, self.command_lower, self.command_upper)
        correction_deg = float(np.max(np.abs(np.rad2deg(bounded - measured))))
        if correction_deg > self.config["max_joint_step_deg"] + 1e-10:
            raise ValueError(f"feedback cannot seed an in-limit command within the joint step limit: "
                             f"correction={correction_deg:.6f} deg, "
                             f"step_limit={self.config['max_joint_step_deg']:.6f} deg")
        return self.quantize_command(bounded)

    def pose(self, q, frame=None):
        pin.framesForwardKinematics(self.model, self.data, vector(q, 7))
        return self.data.oMf[self.tcp if frame is None else frame].copy()


class NeroIK(NeroKinematics):
    """Pinocchio/CasADi pose IK with seven-joint continuity bounds."""

    def __init__(self, config):
        super().__init__(config)
        cmodel = cpin.Model(self.model)
        cdata = cmodel.createData()
        q = ca.SX.sym("q", 7)
        cpin.framesForwardKinematics(cmodel, cdata, q)
        position = ca.SX.sym("position", 3)
        rotation = ca.SX.sym("rotation", 3, 3)
        # Holding orientation needs a smooth residual even at exactly zero error.
        error = ca.vertcat(cdata.oMf[self.tcp].translation - position,
                          ca.reshape(cdata.oMf[self.tcp].rotation - rotation, 9, 1))
        error_fn = ca.Function("nero_pose_error", [q, position, rotation], [error])
        tcp_fn = ca.Function("nero_tcp_position", [q], [cdata.oMf[self.tcp].translation])
        rotation_fn = ca.Function("nero_tcp_rotation", [q], [cdata.oMf[self.tcp].rotation])
        self.opti = ca.Opti()
        self.q = self.opti.variable(7)
        self.position = self.opti.parameter(3)
        self.rotation = self.opti.parameter(3, 3)
        self.previous = self.opti.parameter(7)
        self.lower = self.opti.parameter(7)
        self.upper = self.opti.parameter(7)
        self.previous_tcp = self.opti.parameter(3)
        self.tcp_step = self.opti.parameter()
        residual = error_fn(self.q, self.position, self.rotation)
        self.opti.minimize(1e6 * ca.sumsqr(residual[:3]) + 5e3 * ca.sumsqr(residual[3:])
                           + 0.05 * ca.sumsqr(self.q - self.previous))
        self.opti.subject_to(self.opti.bounded(self.lower, self.q, self.upper))
        # Bound FK motion inside the optimizer, including correction of prior IK error.
        self.opti.subject_to(ca.sumsqr((tcp_fn(self.q) - self.previous_tcp) / self.tcp_step) <= 1)
        # ||R-R0||_F^2 = 8 sin(theta/2)^2. Keep the hold tolerance in the solve,
        # with a small numerical margin for the independent angular postcheck.
        orientation_bound = 8 * math.sin(math.radians(config["orientation_tolerance_deg"] * .99) / 2) ** 2
        self.opti.subject_to(ca.sumsqr(residual[3:]) / orientation_bound <= 1)
        if config.get("orientation_mode") == "relative_link7":
            self.previous_rotation = self.opti.parameter(3, 3)
            self.session_rotation = self.opti.parameter(3, 3)
            self.rotation_step_bound = self.opti.parameter()
            solved_rotation = rotation_fn(self.q)
            angular_step = ca.reshape(solved_rotation - self.previous_rotation, 9, 1)
            self.opti.subject_to(ca.sumsqr(angular_step) / self.rotation_step_bound <= 1)
            angular_travel = ca.reshape(solved_rotation - self.session_rotation, 9, 1)
            session_bound = 8 * math.sin(math.radians(config["max_rotation_session_deg"]) / 2) ** 2
            self.opti.subject_to(ca.sumsqr(angular_travel) / session_bound <= 1)
        self.opti.solver("ipopt", {"print_time": False,
                         "ipopt": {"print_level": 0, "sb": "yes", "max_iter": 50,
                                   "tol": 1e-7, "bound_relax_factor": 0., "max_cpu_time": 0.08}})

    def solve(self, position, rotation, previous, session_seed, dt):
        started = time.perf_counter()
        previous = self.validate_joints(previous)
        session_seed = self.validate_joints(session_seed)
        position = vector(position, 3)
        rotation = basis(rotation)
        if not math.isfinite(dt) or not 0 < dt <= self.config["max_gap_s"]:
            return {"ok": False, "reason": "invalid_ik_interval", "joints_rad": None}
        step = math.radians(min(self.config["max_joint_step_deg"],
                                self.config["max_joint_speed_deg_s"] * dt))
        lower = np.maximum(self.model.lowerPositionLimit, previous - step)
        upper = np.minimum(self.model.upperPositionLimit, previous + step)
        if self.config["max_joint_session_deg"] is not None:
            travel = math.radians(self.config["max_joint_session_deg"])
            lower = np.maximum(lower, session_seed - travel)
            upper = np.minimum(upper, session_seed + travel)
        if np.any(lower >= upper):
            return {"ok": False, "reason": "empty_joint_bounds", "joints_rad": None}
        for parameter, value in ((self.position, position), (self.rotation, rotation),
                                 (self.previous, previous), (self.lower, lower), (self.upper, upper),
                                 (self.previous_tcp, self.pose(previous).translation),
                                 (self.tcp_step, self.config["max_tcp_speed_m_s"] * dt)):
            self.opti.set_value(parameter, value)
        self.opti.set_initial(self.q, previous)
        if self.config.get("orientation_mode") == "relative_link7":
            angle = math.radians(self.config["max_angular_speed_deg_s"]) * dt
            self.opti.set_value(self.previous_rotation, self.pose(previous).rotation)
            self.opti.set_value(self.session_rotation, self.pose(session_seed).rotation)
            self.opti.set_value(self.rotation_step_bound, 8 * math.sin(angle * .99 / 2) ** 2)
        try:
            solution = self.opti.solve()
            q = np.asarray(solution.value(self.q)).reshape(7)
            if not self.opti.stats().get("success") or not np.isfinite(q).all():
                raise ValueError("solver did not report a finite successful solution")
            pose = self.pose(q)
            pos_error = float(np.linalg.norm(pose.translation - position))
            ori_error = float(np.linalg.norm(pin.log3(rotation.T @ pose.rotation)))
            achieved_step = float(np.linalg.norm(pose.translation - self.pose(previous).translation))
            checks = {"joint_bounds": bool(np.all(q >= lower - 1e-8) and np.all(q <= upper + 1e-8)),
                      "position_error": pos_error <= self.config["position_tolerance_m"],
                      "orientation_error": ori_error <= math.radians(self.config["orientation_tolerance_deg"]),
                      "tcp_speed": achieved_step <= self.config["max_tcp_speed_m_s"] * dt + 1e-8}
            if self.config.get("orientation_mode") == "relative_link7":
                checks["angular_speed"] = (np.linalg.norm(pin.log3(self.pose(previous).rotation.T @ pose.rotation))
                                           <= math.radians(self.config["max_angular_speed_deg_s"]) * dt + 1e-8)
                checks["angular_travel"] = (np.linalg.norm(pin.log3(self.pose(session_seed).rotation.T @ pose.rotation))
                                            <= math.radians(self.config["max_rotation_session_deg"]) + 1e-8)
            failed_checks = [name for name, passed in checks.items() if not passed]
            ok = not failed_checks
            return {"ok": bool(ok), "reason": "solved" if ok else "pose_or_joint_limit_rejected",
                    "failed_checks": failed_checks,
                    "joints_rad": q.tolist() if ok else None,
                    "position_error_m": pos_error, "orientation_error_deg": math.degrees(ori_error),
                    "achieved_tcp_speed_m_s": achieved_step / dt,
                    "solve_ms": (time.perf_counter() - started) * 1000}
        except (RuntimeError, ValueError) as exc:
            return {"ok": False, "reason": "ik_failed", "joints_rad": None,
                    "detail": str(exc).splitlines()[-1],
                    "solve_ms": (time.perf_counter() - started) * 1000}


class RelativeTeleop:
    """Read-only virtual arm, initially seeded by a real or historical snapshot."""

    def __init__(self, config, seed, ik=None):
        self.config = config
        self.ik = ik or NeroIK(config)
        self.seed = self.ik.validate_joints(seed).copy()
        self.q = self.seed.copy()
        initial = self.ik.pose(self.seed)
        self.origin = initial.translation.copy()
        self.target = self.origin.copy()
        self.rotation = initial.rotation.copy()
        self.origin_rotation = self.rotation.copy()
        self.anchor_rotation = None
        self.head_anchor_rotation = None
        self.head_tracking_to_base = None
        self.mapping_mode = config.get("mapping_mode", "legacy_reference_mapping")
        self.head_to_tcp_axes = (basis(config["head_to_tcp_axes"])
                                 if self.mapping_mode == "head_relative_tcp" else None)
        self.guard = InputGuard(stale_s=config["max_gap_s"],
                               require_head_pose=requires_head(config))
        self.pose_basis = basis(config["pose_basis_change"], allow_reflection=True)
        self.mapping = basis(config["controller_reference_to_base"])
        self.tracking_to_base = (basis(config["tracking_to_base"])
                                 if self.mapping_mode == "fixed_tracking_to_base" else None)
        self.mapping_candidate = (basis(config["mapping_matrix_source_to_right_arm_base"])
                                  if self.mapping_mode == "direction_candidate_source_basis" else None)
        self.mapping_source_basis = (basis(config["mapping_source_basis"], allow_reflection=True)
                                     if self.mapping_mode == "direction_candidate_source_basis" else None)
        self.anchor = None
        self.anchor_target = None
        self.last_received = self.last_processed = None
        self.generation = 0
        self.last_ik = None

    def limit_requested_target(self, requested):
        return requested

    def snapshot(self, state, reason, **extra):
        return {"state": state, "reason": reason, "arm_id": self.config["arm_id"],
                "guard": self.guard.snapshot(), "anchor_generation": self.generation,
                "tcp_target_base_m": self.target.tolist(),
                "ik_tcp_base_m": self.ik.pose(self.q).translation.tolist(),
                "tcp_displacement_base_mm": ((self.target - self.origin) * 1000).tolist(),
                "joint_target_rad": self.q.tolist(), "joint_target_deg": np.rad2deg(self.q).tolist(),
                "orientation_mode": self.config["orientation_mode"], "gripper_target": None,
                "tcp_target_rotation": self.rotation.tolist(),
                "mapping_status": self.config["mapping_status"], "ik": self.last_ik,
                "mapping_mode": self.mapping_mode,
                "clutch_tracking_to_base": (self.head_tracking_to_base.tolist()
                                            if self.head_tracking_to_base is not None else None),
                "collision_checked": False, "read_only": True,
                "real_motion_allowed": False, "can_commands_sent": 0, **extra}

    def hold(self, reason):
        self.anchor = None
        self.head_anchor_rotation = None
        self.head_tracking_to_base = None
        self.last_received = self.last_processed = None
        return self.snapshot("HOLD", reason)

    def process(self, event):
        now = event["processed_monotonic"]
        kind = event["kind"]
        if kind == "tracking":
            guard = self.guard.step(event["tracking"], event["received_monotonic"], now, event["device_id"])
        elif kind == "tick":
            guard = self.guard.tick(now)
        else:
            guard = self.guard.invalidate(kind, disconnected=kind == "disconnect")
        if not guard["input_held"]:
            return self.hold(guard["reason"])
        if kind != "tracking":
            return self.snapshot("WAIT_SAMPLE", "no_new_tracking")
        return self.process_pose(event)

    def process_pose(self, event):
        """Process a tracking pose after the caller has applied InputGuard."""
        now = event["processed_monotonic"]
        hand = self.config.get("arm", {}).get("controller_hand", "right")
        pose = valid_pose(event["tracking"]["Controller"][hand]["pose"])
        received = event["received_monotonic"]
        head = None
        if requires_head(self.config):
            head = valid_head_pose(event["tracking"])
            if head is None:
                self.guard.invalidate("head_pose_invalid")
                return self.hold("head_pose_invalid")
        if self.anchor is None:
            self.anchor = np.asarray(pose)
            self.anchor_target = self.target.copy()
            self.anchor_rotation = self.rotation.copy()
            if requires_head(self.config):
                head_quat = np.asarray(head[3:]) / np.linalg.norm(head[3:])
                self.head_anchor_rotation = pin.Quaternion(head_quat[3], *head_quat[:3]).matrix()
                self.anchor_rotation = self.ik.pose(self.q).rotation.copy()
                try:
                    self.head_tracking_to_base = (head_to_arm(self.config, self.head_anchor_rotation)
                        if self.mapping_mode == "head_relative_body" else
                        self.anchor_rotation @ self.head_to_tcp_axes @ self.head_anchor_rotation.T)
                except ValueError as exc:
                    self.guard.invalidate(str(exc))
                    return self.hold(str(exc))
            self.last_received, self.last_processed = received, now
            self.generation += 1
            return self.snapshot("ANCHOR", "current_virtual_arm_and_controller_reference")
        received_dt = received - self.last_received
        processed_dt = now - self.last_processed
        if self.config.get("latest_sample_control", False):
            if received_dt <= 0 or processed_dt <= 0:
                return self.snapshot("WAIT_SAMPLE", "no_new_tracking")
            dt = processed_dt
        else:
            if min(received_dt, processed_dt) < 1 / self.config["control_hz"]:
                return self.snapshot("WAIT_SAMPLE", "control_rate_limit")
            dt = min(received_dt, processed_dt)
        if max(received_dt, processed_dt) > self.config["max_gap_s"]:
            self.guard.invalidate("control_gap_requires_release")
            return self.hold("control_gap_requires_release")
        # Cap delayed progress; streaming uses elapsed time within a short budget.
        dt = min(dt, self.config.get("max_step_interval_s", dt))
        self.last_received, self.last_processed = received, now
        quat = self.anchor[3:] / np.linalg.norm(self.anchor[3:])
        ref_rotation = pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).matrix()
        # This is the translation of T_reference.inverse() * T_current.
        raw_delta = np.asarray(pose[:3]) - self.anchor[:3]
        if self.mapping_mode == "fixed_tracking_to_base":
            requested = self.anchor_target + self.config["translation_scale"] * self.tracking_to_base @ raw_delta
        elif requires_head(self.config):
            # The clutch mapping is frozen: body axes or grip-time TCP axes.
            requested = self.anchor_target + self.config["translation_scale"] * self.head_tracking_to_base @ raw_delta
        elif self.mapping_mode == "direction_candidate_source_basis":
            semantic_delta = self.mapping_source_basis.T @ raw_delta
            requested = (self.anchor_target + self.config["translation_scale"]
                         * self.mapping_candidate @ semantic_delta)
        else:
            delta_local = self.pose_basis @ ref_rotation.T @ raw_delta
            requested = self.anchor_target + self.config["translation_scale"] * self.mapping @ delta_local
        requested = self.limit_requested_target(requested)
        radius = self.config["max_displacement_m"]
        if radius is not None and np.linalg.norm(requested - self.origin) > radius:
            self.guard.invalidate("workspace_limit_requires_release")
            return self.hold("workspace_limit_requires_release")
        delta = requested - self.target
        distance = np.linalg.norm(delta)
        target = self.target + delta * min(1., self.config["max_tcp_speed_m_s"] * dt / distance) if distance else self.target.copy()
        rotation = self.rotation
        if self.config.get("orientation_mode") == "relative_link7":
            current_quat = np.asarray(pose[3:]) / np.linalg.norm(pose[3:])
            current_rotation = pin.Quaternion(current_quat[3], *current_quat[:3]).matrix()
            if self.mapping_mode in ("fixed_tracking_to_base", "head_relative_tcp", "head_relative_body"):
                mapping = (self.head_tracking_to_base if requires_head(self.config)
                           else self.tracking_to_base)
                # Spatial delta is current * reference^-1, expressed in arm base.
                relative = mapping @ (current_rotation @ ref_rotation.T) @ mapping.T
            else:
                mapping = self.mapping @ self.pose_basis
                relative = mapping @ (ref_rotation.T @ current_rotation) @ mapping.T
            requested_rotation = pin.exp3(pin.log3(relative) * self.config["rotation_scale"]) @ self.anchor_rotation
            from_origin = pin.log3(self.origin_rotation.T @ requested_rotation)
            angle = np.linalg.norm(from_origin)
            limit = math.radians(self.config["max_rotation_session_deg"])
            if angle > limit:
                requested_rotation = self.origin_rotation @ pin.exp3(from_origin * (limit / angle))
            step_rotation = pin.log3(self.rotation.T @ requested_rotation)
            step_angle = np.linalg.norm(step_rotation)
            step_limit = math.radians(self.config["max_angular_speed_deg_s"]) * dt
            rotation = self.rotation @ pin.exp3(step_rotation * min(1., step_limit / step_angle)) if step_angle else self.rotation
        if np.linalg.norm(target - self.target) < 1e-10 and np.linalg.norm(rotation - self.rotation) < 1e-10:
            return self.snapshot("FOLLOW", "unchanged_target")
        self.last_ik = self.ik.solve(target, rotation, self.q, self.seed, dt)
        # A successful bounded solve can still miss the requested position.
        # Try less progress from the last accepted target, without relaxing limits.
        attempts = [self.last_ik]
        requested_step_target = target.copy()
        requested_step_rotation = rotation.copy()
        progress = 1.
        for progress_candidate in (.5, .25):
            if self.last_ik.get("failed_checks") != ["position_error"]:
                break
            target = self.target + progress_candidate * (requested_step_target - self.target)
            rotation = self.rotation @ pin.exp3(progress_candidate * pin.log3(self.rotation.T @ requested_step_rotation))
            self.last_ik = self.ik.solve(target, rotation, self.q, self.seed, dt)
            attempts.append(self.last_ik)
            progress = progress_candidate
        if len(attempts) > 1:
            self.last_ik = {**self.last_ik, "attempts": attempts,
                            "solve_ms": sum(item.get("solve_ms", 0.) for item in attempts),
                            "target_progress_fraction": progress}
        if not self.last_ik["ok"]:
            if self.last_ik.get("failed_checks") == ["position_error"]:
                return self.snapshot("LIMITED", "position_target_limited",
                                     requested_tcp_base_m=requested.tolist())
            self.guard.invalidate("ik_rejected_requires_release")
            return self.hold("ik_rejected_requires_release")
        self.target = target
        self.rotation = rotation
        self.q = np.asarray(self.last_ik["joints_rad"])
        if self.last_ik.get("limited", False):
            return self.snapshot("LIMITED", self.last_ik["reason"], requested_tcp_base_m=requested.tolist())
        return self.snapshot("FOLLOW", "relative_tcp_and_seven_joint_target" if progress == 1. else "reduced_target_progress",
                             requested_tcp_base_m=requested.tolist())
