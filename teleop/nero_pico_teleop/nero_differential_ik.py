"""Bounded differential IK using Pinocchio and SciPy's dense BVLS solver."""

import math
import time

import numpy as np
import pinocchio as pin
from scipy.optimize import lsq_linear

from .nero_relative_core import NeroKinematics, basis, vector


class NeroDifferentialIK(NeroKinematics):
    """Solve one local pose increment, then check its nonlinear FK before use."""

    def solve(self, position, rotation, previous, session_seed, dt):
        started = time.perf_counter()
        previous = self.validate_joints(previous)
        session_seed = self.validate_joints(session_seed)
        position, rotation = vector(position, 3), basis(rotation)
        if not math.isfinite(dt) or not 0 < dt <= self.config["max_gap_s"]:
            raise ValueError("invalid differential IK interval")
        initial, origin = self.pose(previous), self.pose(session_seed)
        step = math.radians(min(self.config["max_joint_step_deg"], self.config["max_joint_speed_deg_s"] * dt))
        lower = np.maximum(self.command_lower, previous - step)
        upper = np.minimum(self.command_upper, previous + step)
        if self.config["max_joint_session_deg"] is not None:
            travel = math.radians(self.config["max_joint_session_deg"])
            lower = np.maximum(lower, session_seed - travel)
            upper = np.minimum(upper, session_seed + travel)
        if np.any(lower >= upper):
            raise ValueError("empty differential IK joint bounds")

        # LOCAL_WORLD_ALIGNED gives TCP linear and angular velocities in base axes.
        jacobian = pin.computeFrameJacobian(self.model, self.data, previous, self.tcp,
                                           pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        weights = np.array([1., 1., 1., .1, .1, .1])
        relative = self.config["orientation_mode"] == "relative_link7"
        # Translation-only commissioning previews may explicitly prioritize TCP
        # position while keeping orientation inside the configured tolerance.
        position_priority = self.config.get("ik_priority") == "translation"
        priority_slack = self.config.get("translation_priority_slack_m", .00005)

        def residual(pose):
            return weights * np.concatenate((position - pose.translation,
                                              pin.log3(rotation @ pose.rotation.T)))

        error = residual(initial)
        iterations = 0

        def bounded_increment(matrix, rhs):
            nonlocal iterations
            result = lsq_linear(np.vstack((matrix, np.eye(7) * .001)),
                                np.concatenate((rhs, np.zeros(7))),
                                bounds=(lower - previous, upper - previous),
                                method="bvls", tol=1e-10, max_iter=20)
            if not result.success or not np.isfinite(result.x).all():
                raise RuntimeError(f"differential IK failed: {result.message}")
            iterations += int(result.nit or 0)
            return np.clip(result.x, lower - previous, upper - previous)

        orientation_fraction = 1.
        if position_priority:
            # First reserve the attainable translation. Let orientation change
            # that linear TCP increment by at most the explicit position slack.
            primary = bounded_increment(jacobian[:3], position - initial.translation)
            secondary_weights = np.array([10., 10., 10., .1, .1, .1])
            secondary = bounded_increment(secondary_weights[:, None] * jacobian,
                                          secondary_weights * np.concatenate((
                                              jacobian[:3] @ primary,
                                              pin.log3(rotation @ initial.rotation.T))))
            position_change = float(np.linalg.norm(jacobian[:3] @ (secondary - primary)))
            if position_change > priority_slack:
                orientation_fraction = priority_slack / position_change
            increment = primary + orientation_fraction * (secondary - primary)
        else:
            increment = bounded_increment(weights[:, None] * jacobian, error)
        candidate = np.clip(previous + increment, lower, upper)

        rotation_limit = math.radians(self.config.get("max_rotation_session_deg",
                                                     self.config["orientation_tolerance_deg"]))
        initial_rotation_error = float(np.linalg.norm(pin.log3(origin.rotation.T @ initial.rotation)))
        radius = self.config["max_displacement_m"]
        if radius is not None:
            radius = max(radius, float(np.linalg.norm(initial.translation - origin.translation)))
        # Previous millidegree rounding can lie just outside the generator's bound.
        # Permit convergence back inside, without increasing that existing error.
        rotation_limit = max(rotation_limit, initial_rotation_error)
        chosen, achieved, fraction = previous.copy(), initial, 0.
        scale = 1.
        for _ in range(8):
            q = np.clip(previous + scale * (candidate - previous), lower, upper)
            pose = self.pose(q)
            tcp_step = float(np.linalg.norm(pose.translation - initial.translation))
            angular_step = float(np.linalg.norm(pin.log3(initial.rotation.T @ pose.rotation)))
            tcp_budget = self.config["max_tcp_speed_m_s"] * dt
            angular_budget = math.radians(self.config["max_angular_speed_deg_s"]) * dt if relative else math.inf
            if tcp_step > tcp_budget + 1e-10 or angular_step > angular_budget + 1e-10:
                scale *= .995 * min(1., tcp_budget / max(tcp_step, 1e-15),
                                    angular_budget / max(angular_step, 1e-15))
                continue
            if (np.any(q < lower - 1e-10) or np.any(q > upper + 1e-10)
                    or (radius is not None and np.linalg.norm(pose.translation - origin.translation) > radius + 1e-10)
                    or np.linalg.norm(pin.log3(origin.rotation.T @ pose.rotation)) > rotation_limit + 1e-10):
                scale *= .5
                continue
            if position_priority:
                primary_pose = self.pose(previous + scale * primary)
                position_error = float(np.linalg.norm(position - pose.translation))
                position_budget = min(float(np.linalg.norm(position - primary_pose.translation)) + priority_slack,
                                      max(float(np.linalg.norm(position - initial.translation)), priority_slack))
                worsened = position_error > position_budget + 1e-10
            else:
                remaining = residual(pose)
                worsened = float(remaining @ remaining) > float(error @ error) + 1e-14
            if worsened:
                scale *= .5
                continue
            chosen, achieved, fraction = q, pose, scale
            break

        position_error = float(np.linalg.norm(position - achieved.translation))
        orientation_error = math.degrees(float(np.linalg.norm(pin.log3(rotation.T @ achieved.rotation))))
        limited = (fraction < 1. or position_error > self.config["position_tolerance_m"]
                   or orientation_error > self.config["orientation_tolerance_deg"])
        return {"ok": True, "reason": "bounded_differential_progress" if limited else "differential_solved",
                "solver": "pinocchio_scipy_bvls", "limited": limited, "failed_checks": [],
                "joints_rad": chosen.tolist(), "position_error_m": position_error,
                "orientation_error_deg": orientation_error, "step_fraction": fraction,
                "priority": "translation" if position_priority else "pose",
                "translation_priority_slack_m": priority_slack if position_priority else None,
                "orientation_increment_fraction": orientation_fraction,
                "joint_limit_active": (np.flatnonzero(np.minimum(chosen - self.command_lower,
                                                                 self.command_upper - chosen)
                                                      <= math.radians(.002)) + 1).tolist(),
                "iterations": iterations,
                "achieved_tcp_speed_m_s": float(np.linalg.norm(achieved.translation - initial.translation)) / dt,
                "solve_ms": (time.perf_counter() - started) * 1000}
