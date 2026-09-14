"""Placo IPC adapter with independent Pinocchio FK and output-bound checks."""

import math
import multiprocessing
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import pinocchio as pin

from .nero_relative_core import NeroKinematics, basis, vector
from .paths import PROJECT_ROOT, environment_root


class ProcessPlacoIK(NeroKinematics):
    def __init__(self, config, *, stopping=lambda: False):
        super().__init__(config)
        self.stopping = stopping
        self.process = self.connection = None
        self.request_id = 0
        self.solve_timeout_s = .15
        self.failed = False

    def __enter__(self):
        executable = Path(os.environ.get("NERO_PLACO_PYTHON", environment_root() / "placo/bin/python"))
        if not executable.is_file():
            raise RuntimeError("Placo runtime missing; run bash scripts/setup_placo.sh")
        self.connection, child = multiprocessing.Pipe()
        env = {k: v for k, v in os.environ.items() if k not in ("LD_LIBRARY_PATH", "PYTHONPATH")}
        env.update(PYTHONPATH=str(PROJECT_ROOT), PYTHONNOUSERSITE="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        try:
            self.process = subprocess.Popen(
                [str(executable), "-m", "nero_pico_teleop.nero_placo_worker", str(child.fileno())],
                pass_fds=(child.fileno(),), env=env, cwd=PROJECT_ROOT)
            child.close()
            self.connection.send({**self.config, "command_lower": self.command_lower.tolist(),
                                  "command_upper": self.command_upper.tolist()})
            if self._receive(30.).get("kind") != "ready":
                raise RuntimeError("invalid Placo startup response")
            return self
        except BaseException:
            child.close()
            self.close()
            raise

    def _receive(self, timeout):
        deadline = time.monotonic() + timeout
        try:
            while not self.stopping():
                if time.monotonic() >= deadline:
                    raise RuntimeError("Placo worker timed out")
                if self.connection.poll(.005):
                    response = self.connection.recv()
                    if self.stopping() or time.monotonic() >= deadline:
                        raise RuntimeError("Placo result cancelled or expired")
                    if response.get("kind") == "error":
                        raise RuntimeError(f"Placo worker failed: {response['error']}")
                    return response
                if self.process.poll() is not None:
                    raise RuntimeError("Placo worker exited")
            raise RuntimeError("Placo solve cancelled")
        except (EOFError, OSError, RuntimeError):
            self.failed = True
            raise

    def solve(self, position, rotation, previous, session_seed, dt):
        started = time.monotonic()
        if self.failed or self.process is None or self.process.poll() is not None:
            raise RuntimeError("Placo worker is unavailable")
        previous, seed = self.validate_joints(previous), self.validate_joints(session_seed)
        position, rotation = vector(position, 3), basis(rotation)
        if not math.isfinite(dt) or not 0 < dt <= self.config["max_gap_s"]:
            raise ValueError("invalid Placo interval")
        self.request_id += 1
        self.connection.send((self.request_id, dict(position=position.tolist(), rotation=rotation.tolist(),
                              previous=previous.tolist(), session_seed=seed.tolist(), dt=dt)))
        response = self._receive(self.solve_timeout_s)
        if response.get("kind") != "result" or response.get("id") != self.request_id:
            self.failed = True
            raise RuntimeError("Placo response does not match request")
        result = response["result"]
        candidate = vector(result["joints_rad"], 7)
        # Independent FK catches different joint order, TCP or model conventions.
        actual = self.pose(candidate)
        if not np.allclose(actual.homogeneous, result["tcp_transform"], atol=1e-8, rtol=0):
            raise RuntimeError("Placo/Pinocchio FK mismatch")
        step = math.radians(min(self.config["max_joint_step_deg"], self.config["max_joint_speed_deg_s"] * dt))
        lower, upper = np.maximum(self.command_lower, previous - step), np.minimum(self.command_upper, previous + step)
        travel = self.config["max_joint_session_deg"]
        if travel is not None:
            lower, upper = np.maximum(lower, seed - math.radians(travel)), np.minimum(upper, seed + math.radians(travel))
        if np.any(candidate < lower - 1e-7) or np.any(candidate > upper + 1e-7):
            raise RuntimeError("Placo exceeded joint position or velocity bounds")
        candidate = np.clip(candidate, lower, upper)
        initial, origin = self.pose(previous), self.pose(seed)
        chosen, fraction = previous.copy(), 0.
        scale = 1.
        for _ in range(16):
            q = previous + scale * (candidate - previous)
            pose = self.pose(q)
            tcp_step = np.linalg.norm(pose.translation - initial.translation)
            angular_step = np.linalg.norm(pin.log3(initial.rotation.T @ pose.rotation))
            angular_budget = math.radians(self.config.get("max_angular_speed_deg_s", 180.)) * dt
            tcp_budget = self.config["max_tcp_speed_m_s"] * dt
            if tcp_step > tcp_budget + 1e-10 or angular_step > angular_budget + 1e-10:
                scale *= .99 * min(1., tcp_budget / max(tcp_step, 1e-15), angular_budget / max(angular_step, 1e-15))
                continue
            radius = self.config["max_displacement_m"]
            rotation_limit = self.config.get("max_rotation_session_deg", self.config["orientation_tolerance_deg"])
            if ((radius is not None and np.linalg.norm(pose.translation - origin.translation) > radius + 1e-10)
                    or np.linalg.norm(pin.log3(origin.rotation.T @ pose.rotation)) > math.radians(rotation_limit) + 1e-10):
                scale *= .5
                continue
            chosen, fraction = q, scale
            break
        achieved = self.pose(chosen)
        pos_error = float(np.linalg.norm(position - achieved.translation))
        ori_error = math.degrees(float(np.linalg.norm(pin.log3(rotation.T @ achieved.rotation))))
        limited = fraction < .999 or pos_error > self.config["position_tolerance_m"] or ori_error > self.config["orientation_tolerance_deg"]
        return {"ok": True, "reason": "placo_bounded_progress" if limited else "placo_solved",
                "solver": "placo", "limited": bool(limited), "failed_checks": [], "joints_rad": chosen.tolist(),
                "position_error_m": pos_error, "orientation_error_deg": ori_error, "step_fraction": fraction,
                "joint_limit_active": (np.flatnonzero(np.minimum(chosen - self.command_lower, self.command_upper - chosen)
                                                     < math.radians(.002)) + 1).tolist(),
                "worker_solve_ms": result["worker_solve_ms"], "solve_ms": (time.monotonic() - started) * 1000}

    def close(self):
        try:
            if self.process is not None:
                if self.process.poll() is None:
                    try:
                        self.connection.send(None)
                        self.process.wait(timeout=.2)
                    except (OSError, subprocess.TimeoutExpired):
                        self.process.terminate()
                        try:
                            self.process.wait(timeout=.5)
                        except subprocess.TimeoutExpired:
                            self.process.kill()
                            self.process.wait(timeout=.5)
        finally:
            if self.connection is not None:
                self.connection.close()
            self.process = self.connection = None

    def __exit__(self, *_args):
        self.close()
