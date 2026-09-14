"""Compute-only Placo process. No XR SDK, robot driver or CAN socket is loaded.

Frame and posture tasks follow moonbot_isaacsim/nero_vr_control's Placo setup.
Uses a separate interpreter because its Pinocchio ABI differs from CasADi's.
"""

import math
from multiprocessing.connection import Connection
import signal
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import placo


class PlacoSolver:
    def __init__(self, config, directory):
        self.config = config
        root = ET.parse(config["urdf"]).getroot()
        for link in root.findall("link"):
            for element in list(link):
                if element.tag in ("visual", "collision"):
                    link.remove(element)
        ET.SubElement(root, "link", name="teleop_tcp")
        joint = ET.SubElement(root, "joint", name="teleop_tcp_fixed", type="fixed")
        ET.SubElement(joint, "parent", link="link7")
        ET.SubElement(joint, "child", link="teleop_tcp")
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, config["tcp_offset_m"])), rpy="0 0 0")
        path = Path(directory) / "nero_ik.urdf"
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
        self.robot = placo.RobotWrapper(str(path))
        self.names = [f"joint{i}" for i in range(1, 8)]
        if set(self.robot.joint_names()) != set(self.names):
            raise ValueError("Placo model must contain exactly the seven Nero joints")
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)
        self.solver.enable_velocity_limits(True)
        self.robot.update_kinematics()
        self.task = self.solver.add_frame_task("teleop_tcp", self.robot.get_T_world_frame("teleop_tcp"))
        self.task.position().configure("tcp_position", "soft", config.get("placo_position_weight", 1.))
        # Holding orientation is an equality in the solve, not a rejected
        # position-only step after it has reached an orientation boundary.
        holding = config["orientation_mode"] == "hold_initial_link7"
        self.task.orientation().configure("tcp_orientation", "hard" if holding else "soft",
                                          config.get("placo_orientation_weight", .45))
        self.posture = self.solver.add_joints_task()
        self.posture.configure("joint_continuity", "soft", config.get("placo_posture_weight", .001))
        self.lower = np.asarray(config["command_lower"])
        self.upper = np.asarray(config["command_upper"])

    def solve(self, position, rotation, previous, session_seed, dt):
        started = time.monotonic()
        previous, seed = np.asarray(previous), np.asarray(session_seed)
        if not math.isfinite(dt) or not 0 < dt <= self.config["max_gap_s"]:
            raise ValueError("invalid Placo control interval")
        step = math.radians(min(self.config["max_joint_step_deg"], self.config["max_joint_speed_deg_s"] * dt))
        lower, upper = self.lower.copy(), self.upper.copy()
        travel = self.config["max_joint_session_deg"]
        if travel is not None:
            lower = np.maximum(lower, seed - math.radians(travel))
            upper = np.minimum(upper, seed + math.radians(travel))
        for i, name in enumerate(self.names):
            self.robot.set_joint(name, float(previous[i]))
            self.robot.set_joint_limits(name, float(lower[i]), float(upper[i]))
            self.robot.set_velocity_limit(name, step / dt)
        self.robot.update_kinematics()
        self.posture.set_joints(dict(zip(self.names, previous.tolist())))
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotation, position
        self.task.T_world_frame = target
        self.solver.dt = dt
        self.solver.solve(True)
        self.robot.update_kinematics()
        return {"joints_rad": [self.robot.get_joint(name) for name in self.names],
                "tcp_transform": self.robot.get_T_world_frame("teleop_tcp").tolist(),
                "worker_solve_ms": (time.monotonic() - started) * 1000}


def main():
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    connection = Connection(int(sys.argv[1]))
    try:
        config = connection.recv()
        with tempfile.TemporaryDirectory(prefix="nero-placo-") as directory:
            solver = PlacoSolver(config, directory)
            connection.send({"kind": "ready"})
            while True:
                request = connection.recv()
                if request is None:
                    break
                request_id, arguments = request
                result = solver.solve(**arguments)
                connection.send({"kind": "result", "id": request_id, "result": result})
    except EOFError:
        pass
    except Exception as exc:
        connection.send({"kind": "error", "error": str(exc)})
    finally:
        connection.close()


if __name__ == "__main__":
    main()
