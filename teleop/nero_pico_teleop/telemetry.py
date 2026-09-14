"""Optional local telemetry. Never reads hardware or sends robot commands."""

import hashlib
import json
import socket
import threading
import time
import uuid


class TelemetryPublisher:
    def __init__(self, outputs, path, hz=50.):
        self.outputs, self.path, self.hz = outputs, str(path), hz
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.stop = threading.Event()
        self.worker = None
        self.socket = None
        self.configurations = {name: {
            "can_channel": output.config["arm"]["can_channel"],
            "controller_hand": output.config["arm"]["controller_hand"],
            "tcp_offset_m": output.config["tcp_offset_m"],
            "mapping_mode": output.config.get("mapping_mode"),
            "config_sha256": hashlib.sha256(json.dumps(output.config, sort_keys=True).encode()).hexdigest(),
        } for name, output in outputs.items()}

    def snapshot(self):
        arms = {}
        for name, output in self.outputs.items():
            # Skip a busy sender; camera/collector work cannot hold the control lock.
            if not output.lock.acquire(blocking=False):
                return None
            try:
                now, wall = time.monotonic(), time.time()
                feedback = output.last_feedback_snapshot
                source = None
                if output.input_source is not None:
                    if not output.input_source.lock.acquire(blocking=False):
                        return None
                    try:
                        source = output.input_source.snapshot()
                    finally:
                        output.input_source.lock.release()
                guard = source["guard"] if source else None
                healthy = bool(guard and guard.last_valid_received is not None
                    and 0 <= now - guard.last_valid_received <= output.config["max_gap_s"]
                    and not source["terminal_reason"] and not source["recovery_reason"]
                    and not guard.paused and not guard.reference_requests
                    and guard.state in ("READY_IDLE", "WAIT_RELEASE", "INPUT_HELD"))
                gripper_stamp = output.gripper_feedback_timestamp_s
                arms[name] = {
                    "observed_monotonic": now,
                    "state_joints_rad": list(feedback["joints_rad"]) if feedback else None,
                    "state_gripper_m": output.gripper_measured,
                    "feedback_monotonic": now - (wall - min(feedback["group_timestamps_s"])) if feedback else None,
                    "gripper_feedback_monotonic": now - (wall - gripper_stamp) if gripper_stamp is not None else None,
                    "action_joints_rad": output.last_target.tolist() if output.last_target is not None else None,
                    "action_gripper_m": output.gripper_target,
                    "gripper_frames_sent": output.gripper_frames,
                    "action_gripper_initialized_from_feedback": output.gripper_frames == 0,
                    "last_command_monotonic": output.last_sent,
                    "target_batches_sent": output.targets_sent,
                    "output_state": output.state,
                    "ready": output.teleop_ready,
                    "home_state": output.home_state,
                    "input_healthy": healthy,
                    "clutch_held": bool(source and source["held"]),
                    "clutch_epoch": source["epoch"] if source else None,
                    "gripper_enabled": output.gripper is not None,
                }
            finally:
                output.lock.release()
        self.sequence += 1
        return {"schema_version": 1, "session_id": self.session_id, "sequence": self.sequence,
                "monotonic": time.monotonic(), "wall_time_ns": time.time_ns(),
                "configurations": self.configurations, "arms": arms}

    def __enter__(self):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

        def run():
            while not self.stop.is_set():
                try:
                    packet = self.snapshot()
                    if packet is not None:
                        self.socket.sendto(json.dumps(packet, allow_nan=False).encode(), self.path)
                except (OSError, ValueError, TypeError):
                    # Receiver absent/full: drop telemetry, never delay robot output.
                    pass
                self.stop.wait(1. / self.hz)

        self.worker = threading.Thread(target=run, name="nero-telemetry", daemon=True)
        self.worker.start()
        return self

    def __exit__(self, *_args):
        self.stop.set()
        self.worker.join(timeout=1.)
        self.socket.close()
