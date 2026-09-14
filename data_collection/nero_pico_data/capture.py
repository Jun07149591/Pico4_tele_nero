"""Background capture and episode commands, independent of robot control."""

from concurrent.futures import Future
import queue
import threading
import time

from .quality import episode_path, summary, validate_episode, write_review
from .schema import capture_fps_limit
from .sync import Synchronizer, check_packet

LOOKAHEAD_S = .06


class CaptureController:
    def __init__(self, config, store, receiver, cameras, *, review_only=False):
        self.config, self.store, self.receiver, self.cameras = config, store, receiver, cameras
        self.review_only = review_only
        self.jobs = queue.Queue(maxsize=16)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.info = {"state": "idle", "frames": 0, "elapsed_s": 0., "last_episode": None, "error": None}
        self.worker = None

    def _update(self, **values):
        with self.lock:
            self.info.update(values)

    def status(self):
        packet = self.receiver.latest()
        now = time.monotonic()
        camera_status = {}
        for role, camera in self.cameras.items():
            frame = camera.latest_before(now)
            camera_status[role] = {"age_ms": (now - frame.monotonic) * 1000 if frame else None,
                                   "error": camera.error,
                                   "ready": not camera.error and frame is not None
                                   and 0 <= now - frame.monotonic <= self.config["max_camera_age_s"]}
        reason = None
        try:
            if self.review_only:
                raise ValueError("offline review")
            if self.receiver.error:
                raise ValueError(self.receiver.error)
            check_packet(packet, now, self.config)
            for role, status in camera_status.items():
                if not status["ready"]:
                    raise ValueError(f"camera {role} missing/stale: {status['error'] or ''}")
        except (ValueError, KeyError, TypeError) as exc:
            reason = str(exc)
        with self.lock:
            info = dict(self.info)
        return {**info, "ready": reason is None, "readiness_reason": reason,
                "review_only": self.review_only,
                "mode": self.config["mode"], "fps": self.config["fps"],
                "max_capture_fps": capture_fps_limit(self.config),
                "synthetic": self.store.metadata["synthetic"], "root": str(self.store.root),
                "cameras": list(self.cameras), "robot_age_ms": (now - packet["monotonic"]) * 1000 if packet else None,
                "camera_status": camera_status,
                "telemetry_socket": str(self.receiver.path),
                "arm_status": {name: {key: arm.get(key) for key in
                                      ("ready", "output_state", "home_state", "input_healthy", "clutch_held",
                                       "gripper_enabled", "state_gripper_m", "action_gripper_m")}
                               | {"feedback_age_ms": (now - arm["feedback_monotonic"]) * 1000
                                  if arm.get("feedback_monotonic") is not None else None,
                                  "gripper_feedback_age_ms": (now - arm["gripper_feedback_monotonic"]) * 1000
                                  if arm.get("gripper_feedback_monotonic") is not None else None}
                               for name, arm in packet["arms"].items()} if packet else {},
                "rejected_packets": self.receiver.rejected_packets}

    def command(self, command, **kwargs):
        if self.stop.is_set() or not self.worker or not self.worker.is_alive():
            raise RuntimeError("capture worker is stopped")
        future = Future()
        self.jobs.put_nowait((command, kwargs, future))
        return future.result(timeout=30.)

    def _finish(self, outcome, reason=""):
        self._update(state="saving")
        try:
            path = self.store.finish(outcome, reason)
        except Exception as exc:
            self._update(state="error", error=str(exc))
            raise
        self._update(state="idle", last_episode=path.name, error=reason or None)
        return summary(path)

    def __enter__(self):
        def run():
            synchronizer, next_tick, started = None, None, None
            try:
                while not self.stop.is_set():
                    try:
                        command, kwargs, future = self.jobs.get(timeout=.003)
                    except queue.Empty:
                        command = None
                    if command is not None:
                        try:
                            if command == "start":
                                status = self.status()
                                if not status["ready"]:
                                    raise ValueError(status["readiness_reason"])
                                self.store.start(kwargs["task"], {"telemetry": self.receiver.latest(),
                                                               "alignment": self.store.metadata})
                                synchronizer = Synchronizer(self.config, self.receiver, self.cameras)
                                started = next_tick = time.monotonic()
                                self._update(state="recording", frames=0, elapsed_s=0., error=None)
                                result = {"started": True}
                            elif command == "configure":
                                if self.review_only:
                                    raise ValueError("offline review")
                                result = self.store.set_fps(kwargs["fps"])
                            elif command == "finish":
                                if kwargs["outcome"] not in ("success", "failure", "discarded", "interrupted"):
                                    raise ValueError("invalid episode outcome")
                                if self.store.episode is None:
                                    raise ValueError("no episode is recording")
                                if kwargs["outcome"] in ("success", "failure") and self.store.episode.enqueued < self.config["min_episode_frames"]:
                                    raise ValueError("episode too short; continue recording or discard")
                                result = self._finish(kwargs["outcome"])
                            elif command == "review":
                                if self.store.episode is not None:
                                    raise ValueError("finish recording before reviewing")
                                path = episode_path(self.store.root, kwargs["id"])
                                qc = validate_episode(path, self.store.metadata)
                                if kwargs["verdict"] == "pass" and not qc["ok"]:
                                    raise ValueError("quality check failed: " + ", ".join(qc["issues"]))
                                write_review(path, kwargs["verdict"], kwargs.get("notes", ""))
                                result = {**summary(path), "quality": qc}
                            else:
                                raise ValueError("unknown capture command")
                            future.set_result(result)
                        except Exception as exc:
                            future.set_exception(exc)
                    if self.store.episode is None or next_tick is None:
                        continue
                    now = time.monotonic()
                    if now < next_tick + LOOKAHEAD_S:
                        continue
                    try:
                        if now - (next_tick + LOOKAHEAD_S) > self.config["max_schedule_lateness_s"]:
                            raise RuntimeError("capture deadline missed")
                        if next_tick - started >= self.config["max_episode_seconds"]:
                            raise RuntimeError("maximum episode duration reached")
                        sample = synchronizer.sample(next_tick)
                        self.store.episode.append(sample)
                        self._update(frames=self.store.episode.enqueued,
                                     elapsed_s=self.store.episode.enqueued / self.config["fps"])
                        next_tick = started + self.store.episode.enqueued / self.config["fps"]
                    except Exception as exc:
                        try:
                            self._finish("interrupted", str(exc))
                        except Exception:
                            pass
            finally:
                if self.store.episode is not None:
                    try:
                        self._finish("interrupted", "collector_closed")
                    except Exception:
                        pass
                while not self.jobs.empty():
                    _, _, future = self.jobs.get_nowait()
                    future.set_exception(RuntimeError("collector_closed"))

        self.worker = threading.Thread(target=run, name="data-capture", daemon=True)
        self.worker.start()
        return self

    def __exit__(self, *_args):
        self.stop.set()
        self.worker.join()
