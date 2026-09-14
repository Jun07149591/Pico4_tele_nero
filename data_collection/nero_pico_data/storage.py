import copy
import fcntl
import json
import os
from pathlib import Path
import queue
import re
import shutil
import threading
import uuid

import cv2
import h5py
import numpy as np

from .schema import manifest, validate_capture_fps, vector_names


def _fixed_schema(metadata):
    result = copy.deepcopy(metadata)
    result.pop("fps", None)
    result["config"].pop("fps", None)
    return result


class DatasetStore:
    def __init__(self, root, config, *, synthetic=False):
        self.root, self.config = Path(root).resolve(), config
        self.metadata = manifest(config, synthetic)
        self.lock_file = None
        self.episode = None

    def __enter__(self):
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock_file = (self.root / ".writer.lock").open("a+")
        try:
            try:
                fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                self.lock_file.seek(0)
                try:
                    owner = json.load(self.lock_file)
                except (ValueError, OSError):
                    owner = {}
                pid = owner.get("pid") if isinstance(owner, dict) else None
                process = f" (PID {pid})" if type(pid) is int and pid > 0 else ""
                raise BlockingIOError(
                    f"dataset already in use{process}: {self.root}. "
                    "Use the existing collector webpage, or stop that collector before restarting. "
                    "Do not delete .writer.lock while a collector is running.") from exc
            self.lock_file.seek(0)
            self.lock_file.truncate()
            json.dump({"pid": os.getpid(), "root": str(self.root)}, self.lock_file)
            self.lock_file.flush()
            path = self.root / "manifest.json"
            if path.exists():
                existing = json.loads(path.read_text())
                if _fixed_schema(existing) != _fixed_schema(self.metadata):
                    raise ValueError("dataset schema/cameras/mode differ; use a new dataset directory")
                self.metadata = existing
            else:
                with path.open("x") as stream:
                    json.dump(self.metadata, stream, indent=2)
            for folder in ("episodes", ".inprogress"):
                (self.root / folder).mkdir(exist_ok=True)
            settings = self.root / "capture_settings.json"
            if settings.exists():
                self.config["fps"] = validate_capture_fps(json.loads(settings.read_text())["fps"], self.config)
        except Exception:
            self.lock_file.close()
            raise
        return self

    def set_fps(self, fps):
        if self.episode is not None:
            raise ValueError("finish recording before changing capture fps")
        fps = validate_capture_fps(fps, self.config)
        destination = self.root / "capture_settings.json"
        temporary = destination.with_suffix(f".{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("x") as stream:
                json.dump({"fps": fps}, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)
        self.config["fps"] = fps
        return {"fps": fps}

    def start(self, task, provenance):
        if self.episode is not None:
            raise ValueError("an episode is already recording")
        if not task.strip():
            raise ValueError("a nonempty task instruction is required")
        if shutil.disk_usage(self.root).free < self.config["minimum_free_gb"] * 1e9:
            raise RuntimeError("insufficient free disk space")
        self.episode = EpisodeWriter(self.root, self.config, task.strip(), provenance)
        return self.episode

    def finish(self, outcome, reason=""):
        if outcome not in ("success", "failure", "discarded", "interrupted"):
            raise ValueError("invalid episode outcome")
        episode = self.episode
        if episode is None:
            raise ValueError("no episode is recording")
        if outcome in ("success", "failure") and episode.enqueued < self.config["min_episode_frames"]:
            raise ValueError("episode too short; continue recording or discard")
        try:
            return episode.finish(outcome, reason)
        finally:
            self.episode = None

    def __exit__(self, *_args):
        try:
            if self.episode is not None:
                self.finish("interrupted", "collector_closed")
        finally:
            self.lock_file.close()


class EpisodeWriter:
    def __init__(self, root, config, task, provenance):
        config = copy.deepcopy(config)
        self.config = config
        paths = [*(root / "episodes").glob("episode_*.h5"), *(root / ".inprogress").glob("episode_*.h5")]
        numbers = [int(match[1]) for path in paths if (match := re.fullmatch(r"episode_(\d+)\.h5", path.name))]
        self.identifier = f"episode_{max([len(paths), *numbers]) + 1:02d}"
        self.partial = root / ".inprogress" / f"{self.identifier}.h5"
        self.destination = root / "episodes" / f"{self.identifier}.h5"
        self.queue = queue.Queue(maxsize=config["writer_queue_frames"])
        self.error = None
        self.enqueued = 0
        self.outcome, self.reason = None, ""
        ready = threading.Event()

        def run():
            try:
                with h5py.File(self.partial, "x") as file:
                    file.attrs.update(schema_version=1, task=task, outcome="inprogress", fps=config["fps"],
                                      mode=config["mode"], provenance=json.dumps({**provenance, "capture_fps": config["fps"]}, allow_nan=False))
                    dim = len(vector_names(config["mode"]))
                    for key in ("state", "action"):
                        file.create_dataset(key, shape=(0, dim), maxshape=(None, dim), chunks=(1, dim), dtype="f4")
                    for key in ("timestamp", "monotonic", "action_monotonic", "wall_time_ns"):
                        file.create_dataset(key, shape=(0,), maxshape=(None,), chunks=True,
                                            dtype="i8" if key == "wall_time_ns" else "f8")
                    file.create_dataset("diagnostics", shape=(0,), maxshape=(None,), dtype=h5py.string_dtype())
                    for role in config["cameras"]:
                        file.create_dataset(f"images/{role}", shape=(0,), maxshape=(None,), dtype=h5py.vlen_dtype(np.dtype("uint8")))
                        file.create_dataset(f"image_timestamps/{role}", shape=(0, 3), maxshape=(None, 3), dtype="f8")
                    ready.set()
                    count = 0
                    while (sample := self.queue.get()) is not None:
                        if count % config["fps"] == 0 and shutil.disk_usage(root).free < config["minimum_free_gb"] * 1e9:
                            raise RuntimeError("disk space reserve reached")
                        values = {key: sample[key] for key in ("state", "action", "monotonic", "action_monotonic", "wall_time_ns")}
                        values["timestamp"] = count / config["fps"]
                        values["diagnostics"] = json.dumps(sample["diagnostics"], allow_nan=False)
                        for role, frame in sample["images"].items():
                            ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR),
                                                       [cv2.IMWRITE_JPEG_QUALITY, config["jpeg_quality"]])
                            if not ok:
                                raise RuntimeError(f"image encoding failed: {role}")
                            values[f"images/{role}"] = encoded
                            values[f"image_timestamps/{role}"] = [frame.monotonic, frame.device_timestamp_ms, frame.sequence]
                        for key, value in values.items():
                            file[key].resize(count + 1, axis=0)
                            file[key][count] = value
                        count += 1
                        if count % config["fps"] == 0:
                            file.flush()
                    file.attrs.update(outcome=self.outcome, end_reason=self.reason, frame_count=count)
                    file.flush()
                    os.fsync(file.id.get_vfd_handle())
                os.rename(self.partial, self.destination)
                directory_fd = os.open(self.destination.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except Exception as exc:
                self.error = str(exc)
            finally:
                ready.set()

        self.worker = threading.Thread(target=run, name="data-writer", daemon=True)
        self.worker.start()
        ready.wait(timeout=10.)
        if not ready.is_set() or self.error:
            raise RuntimeError(self.error or "episode writer initialization timed out")

    def append(self, sample):
        if self.error:
            raise RuntimeError(f"writer failed: {self.error}")
        try:
            self.queue.put_nowait(sample)
        except queue.Full as exc:
            raise RuntimeError("writer queue full; episode interrupted instead of silently dropping frames") from exc
        self.enqueued += 1

    def finish(self, outcome, reason=""):
        if outcome not in ("success", "failure", "discarded", "interrupted"):
            raise ValueError("invalid episode outcome")
        self.outcome, self.reason = outcome, reason
        while self.worker.is_alive():
            try:
                self.queue.put(None, timeout=.1)
                break
            except queue.Full:
                continue
        self.worker.join()
        if self.error:
            raise RuntimeError(f"episode kept in .inprogress: {self.error}")
        return self.destination


def read_rgb(file, role, index):
    bgr = cv2.imdecode(np.asarray(file[f"images/{role}"][index]), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"invalid JPEG: {role} frame {index}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
