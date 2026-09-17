"""Streaming LeRobot v2.1 episodes without intermediate image files."""

from contextlib import ExitStack
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from .schema import vector_names

LEROBOT_REVISION = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5"
IMAGE_STORAGE = "lerobot_video"
RECEIPT = "meta/nero_files.json"


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def create_metadata(root, config, repo_id):
    from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDatasetMetadata

    if CODEBASE_VERSION != "v2.1":
        raise RuntimeError(f"OpenPI requires pinned LeRobot v2.1; installed {CODEBASE_VERSION}")
    names = vector_names(config["mode"])
    features = {key: {"dtype": "float32", "shape": (len(names),), "names": names}
                for key in ("observation.state", "action")}
    features.update({f"observation.images.{role}": {"dtype": "video",
                     "shape": (config["image_height"], config["image_width"], 3),
                     "names": ["height", "width", "channels"]} for role in config["cameras"]})
    return LeRobotDatasetMetadata.create(repo_id=repo_id, root=root, fps=config["fps"],
                                        robot_type=f"nero_{config['mode']}", features=features, use_videos=True)


def table_stats(table, features):
    from lerobot.common.datasets.compute_stats import compute_episode_stats

    return compute_episode_stats({key: np.asarray(table[key].to_pylist()) for key in table.column_names}, features)


class ImageStats:
    def __init__(self):
        self.count = 0
        self.pixels = 0
        self.mean = np.zeros(3)
        self.m2 = np.zeros(3)
        self.minimum = np.full(3, np.inf)
        self.maximum = np.full(3, -np.inf)

    def add(self, rgb):
        # Match LeRobot's spatial sampling, accumulating every frame in bounded memory.
        factor = max(1, max(rgb.shape[:2]) // 150) if max(rgb.shape[:2]) >= 300 else 1
        pixels = rgb[::factor, ::factor].reshape(-1, 3).astype(np.float64) / 255.
        mean = pixels.mean(axis=0)
        delta = mean - self.mean
        total = self.pixels + len(pixels)
        self.m2 += ((pixels - mean) ** 2).sum(axis=0) + delta ** 2 * self.pixels * len(pixels) / total
        self.mean += delta * len(pixels) / total
        self.minimum = np.minimum(self.minimum, pixels.min(axis=0))
        self.maximum = np.maximum(self.maximum, pixels.max(axis=0))
        self.pixels = total
        self.count += 1

    def result(self):
        values = {"min": self.minimum, "max": self.maximum, "mean": self.mean,
                  "std": np.sqrt(self.m2 / self.pixels)}
        return {key: value.reshape(3, 1, 1) for key, value in values.items()} | {"count": np.array([self.count])}


class StreamingEpisode:
    """One standalone episode; callers publish its directory only after finish()."""

    def __init__(self, root, config, task):
        import av
        import pyarrow.parquet as pq
        from lerobot.common.datasets.utils import get_hf_features_from_features

        av.logging.set_level(av.logging.ERROR)
        self.root, self.config, self.task = Path(root), config, task
        self.resources = ExitStack()
        self.videos = {}
        self.count = 0
        self.rows = []
        self.stats = {}
        self.image_stats = {role: ImageStats() for role in config["cameras"]}
        self.meta = create_metadata(self.root, config, "local/recording")
        (self.root / ".recording-incomplete").touch()
        self.meta.add_task(task)
        self.schema = get_hf_features_from_features(self.meta.features).arrow_schema
        path = self.root / self.meta.get_data_file_path(0)
        path.parent.mkdir(parents=True)
        self.parquet = self.resources.enter_context(pq.ParquetWriter(path, self.schema, compression="snappy"))

    def append(self, state, action, images):
        import av

        cfg = self.config
        dim = len(vector_names(cfg["mode"]))
        if any(np.shape(value) != (dim,) or not np.isfinite(value).all() for value in (state, action)):
            raise ValueError("invalid state/action vector")
        if set(images) != set(cfg["cameras"]):
            raise ValueError("recording camera roles do not match configuration")
        for role, rgb in images.items():
            if rgb.dtype != np.uint8 or rgb.shape != (cfg["image_height"], cfg["image_width"], 3):
                raise ValueError(f"invalid RGB frame: {role}")
            if role not in self.videos:
                path = self.root / self.meta.get_video_file_path(0, f"observation.images.{role}")
                path.parent.mkdir(parents=True)
                container = self.resources.enter_context(av.open(str(path), "w"))
                stream = container.add_stream("libx264", rate=cfg["fps"], options={
                    "crf": "18", "preset": "veryfast", "tune": "zerolatency", "g": "2", "bf": "0"})
                stream.width, stream.height = cfg["image_width"], cfg["image_height"]
                stream.pix_fmt = "yuv420p" if stream.width % 2 == stream.height % 2 == 0 else "yuv444p"
                stream.codec_context.thread_count = 2
                self.videos[role] = (container, stream)
            container, stream = self.videos[role]
            frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
            frame.pts, frame.time_base = self.count, Fraction(1, cfg["fps"])
            for packet in stream.encode(frame):
                container.mux(packet)
            self.image_stats[role].add(rgb)
        self.rows.append({"observation.state": np.asarray(state, dtype=np.float32).tolist(),
                          "action": np.asarray(action, dtype=np.float32).tolist(),
                          "timestamp": self.count / cfg["fps"], "frame_index": self.count,
                          "episode_index": 0, "index": self.count, "task_index": 0})
        self.count += 1
        if len(self.rows) >= cfg["fps"]:
            self.flush_table()

    def flush_table(self):
        import pyarrow as pa
        from lerobot.common.datasets.compute_stats import aggregate_stats

        if self.rows:
            table = pa.Table.from_pylist(self.rows, schema=self.schema)
            self.parquet.write_table(table)
            stats = table_stats(table, self.meta.features)
            self.stats = aggregate_stats([self.stats, stats]) if self.stats else stats
            self.rows.clear()

    def _flush_videos(self):
        for container, stream in self.videos.values():
            for packet in stream.encode():
                container.mux(packet)

    def finish(self, recording):
        try:
            self.flush_table()
            self._flush_videos()
        finally:
            self.close()
        if self.count:
            self.stats.update({f"observation.images.{role}": stats.result() for role, stats in self.image_stats.items()})
            self.meta.save_episode(0, self.count, [self.task], self.stats)
        (self.root / "meta/nero_recording.json").write_text(json.dumps(recording, ensure_ascii=False, indent=2))
        files = {}
        for path in sorted(self.root.rglob("*")):
            if path.is_file() and path.name != ".recording-incomplete":
                with path.open("rb") as stream:
                    os.fsync(stream.fileno())
                files[str(path.relative_to(self.root))] = {"bytes": path.stat().st_size, "sha256": sha256(path)}
        receipt = self.root / RECEIPT
        with receipt.open("x") as stream:
            json.dump({"frames": self.count, "files": files}, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        (self.root / ".recording-incomplete").unlink()
        for directory in [*sorted((p for p in self.root.rglob("*") if p.is_dir()), reverse=True), self.root]:
            fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        return sha256(receipt)

    def close(self):
        self.resources.close()
        self.videos.clear()


def native_root(file):
    root = Path(file.filename).with_suffix("")
    if root.is_symlink() or not root.is_dir() or (root / ".recording-incomplete").exists():
        raise ValueError("native episode missing or incomplete")
    return root


def native_video(file, role):
    if role not in file["image_timestamps"]:
        raise ValueError("unknown camera role")
    return native_root(file) / f"videos/chunk-000/observation.images.{role}/episode_000000.mp4"


def verify_native(file, metadata, *, hashes=True):
    import pyarrow.parquet as pq

    root = native_root(file)
    receipt_path = root / RECEIPT
    if sha256(receipt_path) != file.attrs["native_receipt_sha256"]:
        raise ValueError("native episode manifest changed")
    receipt = json.loads(receipt_path.read_text())
    if receipt["frames"] != len(file["timestamp"]):
        raise ValueError("native episode length mismatch")
    expected = {"data/chunk-000/episode_000000.parquet", "meta/info.json", "meta/tasks.jsonl",
                "meta/episodes.jsonl", "meta/episodes_stats.jsonl", "meta/nero_recording.json"}
    expected.update(str(native_video(file, role).relative_to(root)) for role in metadata["config"]["cameras"])
    if not expected.issubset(receipt["files"]):
        raise ValueError("native episode files missing")
    for relative, info in receipt["files"].items():
        path = root / relative
        if (Path(relative).is_absolute() or ".." in Path(relative).parts
                or any(p.is_symlink() for p in (path, *path.parents))
                or not path.is_file() or path.stat().st_size != info["bytes"]):
            raise ValueError(f"native episode file missing or changed: {relative}")
        if hashes and sha256(path) != info["sha256"]:
            raise ValueError(f"native episode file checksum mismatch: {relative}")
    table = pq.read_table(root / "data/chunk-000/episode_000000.parquet")
    for target, source in (("observation.state", "state"), ("action", "action")):
        if not np.array_equal(np.asarray(table[target].to_pylist(), dtype=np.float32), file[source][:]):
            raise ValueError(f"native episode {source} mismatch")
    count = len(file["timestamp"])
    columns = {"frame_index": np.arange(count), "index": np.arange(count),
               "episode_index": np.zeros(count), "task_index": np.zeros(count),
               "timestamp": np.asarray(file["timestamp"][:], dtype=np.float32)}
    if table.num_rows != count or any(not np.array_equal(table[key].to_numpy(), value) for key, value in columns.items()):
        raise ValueError("native episode indices/timestamps mismatch")
    info = json.loads((root / "meta/info.json").read_text())
    tasks = [json.loads(line) for line in (root / "meta/tasks.jsonl").read_text().splitlines()]
    if (info["fps"] != file.attrs["fps"] or info["total_frames"] != count
            or info["robot_type"] != metadata["robot_type"]
            or info["features"]["observation.state"]["names"] != metadata["vector_names"]
            or tasks != [{"task_index": 0, "task": str(file.attrs["task"])}]):
        raise ValueError("native episode metadata mismatch")
    return root


def read_video_frame(file, role, index):
    import av

    with av.open(str(native_video(file, role))) as container:
        stream = container.streams.video[0]
        timestamp = Fraction(index, int(file.attrs["fps"]))
        container.seek(int(timestamp / stream.time_base), stream=stream, backward=True)
        for frame in container.decode(stream):
            if frame.pts is not None and abs(float(frame.pts * frame.time_base - timestamp)) < 1e-5:
                return frame.to_ndarray(format="rgb24")
    raise ValueError(f"video frame missing: {role}:{index}")


def validate_video(file, role, config):
    import av

    fps = int(file.attrs["fps"])
    count = 0
    with av.open(str(native_video(file, role))) as container:
        stream = container.streams.video[0]
        if stream.average_rate != fps:
            raise ValueError("video rate mismatch")
        for index, frame in enumerate(container.decode(stream)):
            if ((frame.height, frame.width) != (config["image_height"], config["image_width"])
                    or frame.pts is None or abs(float(frame.pts * frame.time_base) - index / fps) > 1e-5):
                raise ValueError(f"invalid video frame: {index}")
            count += 1
    if count != len(file["timestamp"]):
        raise ValueError("video frame count mismatch")
