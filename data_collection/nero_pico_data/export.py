import hashlib
import json
from pathlib import Path
import re
import shutil

import h5py

from .quality import episode_path, list_episodes, validate_episode
from .storage import read_rgb


LEROBOT_REVISION = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5"


def select_episodes(root, episode_ids=None):
    available = list_episodes(root)
    if episode_ids is not None:
        if (not isinstance(episode_ids, list) or not episode_ids
                or any(not isinstance(identifier, str) for identifier in episode_ids)
                or len(set(episode_ids)) != len(episode_ids)):
            raise ValueError("select at least one episode; duplicate episode IDs are not allowed")
        for identifier in episode_ids:
            episode_path(root, identifier)
        selected = [info for info in available if info["id"] in episode_ids]
        if any(info["outcome"] != "success" or info["review"]["verdict"] != "pass" for info in selected):
            raise ValueError("selected episodes must be successful with PASS review")
    else:
        selected = [info for info in available if info["outcome"] == "success" and info["review"]["verdict"] == "pass"]
    if not selected:
        raise ValueError("no successful episodes with PASS review")
    if len({info["fps"] for info in selected}) != 1:
        raise ValueError("selected episodes have different capture rates; export each rate separately")
    return selected


def export_dataset(root, destination, repo_id, *, allow_synthetic=False, episode_ids=None, progress=None):
    import pyarrow.parquet as pq
    from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    from lerobot.common.datasets.video_utils import encode_video_frames

    class VideoDataset(LeRobotDataset):
        def encode_episode_videos(self, episode_index):
            # The pinned writer has no codec option; use its encoder with H.264
            # for both standard LeRobot decoding and ordinary video players.
            paths = {}
            for key in self.meta.video_keys:
                path = self.root / self.meta.get_video_file_path(episode_index, key)
                images = self._get_image_file_path(episode_index, key, 0).parent
                encode_video_frames(images, path, self.fps, vcodec="h264", crf=18, overwrite=True)
                paths[key] = str(path)
            return paths

    if CODEBASE_VERSION != "v2.1":
        raise RuntimeError(f"OpenPI requires pinned LeRobot v2.1; installed {CODEBASE_VERSION}")
    if not re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_.-]+", repo_id):
        raise ValueError("repo_id must be namespace/dataset")
    root, destination = Path(root).resolve(), Path(destination).resolve()
    if destination.exists():
        raise ValueError("export destination already exists; choose a new directory")
    metadata = json.loads((root / "manifest.json").read_text())
    if metadata["synthetic"] and not allow_synthetic:
        raise ValueError("synthetic dataset requires explicit --allow-synthetic")
    selected = select_episodes(root, episode_ids)
    fps = selected[0]["fps"]
    checked = []
    for info in selected:
        path = episode_path(root, info["id"])
        qc = validate_episode(path, metadata)
        if not qc["ok"]:
            raise ValueError(f"accepted episode failed validation: {path.name}: {qc['issues']}")
        checked.append((path, info, qc))
    config = metadata["config"]
    names, dim = metadata["vector_names"], len(metadata["vector_names"])
    features = {"observation.state": {"dtype": "float32", "shape": (dim,), "names": names},
                "action": {"dtype": "float32", "shape": (dim,), "names": names}}
    for role in config["cameras"]:
        features[f"observation.images.{role}"] = {"dtype": "video",
            "shape": (config["image_height"], config["image_width"], 3), "names": ["height", "width", "channels"]}
    dataset = VideoDataset.create(repo_id=repo_id, root=destination, fps=fps,
                                 robot_type=metadata["robot_type"], features=features, use_videos=True,
                                 image_writer_threads=2, video_backend="pyav")
    marker = destination / ".export-incomplete"
    marker.touch()
    sources = []
    try:
        for episode_index, (path, info, qc) in enumerate(checked):
            name = Path(dataset.meta.get_data_file_path(episode_index)).stem
            if progress:
                progress({"completed": episode_index, "total": len(checked), "episode": name})
            with h5py.File(path, "r") as file:
                for index in range(len(file["timestamp"])):
                    if index % fps == 0 and shutil.disk_usage(destination).free < config["minimum_free_gb"] * 1e9:
                        raise RuntimeError("disk space reserve reached; export kept incomplete")
                    frame = {"observation.state": file["state"][index], "action": file["action"][index],
                             "task": str(file.attrs["task"])}
                    frame.update({f"observation.images.{role}": read_rgb(file, role, index) for role in config["cameras"]})
                    dataset.add_frame(frame)
                dataset.save_episode()
            with path.open("rb") as stream:
                digest = hashlib.file_digest(stream, "sha256").hexdigest()
            videos = {role: str(dataset.meta.get_video_file_path(episode_index, f"observation.images.{role}"))
                      for role in config["cameras"]}
            sources.append({"name": name, "episode_index": episode_index, "source": path.name,
                            "fps": fps, "frames": info["frames"], "videos": videos,
                            "parquet": str(dataset.meta.get_data_file_path(episode_index)),
                            "sha256": digest, "review": info["review"], "quality": qc})
    finally:
        dataset.stop_image_writer()
    # Validate files on disk before declaring the export complete.
    for source in sources:
        table = pq.read_table(destination / source["parquet"], columns=["episode_index"])
        if (table.num_rows != source["frames"]
                or table["episode_index"].unique().to_pylist() != [source["episode_index"]]):
            raise RuntimeError(f"exported Parquet does not match source: {source['source']}")
        for video in source["videos"].values():
            if (destination / video).stat().st_size == 0:
                raise RuntimeError(f"exported video is empty: {video}")
    parquet_files = len(list((destination / "data").rglob("*.parquet")))
    video_files = len(list((destination / "videos").rglob("*.mp4")))
    if (parquet_files != len(sources) or video_files != len(sources) * len(config["cameras"])
            or dataset.meta.total_frames != sum(source["frames"] for source in sources)):
        raise RuntimeError("exported file/frame counts do not match selected episodes")
    report = {"repo_id": repo_id, "episodes": len(sources), "frames": dataset.meta.total_frames,
              "fps": fps, "image_storage": "video", "video_codec": "h264",
              "parquet_files": parquet_files, "video_files": video_files,
              "lerobot_revision": LEROBOT_REVISION, "source_manifest": metadata, "sources": sources}
    (destination / "meta/nero_provenance.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    spec = {"repo_id": repo_id, "mode": metadata["mode"], "cameras": list(config["cameras"]),
            "fps": fps, "action_horizon": 20, "synthetic": metadata["synthetic"],
            "dataset_root": str(destination)}
    (destination / "meta/nero_openpi.json").write_text(json.dumps(spec, indent=2))
    marker.unlink()
    if progress:
        progress({"completed": len(sources), "total": len(sources), "episode": None})
    return {"root": str(destination), "episodes": len(sources), "frames": dataset.meta.total_frames,
            "fps": fps, "image_storage": "video", "videos_root": str(destination / "videos"),
            "parquet_files": parquet_files, "video_files": video_files,
            "files": [{key: source[key] for key in ("source", "name", "frames", "parquet", "videos")}
                      for source in sources],
            "repo_id": repo_id, "openpi_spec": str(destination / "meta/nero_openpi.json")}
