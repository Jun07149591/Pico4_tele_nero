import copy
import json
from pathlib import Path
import re
import shutil
import tempfile

import h5py

from .quality import episode_path, list_episodes, validate_episode
from .storage import read_rgb
from .lerobot_io import IMAGE_STORAGE, LEROBOT_REVISION, StreamingEpisode, create_metadata, native_root, sha256, table_stats


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
    import numpy as np
    import pyarrow as pa
    import pyarrow.parquet as pq
    from lerobot.common.datasets.lerobot_dataset import LeRobotDatasetMetadata

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
    for index, info in enumerate(selected):
        if progress:
            progress({"completed": index, "total": len(selected), "episode": info["id"], "phase": "validating"})
        path = episode_path(root, info["id"])
        # Native recordings already contain encoded frames. Check all file hashes
        # and table alignment instead of decoding every pixel again on each export.
        qc = validate_episode(path, metadata, decode_images=info["image_storage"] != IMAGE_STORAGE, verify_hashes=True)
        if not qc["ok"]:
            raise ValueError(f"accepted episode failed validation: {path.name}: {qc['issues']}")
        checked.append((path, info, qc))
    config = {**metadata["config"], "fps": fps}
    output = create_metadata(destination, config, repo_id)
    marker = destination / ".export-incomplete"
    marker.touch()
    sources = []
    reused = 0
    for episode_index, (path, info, qc) in enumerate(checked):
        name = Path(output.get_data_file_path(episode_index)).stem
        native = info["image_storage"] == IMAGE_STORAGE
        if progress:
            progress({"completed": episode_index, "total": len(checked), "episode": name,
                      "phase": "copying" if native else "encoding_legacy"})
        with tempfile.TemporaryDirectory(prefix=".export-work-", dir=destination) as temporary:
            if native:
                with h5py.File(path, "r") as file:
                    bundle = native_root(file)
                reused += 1
            else:
                bundle = Path(temporary) / "episode"
                with h5py.File(path, "r") as file:
                    writer = StreamingEpisode(bundle, config, str(file.attrs["task"]))
                    try:
                        for index in range(len(file["timestamp"])):
                            if index % fps == 0 and shutil.disk_usage(destination).free < config["minimum_free_gb"] * 1e9:
                                raise RuntimeError("disk space reserve reached; export kept incomplete")
                            writer.append(file["state"][index], file["action"][index],
                                          {role: read_rgb(file, role, index) for role in config["cameras"]})
                        writer.finish({"source": path.name, "outcome": info["outcome"]})
                    finally:
                        writer.close()
            original = LeRobotDatasetMetadata("local/recording", root=bundle)
            table = pq.read_table(bundle / original.get_data_file_path(0))
            task = info["task"]
            if output.get_task_index(task) is None:
                output.add_task(task)
            changes = {"episode_index": np.full(table.num_rows, episode_index, dtype=np.int64),
                       "index": np.arange(output.total_frames, output.total_frames + table.num_rows, dtype=np.int64),
                       "task_index": np.full(table.num_rows, output.get_task_index(task), dtype=np.int64)}
            for key, values in changes.items():
                field = table.schema.field(key)
                table = table.set_column(table.schema.get_field_index(key), field, pa.array(values, type=field.type))
            data_path = destination / output.get_data_file_path(episode_index)
            data_path.parent.mkdir(parents=True, exist_ok=True)
            pq.write_table(table, data_path, compression="snappy")
            stats = table_stats(table, output.features)
            videos = {}
            for role in config["cameras"]:
                key = f"observation.images.{role}"
                source = bundle / original.get_video_file_path(0, key)
                relative = output.get_video_file_path(episode_index, key)
                target = destination / relative
                if shutil.disk_usage(destination).free < config["minimum_free_gb"] * 1e9 + source.stat().st_size:
                    raise RuntimeError("disk space reserve reached; export kept incomplete")
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
                if sha256(target) != sha256(source):
                    raise RuntimeError(f"exported video checksum mismatch: {relative}")
                videos[role] = str(relative)
                stats[key] = copy.deepcopy(original.episodes_stats[0][key])
            output.save_episode(episode_index, table.num_rows, [task], stats)
            sources.append({"name": name, "episode_index": episode_index, "source": path.name,
                            "fps": fps, "frames": info["frames"], "videos": videos,
                            "parquet": str(output.get_data_file_path(episode_index)),
                            "sha256": sha256(path), "review": info["review"], "quality": qc,
                            "video_reused": native})
            if progress:
                progress({"completed": episode_index + 1, "total": len(checked), "episode": name, "phase": "copying"})
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
            or output.total_frames != sum(source["frames"] for source in sources)):
        raise RuntimeError("exported file/frame counts do not match selected episodes")
    report = {"repo_id": repo_id, "episodes": len(sources), "frames": output.total_frames,
              "fps": fps, "image_storage": "video", "video_codec": "h264",
              "reused_video_episodes": reused, "converted_legacy_episodes": len(sources) - reused,
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
    return {"root": str(destination), "episodes": len(sources), "frames": output.total_frames,
            "fps": fps, "image_storage": "video", "videos_root": str(destination / "videos"),
            "reused_video_episodes": reused, "converted_legacy_episodes": len(sources) - reused,
            "parquet_files": parquet_files, "video_files": video_files,
            "files": [{key: source[key] for key in ("source", "name", "frames", "parquet", "videos")}
                      for source in sources],
            "repo_id": repo_id, "openpi_spec": str(destination / "meta/nero_openpi.json")}
