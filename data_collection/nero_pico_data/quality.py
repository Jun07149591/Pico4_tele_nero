import json
import os
from pathlib import Path
import uuid

import h5py
import numpy as np

from .eva_alignment import stream_stats
from .storage import read_rgb


def episode_path(root, identifier):
    if not identifier.startswith("episode_") or Path(identifier).name != identifier or not identifier.endswith(".h5"):
        raise ValueError("invalid episode identifier")
    path = Path(root) / "episodes" / identifier
    if not path.is_file() or path.is_symlink():
        raise ValueError("episode does not exist")
    return path


def review_path(path):
    return path.with_suffix(".review.json")


def read_review(path):
    return json.loads(review_path(path).read_text()) if review_path(path).exists() else {"verdict": "unreviewed", "notes": ""}


def write_review(path, verdict, notes):
    if verdict not in ("pass", "fail", "unreviewed") or len(notes) > 4000:
        raise ValueError("invalid review")
    destination = review_path(path)
    temporary = destination.with_suffix(f".{uuid.uuid4().hex}.tmp")
    with temporary.open("x") as stream:
        json.dump({"verdict": verdict, "notes": notes}, stream, ensure_ascii=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, destination)


def summary(path):
    with h5py.File(path, "r") as file:
        count = len(file["timestamp"])
        return {"id": path.name, "task": str(file.attrs["task"]), "outcome": str(file.attrs["outcome"]),
                "frames": count, "seconds": count / file.attrs["fps"], "fps": int(file.attrs["fps"]),
                "created_ns": int(file["wall_time_ns"][0]) if count else path.stat().st_mtime_ns,
                "mode": str(file.attrs["mode"]), "reason": str(file.attrs.get("end_reason", "")),
                "cameras": list(file["images"]), "review": read_review(path)}


def list_episodes(root):
    return sorted((summary(path) for path in (Path(root) / "episodes").glob("episode_*.h5")),
                  key=lambda info: (info["created_ns"], info["id"]))


def validate_episode(path, metadata, *, decode_images=True):
    issues = []
    config = metadata["config"]
    with h5py.File(path, "r") as file:
        times = file["monotonic"][:]
        fps = file.attrs["fps"]
        count = len(times)
        dim = len(metadata["vector_names"])
        if count < config["min_episode_frames"]:
            issues.append("episode_too_short")
        if str(file.attrs["mode"]) != metadata["mode"]:
            issues.append("schema_mismatch")
        if not isinstance(fps, (int, np.integer)) or not 1 <= fps <= min(c["fps"] for c in config["cameras"].values()):
            issues.append("invalid_fps")
            fps = metadata["fps"]
        if file.attrs["outcome"] in ("interrupted", "discarded", "inprogress"):
            issues.append(f"episode_{file.attrs['outcome']}")
        if not str(file.attrs["task"]).strip():
            issues.append("missing_task")
        if not np.isfinite(times).all() or (len(times) > 1 and not np.allclose(np.diff(times), 1 / fps, atol=1e-6, rtol=0)):
            issues.append("invalid_fixed_timeline")
        for key in ("state", "action"):
            data = file[key][:]
            if data.shape != (count, dim) or not np.isfinite(data).all():
                issues.append(f"invalid_{key}")
            elif np.any((data[:, 7::8] < 0) | (data[:, 7::8] > .1)):
                issues.append(f"invalid_{key}_gripper")
        if not np.allclose(file["timestamp"][:], np.arange(count) / fps, atol=1e-6, rtol=0):
            issues.append("invalid_relative_timestamps")
        if file["action_monotonic"].shape != times.shape or not np.array_equal(file["action_monotonic"][:], times):
            issues.append("action_time_mismatch")
        camera_times = []
        for role in config["cameras"]:
            if role not in file["images"] or len(file[f"images/{role}"]) != count:
                issues.append(f"missing_camera:{role}")
                continue
            stamps = file[f"image_timestamps/{role}"][:]
            if stamps.shape != (count, 3) or not np.isfinite(stamps).all():
                issues.append(f"invalid_camera_timestamps:{role}")
                continue
            camera_times.append(stamps[:, 0])
            if np.any(np.abs(stamps[:, 0] - times) > config["max_camera_age_s"]):
                issues.append(f"camera_skew:{role}")
            if np.any(np.diff(stamps[:, 2]) <= 0):
                issues.append(f"repeated_camera_frame:{role}")
            if decode_images:
                for index in range(count):
                    try:
                        image = read_rgb(file, role, index)
                        if image.shape != (config["image_height"], config["image_width"], 3):
                            raise ValueError("wrong image dimensions")
                    except (ValueError, OSError):
                        issues.append(f"invalid_image:{role}:{index}")
                        break
        if camera_times and count and np.max(np.ptp(camera_times, axis=0)) > config["max_camera_skew_s"]:
            issues.append("cross_camera_skew")
    return {"id": path.name, "ok": not issues, "issues": issues, "timing": stream_stats(times)}
