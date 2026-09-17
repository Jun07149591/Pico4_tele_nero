"""Read-only replay benchmark; all generated data lives in a temporary directory."""

import argparse
import json
from pathlib import Path
import shutil
import tempfile
import time
from unittest.mock import patch

import h5py
import numpy as np

from nero_pico_data.cameras import Frame
from nero_pico_data.export import export_dataset
from nero_pico_data.lerobot_io import StreamingEpisode, sha256
from nero_pico_data.quality import validate_episode, write_review
from nero_pico_data.storage import DatasetStore, read_rgb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()
    source = args.episode.resolve()
    metadata = json.loads((source.parent.parent / "manifest.json").read_text())
    config = metadata["config"].copy()
    digest = sha256(source)
    measurements = []
    original_append = StreamingEpisode.append

    def timed_append(writer, *args, **kwargs):
        started = time.perf_counter()
        result = original_append(writer, *args, **kwargs)
        measurements.append(time.perf_counter() - started)
        return result

    with tempfile.TemporaryDirectory(prefix="nero-recording-benchmark-") as temporary, h5py.File(source, "r") as file:
        root = Path(temporary)
        config["fps"] = int(file.attrs["fps"])
        count = len(file["timestamp"])
        max_queue = retries = 0
        with DatasetStore(root / "native", config, synthetic=True) as store, patch.object(StreamingEpisode, "append", timed_append):
            writer = store.start(str(file.attrs["task"]), {"benchmark": True})
            started = time.perf_counter()
            for index in range(count):
                entry = {key: file[key][index] for key in ("state", "action", "monotonic", "action_monotonic", "wall_time_ns")}
                entry["diagnostics"] = json.loads(file["diagnostics"][index])
                entry["images"] = {role: Frame(read_rgb(file, role, index), *file[f"image_timestamps/{role}"][index])
                                   for role in config["cameras"]}
                if args.realtime:
                    time.sleep(max(0., started + index / config["fps"] - time.perf_counter()))
                while True:
                    try:
                        writer.append(entry)
                        break
                    except RuntimeError as exc:
                        if str(exc) != "writer queue full":
                            raise
                        retries += 1
                        time.sleep(.001)
                max_queue = max(max_queue, writer.queue.qsize())
            replay_s = time.perf_counter() - started
            started = time.perf_counter()
            path = store.finish("success")
            save_s = time.perf_counter() - started
            qc = validate_episode(path, store.metadata)
            if not qc["ok"]:
                raise ValueError(qc)
            write_review(path, "pass", "temporary benchmark data")
            started = time.perf_counter()
            export_dataset(store.root, root / "native_export", "local/benchmark", allow_synthetic=True)
            export_s = time.perf_counter() - started
            native_bytes = path.stat().st_size + sum(p.stat().st_size for p in path.with_suffix("").rglob("*") if p.is_file())
        legacy_root = root / "legacy"
        (legacy_root / "episodes").mkdir(parents=True)
        (legacy_root / "manifest.json").write_text(json.dumps(metadata))
        legacy = legacy_root / "episodes" / source.name
        shutil.copyfile(source, legacy)
        write_review(legacy, "pass", "temporary benchmark data")
        started = time.perf_counter()
        export_dataset(legacy_root, root / "legacy_export", "local/legacy_benchmark", allow_synthetic=True)
        legacy_s = time.perf_counter() - started
        assert sha256(source) == digest
        print(json.dumps({"frames": count, "fps": config["fps"], "cameras": len(config["cameras"]),
                          "image_size": [config["image_width"], config["image_height"]],
                          "realtime": args.realtime, "replay_s": replay_s, "finish_s": save_s,
                          "native_export_s": export_s, "legacy_export_s": legacy_s,
                          "encode_and_table_mean_ms": np.mean(measurements) * 1000,
                          "encode_and_table_p95_ms": np.percentile(measurements, 95) * 1000,
                          "max_writer_queue": max_queue, "queue_full_retries": retries,
                          "source_bytes": source.stat().st_size, "native_bytes": native_bytes,
                          "source_unchanged": True}, indent=2))


if __name__ == "__main__":
    main()
