import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest
from unittest.mock import patch

import av
import cv2
import h5py
import numpy as np
import pyarrow.parquet as pq

from nero_pico_data.export import export_dataset
from nero_pico_data.lerobot_io import IMAGE_STORAGE, ImageStats, StreamingEpisode
from nero_pico_data.quality import validate_episode, write_review
from nero_pico_data.storage import DatasetStore, read_rgb
from test_pipeline import config, sample


def legacy_episode(store, identifier, count=5):
    """A pre-streaming recording, including its JPEG camera payloads."""
    cfg = store.config
    path = store.root / "episodes" / identifier
    samples = [sample(index, cfg) for index in range(count)]
    with h5py.File(path, "x") as file:
        file.attrs.update(schema_version=1, task="legacy task", outcome="success", fps=cfg["fps"], mode=cfg["mode"])
        for key in ("state", "action", "monotonic", "action_monotonic", "wall_time_ns"):
            file.create_dataset(key, data=np.asarray([s[key] for s in samples]))
        file.create_dataset("timestamp", data=np.arange(count) / cfg["fps"])
        for role in cfg["cameras"]:
            images = file.create_dataset(f"images/{role}", (count,), dtype=h5py.vlen_dtype(np.dtype("uint8")))
            for index, entry in enumerate(samples):
                images[index] = cv2.imencode(".jpg", cv2.cvtColor(entry["images"][role].rgb, cv2.COLOR_RGB2BGR))[1]
            file.create_dataset(f"image_timestamps/{role}", data=[
                [s["images"][role].monotonic, s["images"][role].device_timestamp_ms, s["images"][role].sequence]
                for s in samples])
    write_review(path, "pass", "legacy fixture")
    return path


class NativeRecordingTests(unittest.TestCase):
    def test_streams_before_finish_and_loads_without_export(self):
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

        cfg = config()
        count = cfg["fps"] + 7
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            writer = store.start("native task", {})
            for index in range(count):
                value = sample(index, cfg)
                for frame in value["images"].values():
                    frame.rgb[:] = [index * 6, 20, 50]
                writer.append(value)
            bundle = writer.partial.with_suffix("")
            parquet = bundle / "data/chunk-000/episode_000000.parquet"
            deadline = time.monotonic() + 3
            while (not parquet.exists() or parquet.stat().st_size <= 4) and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertGreater(parquet.stat().st_size, 4)
            self.assertEqual(len(list(bundle.rglob("*.mp4"))), 2)
            self.assertFalse(list(store.root.glob("episodes/*.h5")))
            path = store.finish("success")
            self.assertTrue(validate_episode(path, store.metadata)["ok"])
            with h5py.File(path) as file:
                self.assertEqual(file.attrs["image_storage"], IMAGE_STORAGE)
                self.assertNotIn("images", file)
                for index in (0, 13, count - 1, 2):
                    rgb = read_rgb(file, "front", index)
                    np.testing.assert_allclose(rgb[0, 0], [index * 6, 20, 50], atol=5)
            bundle = path.with_suffix("")
            self.assertFalse(list(bundle.rglob("*.png")))
            self.assertFalse(list(bundle.rglob("*.jpg")))
            self.assertFalse((bundle / ".recording-incomplete").exists())
            self.assertEqual(pq.ParquetFile(bundle / "data/chunk-000/episode_000000.parquet").num_row_groups, 2)
            dataset = LeRobotDataset("local/recording", root=bundle, video_backend="pyav",
                                    delta_timestamps={"action": [0., 1 / cfg["fps"]]})
            self.assertEqual(len(dataset), count)
            self.assertEqual(dataset[13]["task"], "native task")
            np.testing.assert_allclose(dataset[13]["observation.state"], sample(13, cfg)["state"])
            stats = dataset.meta.stats
            expected = np.stack([sample(i, cfg)["state"] for i in range(count)])
            np.testing.assert_allclose(stats["observation.state"]["mean"], expected.mean(axis=0), atol=1e-6)
            np.testing.assert_allclose(stats["observation.state"]["std"], expected.std(axis=0), atol=1e-6)
            self.assertEqual(stats["observation.images.front"]["count"].tolist(), [count])

    def test_legacy_and_native_export_together_and_reindex_stats(self):
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(Path(root) / "raw", cfg, synthetic=True) as store:
            legacy = legacy_episode(store, "episode_01.h5")
            before = legacy.read_bytes()
            self.assertTrue(validate_episode(legacy, store.metadata)["ok"])
            writer = store.start("new task", {})
            for index in range(4):
                writer.append(sample(index, cfg))
            path = store.finish("success")
            write_review(path, "pass", "native fixture")
            destination = Path(root) / "export"
            report = export_dataset(store.root, destination, "local/mixed", allow_synthetic=True)
            self.assertEqual((report["reused_video_episodes"], report["converted_legacy_episodes"]), (1, 1))
            self.assertEqual(legacy.read_bytes(), before)
            self.assertFalse(list(destination.rglob("*.png")))
            dataset = LeRobotDataset("local/mixed", root=destination, video_backend="pyav")
            self.assertEqual(dataset[0]["task"], "legacy task")
            self.assertEqual(dataset[5]["task"], "new task")
            self.assertEqual(dataset.meta.episodes_stats[1]["index"]["min"].tolist(), [5])
            self.assertEqual(dataset.meta.episodes_stats[1]["task_index"]["mean"].tolist(), [1])
            for path in destination.rglob("*.mp4"):
                with av.open(str(path)) as video:
                    self.assertIn(len(list(video.decode(video=0))), (4, 5))

    def test_corrupted_native_files_and_changed_diagnostics_block_export(self):
        cfg = config()
        for corruption in ("missing", "checksum", "diagnostics", "manifest"):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as root, \
                    DatasetStore(Path(root) / "raw", cfg, synthetic=True) as store:
                writer = store.start("integrity", {})
                for index in range(4):
                    writer.append(sample(index, cfg))
                path = store.finish("success")
                write_review(path, "pass", "before corruption")
                video = next(path.with_suffix("").rglob("*.mp4"))
                if corruption == "missing":
                    video.unlink()
                elif corruption == "checksum":
                    data = bytearray(video.read_bytes())
                    data[len(data) // 2] ^= 1
                    video.write_bytes(data)
                elif corruption == "diagnostics":
                    with h5py.File(path, "r+") as file:
                        file["action"][0, 0] += .1
                else:
                    (path.with_suffix("") / "meta/nero_files.json").write_text("{}")
                with self.assertRaisesRegex(ValueError, "failed validation.*invalid_native"):
                    export_dataset(store.root, Path(root) / "export", "local/bad", allow_synthetic=True)
                self.assertFalse((Path(root) / "export").exists())

    def test_encoder_finalize_failure_does_not_publish_episode(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            writer = store.start("partial", {})
            writer.append(sample(0, cfg))
            with patch.object(StreamingEpisode, "finish", side_effect=OSError("test disk failure")):
                with self.assertRaisesRegex(RuntimeError, "inprogress.*test disk failure"):
                    store.finish("interrupted")
            self.assertFalse(writer.worker.is_alive())
            self.assertTrue(writer.partial.exists())
            self.assertFalse(writer.destination.exists())
            self.assertTrue((writer.partial.with_suffix("") / ".recording-incomplete").exists())

    def test_deletion_rejects_native_directory_symlink(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            writer = store.start("protected files", {})
            writer.append(sample(0, cfg))
            path = store.finish("discarded")
            native = path.with_suffix("")
            elsewhere = Path(root) / "keep"
            shutil.move(native, elsewhere)
            native.symlink_to(elsewhere, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "native episode directory"):
                store.delete_episodes([path.name])
            self.assertTrue(path.exists())
            self.assertTrue((elsewhere / "meta/info.json").is_file())

    def test_online_image_statistics_match_pixels(self):
        stats = ImageStats()
        rng = np.random.default_rng(7)
        frames = rng.integers(0, 256, (5, 48, 64, 3), dtype=np.uint8)
        for rgb in frames:
            stats.add(rgb)
        actual = stats.result()
        for key, operation in (("mean", np.mean), ("std", np.std), ("min", np.min), ("max", np.max)):
            expected = operation(frames.astype(np.float64) / 255., axis=(0, 1, 2)).reshape(3, 1, 1)
            np.testing.assert_allclose(actual[key], expected, atol=1e-12)
        self.assertEqual(actual["count"].tolist(), [len(frames)])


if __name__ == "__main__":
    unittest.main()
