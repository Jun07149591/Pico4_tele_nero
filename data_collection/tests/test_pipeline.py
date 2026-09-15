from contextlib import ExitStack
import copy
import json
import os
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import h5py
import numpy as np

from nero_pico_data.cameras import Camera, CameraRig, Frame
from nero_pico_data.capture import CaptureController
from nero_pico_data.eva_alignment import hold, interpolate, nearest_index
from nero_pico_data.export import export_dataset, select_episodes
from nero_pico_data.openpi_policy import NeroInputs, NeroOutputs
from nero_pico_data.quality import summary, validate_episode, write_review
from nero_pico_data.receiver import TelemetryReceiver
from nero_pico_data.schema import load_config, manifest, vector, vector_names
from nero_pico_data.server import create_server
from nero_pico_data.storage import DatasetStore, read_rgb
from nero_pico_data.sync import SampleInvalid, Synchronizer
from nero_pico_data.synthetic import SyntheticSource


def config(mode="single"):
    value = load_config(Path(__file__).resolve().parents[1] / "config/demo.json", mode=mode)
    value.update(image_width=64, image_height=48, minimum_free_gb=.001, min_episode_frames=3)
    return value


def packet(timestamp, mode="single", session="test", healthy=True):
    names = ["right_arm"] if mode == "single" else ["left_arm", "right_arm"]
    arms = {}
    for i, name in enumerate(names):
        arms[name] = {"state_joints_rad": [timestamp + i] * 7, "state_gripper_m": .03,
            "action_joints_rad": [timestamp + i + .1] * 7, "action_gripper_m": .04,
            "feedback_monotonic": timestamp, "gripper_feedback_monotonic": timestamp,
            "last_command_monotonic": timestamp, "ready": True, "home_state": "complete",
            "output_state": "ACTIVE", "input_healthy": healthy, "gripper_enabled": True}
    return {"monotonic": timestamp, "session_id": session, "arms": arms, "wall_time_ns": 1000000000,
            "schema_version": 1, "sequence": int(timestamp * 1000), "configurations": {}}


def sample(index, cfg):
    stamp = 1 + index / cfg["fps"]
    p = packet(stamp, cfg["mode"])
    image = np.zeros((cfg["image_height"], cfg["image_width"], 3), dtype=np.uint8)
    image[:, :image.shape[1] // 2, 0] = 220
    return {"monotonic": stamp, "action_monotonic": stamp, "wall_time_ns": 1000000000 + int(stamp * 1e9),
            "state": vector(p, cfg["mode"], "state"), "action": vector(p, cfg["mode"], "action"),
            "images": {role: Frame(image, stamp, stamp * 1000, index) for role in cfg["cameras"]},
            "diagnostics": {"observation": p}}


class AlignmentTests(unittest.TestCase):
    def test_equal_rate_camera_jitter_uses_next_unique_frame_with_bounded_offset(self):
        cfg, receiver, cameras = self.streams()
        self.assertEqual(cfg["fps"], 30)
        for camera in cameras.values():
            camera.history.clear()
            camera.history.extend(Frame(np.zeros((48, 64, 3), np.uint8), stamp, stamp * 1000, index)
                                  for index, stamp in enumerate((.98, 1.01, 1.06)))
        sync = Synchronizer(cfg, receiver, cameras)
        first = sync.sample(1.)
        second = sync.sample(1. + 1 / 30)
        for role in cameras:
            self.assertEqual(first["images"][role].sequence, 1)
            self.assertEqual(second["images"][role].sequence, 2)
            self.assertLess(abs(second["images"][role].monotonic - second["monotonic"]), 1 / 30)
        with self.assertRaisesRegex(SampleInvalid, "repeated"):
            sync.sample(1. + 1 / 30)

    def test_eva_nearest_and_bounded_interpolation(self):
        self.assertEqual(nearest_index([1., 2.], 1.5), 0)
        np.testing.assert_allclose(interpolate([1., 2.], [[0.], [4.]], 1.25, 1.), [1.])
        self.assertEqual(hold([1., 2.], [.02, .05], 1.9), .02)
        with self.assertRaises(ValueError):
            interpolate([1., 2.], [[0.], [4.]], 1.25, .2)
        with self.assertRaises(ValueError):
            hold([2.], [.05], 1.9)

    def streams(self, mode="single"):
        cfg = config(mode)
        receiver = TelemetryReceiver("/unused")
        receiver.history.extend([packet(t, mode) for t in (.96, .98, 1.02, 1.04)])
        cameras = {role: Camera({}, 64, 48) for role in cfg["cameras"]}
        for camera in cameras.values():
            camera.history.extend([Frame(np.zeros((48, 64, 3), np.uint8), t, t * 1000, i) for i, t in enumerate((.97, 1.01, 1.04))])
        return cfg, receiver, cameras

    def test_state_interpolates_action_holds_and_dual_order(self):
        cfg, receiver, cameras = self.streams("dual")
        result = Synchronizer(cfg, receiver, cameras).sample(1.)
        self.assertEqual(result["state"].shape, (16,))
        np.testing.assert_allclose(result["state"][:7], 1.)
        np.testing.assert_allclose(result["state"][8:15], 2.)
        np.testing.assert_allclose(result["action"][:7], 1.08)
        self.assertAlmostEqual(result["state"][7], .03)
        self.assertAlmostEqual(result["action"][7], .04)
        self.assertEqual(result["images"]["front"].monotonic, 1.01)

    def test_invalid_input_future_session_change_and_duplicate_camera(self):
        for field, value in (("input_healthy", False), ("gripper_enabled", False)):
            cfg, receiver, cameras = self.streams()
            receiver.history[-1]["arms"]["right_arm"][field] = value
            with self.subTest(field=field), self.assertRaises(SampleInvalid):
                Synchronizer(cfg, receiver, cameras).sample(1.)
        cfg, receiver, cameras = self.streams()
        receiver.history[-1]["session_id"] = "restart"
        with self.assertRaises(SampleInvalid):
            Synchronizer(cfg, receiver, cameras).sample(1.)
        cfg, receiver, cameras = self.streams()
        sync = Synchronizer(cfg, receiver, cameras)
        sync.sample(1.)
        with self.assertRaisesRegex(SampleInvalid, "repeated"):
            sync.sample(1.001)

    def test_home_return_keeps_recording_measured_state_and_sent_action(self):
        for mode in ("single", "dual"):
            cfg, receiver, cameras = self.streams(mode)
            for value in receiver.history:
                for arm in value["arms"].values():
                    arm.update(home_state="returning", clutch_held=False)
            result = Synchronizer(cfg, receiver, cameras).sample(1.)
            np.testing.assert_allclose(result["state"][:7], 1.)
            np.testing.assert_allclose(result["action"][:7], 1.08)
            self.assertEqual(result["diagnostics"]["observation"]["arms"]["right_arm"]["home_state"], "returning")

    def test_policy_masks_missing_camera_and_keeps_gripper_units(self):
        cfg = config("dual")
        data = sample(0, cfg)
        result = NeroInputs("dual")({"state": data["state"], "actions": data["action"][None, :],
            "images": {r: np.transpose(f.rgb, (2, 0, 1)) / 255 for r, f in data["images"].items()}, "prompt": "pick"})
        self.assertEqual(result["image"]["base_0_rgb"].shape, (48, 64, 3))
        self.assertFalse(result["image_mask"]["left_wrist_0_rgb"])
        self.assertTrue(result["image_mask"]["right_wrist_0_rgb"])
        self.assertAlmostEqual(result["actions"][0, 15], .04)
        self.assertEqual(NeroOutputs("dual")({"actions": np.zeros((20, 32))})["actions"].shape, (20, 16))


class StorageTests(unittest.TestCase):
    def test_delete_selected_episodes_preserves_exports_and_never_reuses_identifiers(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root:
            with DatasetStore(root, cfg, synthetic=True) as store:
                paths = []
                for _ in range(3):
                    writer = store.start("pick", {})
                    for index in range(4):
                        writer.append(sample(index, cfg))
                    path = store.finish("success")
                    write_review(path, "pass", "kept with episode")
                    paths.append(path)
                export = Path(root) / "exports" / "independent.mp4"
                export.parent.mkdir()
                export.write_bytes(b"independent exported copy")
                (Path(root) / "episode_sequence.json").unlink()
                for invalid in ([], [paths[0].name] * 2, [paths[0].name, "episode_99.h5"],
                                ["../episode_01.h5"], ["episode_../episode_01.h5"], "all", [None]):
                    with self.subTest(identifiers=invalid), self.assertRaises(ValueError):
                        store.delete_episodes(invalid)
                    self.assertTrue(all(path.exists() and path.with_suffix(".review.json").exists() for path in paths))
                deleted = store.delete_episodes([paths[2].name, paths[0].name])
                self.assertEqual(deleted["deleted"], [paths[2].name, paths[0].name])
                for path in (paths[0], paths[2]):
                    self.assertFalse(path.exists())
                    self.assertFalse(path.with_suffix(".review.json").exists())
                self.assertTrue(paths[1].exists())
                self.assertEqual(export.read_bytes(), b"independent exported copy")
            with DatasetStore(root, cfg, synthetic=True) as store:
                writer = store.start("next", {})
                self.assertEqual(writer.identifier, "episode_04")
                with self.assertRaisesRegex(ValueError, "finish recording"):
                    store.delete_episodes([paths[1].name])
                self.assertTrue(paths[1].exists())
                store.finish("discarded")
                store.delete_episodes([paths[1].name, "episode_04.h5"])
            with DatasetStore(root, cfg, synthetic=True) as store:
                self.assertEqual(store.start("after deleting all", {}).identifier, "episode_05")
                store.finish("discarded")

    def test_delete_rejects_symlinks_without_touching_other_files(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            target = Path(root) / "unrelated.h5"
            target.write_bytes(b"keep")
            linked = Path(root) / "episodes" / "episode_01.h5"
            linked.symlink_to(target)
            with self.assertRaises(ValueError):
                store.delete_episodes([linked.name])
            self.assertEqual(target.read_bytes(), b"keep")
            linked.unlink()
            writer = store.start("review symlink", {})
            writer.append(sample(0, cfg))
            path = store.finish("discarded")
            path.with_suffix(".review.json").symlink_to(target)
            with self.assertRaisesRegex(ValueError, "review file"):
                store.delete_episodes([path.name])
            self.assertTrue(path.exists())
            self.assertEqual(target.read_bytes(), b"keep")

    def test_legacy_time_limit_does_not_require_a_new_dataset(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root:
            cfg["max_episode_seconds"] = 180
            with DatasetStore(root, cfg, synthetic=True):
                original = (Path(root) / "manifest.json").read_bytes()
            cfg["max_episode_seconds"] = None
            with DatasetStore(root, cfg, synthetic=True):
                self.assertEqual((Path(root) / "manifest.json").read_bytes(), original)

    def test_capture_rate_changes_preserve_old_episodes_and_survive_restart(self):
        cfg = config()
        self.assertEqual(cfg["fps"], 30)
        with tempfile.TemporaryDirectory() as root:
            cfg["fps"] = 20
            with DatasetStore(root, cfg, synthetic=True) as store:
                original_manifest = (Path(root) / "manifest.json").read_bytes()
                old_writer = store.start("old rate", {})
                for index in range(4):
                    old_writer.append(sample(index, cfg))
                old_path = store.finish("success")
                original_data = old_path.read_bytes()
                write_review(old_path, "pass", "old")
            cfg = config()
            with DatasetStore(root, cfg, synthetic=True) as store:
                self.assertEqual(cfg["fps"], 30)
                for fps in (0, 31, 60, True, 20.5, "30"):
                    with self.subTest(fps=fps), self.assertRaises(ValueError):
                        store.set_fps(fps)
                self.assertEqual(store.set_fps(15), {"fps": 15})
                writer = store.start("new rate", {})
                with self.assertRaisesRegex(ValueError, "finish recording"):
                    store.set_fps(30)
                for index in range(4):
                    writer.append(sample(index, cfg))
                new_path = store.finish("success")
                write_review(new_path, "pass", "new")
                self.assertEqual(old_path.name, "episode_01.h5")
                self.assertEqual(new_path.name, "episode_02.h5")
                self.assertEqual(summary(old_path)["fps"], 20)
                self.assertEqual(summary(new_path)["fps"], 15)
                self.assertTrue(validate_episode(old_path, store.metadata)["ok"])
                self.assertTrue(validate_episode(new_path, store.metadata)["ok"])
                with self.assertRaisesRegex(ValueError, "different capture rates"):
                    select_episodes(root, [old_path.name, new_path.name])
                self.assertEqual(select_episodes(root, [new_path.name])[0]["fps"], 15)
                self.assertEqual(old_path.read_bytes(), original_data)
                self.assertEqual((Path(root) / "manifest.json").read_bytes(), original_manifest)
            with DatasetStore(root, config(), synthetic=True) as store:
                self.assertEqual(store.config["fps"], 15)

    def test_selected_export_renumbers_only_chosen_episodes_and_contains_decodable_videos(self):
        import av
        import pyarrow.parquet as pq
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        cfg = config()
        with tempfile.TemporaryDirectory() as root:
            source, destination = Path(root) / "raw", Path(root) / "local" / "selected"
            identifiers = []
            with DatasetStore(source, cfg, synthetic=True) as store:
                for episode in range(3):
                    writer = store.start(f"task {episode + 1}", {})
                    for index in range(5 + episode):
                        writer.append(sample(index, cfg))
                    path = store.finish("success")
                    write_review(path, "pass", "verified")
                    identifiers.append(path.name)
                for invalid in ([], [identifiers[0]] * 2, ["../episode_01.h5"], ["episode_99.h5"], "all"):
                    with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                        select_episodes(source, invalid)
                write_review(source / "episodes" / identifiers[1], "fail", "excluded")
                with self.assertRaisesRegex(ValueError, "successful with PASS"):
                    select_episodes(source, [identifiers[1]])
            report = export_dataset(source, destination, "local/selected", allow_synthetic=True,
                                    episode_ids=[identifiers[2], identifiers[0]])
            self.assertEqual((report["episodes"], report["frames"], report["fps"]), (2, 12, 30))
            provenance = json.loads((destination / "meta/nero_provenance.json").read_text())
            spec = json.loads((destination / "meta/nero_openpi.json").read_text())
            self.assertEqual(report["repo_id"], "local/selected")
            self.assertEqual(provenance["repo_id"], report["repo_id"])
            self.assertEqual((spec["repo_id"], spec["dataset_root"]), (report["repo_id"], str(destination)))
            self.assertEqual([s["source"] for s in provenance["sources"]], [identifiers[0], identifiers[2]])
            self.assertEqual([s["name"] for s in provenance["sources"]], ["episode_000000", "episode_000001"])
            self.assertEqual((report["parquet_files"], report["video_files"]), (2, 4))
            self.assertEqual(len(list(destination.rglob("*.parquet"))), 2)
            self.assertEqual(len(list(destination.rglob("*.mp4"))), 4)
            self.assertFalse((destination / "clips").exists())
            self.assertNotIn("clips_root", report)
            self.assertEqual(report["videos_root"], str(destination / "videos"))
            self.assertEqual([f["parquet"] for f in report["files"]],
                             ["data/chunk-000/episode_000000.parquet", "data/chunk-000/episode_000001.parquet"])
            for item in provenance["sources"]:
                self.assertNotIn("clips", item)
                table = pq.read_table(destination / item["parquet"])
                self.assertEqual(table.num_rows, item["frames"])
                self.assertEqual(table["episode_index"].unique().to_pylist(), [item["episode_index"]])
                self.assertEqual(table["frame_index"].to_pylist(), list(range(item["frames"])))
                for filename in item["videos"].values():
                    with av.open(str(destination / filename)) as video:
                        self.assertEqual(video.streams.video[0].average_rate, 30)
                        self.assertEqual(video.streams.video[0].codec_context.name, "h264")
                        frames = list(video.decode(video=0))
                        self.assertEqual(len(frames), item["frames"])
                        self.assertEqual((frames[0].width, frames[0].height), (64, 48))
            dataset = LeRobotDataset("local/selected", root=destination, video_backend="pyav")
            self.assertEqual(dataset[0]["task"], "task 1")
            self.assertEqual(dataset[5]["task"], "task 3")
            self.assertEqual(tuple(dataset[5]["observation.images.front"].shape), (3, 48, 64))

    def test_invalid_outcome_does_not_orphan_the_writer(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            writer = store.start("pick", {})
            writer.append(sample(0, cfg))
            with self.assertRaisesRegex(ValueError, "invalid episode outcome"):
                store.finish("unknown")
            self.assertIs(store.episode, writer)
            store.finish("discarded")
            self.assertFalse(writer.worker.is_alive())

    def test_lock_mode_integrity_and_interrupted_retention(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root:
            with DatasetStore(root, cfg, synthetic=True) as store:
                lock_path = Path(root) / ".writer.lock"
                owner = lock_path.read_bytes()
                with self.assertRaisesRegex(BlockingIOError, f"dataset already in use.*PID {os.getpid()}"):
                    with DatasetStore(root, cfg, synthetic=True):
                        pass
                self.assertEqual(lock_path.read_bytes(), owner)
                writer = store.start("pick", {})
                writer.append(sample(0, cfg))
                with self.assertRaisesRegex(ValueError, "too short"):
                    store.finish("success")
            path = next((Path(root) / "episodes").glob("*.h5"))
            self.assertEqual(summary(path)["outcome"], "interrupted")
            self.assertFalse(validate_episode(path, manifest(cfg, True))["ok"])
            with self.assertRaisesRegex(ValueError, "differ"):
                with DatasetStore(root, config("dual"), synthetic=True):
                    pass
            with DatasetStore(root, cfg, synthetic=True):
                self.assertEqual(json.loads(lock_path.read_text())["pid"], os.getpid())

    def test_real_lerobot_export_load_and_action_chunks(self):
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        for mode in ("single", "dual"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as root:
                cfg = config(mode)
                source = Path(root) / "raw"
                with DatasetStore(source, cfg, synthetic=True) as store:
                    for outcome in ("success", "failure"):
                        writer = store.start("put the block in the tray", {})
                        for i in range(4):
                            writer.append(sample(i, cfg))
                        path = store.finish(outcome)
                        self.assertTrue(validate_episode(path, store.metadata)["ok"])
                        write_review(path, "pass", "verified")
                destination = Path(root) / "local" / f"nero_{mode}"
                with self.assertRaisesRegex(ValueError, "synthetic"):
                    export_dataset(source, destination, f"local/nero_{mode}")
                report = export_dataset(source, destination, f"local/nero_{mode}", allow_synthetic=True)
                self.assertEqual(report["episodes"], 1)
                self.assertFalse((destination / ".export-incomplete").exists())
                dataset = LeRobotDataset(f"local/nero_{mode}", root=destination,
                    delta_timestamps={"action": [i / cfg["fps"] for i in range(3)]})
                frame = dataset[0]
                self.assertEqual(tuple(frame["action"].shape), (3, len(vector_names(mode))))
                self.assertEqual(tuple(frame["observation.images.front"].shape), (3, 48, 64))
                np.testing.assert_allclose(frame["action"][0], sample(0, cfg)["action"])
                self.assertGreater(float(frame["observation.images.front"][0, 0, 0]), .7)
                self.assertIn("put the block", frame["task"])
                self.assertTrue(dataset[3]["action_is_pad"][-1])
                with self.assertRaisesRegex(ValueError, "already exists"):
                    export_dataset(source, destination, f"local/nero_{mode}", allow_synthetic=True)

    def test_writer_failure_keeps_partial_and_rejects_invalid_image(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            writer = store.start("pick", {})
            with patch("nero_pico_data.storage.cv2.imencode", return_value=(False, None)):
                writer.append(sample(0, cfg))
                with self.assertRaisesRegex(RuntimeError, "inprogress"):
                    store.finish("interrupted")
            self.assertEqual(len(list((Path(root) / ".inprogress").glob("*.h5"))), 1)
            self.assertFalse(list((Path(root) / "episodes").glob("*.h5")))


class ControllerTests(unittest.TestCase):
    def wait_for(self, predicate, timeout=3.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(predicate())

    def test_unavailable_camera_keeps_workbench_open_without_allowing_recording(self):
        cfg = config()
        def start(camera):
            if camera.settings["serial"] == cfg["cameras"]["front"]["serial"]:
                raise RuntimeError("configured front camera is disconnected")
            return camera
        with patch.object(Camera, "__enter__", start), patch.object(Camera, "__exit__") as close:
            with CameraRig(cfg, allow_unavailable=True) as rig:
                self.assertIn("disconnected", rig.cameras["front"].error)
                self.assertIsNone(rig.cameras["front"].latest_before(time.monotonic()))
                self.assertIsNone(rig.cameras["right_wrist"].error)
            self.assertEqual(close.call_count, 1)
            with self.assertRaisesRegex(RuntimeError, "disconnected"), CameraRig(cfg):
                pass

    def test_idle_teleop_with_released_grip_is_ready_and_reports_precise_blockers(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            receiver = TelemetryReceiver("/unused")
            cameras = {role: Camera({}, 64, 48) for role in cfg["cameras"]}
            controller = CaptureController(cfg, store, receiver, cameras)
            now = time.monotonic()
            data = packet(now)
            arm = data["arms"]["right_arm"]
            arm.update(output_state="HOLDING", clutch_held=False)
            receiver.history.append(data)
            for camera in cameras.values():
                camera.history.append(Frame(np.zeros((48, 64, 3), np.uint8), now, 0., 1))
            self.assertTrue(controller.status()["ready"])
            self.assertFalse(controller.status()["arm_status"]["right_arm"]["clutch_held"])
            arm["home_state"] = "returning"
            self.assertTrue(controller.status()["ready"])
            for field, value, reason in (("input_healthy", False, "input lost"),
                                         ("gripper_enabled", False, "gripper unavailable"),
                                         ("gripper_feedback_monotonic", now - 1., "gripper_feedback_monotonic missing/stale")):
                old = arm[field]
                arm[field] = value
                with self.subTest(field=field):
                    self.assertEqual(controller.status()["readiness_reason"], "right_arm: " + reason)
                arm[field] = old
            receiver.history.clear()
            status = controller.status()
            self.assertEqual(status["readiness_reason"], "telemetry missing or stale")
            self.assertTrue(all(camera["ready"] for camera in status["camera_status"].values()))
            self.assertEqual(status["telemetry_socket"], "/unused")

    def test_offline_review_uses_no_camera_or_socket(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, DatasetStore(root, cfg, synthetic=True) as store:
            writer = store.start("pick", {})
            for i in range(3):
                writer.append(sample(i, cfg))
            path = store.finish("success")
            receiver = TelemetryReceiver("/unused")
            cameras = {role: Camera({}, 64, 48) for role in cfg["cameras"]}
            with CaptureController(cfg, store, receiver, cameras, review_only=True) as controller:
                self.assertFalse(controller.status()["ready"])
                result = controller.command("review", id=path.name, verdict="pass", notes="offline")
                self.assertEqual(result["review"]["verdict"], "pass")
                with self.assertRaisesRegex(ValueError, "offline review"):
                    controller.command("start", task="cannot record")
                with self.assertRaisesRegex(ValueError, "offline review"):
                    controller.command("configure", fps=15)
            self.assertIsNone(receiver.sock)
            self.assertTrue(all(camera.device is None for camera in cameras.values()))

    def test_manual_finish_survives_time_limit_and_input_loss_with_explicit_gaps(self):
        cfg = config()
        cfg["max_episode_seconds"] = 1
        with tempfile.TemporaryDirectory() as root, ExitStack() as stack:
            store = stack.enter_context(DatasetStore(root, cfg, synthetic=True))
            source = stack.enter_context(SyntheticSource(cfg))
            controller = stack.enter_context(CaptureController(cfg, store, source, source.cameras))
            self.wait_for(lambda: controller.status()["ready"])
            controller.command("start", task="test movement")
            writer = store.episode
            self.wait_for(lambda: controller.status()["elapsed_s"] > 1.2)
            self.assertGreater(controller.status()["frames"], 20)
            self.assertEqual(controller.status()["state"], "recording", controller.status())
            source.stop.set()
            source.worker.join()
            self.wait_for(lambda: controller.status()["skipped_frames"] > 0)
            self.assertEqual(controller.status()["state"], "recording")
            self.assertTrue(controller.status()["capture_wait_reason"])
            self.assertIs(store.episode, writer)
            self.assertFalse(list((Path(root) / "episodes").glob("*.h5")))
            frames = controller.status()["frames"]
            source.stop.clear()
            source.__enter__()
            self.wait_for(lambda: controller.status()["frames"] >= frames + 4)
            self.assertIsNone(controller.status()["capture_wait_reason"])
            result = controller.command("finish", outcome="success")
            self.assertEqual(result["outcome"], "success")
            self.assertEqual(controller.status()["state"], "idle")
            self.assertGreater(result["capture_gaps"], 0)
            path = Path(root) / "episodes" / result["id"]
            with h5py.File(path, "r") as file:
                gaps = [json.loads(value) for value in file["capture_gaps"]]
                self.assertGreater(gaps[0]["skipped_frames"], 0)
                self.assertGreater(gaps[0]["end_monotonic"], gaps[0]["start_monotonic"])
                self.assertEqual(file.attrs["recording_policy"], "manual_finish")
            self.assertIn("capture_data_gaps", validate_episode(path, store.metadata)["issues"])
            with self.assertRaisesRegex(ValueError, "capture_data_gaps"):
                controller.command("review", id=result["id"], verdict="pass")

    def test_manual_finish_while_data_is_missing_persists_trailing_gap(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, ExitStack() as stack:
            store = stack.enter_context(DatasetStore(root, cfg, synthetic=True))
            source = stack.enter_context(SyntheticSource(cfg))
            controller = stack.enter_context(CaptureController(cfg, store, source, source.cameras))
            self.wait_for(lambda: controller.status()["ready"])
            controller.command("start", task="stop during dropout")
            self.wait_for(lambda: controller.status()["frames"] >= 4)
            source.stop.set()
            source.worker.join()
            self.wait_for(lambda: controller.status()["skipped_frames"] > 0)
            result = controller.command("finish", outcome="failure")
            self.assertEqual(result["outcome"], "failure")
            self.assertGreater(result["capture_gaps"], 0)
            self.assertEqual(controller.status()["state"], "idle")

    def test_late_feedback_is_retried_without_losing_the_sample(self):
        cfg = config()
        original_sample = Synchronizer.sample
        first_timestamp = []
        def delayed(sync, timestamp):
            if not first_timestamp:
                first_timestamp.append(timestamp)
            if timestamp == first_timestamp[0] and time.monotonic() < timestamp + .14:
                raise SampleInvalid("feedback does not bracket sample time")
            return original_sample(sync, timestamp)
        with tempfile.TemporaryDirectory() as root, ExitStack() as stack:
            store = stack.enter_context(DatasetStore(root, cfg, synthetic=True))
            source = stack.enter_context(SyntheticSource(cfg))
            controller = stack.enter_context(CaptureController(cfg, store, source, source.cameras))
            self.wait_for(lambda: controller.status()["ready"])
            with patch.object(Synchronizer, "sample", delayed):
                controller.command("start", task="late feedback")
                self.wait_for(lambda: controller.status()["frames"] >= 4)
                result = controller.command("finish", outcome="success")
            self.assertEqual(controller.status()["skipped_frames"], 0)
            path = Path(root) / "episodes" / result["id"]
            self.assertTrue(validate_episode(path, store.metadata)["ok"])

    def test_writer_backpressure_retries_the_same_aligned_sample(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as root, ExitStack() as stack:
            store = stack.enter_context(DatasetStore(root, cfg, synthetic=True))
            source = stack.enter_context(SyntheticSource(cfg))
            controller = stack.enter_context(CaptureController(cfg, store, source, source.cameras))
            self.wait_for(lambda: controller.status()["ready"])
            controller.command("start", task="temporary writer backlog")
            writer = store.episode
            original_append = writer.append
            attempts = []
            def delayed(sample):
                attempts.append((sample["monotonic"], sample["images"]["front"].sequence))
                if len(attempts) < 3:
                    raise RuntimeError("writer queue full")
                original_append(sample)
            with patch.object(writer, "append", delayed):
                self.wait_for(lambda: controller.status()["frames"] >= 4)
                result = controller.command("finish", outcome="success")
            self.assertEqual(attempts[:3], [attempts[0]] * 3)
            self.assertEqual(controller.status()["skipped_frames"], 0)
            path = Path(root) / "episodes" / result["id"]
            self.assertTrue(validate_episode(path, store.metadata)["ok"])


class ServerTests(unittest.TestCase):
    def test_delete_waits_for_export_and_works_in_offline_review(self):
        cfg = config()
        release, exporting = threading.Event(), threading.Event()
        def export(*_args, **_kwargs):
            exporting.set()
            if not release.wait(5):
                raise RuntimeError("test export timed out")
            return {"episodes": 1}
        with tempfile.TemporaryDirectory() as root, ExitStack() as stack:
            store = stack.enter_context(DatasetStore(root, cfg, synthetic=True))
            writer = store.start("pick", {})
            for index in range(4):
                writer.append(sample(index, cfg))
            path = store.finish("success")
            write_review(path, "pass", "")
            controller = stack.enter_context(CaptureController(cfg, store, TelemetryReceiver("/unused"), {}, review_only=True))
            stack.enter_context(patch("nero_pico_data.server.export_dataset", side_effect=export))
            server = create_server(controller, 0)
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            def request(route, body=None, headers=None):
                data = json.dumps(body).encode() if body is not None else None
                req = Request(f"http://127.0.0.1:{server.server_port}{route}", data=data,
                              headers=headers if headers is not None else {"Content-Type": "application/json", "X-Nero-Request": "1"})
                try:
                    response = urlopen(req, timeout=3)
                except HTTPError as exc:
                    response = exc
                with response:
                    return response.status, json.load(response)
            deletion = {"command": "delete", "episode_ids": [path.name]}
            try:
                status, _ = request("/api/export", {"repo_id": "local/test", "episode_ids": [path.name], "allow_synthetic": True})
                self.assertEqual(status, 202)
                self.assertTrue(exporting.wait(2))
                status, result = request("/api/command", deletion)
                self.assertEqual(status, 400)
                self.assertIn("export is running", result["error"])
                self.assertTrue(path.exists())
                release.set()
                deadline = time.monotonic() + 3
                while request("/api/status")[1]["export"]["state"] == "running" and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertEqual(request("/api/command", deletion, headers={"Content-Type": "application/json"})[0], 403)
                self.assertTrue(path.exists())
                status, result = request("/api/command", deletion)
                self.assertEqual(status, 200)
                self.assertEqual(result["deleted"], [path.name])
                self.assertEqual(request("/api/episodes")[1], [])
                self.assertEqual(request("/api/episode?id=" + path.name)[0], 400)
                self.assertFalse(path.with_suffix(".review.json").exists())
            finally:
                release.set()
                server.shutdown()
                server.server_close()
                worker.join()
                server.export_pool.shutdown(wait=True)


class ReceiverTests(unittest.TestCase):
    def test_unix_receiver_rejects_stale_and_out_of_order_packets(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "data.sock"
            with TelemetryReceiver(path) as receiver, socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
                now = time.monotonic()
                valid = packet(now)
                for value in (packet(now - 1), valid, valid):
                    sender.sendto(json.dumps(value).encode(), str(path))
                deadline = time.monotonic() + 1
                while receiver.rejected_packets < 2 and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertEqual(receiver.rejected_packets, 2)
                self.assertEqual(len(receiver.snapshot()), 1)
                self.assertEqual(receiver.latest()["monotonic"], now)
            self.assertFalse(path.exists())

    def test_existing_socket_is_not_removed_by_second_collector(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "data.sock"
            with TelemetryReceiver(path):
                with self.assertRaisesRegex(RuntimeError, "exists"):
                    with TelemetryReceiver(path):
                        pass
                self.assertTrue(path.exists())


if __name__ == "__main__":
    unittest.main()
