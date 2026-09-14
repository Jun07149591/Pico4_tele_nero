import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'data_collection'))
sys.path.insert(0, str(ROOT / 'teleop'))
spec = importlib.util.spec_from_file_location('configure_cameras', ROOT / 'scripts/configure_cameras.py')
cameras = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cameras)


class PortabilityTests(unittest.TestCase):
    def test_archive_excludes_recordings_and_machine_files(self):
        spec = importlib.util.spec_from_file_location('package', ROOT / 'scripts/package.py')
        packaging = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(packaging)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            keep = ('README.md', 'teleop/config/home_poses.json', 'data_collection/nero_pico_data/export.py')
            skip = ('data_collection/config/capture.local.json', 'teleop/logs/real.jsonl',
                    '.runtime/library.so', 'data_collection/data/episode.h5', 'scripts/__pycache__/test.pyc',
                    'vendor/.git/config', 'data_collection/nero.egg-info/SOURCES.txt')
            for name in (*keep, *skip):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            self.assertEqual({str(path.relative_to(root)) for path in packaging.source_files(root)}, set(keep))

    def test_dual_can_dry_run_does_not_change_either_interface(self):
        stub = types.ModuleType('nero_pico_teleop.preflight')
        stub.can_interface = lambda *_: {}
        with patch.dict(sys.modules, {'nero_pico_teleop.preflight': stub}):
            from nero_pico_teleop import can_setup
        with patch.object(sys, 'argv', ['can_setup', '--dual', '--dry-run']), \
                patch.object(can_setup.subprocess, 'check_output', return_value='[{"flags": []}]') as read, \
                patch.object(can_setup.subprocess, 'run') as change, patch('builtins.print'):
            can_setup.main()
            self.assertEqual(read.call_count, 2)
            change.assert_not_called()

    def test_runtime_config_files_are_self_contained(self):
        directory = ROOT / 'teleop/config'
        for name in ('nero_relative_config.json', 'nero_humanoid_config.json'):
            config = json.loads((directory / name).read_text())
            for key in ('robot_config', 'urdf', 'replay_seed_report', 'humanoid_frames', 'home_poses'):
                if key in config:
                    path = (directory / config[key]).resolve()
                    self.assertTrue(path.is_relative_to(ROOT), path)
                    self.assertTrue(path.is_file(), path)

    def test_wrapper_resolves_moved_directory_with_spaces(self):
        with tempfile.TemporaryDirectory() as temporary:
            moved = Path(temporary) / 'renamed project'
            shutil.copytree(ROOT / 'scripts', moved / 'scripts', ignore=shutil.ignore_patterns('__pycache__'))
            environment = dict(os.environ, PICO_NERO_PYTHON=sys.executable, NERO_DATA_PYTHON=sys.executable)
            for script, package in (('teleop_python.sh', 'teleop'), ('data_python.sh', 'data_collection')):
                output = subprocess.check_output(['bash', str(moved / 'scripts' / script), '-c',
                    'import os; print(os.environ["PYTHONPATH"])'], cwd=temporary, env=environment, text=True)
                self.assertEqual(output.strip(), str(moved / package))

    def test_setup_rejects_conflicting_modes_without_installing(self):
        result = subprocess.run(['bash', str(ROOT / 'scripts/setup.sh'), '--data-only', '--teleop-only'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('cannot be combined', result.stderr)

    def test_camera_auto_selection_and_ambiguous_models(self):
        config = json.loads((ROOT / 'data_collection/config/capture.json').read_text())
        devices = [{'name': 'Intel RealSense D455', 'serial': 'front'},
                   {'name': 'Intel RealSense D405', 'serial': 'right'}]
        selected = cameras.assign_cameras(config, devices, {}, True)
        self.assertEqual(selected['cameras']['front']['serial'], 'front')
        self.assertTrue(config['cameras']['front']['serial'].startswith('SET_'))
        devices.append({'name': 'Intel RealSense D405', 'serial': 'left'})
        with self.assertRaisesRegex(ValueError, 'specify serial'):
            cameras.assign_cameras(config, devices, {}, True)
        selected = cameras.assign_cameras(config, devices, {'right_wrist': 'right', 'left_wrist': 'left'}, True)
        self.assertEqual(len(selected['cameras']), 3)

    def test_camera_duplicate_and_disconnected_serials_rejected(self):
        config = json.loads((ROOT / 'data_collection/config/capture.json').read_text())
        devices = [{'name': 'Camera', 'serial': 'only'}]
        with self.assertRaisesRegex(ValueError, 'multiple roles'):
            cameras.assign_cameras(config, devices, {'front': 'only', 'right_wrist': 'only'})
        with self.assertRaisesRegex(ValueError, 'not connected'):
            cameras.assign_cameras(config, devices, {'front': 'absent'})

    def test_openpi_uses_moved_dataset_instead_of_old_machine_path(self):
        from nero_pico_data import openpi_bridge
        registered = []
        stub = types.ModuleType('nero_pico_data.openpi_config')
        stub.register = lambda value: registered.append(value.copy()) or 'pi05_nero_single'
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'new location/local/test'
            (root / 'meta').mkdir(parents=True)
            (root / 'meta/info.json').write_text('{}')
            spec_path = root / 'meta/nero_openpi.json'
            spec_path.write_text(json.dumps({'repo_id': 'local/test', 'synthetic': False,
                                            'dataset_root': '/unavailable/old/local/test'}))
            with patch.dict(sys.modules, {'nero_pico_data.openpi_config': stub}), \
                    patch.object(sys, 'argv', ['openpi_bridge', 'train', '--spec', str(spec_path),
                                               '--openpi', str(Path(temporary) / 'openpi')]), \
                    patch.object(sys, 'path', sys.path.copy()), patch.dict(os.environ), \
                    patch.object(openpi_bridge.os, 'chdir'), patch.object(openpi_bridge.runpy, 'run_path') as run:
                openpi_bridge.main()
                self.assertEqual(registered[0]['dataset_root'], str(root))
                self.assertEqual(os.environ['HF_LEROBOT_HOME'], str(root.parent.parent))
                run.assert_called_once()


if __name__ == '__main__':
    unittest.main()
