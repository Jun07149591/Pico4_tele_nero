from contextlib import ExitStack, redirect_stdout
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from nero_pico_data import cli


class CaptureDirectoryTests(unittest.TestCase):
    def run_capture(self, command, options, expected, config_mode="single", absolute_root=False):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            base = Path(directory)
            project = base / "relocated project"
            caller = base / "unrelated working directory"
            caller.mkdir()
            config = json.loads((Path(__file__).resolve().parents[1] / "config/demo.json").read_text())
            config.update(mode=config_mode, image_width=64, image_height=48)
            config_path = base / "capture.json"
            config_path.write_text(json.dumps(config))
            stack.enter_context(patch.object(cli, "ROOT", project / "data_collection"))
            cameras = stack.enter_context(patch("nero_pico_data.cameras.CameraRig"))
            cameras.return_value.__enter__.return_value.cameras = {}
            stack.enter_context(patch("nero_pico_data.receiver.TelemetryReceiver"))
            server = stack.enter_context(patch("nero_pico_data.server.serve"))
            stack.enter_context(patch("builtins.input", return_value="quit"))
            output = stack.enter_context(redirect_stdout(io.StringIO()))
            previous_cwd = Path.cwd()
            stack.callback(os.chdir, previous_cwd)
            os.chdir(caller)
            argv = [command, "--config", str(config_path), *options]
            if absolute_root:
                expected_root = base / "external data"
                argv.extend(["--root", str(expected_root)])
            else:
                expected_root = base / expected
            self.assertEqual(cli.main(argv), 0)
            metadata = json.loads((expected_root / "manifest.json").read_text())
            self.assertEqual(metadata["synthetic"], command == "demo")
            self.assertIn(str(expected_root), output.getvalue())
            self.assertEqual(list(base.rglob("manifest.json")), [expected_root / "manifest.json"])
            if command == "record":
                server.assert_not_called()
            else:
                self.assertEqual(server.call_args.args[0].store.root, expected_root)
            if command == "demo":
                cameras.assert_not_called()

    def test_defaults_use_relocated_project_and_effective_mode(self):
        cases = [
            ("serve", ["--mode", "single"], "right_arm", "single"),
            ("serve", ["--mode", "dual"], "dual_arm", "single"),
            ("serve", [], "dual_arm", "dual"),
            ("serve", ["--mode", "single"], "right_arm", "dual"),
            ("record", ["--mode", "dual", "--task", "pick"], "dual_arm", "single"),
            ("demo", [], "demo", "single"),
        ]
        for command, options, folder, mode in cases:
            with self.subTest(command=command, options=options, config_mode=mode):
                self.run_capture(command, options, f"relocated project/data/{folder}", mode)

    def test_explicit_roots_preserve_absolute_and_caller_relative_paths(self):
        self.run_capture("serve", ["--root", "chosen data"], "unrelated working directory/chosen data")
        self.run_capture("serve", [], None, absolute_root=True)


if __name__ == "__main__":
    unittest.main()
