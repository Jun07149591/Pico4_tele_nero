import argparse
from contextlib import ExitStack
import json
from pathlib import Path

from .schema import DEFAULT_CONFIG, ROOT, default_socket, load_config


def main(argv=None):
    parser = argparse.ArgumentParser(description="Nero/PICO demonstration collection and OpenPI export")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("cameras", help="list RealSense device serials")
    command = commands.add_parser("review", help="review/export an existing dataset without hardware")
    command.add_argument("--root", type=Path, required=True)
    command.add_argument("--port", type=int, default=8765)
    for name in ("serve", "record", "demo"):
        command = commands.add_parser(name)
        command.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
        command.add_argument("--mode", choices=("single", "dual"))
        command.add_argument("--root", type=Path,
                             help="dataset directory (default: project/data/right_arm, dual_arm, or demo; "
                                  "explicit relative paths use the current working directory)")
        command.add_argument("--socket", type=Path, default=default_socket())
        if name != "record":
            command.add_argument("--port", type=int, default=8765)
        else:
            command.add_argument("--task", required=True)
    command = commands.add_parser("validate")
    command.add_argument("--root", type=Path, required=True)
    command = commands.add_parser("export")
    command.add_argument("--root", type=Path, required=True)
    command.add_argument("--destination", type=Path, required=True)
    command.add_argument("--repo-id", required=True)
    command.add_argument("--allow-synthetic", action="store_true")
    command.add_argument("--episodes", nargs="+", help="raw episode filenames to include, e.g. episode_01.h5 episode_03.h5")
    args = parser.parse_args(argv)
    try:
        if args.command == "cameras":
            from .cameras import discover
            print(json.dumps(discover(), indent=2))
            return 0
        if args.command == "validate":
            from .quality import validate_episode
            metadata = json.loads((args.root / "manifest.json").read_text())
            reports = [validate_episode(path, metadata) for path in sorted((args.root / "episodes").glob("episode_*.h5"))]
            print(json.dumps({"episodes": reports, "unfinished": [p.name for p in (args.root / ".inprogress").glob("*.h5")]}, indent=2))
            return 0 if reports and all(r["ok"] for r in reports) else 1
        if args.command == "export":
            from .export import export_dataset
            print(json.dumps(export_dataset(args.root, args.destination, args.repo_id,
                                            allow_synthetic=args.allow_synthetic, episode_ids=args.episodes), indent=2))
            return 0
        from .cameras import Camera, CameraRig
        from .capture import CaptureController
        from .receiver import TelemetryReceiver
        from .storage import DatasetStore
        if args.command == "review":
            metadata = json.loads((args.root / "manifest.json").read_text())
            config, synthetic = metadata["config"], metadata["synthetic"]
        else:
            config, synthetic = load_config(args.config, args.mode), args.command == "demo"
            if args.root is None:
                folder = "demo" if synthetic else {"single": "right_arm", "dual": "dual_arm"}[config["mode"]]
                args.root = ROOT.parent / "data" / folder
        with ExitStack() as stack:
            store = stack.enter_context(DatasetStore(args.root, config, synthetic=synthetic))
            print(f"NeroPicoData: dataset root: {store.root}", flush=True)
            if args.command == "review":
                receiver = TelemetryReceiver(default_socket())
                cameras = {role: Camera(settings, config["image_width"], config["image_height"])
                           for role, settings in config["cameras"].items()}
            elif args.command == "demo":
                from .synthetic import SyntheticSource
                receiver = stack.enter_context(SyntheticSource(config))
                cameras = receiver.cameras
            else:
                receiver = stack.enter_context(TelemetryReceiver(args.socket))
                cameras = stack.enter_context(CameraRig(config, allow_unavailable=args.command == "serve")).cameras
            controller = stack.enter_context(CaptureController(config, store, receiver, cameras, review_only=args.command == "review"))
            if args.command in ("serve", "demo", "review"):
                from .server import serve
                serve(controller, args.port)
            else:
                print("Commands: start, success, failure, discard, status, quit", flush=True)
                while True:
                    try:
                        line = input("> ").strip()
                        if line == "quit":
                            break
                        if line == "status":
                            print(json.dumps(controller.status(), ensure_ascii=False))
                        elif line == "start":
                            print(controller.command("start", task=args.task))
                        elif line in ("success", "failure", "discard"):
                            print(controller.command("finish", outcome="discarded" if line == "discard" else line))
                    except (ValueError, RuntimeError) as exc:
                        print(f"Capture: {exc}", flush=True)
                    except (EOFError, KeyboardInterrupt):
                        break
        return 0
    except (ValueError, RuntimeError, OSError) as exc:
        parser.exit(1, f"NeroPicoData: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
