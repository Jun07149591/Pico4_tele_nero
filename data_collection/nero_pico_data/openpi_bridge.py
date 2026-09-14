"""Run official OpenPI scripts with our config registered, without patching OpenPI."""

import argparse
import json
import os
from pathlib import Path
import runpy
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=("norm", "train", "serve", "check"))
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--openpi", type=Path, required=True)
    parser.add_argument("--allow-synthetic", action="store_true")
    args, extra = parser.parse_known_args()
    spec = json.loads(args.spec.resolve().read_text())
    if spec["synthetic"] and not args.allow_synthetic:
        parser.error("synthetic dataset requires --allow-synthetic")
    root = args.spec.resolve().parent.parent
    spec["dataset_root"] = str(root)
    if (root / ".export-incomplete").exists() or not (root / "meta/info.json").is_file():
        parser.error("dataset export is incomplete")
    namespace, dataset_name = spec["repo_id"].split("/")
    if root.name != dataset_name or root.parent.name != namespace:
        parser.error("export under <dataset-home>/<namespace>/<dataset> to use OpenPI's local loader")
    os.environ["HF_LEROBOT_HOME"] = str(root.parent.parent)
    project = args.openpi.resolve()
    sys.path.insert(0, str(project / "src"))
    from .openpi_config import register
    name = register(spec)
    os.chdir(project)
    if args.operation == "check":
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        from .openpi_policy import NeroInputs
        dataset = LeRobotDataset(spec["repo_id"], root=root, delta_timestamps={"action": [i / spec["fps"] for i in range(spec["action_horizon"])]})
        frame = dataset[0]
        model_input = NeroInputs(spec["mode"], tuple(spec["cameras"]))({"state": frame["observation.state"],
            "actions": frame["action"], "images": {r: frame[f"observation.images.{r}"] for r in spec["cameras"]}})
        print(json.dumps({"config": name, "frames": len(dataset), "state_shape": list(model_input["state"].shape),
                          "action_shape": list(model_input["actions"].shape)}))
        return
    scripts = {"norm": "compute_norm_stats.py", "train": "train.py", "serve": "serve_policy.py"}
    script = project / "scripts" / scripts[args.operation]
    prefix = {"norm": ["--config-name", name], "train": [name], "serve": ["policy:checkpoint", f"--policy.config={name}"]}
    sys.argv = [str(script), *prefix[args.operation], *extra]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
