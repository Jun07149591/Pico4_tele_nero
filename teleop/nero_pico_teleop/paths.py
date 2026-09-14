"""Paths shared by preview, commissioning and continuous control."""

import os
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parent
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "nero_relative_config.json"


def environment_root():
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return Path(os.environ.get("PICO_ENV_ROOT", cache / "pico4_tele_nero/envs")).expanduser().resolve()


def pico_sdk_path():
    return Path(os.environ.get("PICO_XR_SDK", REPOSITORY_ROOT / ".runtime/roboticsservice/SDK/x64/libPXREARobotSDK.so")).resolve()


def agx_sdk_path():
    return Path(os.environ.get("NERO_AGX_SDK_ROOT", REPOSITORY_ROOT / "vendor/pyAgxArm")).resolve()


def use_agx_sdk():
    root = agx_sdk_path()
    if not (root / "pyAgxArm/__init__.py").is_file():
        raise FileNotFoundError(f"Nero SDK missing: {root}; run scripts/setup.sh")
    loaded = sys.modules.get("pyAgxArm")
    if loaded is not None and not Path(loaded.__file__).resolve().is_relative_to(root):
        raise RuntimeError(f"Another Nero SDK is already imported: {loaded.__file__}")
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return root
