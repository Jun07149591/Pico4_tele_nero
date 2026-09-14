#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
install_teleop=true
install_data=true
install_xr=true
for arg in "$@"; do
    case "$arg" in
        --data-only) install_teleop=false; install_xr=false ;;
        --teleop-only) install_data=false ;;
        --skip-xr) install_xr=false ;;
        --help|-h) printf '%s\n' 'Usage: bash scripts/setup.sh [--data-only | --teleop-only] [--skip-xr]'; exit 0 ;;
        *) printf 'Unknown setup option: %s\n' "$arg" >&2; exit 2 ;;
    esac
done
if ! $install_teleop && ! $install_data; then
    printf '%s\n' '--data-only and --teleop-only cannot be combined.' >&2; exit 2
fi
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
    printf '%s\n' 'This installer targets Ubuntu x86_64. See README.md for platform requirements.' >&2; exit 2
fi
for prerequisite in git curl python3 dpkg-deb; do
    if ! command -v "$prerequisite" >/dev/null; then
        printf 'Missing system command: %s. See README.md system packages.\n' "$prerequisite" >&2; exit 2
    fi
done
conda_executable="${CONDA_EXE:-$(command -v conda || true)}"
if [[ ! -x "$conda_executable" ]]; then
    printf '%s\n' 'Install Miniforge/Conda first, then rerun. See README.md.' >&2; exit 2
fi
if [[ "$PICO_ENV_ROOT" == *' '* ]]; then
    printf '%s\n' 'PICO_ENV_ROOT must not contain spaces (Conda prefix requirement); the repository may contain spaces.' >&2; exit 2
fi
mkdir -p "$PICO_ENV_ROOT"
if $install_teleop; then
    if [[ ! -x "$PICO_ENV_ROOT/teleop/bin/python" ]]; then
        "$conda_executable" env create --yes --prefix "$PICO_ENV_ROOT/teleop" --file "$PICO_PROJECT_DIR/environment-teleop.yml"
    fi
    env -u PYTHONPATH -u LD_LIBRARY_PATH "$PICO_ENV_ROOT/teleop/bin/python" -m pip install --no-deps --no-build-isolation --editable "$PICO_PROJECT_DIR/teleop"
    if [[ ! -x "$PICO_ENV_ROOT/placo/bin/python" ]]; then
        "$PICO_ENV_ROOT/teleop/bin/python" -m venv "$PICO_ENV_ROOT/placo"
    fi
    env -u PYTHONPATH -u LD_LIBRARY_PATH "$PICO_ENV_ROOT/placo/bin/python" -m pip install -r "$PICO_PROJECT_DIR/teleop/requirements-placo.txt"
    PICO_NERO_PYTHON="$PICO_ENV_ROOT/teleop/bin/python" bash "$PICO_PROJECT_DIR/scripts/teleop_python.sh" -c 'import pinocchio, casadi, can; from pinocchio import casadi as cpin; from nero_pico_teleop.paths import use_agx_sdk; use_agx_sdk(); print("Teleop dependencies ready")'
fi
if $install_data; then
    if [[ ! -x "$PICO_ENV_ROOT/data/bin/python" ]]; then
        "$conda_executable" create --yes --prefix "$PICO_ENV_ROOT/data" --override-channels -c conda-forge python=3.12 pip
    fi
    env -u PYTHONPATH -u LD_LIBRARY_PATH "$PICO_ENV_ROOT/data/bin/python" -m pip install \
        --index-url https://download.pytorch.org/whl/cpu 'torch==2.10.0' 'torchvision==0.25.0'
    env -u PYTHONPATH -u LD_LIBRARY_PATH "$PICO_ENV_ROOT/data/bin/python" -m pip install \
        -r "$PICO_PROJECT_DIR/data_collection/requirements-export.txt" \
        'h5py==3.16.0' 'pyrealsense2==2.58.4.10922' 'opencv-python==4.13.0.92'
    GIT_LFS_SKIP_SMUDGE=1 env -u PYTHONPATH -u LD_LIBRARY_PATH "$PICO_ENV_ROOT/data/bin/python" -m pip install --no-deps \
        'lerobot @ git+https://github.com/huggingface/lerobot.git@0cf864870cf29f4738d3ade893e6fd13fbd7cdb5' \
        --editable "$PICO_PROJECT_DIR/data_collection"
    NERO_DATA_PYTHON="$PICO_ENV_ROOT/data/bin/python" bash "$PICO_PROJECT_DIR/scripts/data_python.sh" -c 'from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION; import h5py, cv2, pyrealsense2; assert CODEBASE_VERSION == "v2.1"; print("Capture/export dependencies ready")'
fi
if $install_xr; then
    bash "$PICO_PROJECT_DIR/scripts/fetch_xr.sh"
fi
printf 'Setup complete. Environments: %s\n' "$PICO_ENV_ROOT"
