#!/usr/bin/env bash
set -euo pipefail
PICO_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PICO_ENV_ROOT="${PICO_ENV_ROOT:-${XDG_CACHE_HOME:-$HOME/.cache}/pico4_tele_nero/envs}"
PICO_NERO_PYTHON="${PICO_NERO_PYTHON:-$PICO_ENV_ROOT/teleop/bin/python}"
NERO_PLACO_PYTHON="${NERO_PLACO_PYTHON:-$PICO_ENV_ROOT/placo/bin/python}"
NERO_DATA_PYTHON="${NERO_DATA_PYTHON:-$PICO_ENV_ROOT/data/bin/python}"
export PICO_ENV_ROOT NERO_PLACO_PYTHON
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

require_python() {
    if [[ ! -x "$1" ]]; then
        printf 'Python environment missing: %s\nRun bash scripts/setup.sh first.\n' "$1" >&2
        exit 2
    fi
}
