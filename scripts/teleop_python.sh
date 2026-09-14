#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
require_python "$PICO_NERO_PYTHON"
exec env -u LD_LIBRARY_PATH PYTHONPATH="$PICO_PROJECT_DIR/teleop" "$PICO_NERO_PYTHON" "$@"
