#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
require_python "$NERO_DATA_PYTHON"
exec env -u LD_LIBRARY_PATH PYTHONPATH="$PICO_PROJECT_DIR/data_collection" "$NERO_DATA_PYTHON" "$@"
