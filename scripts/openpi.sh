#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ -z "${OPENPI_PROJECT_DIR:-}" ]]; then
    printf '%s\n' 'Set OPENPI_PROJECT_DIR to your OpenPI checkout; see docs/DATA_FORMAT.md.' >&2
    exit 2
fi
OPENPI_PYTHON="${OPENPI_PYTHON:-$OPENPI_PROJECT_DIR/.venv/bin/python}"
require_python "$OPENPI_PYTHON"
export PYTHONPATH="$PICO_PROJECT_DIR/data_collection:$OPENPI_PROJECT_DIR/src"
exec "$OPENPI_PYTHON" -m nero_pico_data.openpi_bridge --openpi "$OPENPI_PROJECT_DIR" "$@"
