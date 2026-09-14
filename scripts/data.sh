#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == demo ]]; then
    shift
    set -- demo --config "$PICO_PROJECT_DIR/data_collection/config/demo.json" "$@"
fi
exec bash "$PICO_PROJECT_DIR/scripts/data_python.sh" -m nero_pico_data.cli "$@"
