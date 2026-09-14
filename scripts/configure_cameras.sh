#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec bash "$PICO_PROJECT_DIR/scripts/data_python.sh" "$PICO_PROJECT_DIR/scripts/configure_cameras.py" "$@"
