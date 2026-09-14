#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec bash "$PICO_PROJECT_DIR/scripts/teleop_python.sh" -m nero_pico_teleop.deploy \
    --telemetry-socket "/tmp/nero_pico_data_$(id -u).sock" "$@"
