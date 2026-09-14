#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec bash "$PICO_PROJECT_DIR/scripts/teleop_python.sh" -m nero_pico_teleop.preflight \
    --config "$PICO_PROJECT_DIR/teleop/config/nero_humanoid_config.json" "$@"
