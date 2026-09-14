#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec bash "$PICO_PROJECT_DIR/scripts/teleop_python.sh" -m nero_pico_teleop.can_setup "$@"
