#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
service_dir="$PICO_PROJECT_DIR/.runtime/roboticsservice"
if [[ ! -x "$service_dir/RoboticsServiceProcess" ]]; then
    printf '%s\n' 'XR service missing. Run bash scripts/fetch_xr.sh.' >&2; exit 2
fi
if pgrep -x -f '(.*/)?RoboticsServiceProcess' >/dev/null; then
    printf '%s\n' 'XRoboToolkit PC service is already running.'; exit 0
fi
cd "$service_dir"
exec env LD_LIBRARY_PATH="$service_dir:$service_dir/lib:$service_dir/SDK/x64" \
    QT_QPA_PLATFORM=offscreen QT_PLUGIN_PATH="$service_dir/plugins" "$service_dir/RoboticsServiceProcess"
