#!/usr/bin/env bash
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/start_teleop.sh" "$@"
