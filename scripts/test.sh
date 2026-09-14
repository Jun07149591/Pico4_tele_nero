#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
suite="${1:-all}"
case "$suite" in
    all|teleop|data|browser|packaging) ;;
    *) printf '%s\n' 'Usage: bash scripts/test.sh [all|teleop|data|browser|packaging]' >&2; exit 2 ;;
esac
cd "$PICO_PROJECT_DIR"
if [[ "$suite" == all || "$suite" == packaging ]]; then
    python3 -m unittest discover -s tests -v
fi
if [[ "$suite" == all || "$suite" == teleop ]]; then
    bash scripts/teleop_python.sh -m unittest discover -s teleop/tests -v
fi
if [[ "$suite" == all || "$suite" == data ]]; then
    bash scripts/data_python.sh -m unittest discover -s data_collection/tests -p 'test_*.py' -v
fi
if [[ "$suite" == browser ]]; then
    bash scripts/data_python.sh data_collection/tests/browser_check.py
fi
