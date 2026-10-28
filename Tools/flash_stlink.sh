#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BIN_FILE="$PROJECT_ROOT/build/Debug/self_balancing_car.bin"

PYTHON_EXEC="$PROJECT_ROOT/venv/bin/python3"
if [ ! -f "$PYTHON_EXEC" ]; then
    PYTHON_EXEC="python3"
fi

"$PYTHON_EXEC" "$SCRIPT_DIR/flash_stlink.py" "$@"
