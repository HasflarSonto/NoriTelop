#!/usr/bin/env bash
# Pi-side teleop launcher. Stops ser2net so /dev/xlerobot_bus{1,2} are free,
# runs teleop_server.py, then restores ser2net on exit so the Phase 1
# bridge scripts (spin_motor.py, bridge_smoke_test.py) keep working.
#
# Uses the venv at rpi4/venv because Debian 13 enforces PEP 668
# (externally-managed system Python). Create it once with:
#   python3 -m venv rpi4/venv
#   rpi4/venv/bin/pip install -r rpi4/requirements.txt
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$SCRIPT_DIR/venv/bin/python"

if [[ ! -x "$PYTHON" ]]; then
    echo "venv not found at $SCRIPT_DIR/venv"
    echo "run: python3 -m venv $SCRIPT_DIR/venv && $SCRIPT_DIR/venv/bin/pip install -r $SCRIPT_DIR/requirements.txt"
    exit 1
fi

sudo systemctl stop ser2net
trap 'sudo systemctl start ser2net' EXIT

exec "$PYTHON" "$SCRIPT_DIR/teleop_server.py" "$@"
