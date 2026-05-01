#!/usr/bin/env bash
# Pi-side ZMQ camera server launcher.
#
# Streams USB cameras to the laptop over ZMQ on per-camera ports
# (default 5555 head, 5556 right_wrist — see image_server.py CAMERAS).
#
# Independent of teleop_server.py — they share zero state and use
# different ports, so they can run simultaneously. This wrapper does NOT
# stop ser2net (cameras don't touch the motor buses) and does NOT touch
# the NoriScreen UI.
#
# Uses the same venv at rpi4/venv as teleop_server.py.
# First-time setup:
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

exec "$PYTHON" "$SCRIPT_DIR/image_server.py" "$@"
