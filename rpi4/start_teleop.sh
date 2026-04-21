#!/usr/bin/env bash
# Pi-side teleop launcher. Orchestrates three children:
#   1. python3 -m http.server 8080  (serves noriscreen/index.html to Chromium)
#   2. chromium --kiosk http://localhost:8080/index.html  (the UI on 7" DSI)
#   3. venv/bin/python teleop_server.py  (TCP 7777 + HTTP estop on 9091)
#
# Stops ser2net so /dev/xlerobot_bus{1,2} are free. On any exit path
# (teleop_server exits, Ctrl-C, E-STOP via the UI, SIGTERM) the trap
# kills all children and restarts ser2net so the Phase 1 bridge scripts
# (spin_motor.py, bridge_smoke_test.py) keep working.
#
# Uses the venv at rpi4/venv because Debian 13 enforces PEP 668
# (externally-managed system Python). Create it once with:
#   python3 -m venv rpi4/venv
#   rpi4/venv/bin/pip install -r rpi4/requirements.txt
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$SCRIPT_DIR/venv/bin/python"
NORISCREEN_DIR="$SCRIPT_DIR/noriscreen"

if [[ ! -x "$PYTHON" ]]; then
    echo "venv not found at $SCRIPT_DIR/venv"
    echo "run: python3 -m venv $SCRIPT_DIR/venv && $SCRIPT_DIR/venv/bin/pip install -r $SCRIPT_DIR/requirements.txt"
    exit 1
fi
if [[ ! -f "$NORISCREEN_DIR/index.html" ]]; then
    echo "noriscreen/ not found at $NORISCREEN_DIR"
    exit 1
fi

sudo systemctl stop ser2net

CHILD_PIDS=()
cleanup() {
    for pid in "${CHILD_PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Chromium spawns zygote/renderer children that outlive the parent PID.
    pkill -f "chromium.*localhost:8080/index.html" 2>/dev/null || true
    fuser -k 8080/tcp 2>/dev/null || true
    sudo systemctl start ser2net
}
trap cleanup EXIT INT TERM

# Kill any stale HTTP server on 8080 from prior manual runs.
fuser -k 8080/tcp 2>/dev/null || true
sleep 0.2

# (1) Static HTTP server for the NoriScreen UI.
( cd "$NORISCREEN_DIR" && exec python3 -m http.server 8080 >/dev/null 2>&1 ) &
CHILD_PIDS+=($!)

# (2) Chromium kiosk pointing at the static server.
#     Needs the active graphical session on the Pi's 7" DSI — Xwayland
#     is running under labwc, so DISPLAY=:0 routes through it.
DISPLAY=:0 chromium --kiosk --noerrdialogs --disable-infobars \
    --incognito --start-fullscreen \
    http://localhost:8080/index.html >/dev/null 2>&1 &
CHILD_PIDS+=($!)

# (3) Teleop server in the foreground. When it exits (E-STOP SIGINT,
#     Ctrl-C, crash) the trap fires and tears the other two down.
"$PYTHON" "$SCRIPT_DIR/teleop_server.py" "$@"
