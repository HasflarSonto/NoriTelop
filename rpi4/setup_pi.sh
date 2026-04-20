#!/usr/bin/env bash
# XLeRobot RPi4 bridge setup — Phase 1 of the laptop→Pi migration.
#
# Run this on the RPi4 (Raspberry Pi OS / Debian 13 trixie, or Ubuntu 24.04).
# It:
#   1. Installs ser2net + usbutils + avahi-daemon + python3-serial
#   2. Adds $SUDO_USER to the dialout group (if not already there)
#   3. Auto-identifies which /dev/ttyACM* is bus1 vs bus2 by pinging motors.
#      Requires motors to be POWERED when you run this step.
#   4. Writes /etc/udev/rules.d/99-xlerobot.rules keyed to each board's
#      USB serial number (ATTRS{serial}) so the symlink survives reboots
#      and port swaps.
#   5. Installs /etc/ser2net.yaml
#   6. Reloads udev, enables + starts ser2net
#
# The CH343 "USB Single Serial" boards (VID 1a86 PID 55d3) bind to the
# cdc_acm driver on modern kernels and appear as /dev/ttyACM*, NOT
# /dev/ttyUSB*. That is normal and working.
#
# Usage (on the Pi):
#   cd ~/NoriTelop
#   sudo bash rpi4/setup_pi.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run as root: sudo bash $0"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SER2NET_YAML="$SCRIPT_DIR/ser2net.yaml"
RULES_TEMPLATE="$SCRIPT_DIR/99-xlerobot.rules"
IDENT_SCRIPT="$SCRIPT_DIR/identify_buses.py"

for f in "$SER2NET_YAML" "$RULES_TEMPLATE" "$IDENT_SCRIPT"; do
    [[ -f "$f" ]] || { echo "Missing: $f"; exit 1; }
done

TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo antonio)}"

echo "=== Step 1: Install packages ==="
apt update
apt install -y ser2net usbutils avahi-daemon python3-serial

echo
echo "=== Step 2: Add $TARGET_USER to dialout group ==="
if id -nG "$TARGET_USER" | grep -qw dialout; then
    echo "$TARGET_USER already in dialout"
else
    usermod -aG dialout "$TARGET_USER"
    echo "Added. Log out + back in for group to take effect."
fi

echo
echo "=== Step 3: Identify bus1 vs bus2 (pinging motors) ==="
echo "Motors MUST be powered on for this step. Boards must both be plugged in."
read -r -p "Press ENTER when ready..." _

# Run identifier; capture the shell-style KEY=VALUE lines it prints to stdout.
IDENT_OUT=$(python3 "$IDENT_SCRIPT")
echo "$IDENT_OUT"
# shellcheck disable=SC1090,SC2046
eval "$(printf '%s\n' "$IDENT_OUT" | grep -E '^(BUS[12]_(SERIAL|DEV))=')"

: "${BUS1_SERIAL:?bus1 serial not identified}"
: "${BUS2_SERIAL:?bus2 serial not identified}"

echo
echo "bus1 = $BUS1_DEV (serial $BUS1_SERIAL)"
echo "bus2 = $BUS2_DEV (serial $BUS2_SERIAL)"

echo
echo "=== Step 4: Install udev rules ==="
tmp_rules=$(mktemp)
sed -e "s|__BUS1_SERIAL__|$BUS1_SERIAL|" \
    -e "s|__BUS2_SERIAL__|$BUS2_SERIAL|" \
    "$RULES_TEMPLATE" > "$tmp_rules"
install -m 0644 "$tmp_rules" /etc/udev/rules.d/99-xlerobot.rules
rm -f "$tmp_rules"
udevadm control --reload
udevadm trigger --subsystem-match=tty

echo
echo "=== Step 5: Install ser2net config ==="
install -m 0644 "$SER2NET_YAML" /etc/ser2net.yaml

echo
echo "=== Step 6: Enable + start ser2net ==="
systemctl enable ser2net
systemctl restart ser2net
sleep 1

echo
echo "--- Symlinks ---"
ls -l /dev/xlerobot_bus1 /dev/xlerobot_bus2 2>&1 || true
echo
echo "--- ser2net status ---"
systemctl --no-pager status ser2net | head -15 || true
echo
echo "--- listening ports (expect 3001 + 3002) ---"
ss -tlnp 2>/dev/null | grep -E ':(3001|3002)' || echo "(no listeners — check systemctl status ser2net)"

HOST=$(hostname)
echo
echo "=== Done ==="
echo "Next: on the laptop, point it at this Pi and run the smoke test."
echo "  Windows: set XLEROBOT_BRIDGE=$HOST.local  (or the Pi's IP)"
echo "  Linux/macOS: export XLEROBOT_BRIDGE=$HOST.local"
echo "  python lerobot/examples/xlerobot/bridge_smoke_test.py"
