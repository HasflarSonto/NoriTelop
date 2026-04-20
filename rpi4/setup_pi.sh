#!/usr/bin/env bash
# XLeRobot RPi4 bridge setup — Phase 1 of the laptop→Pi migration.
#
# Run this on the RPi4 (Ubuntu Server 24.04 arm64). It:
#   1. Installs ser2net + usbutils + avahi-daemon
#   2. Adds $USER to the dialout group
#   3. Interactively identifies which USB port is bus1 (head) vs bus2 (wheels)
#   4. Writes /etc/udev/rules.d/99-xlerobot.rules with the discovered KERNELS
#   5. Installs /etc/ser2net.yaml
#   6. Reloads udev, enables + starts ser2net
#
# Usage:
#   sudo bash setup_pi.sh
#
# Copy this whole rpi4/ directory to the Pi first, e.g.:
#   scp -r lerobot/rpi4 ubuntu@xlerobot.local:~/
#   ssh ubuntu@xlerobot.local "sudo bash ~/rpi4/setup_pi.sh"
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run as root: sudo bash $0"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SER2NET_YAML="$SCRIPT_DIR/ser2net.yaml"
RULES_TEMPLATE="$SCRIPT_DIR/99-xlerobot.rules"

for f in "$SER2NET_YAML" "$RULES_TEMPLATE"; do
    [[ -f "$f" ]] || { echo "Missing: $f"; exit 1; }
done

TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo ubuntu)}"

echo "=== Step 1: Install packages ==="
apt update
apt install -y ser2net usbutils avahi-daemon

echo
echo "=== Step 2: Add $TARGET_USER to dialout group ==="
usermod -aG dialout "$TARGET_USER" || true
echo "(log out + back in for group to take effect)"

echo
echo "=== Step 3: Identify bus1 vs bus2 by USB port path ==="

read_kernels() {
    local dev="$1"
    udevadm info -a -n "$dev" 2>/dev/null | awk -F'"' '/KERNELS==/ {print $2; exit}'
}

wait_for_tty() {
    local prompt="$1"
    echo
    echo "$prompt"
    read -r -p "Press ENTER when plugged in..." _
    sleep 1
    local devs=( /dev/ttyUSB* )
    if [[ ! -e "${devs[0]}" ]]; then
        echo "No /dev/ttyUSB* found — check cable/power and retry."
        return 1
    fi
    # Pick the newest ttyUSB* (most recently enumerated)
    local newest
    newest=$(ls -1t /dev/ttyUSB* 2>/dev/null | head -1)
    echo "Detected: $newest"
    local k
    k=$(read_kernels "$newest")
    if [[ -z "$k" ]]; then
        echo "Failed to read KERNELS for $newest"
        return 1
    fi
    echo "KERNELS = $k"
    echo "$k"
}

echo "Unplug BOTH Feetech boards from the Pi/hub now."
read -r -p "Press ENTER once both are unplugged..." _

# Ensure no lingering devices
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    echo "Warning: /dev/ttyUSB* still present — unplug and retry."
    exit 1
fi

BUS1_KERNELS=$(wait_for_tty "Plug in ONLY the bus1 board (left arm + head, IDs 1-8)." | tail -1)
read -r -p "Now unplug that board. Press ENTER when done..." _
sleep 1
BUS2_KERNELS=$(wait_for_tty "Plug in ONLY the bus2 board (right arm + wheels + z-lift, IDs 1-6/9-11)." | tail -1)

if [[ "$BUS1_KERNELS" == "$BUS2_KERNELS" || -z "$BUS1_KERNELS" || -z "$BUS2_KERNELS" ]]; then
    echo "Discovery failed (bus1=$BUS1_KERNELS, bus2=$BUS2_KERNELS). Aborting."
    exit 1
fi

echo
echo "bus1 -> $BUS1_KERNELS"
echo "bus2 -> $BUS2_KERNELS"

echo
echo "=== Step 4: Install udev rules ==="
tmp_rules=$(mktemp)
sed -e "s|__BUS1_KERNELS__|$BUS1_KERNELS|" \
    -e "s|__BUS2_KERNELS__|$BUS2_KERNELS|" \
    "$RULES_TEMPLATE" > "$tmp_rules"
install -m 0644 "$tmp_rules" /etc/udev/rules.d/99-xlerobot.rules
rm -f "$tmp_rules"
udevadm control --reload
udevadm trigger

echo
echo "=== Step 5: Install ser2net config ==="
install -m 0644 "$SER2NET_YAML" /etc/ser2net.yaml

echo
echo "=== Step 6: Enable + start ser2net ==="
systemctl enable ser2net
systemctl restart ser2net

echo
echo "Plug BOTH boards back in, then press ENTER to verify..."
read -r _
sleep 2

echo
echo "--- Symlinks ---"
ls -l /dev/xlerobot_bus1 /dev/xlerobot_bus2 2>&1 || true
echo
echo "--- ser2net status ---"
systemctl --no-pager status ser2net | head -15
echo
echo "--- listening ports ---"
ss -tlnp 2>/dev/null | grep -E ':(3001|3002)' || true

echo
echo "=== Done ==="
echo "Next: on the laptop, run:"
echo "  set XLEROBOT_BRIDGE=$(hostname).local"
echo "  cd lerobot && python examples/xlerobot/detect_buses.py socket://$(hostname).local:3001 socket://$(hostname).local:3002"
