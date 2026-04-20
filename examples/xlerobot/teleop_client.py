"""Phase 2: Laptop-side teleop client.

Captures pressed keys at 50 Hz via lerobot.KeyboardTeleop and ships them as
JSON lines to rpi4/teleop_server.py (port 7777). All control logic lives on
the Pi. Set XLEROBOT_BRIDGE=<host-or-IP> to point at the server (same env
var used by Phase 1).

Usage:
    $env:XLEROBOT_BRIDGE = "xlerobot.local"  # PowerShell
    python examples/xlerobot/teleop_client.py
"""
import json
import os
import socket
import sys
import time

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

SERVER_PORT = 7777
FPS = 50

# Keys that mean "reset the corresponding group" rather than jog motion.
RESET_LEFT_KEY = "c"
RESET_RIGHT_KEY = "0"
RESET_HEAD_KEY = "?"


def print_banner() -> None:
    print("\n" + "=" * 80)
    print("XLeRobot 2Wheels Keyboard Control (Pi-side loop, Phase 2)")
    print("=" * 80)
    print("\nBase Control (Differential Drive):")
    print("    i: Forward,   k: Backward,   j: Rotate Left,   l: Rotate Right")
    print("    u: Speed Up,  o: Speed Down")
    print("\nLeft Arm (bus1):")
    print("    Q/E: shoulder_pan +/-     R/F: wrist_roll +/-    T/G: gripper +/-")
    print("    W/S: x +/-                A/D: y +/-             Z/X: pitch +/-")
    print("    C:   reset to zero")
    print("\nRight Arm (bus2):")
    print("    7/9: shoulder_pan +/-     / *: wrist_roll +/-    +/-: gripper +/-")
    print("    8/2: x +/-                4/6: y +/-             1/3: pitch +/-")
    print("    0:   reset to zero")
    print("\nHead (bus1):")
    print("    </>: head_motor_1 +/-   ,/.: head_motor_2 +/-   ?: reset head")
    print("\nZ-Lift (bus2, motor 11, velocity mode):")
    print("    y: raise,  h: lower  (hold key — release to stop)")
    print("\nCtrl-C to quit.\n")
    print("=" * 80 + "\n")


def main() -> int:
    host = os.environ.get("XLEROBOT_BRIDGE")
    if not host:
        print("ERROR: set XLEROBOT_BRIDGE to the Pi hostname or IP (e.g. xlerobot.local).",
              file=sys.stderr)
        return 2

    print("Which buses to enable?")
    print("  1 = Bus1 only (left arm + head)")
    print("  2 = Bus2 only (right arm + wheels)")
    print("  3 = Both buses")
    bus_choice = input("Enter choice [2]: ").strip() or "2"
    if bus_choice not in ("1", "2", "3"):
        print(f"invalid choice: {bus_choice!r}"); return 2

    print(f"[CLIENT] connecting to {host}:{SERVER_PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((host, SERVER_PORT))
    except OSError as e:
        print(f"[CLIENT] connect failed: {e}"); return 1

    # Handshake
    sock.sendall((json.dumps({"hello": {"bus_choice": bus_choice}}) + "\n").encode())
    sock.settimeout(10.0)
    buf = b""
    ack = None
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            print("[CLIENT] server closed before ack"); return 1
        buf += chunk
    line, buf = buf.split(b"\n", 1)
    try:
        ack = json.loads(line.decode("utf-8"))
    except json.JSONDecodeError as e:
        print(f"[CLIENT] bad ack: {e}"); return 1
    if not ack.get("ack"):
        print(f"[CLIENT] server refused: {ack.get('error','?')}"); return 1
    print(f"[CLIENT] ack received, {len(ack.get('initial_obs', {}))} initial positions")

    # Start keyboard
    keyboard = KeyboardTeleop(KeyboardTeleopConfig())
    keyboard.connect()
    print_banner()

    sock.setblocking(False)
    dt = 1.0 / FPS
    try:
        while True:
            t0 = time.time()
            pressed = set(keyboard.get_action().keys())

            frame = {
                "keys": sorted(pressed),
                "reset_left": RESET_LEFT_KEY in pressed,
                "reset_right": RESET_RIGHT_KEY in pressed,
                "reset_head": RESET_HEAD_KEY in pressed,
            }
            try:
                sock.sendall((json.dumps(frame) + "\n").encode())
            except OSError as e:
                print(f"[CLIENT] send failed: {e}"); break

            # Drain any telemetry
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    print("[CLIENT] server closed"); break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        msg = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError:
                        continue
                    if "stalled" in msg and msg["stalled"]:
                        print(f"[SRV] stalled: {msg['stalled']} ({msg.get('loop_hz','?')} Hz)")
            except BlockingIOError:
                pass
            except OSError:
                pass

            remaining = dt - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\n[CLIENT] Ctrl-C")
    finally:
        try:
            sock.sendall((json.dumps({"bye": True}) + "\n").encode())
        except OSError:
            pass
        try: sock.close()
        except OSError: pass
        keyboard.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
