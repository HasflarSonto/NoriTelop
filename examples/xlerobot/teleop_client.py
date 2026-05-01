"""Phase 2: Laptop-side teleop client.

Captures pressed keys at 50 Hz via a global pynput listener (with suppression
so digits/letters don't leak into the terminal that's running this script)
and ships them as JSON lines to rpi4/teleop_server.py (port 7777). All
control logic lives on the Pi. Set XLEROBOT_BRIDGE=<host-or-IP> to point at
the server (same env var used by Phase 1).

While the listener is active, ALL keystrokes are routed to this script and
hidden from other windows — that's the price of "no terminal echo while
teleoping". Press ESC (or Ctrl-C) to release the keyboard.

Note: on macOS, suppress=True needs both Accessibility AND Input Monitoring
permission for the terminal in System Settings → Privacy & Security. On
Linux/X11 it works without extra setup; on Wayland it's a no-op.

Usage:
    $env:XLEROBOT_BRIDGE = "xlerobot.local"  # PowerShell
    python examples/xlerobot/teleop_client.py
"""
import json
import os
import socket
import sys
import threading
import time

from pynput import keyboard as pyn_keyboard

SERVER_PORT = 7777
FPS = 50

# Keys that mean "reset the corresponding group" rather than jog motion.
RESET_LEFT_KEY = "c"
RESET_RIGHT_KEY = "0"
RESET_HEAD_KEY = "?"


class KeyTracker:
    """Thread-safe pressed-key set fed by a suppressed pynput listener.

    suppress=True makes the OS-level hook consume each keystroke so it
    doesn't echo into the terminal or other foreground apps. The script
    still gets the events via on_press / on_release. ESC sets a quit flag
    instead of being sent as a key.
    """

    def __init__(self) -> None:
        self._pressed: set[str] = set()
        self._lock = threading.Lock()
        self._quit = False
        self._listener: pyn_keyboard.Listener | None = None

    def start(self) -> None:
        self._listener = pyn_keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
            suppress=True,
        )
        self._listener.start()

    def stop(self) -> None:
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception:
                pass
            self._listener = None

    def _key_char(self, key) -> str | None:
        # pynput delivers KeyCode for printable characters and Key for
        # specials. We only care about chars (a-z, digits, punct).
        return getattr(key, "char", None)

    def _on_press(self, key) -> None:
        if key == pyn_keyboard.Key.esc:
            self._quit = True
            return
        ch = self._key_char(key)
        if ch is not None:
            with self._lock:
                self._pressed.add(ch)

    def _on_release(self, key) -> None:
        ch = self._key_char(key)
        if ch is not None:
            with self._lock:
                self._pressed.discard(ch)

    def snapshot(self) -> set[str]:
        with self._lock:
            return set(self._pressed)

    @property
    def quit_requested(self) -> bool:
        return self._quit


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

    # Start keyboard listener AFTER the input() prompt finishes, so the
    # bus-choice readline isn't suppressed.
    tracker = KeyTracker()
    tracker.start()
    print_banner()
    print("[CLIENT] keyboard suppression active — keystrokes won't echo "
          "to the terminal. Press ESC or Ctrl-C to release.\n")

    sock.setblocking(False)
    dt = 1.0 / FPS
    try:
        while True:
            t0 = time.time()
            if tracker.quit_requested:
                print("\n[CLIENT] ESC pressed")
                break

            pressed = tracker.snapshot()

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
        # Always release the keyboard hook, even if something else blew up.
        tracker.stop()
        try:
            sock.sendall((json.dumps({"bye": True}) + "\n").encode())
        except OSError:
            pass
        try: sock.close()
        except OSError: pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
