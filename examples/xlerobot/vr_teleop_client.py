"""Phase 2.5: Laptop-side VR teleop client.

Spawns the XLeVR WebSocket + HTTPS servers via VRMonitor so the Quest
browser can connect, then polls VRMonitor at 50 Hz and ships flattened
ControlGoal dicts as JSON to the Pi's teleop_server.py (port 7777) with
hello.mode="vr". All motor I/O lives on the Pi.

Usage (PowerShell):
    $env:XLEROBOT_BRIDGE = "xlerobot.local"    # Pi host
    $env:XLEVR_PATH = "C:\\Users\\Antonio\\Desktop\\XLeRobot_Setup\\XLeRobot\\XLeVR"
    python examples/xlerobot/vr_teleop_client.py

Put on Quest → browser → https://<laptop-ip>:8443 → accept cert → enter VR.
"""
import asyncio
import json
import os
import socket
import sys
import threading
import time

SERVER_PORT = 7777
FPS = 50

DEFAULT_XLEVR_PATH = r"C:\Users\Antonio\Desktop\XLeRobot_Setup\XLeRobot\XLeVR"


def print_banner(host: str, bus_choice: str) -> None:
    print("\n" + "=" * 80)
    print("XLeRobot VR Teleop (Phase 2.5) — Pi-side control loop")
    print("=" * 80)
    print(f"Pi server: {host}:{SERVER_PORT}   bus_choice={bus_choice}   mode=vr")
    print("\nQuest 3 connection:")
    print("  Put on the headset and open https://<this-laptop-IP>:8443")
    print("  (accept the self-signed cert, then tap 'Enter VR').")
    print("\nMapping:")
    print("  Left controller pose   -> left arm IK  (bus1, if enabled)")
    print("  Right controller pose  -> right arm IK (bus2, if enabled)")
    print("  Trigger (each hand)    -> that side's gripper (hold to close)")
    print("  Left thumbstick        -> head pan/tilt")
    print("  Right thumbstick       -> wheels (forward/back + turn)")
    print("  Right A button         -> Z-lift up")
    print("  Right B button         -> Z-lift down")
    print("\nCtrl-C to quit.")
    print("=" * 80 + "\n")


def flatten_goal(goal) -> dict | None:
    """Flatten a ControlGoal (from VRMonitor) to the plain-dict wire format
    expected by teleop_server.py. Returns None if the goal has no pose."""
    if goal is None:
        return None
    pos = getattr(goal, "target_position", None)
    if pos is None:
        return None
    meta = getattr(goal, "metadata", None) or {}
    out = {
        "position": [float(pos[0]), float(pos[1]), float(pos[2])],
        "trigger": float(meta.get("trigger", 0.0) or 0.0),
        "thumbstick": meta.get("thumbstick") or {},
        "buttons": meta.get("buttons") or {},
    }
    wr = getattr(goal, "wrist_roll_deg", None)
    wf = getattr(goal, "wrist_flex_deg", None)
    if wr is not None:
        out["wrist_roll_deg"] = float(wr)
    if wf is not None:
        out["wrist_flex_deg"] = float(wf)
    return out


def start_vr_monitor():
    """Spawn VRMonitor's asyncio loop in a daemon thread and return the
    monitor instance. Raises RuntimeError if XLeVR can't be imported."""
    os.environ.setdefault("XLEVR_PATH", DEFAULT_XLEVR_PATH)
    xlevr_path = os.environ["XLEVR_PATH"]
    if not os.path.isdir(xlevr_path):
        raise RuntimeError(
            f"XLEVR_PATH does not exist: {xlevr_path}\n"
            "Set $env:XLEVR_PATH to the local XLeVR checkout."
        )

    from lerobot.teleoperators.xlerobot_vr.vr_monitor import VRMonitor
    monitor = VRMonitor()

    def _run():
        try:
            asyncio.run(monitor.start_monitoring())
        except Exception as e:
            print(f"[VR-MON] thread error: {e}")

    threading.Thread(target=_run, daemon=True).start()

    # Wait up to 10 s for monitor to come up (HTTPS + WS bind).
    t0 = time.time()
    while time.time() - t0 < 10.0:
        if monitor.is_running:
            return monitor
        time.sleep(0.1)
    raise RuntimeError("VRMonitor did not start within 10 s (check XLeVR config/certs)")


def main() -> int:
    host = os.environ.get("XLEROBOT_BRIDGE")
    if not host:
        print("ERROR: set XLEROBOT_BRIDGE to the Pi hostname or IP.", file=sys.stderr)
        return 2

    print("Which buses to enable?")
    print("  1 = Bus1 only (left arm + head)")
    print("  2 = Bus2 only (right arm + wheels + z-lift)")
    print("  3 = Both buses")
    bus_choice = input("Enter choice [3]: ").strip() or "3"
    if bus_choice not in ("1", "2", "3"):
        print(f"invalid choice: {bus_choice!r}"); return 2

    print("[CLIENT] starting VRMonitor (XLeVR WebSocket + HTTPS) ...")
    try:
        monitor = start_vr_monitor()
    except RuntimeError as e:
        print(f"[CLIENT] {e}"); return 1
    print("[CLIENT] VRMonitor up.")

    print(f"[CLIENT] connecting to {host}:{SERVER_PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((host, SERVER_PORT))
    except OSError as e:
        print(f"[CLIENT] connect failed: {e}"); return 1

    sock.sendall((json.dumps({"hello": {"mode": "vr", "bus_choice": bus_choice}}) + "\n").encode())
    sock.settimeout(10.0)
    buf = b""
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

    print_banner(host, bus_choice)

    sock.setblocking(False)
    dt = 1.0 / FPS
    try:
        while True:
            t0 = time.time()
            dual = monitor.get_latest_goal_nowait()  # {"left","right","headset","has_*"}
            frame = {
                "vr": {
                    "left": flatten_goal(dual.get("left")),
                    "right": flatten_goal(dual.get("right")),
                }
            }
            try:
                sock.sendall((json.dumps(frame) + "\n").encode())
            except OSError as e:
                print(f"[CLIENT] send failed: {e}"); break

            # Drain telemetry (shared shape with keyboard mode).
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
                    if msg.get("stalled"):
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
