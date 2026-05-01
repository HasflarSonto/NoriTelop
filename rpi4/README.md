# NoriTelop — Pi-side teleop & IL recording for XLeRobot

NoriTelop is a thin overlay on top of [`huggingface/lerobot`](https://github.com/huggingface/lerobot) that runs the XLeRobot 2-wheels control loop on a Raspberry Pi 4. It adds:

- A 50 Hz motor control server (`teleop_server.py`) that owns both Feetech buses directly via `scservo_sdk`, with stall detection, smooth base accel/decel, and a dead-man timeout.
- A multi-camera ZMQ image server (`image_server.py`) that streams Pi-attached USB cameras to the laptop with negligible motor-loop impact.
- Three laptop-side clients (keyboard, VR teleop, IL recording) that talk to the Pi over a small JSON-line TCP protocol.
- An IL recording path that drops demos straight into `LeRobotDataset` so they feed `python -m lerobot.scripts.lerobot_train --policy.type=act ...` with no extra glue.

The repo is an **overlay**, not a fork — only the files NoriTelop changes are tracked. Everything else in the lerobot tree is left as upstream.

---

## Architecture

Three deployment phases, each additive. Pick whichever fits your setup; they coexist on the same hardware.

```
Phase 1 — laptop owns motors, Pi just bridges the buses
   Laptop (full lerobot)                Pi (ser2net only)
   ┌────────────────────┐    socket://   ┌──────────────────┐
   │ teleop / VR loop   │ ────────────►  │ ser2net          │
   │ scservo_sdk        │   3001 / 3002  │   /dev/xlerobot_ │
   │                    │                │   bus{1,2}       │
   └────────────────────┘                └──────────────────┘

Phase 2 — Pi owns motors, laptop sends keys / VR / joint targets
   Laptop                                Pi
   ┌────────────────────┐    TCP 7777    ┌──────────────────┐
   │ teleop_client.py   │ ────JSON────►  │ teleop_server.py │
   │                    │ ◄──telemetry── │ 50 Hz motor loop │
   └────────────────────┘                └──────────────────┘

Phase 3 — Phase 2 + cameras streamed from the Pi
   Laptop                                Pi
   ┌────────────────────┐    TCP 7777    ┌──────────────────┐
   │ 9_phase2_vr_record │ ───control──►  │ teleop_server.py │
   │                    │ ◄──state────── │                  │
   │                    │    ZMQ 5555    ├──────────────────┤
   │ ZMQCamera("head")  │ ◄──JPEG─────── │ image_server.py  │
   │ ZMQCamera("wrist") │ ◄──JPEG────────│   PUB 5555/5556  │
   └────────────────────┘    ZMQ 5556    └──────────────────┘
```

When to use which:

| Phase | Motor loop site | Cameras | Use when |
|---|---|---|---|
| 1 | Laptop | Laptop USB | Quickest bring-up; WiFi RTT tolerable |
| 2 | Pi | Laptop USB | You want a smooth 50 Hz loop without WiFi in the path |
| 3 | Pi | Pi USB | IL recording with Pi-attached cameras |

---

## Hardware

- **Robot**: XLeRobot 2-wheels v0.4.0 — dual SO-101 arms, 2-DOF head, motorized Z-lift, differential drive base.
- **Motors**: 17 × Feetech STS3215 across two USB control boards (CH343 chips).
  - Bus 1 → `/dev/xlerobot_bus1` — left arm IDs 1-6 + head IDs 7-8 (8 motors)
  - Bus 2 → `/dev/xlerobot_bus2` — right arm IDs 1-6 + wheels IDs 9-10 + Z-lift ID 11 (9 motors)
- **Pi**: Raspberry Pi 4, 1 GB RAM, Debian 13. Two USB-C ports for camera USB (separate from the motor bus USB).
- **Power**: Anker SOLIX C300 DC (288 Wh, 300 W, USB-C 140 W per port).
- **Cameras (Phase 3)**: any V4L2-compatible USB cameras that support MJPEG output. Tested at 640×480 @ 30 fps.
- **VR (optional)**: Meta Quest 3 with the XLeVR WebSocket + HTTPS server running on the laptop.

`udev` rules at [`99-xlerobot.rules`](99-xlerobot.rules) bind the two motor boards to deterministic device paths so the bus numbering is stable across reboots and cable swaps.

---

## Pi-side first-time setup

1. **Clone and create venv** (Debian 13 enforces PEP 668, so a venv is mandatory):

   ```bash
   git clone git@github.com:HasflarSonto/NoriTelop.git ~/NoriTelop
   python3 -m venv ~/NoriTelop/rpi4/venv
   ~/NoriTelop/rpi4/venv/bin/pip install -r ~/NoriTelop/rpi4/requirements.txt
   ```

2. **Install udev rules + ser2net** (one-shot script):

   ```bash
   bash ~/NoriTelop/rpi4/setup_pi.sh
   ```

3. **Copy calibration JSON from the laptop** — required, the server refuses to run without it:

   ```bash
   # On the laptop:
   scp ~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json \
       antonio@<pi-host>:~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/
   ```

4. **(Phase 3 only) verify cameras are visible**:

   ```bash
   v4l2-ctl --list-devices
   ```

   Edit `CAMERAS` at the top of `image_server.py` so the indices match what `v4l2-ctl` shows.

---

## Daily ops

### Phase 1 — bus bridge only

`ser2net` runs as a systemd unit and exposes the two buses on TCP 3001 / 3002. Nothing else to start on the Pi. On the laptop:

```bash
export XLEROBOT_BRIDGE=<pi-host>           # bash/zsh
$env:XLEROBOT_BRIDGE = "<pi-host>"         # PowerShell
python examples/xlerobot/4_xlerobot_2wheels_teleop_keyboard.py   # or any laptop teleop
```

`_ports.py` resolves `socket://<pi-host>:3001/3002` and a monkey-patch on `scservo_sdk.PortHandler.setupPort` routes `socket://` URLs through `serial_for_url`. The packet timeout floor is bumped from 50 ms → 500 ms on socket URLs so WiFi jitter doesn't trip serial timeouts.

### Phase 2 — motor server on Pi

```bash
bash ~/NoriTelop/rpi4/start_teleop.sh
```

Wrapper stops `ser2net`, runs `teleop_server.py` on TCP 7777, and restarts `ser2net` when the server exits. It also brings up the [NoriScreen](noriscreen/) kiosk on the Pi's 7" DSI display (status + E-STOP UI on HTTP 9091).

On the laptop (keyboard):

```bash
export XLEROBOT_BRIDGE=<pi-host>
python examples/xlerobot/teleop_client.py
```

The client prompts for bus 1 / 2 / 3 at startup. macOS works — `pynput` needs Accessibility permission for the terminal (System Settings → Privacy & Security → Accessibility → add Terminal.app).

### Phase 3 — cameras

In a second terminal on the Pi:

```bash
bash ~/NoriTelop/rpi4/start_camera_server.sh
```

Independent of the motor server (zero shared state, separate sockets). One PUB socket per camera on its own port. Pi RAM cost: ~85 MB total for both cameras at 640×480 / 30 fps.

---

## Phase 2 wire protocol

JSON-line TCP. Server listens on 7777, accepts one client at a time.

**Handshake:**

```jsonc
// Client → server
{"hello": {"bus_choice": "1"|"2"|"3"}}

// Server → client
{"ack": true, "initial_obs": {"motor_name": pos_deg, ...}}
```

**Client frames (any subset of fields, sent ~50 Hz):**

```jsonc
{"keys": ["w", "a", ...]}                          // keyboard mode (jog)
{"vr": {"left": {...}, "right": {...}}}            // VR mode (Pi-side IK)
{"arm_targets": {"right_arm_shoulder_pan": 12.5}}  // laptop did the IK; Pi P-controls toward this
{"wheel_targets": {"x.vel": 0.3, "theta.vel": 0.0}}// explicit base velocity override
{"reset_left": true, "reset_right": true, "reset_head": true}
{"bye": true}                                      // graceful disconnect
```

**Server telemetry (every Pi loop tick, ~50 Hz):**

```jsonc
{"ts_ns": 12345678901234,                          // Pi monotonic_ns
 "state": {"right_arm_shoulder_pan": 12.4, ...}}   // current motor positions in degrees
```

Every `TELEMETRY_INTERVAL` (10) loops the same line additionally carries:

```jsonc
{"loop_hz": 50.1, "errors": 0, "stalled": []}
```

**Dead-man:** if no client frame for 500 ms, wheels zero out and arm targets freeze in place. Reconnect resumes from the last commanded state.

**Backward compatibility:** clients that only send `keys[]` and ignore unknown telemetry fields keep working unchanged.

---

## Phase 3 ZMQ wire format

Each camera publishes on its own PUB socket. Wire format matches lerobot's `ZMQCamera` parser:

```jsonc
{"timestamps": {"<camera_name>": <pi_monotonic_seconds>},
 "images":     {"<camera_name>": "<base64-jpeg>"}}
```

`CONFLATE=1` on every socket, both publisher and subscriber. WiFi backpressure manifests as dropped frames, not memory growth. Pi-side timestamps are stamped at `cap.read()` so network jitter doesn't bleed into the recorded dataset.

---

## Imitation learning

Two recording paths share the same right-arm 6 DOF / local-dataset shape. Pick based on where your cameras are:

### Direct-USB / Phase 1 bridge — laptop owns everything

```bash
python examples/xlerobot/9_xlerobot_2wheels_teleop_vr_record.py
```

Cameras configured in `src/lerobot/robots/xlerobot_2wheels/config_xlerobot_2wheels.py`. Edit `xlerobot_2wheels_cameras_config()` so the OpenCV indices match `python -m lerobot.scripts.lerobot_find_cameras opencv` output.

### Phase 2 + Phase 3 — Pi owns motors and cameras

Two terminals on the Pi (`start_teleop.sh` + `start_camera_server.sh`), then on the laptop:

```bash
export XLEROBOT_BRIDGE=<pi-host>
python examples/xlerobot/9_phase2_vr_record.py
```

Both recording scripts share these knobs at the top of the file:

```python
TASK = "pick up the red block"
DATASET_REPO = "local/xlerobot_right_arm_pickup"
DATASET_ROOT = "outputs/datasets/xlerobot_right_arm_pickup"
NUM_EPISODES = 30
EPISODE_LEN_SEC = 15
RESET_TIME_SEC = 5
FPS = 30
```

Auto-advance flow: record `EPISODE_LEN_SEC` → save episode → `RESET_TIME_SEC` window for repositioning → next episode. ESC aborts.

### Train ACT once you have demos

```bash
python -m lerobot.scripts.lerobot_train \
    --policy.type=act \
    --dataset.repo_id=local/xlerobot_right_arm_pickup \
    --dataset.root=outputs/datasets/xlerobot_right_arm_pickup \
    --output_dir=outputs/train/act_right_arm_pickup \
    --steps=80000 --batch_size=8
```

CUDA GPU recommended. The dataset format is what `lerobot_train` expects out of the box.

---

## File map

### Pi-side (`rpi4/`)

| File | Purpose |
|---|---|
| [`teleop_server.py`](teleop_server.py) | 50 Hz motor control loop (TCP 7777). Owns both buses via `scservo_sdk`, P-controls arms/head, smooth base accel/decel, stall detection, NoriScreen E-STOP listener on HTTP 9091. |
| [`image_server.py`](image_server.py) | Multi-camera ZMQ publisher. One PUB socket per camera, MJPEG fourcc on the V4L2 capture, `CONFLATE=1`. Standalone — no lerobot import. |
| [`start_teleop.sh`](start_teleop.sh) | Stops `ser2net` → runs `teleop_server.py` + NoriScreen kiosk → restarts `ser2net` on exit. |
| [`start_camera_server.sh`](start_camera_server.sh) | Runs `image_server.py`. Independent of the motor server. |
| [`setup_pi.sh`](setup_pi.sh) | One-shot Pi setup: udev rules, ser2net config, dependencies. |
| [`recalibrate.sh`](recalibrate.sh) | Stops the motor server → runs `recalibrate.py` → restarts. |
| [`identify_buses.py`](identify_buses.py) | Pings head + wheels to discover which `/dev/xlerobot_bus*` is which. |
| [`ser2net.yaml`](ser2net.yaml) | Phase 1 bridge config (TCP 3001/3002 → bus1/bus2). |
| [`99-xlerobot.rules`](99-xlerobot.rules) | udev rules pinning the motor boards to deterministic paths. |
| [`requirements.txt`](requirements.txt) | Pi venv pins. |
| [`noriscreen/`](noriscreen/) | Kiosk UI (status + E-STOP) shown on the Pi's DSI screen. |

### Laptop-side (`examples/xlerobot/`)

| File | Purpose |
|---|---|
| `4_xlerobot_2wheels_teleop_keyboard.py` | Keyboard teleop, direct-USB / Phase 1 bridge. |
| `8_xlerobot_2wheels_teleop_vr.py` | VR teleop, direct-USB / Phase 1 bridge. |
| `teleop_client.py` | Phase 2 keyboard client. macOS-compatible. |
| `9_xlerobot_2wheels_teleop_vr_record.py` | IL recording over direct-USB / Phase 1 bridge — VR + LeRobotDataset writer, right arm 6 DOF. |
| `9_phase2_vr_record.py` | IL recording over Phase 2 + Phase 3 — TCP control + ZMQ cameras. Right arm 6 DOF. |
| `recalibrate.py` | Recalibrate a single arm: `python recalibrate.py right`. |
| `fix_eeprom.py` | Set firmware-level motor protection (run once on a fresh setup). |
| `test_bus.py`, `detect_buses.py` | Diagnostic tools. |
| `set_motor_id.py` | Change a Feetech motor's ID (for replacement motors). |

---

## Motor protection — read this before turning anything on

The XLeRobot uses 1:345 plastic-gear servos that **die quickly under sustained stall**. Two motors have already been killed (left elbow, right gripper). The protection system has four layers:

1. **`Torque_Limit=600`** in SRAM — caps peak current so the C300 power station doesn't trip.
2. **Software stall detection** (`StallDetector` class in `teleop_server.py`) reads `Present_Current` + `Present_Position` each loop. If high current + no movement for 15 loops → reduce `Torque_Limit` (gripper: hold gently at 200; other joints: cut to 100 until target changes).
3. **Calibration clamping** in `motors_bus.py:_unnormalize` — every `Goal_Position` is clamped to the calibrated `[range_min, range_max]`. Motors cannot be commanded beyond physical limits.
4. **Firmware backup** (set via `examples/xlerobot/fix_eeprom.py`):
   - `Protection_Current=450` (~1.8 A) — motor self-disables if software fails
   - `Over_Current_Protection_Time=150` — 1.5 s grace for movements
   - `Max_Temperature_Limit=70` — thermal cutoff
   - `Max_Torque_Limit=600` — power-on default of `Torque_Limit`

**Never** raise `Max_Temperature_Limit` above 70 or `Over_Current_Protection_Time` above 150 — they trade thermal safety for nothing useful.

**Power station rules:**

- Each board on its own USB-C port (separate 140 W budget)
- Head motors enable LAST with a 0.5 s delay after arms — prevents a startup current spike
- Battery <30 % reduces per-port current capacity → trips on the same load that's safe at 100 %

---

## Troubleshooting

**`teleop_client.py` connects but the robot doesn't move (macOS)** — `pynput` needs Accessibility permission. System Settings → Privacy & Security → Accessibility → add Terminal.app, then quit and relaunch the terminal.

**`getaddrinfo failed` on `xlerobot.local`** — mDNS isn't resolving. Use the IP directly: `XLEROBOT_BRIDGE=<ip>`.

**Phase 2 client connect succeeds but handshake times out** — `teleop_server.py` is probably stuck initialising one of the buses. Check the Pi terminal for the actual error. Common cause: a motor isn't responding (loose cable / power off).

**`ser2net` port busy when starting `teleop_server.py`** — `start_teleop.sh` should stop `ser2net` automatically; if it didn't, `sudo systemctl stop ser2net && pkill -f teleop_server` then retry.

**Phase 3 camera streams stop after a few seconds** — almost always Pi 4 USB power. If both motor boards and both cameras share USB, the Pi's USB regulator can't sustain it. Use a powered USB hub for the cameras.

**Motor `loop_hz` drops below 50 with `image_server.py` running** — Pi is starving. First check is `htop` to see if a camera thread is pegging the CPU; if so, drop `JPEG_QUALITY` from 80 → 60 in `image_server.py`, or drop resolution to 320×240.

**`async_read timeout` on a `ZMQCamera` instance** — the laptop subscriber isn't getting frames. Either the camera process is down, the WiFi link is broken, or a firewall is blocking the ZMQ port. `nc -zv <pi-host> 5555` from the laptop to test.

---

## Environment

- **Pi**: Debian 13, Python 3.11+, venv at `~/NoriTelop/rpi4/venv`. PEP 668 is enforced — don't `pip install` outside the venv.
- **Laptop**: Linux / macOS / Windows. Python 3.12 + lerobot v0.5.1+. macOS works for `teleop_client.py` (with the Accessibility caveat above).
- **Network**: laptop and Pi on the same LAN or hotspot. WiFi RTT is the dominant latency in Phase 1; Phase 2/3 is much more tolerant since the control loop lives Pi-side.

---

## Contributing

This is an overlay repo. Only files in `rpi4/`, `examples/xlerobot/9_*`, and `src/lerobot/robots/xlerobot_2wheels/config_xlerobot_2wheels.py` are tracked here — everything else is upstream lerobot. When you change an upstream file deliberately, add it explicitly to the commit (don't `git add -A`, the working tree carries the whole upstream lerobot tree untracked).

Stick to the existing commit-message style: `Phase N: <one-line summary>` for phased features, or `<file>: <one-line summary>` for narrow fixes.
