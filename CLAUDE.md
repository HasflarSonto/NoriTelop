# XLeRobot Setup — Project Knowledge

## Hardware

- **Robot:** XLeRobot v0.4.0, 2-wheel differential drive, dual SO-101 arms, 2-DOF head
- **Motors:** 16x Feetech STS3215 (12V, 2.7A stall current, 180mA idle)
- **Boards:** 2x USB control boards (CH343 chips)
  - Board 1 (COM5): Left arm IDs 1-6 + Head IDs 7-8
  - Board 2 (COM6): Right arm IDs 1-6 + Wheels IDs 9-10
- **Ender 3:** Modified (X axis/extruder removed), Z-lift + Y-bed only, COM9, 115200 baud
- **Power:** Anker SOLIX C300 DC (288Wh, 300W max, USB-C 140W per port)
- **VR:** Meta Quest 3 (XLeVR WebSocket + HTTPS server)

## Motor Protection — CRITICAL

### How motors die
1. Motor gets a goal position far from current position
2. With low Torque_Limit (e.g. 600/29%), it can't move fast enough → stalls
3. Sustained stalling grinds the 1:345 plastic gears → **motor destroyed**
4. Two motors already killed this way (left elbow + right gripper)

### Prevention system
- **Torque_Limit=600** in SRAM — limits peak current to prevent tripping the C300 power station
- **Software stall detection** in teleop scripts (`StallDetector` class):
  - Reads `Present_Current` + `Present_Position` each loop
  - If high current + no movement for 15 loops → motor is stalled
  - **Gripper**: immediately reduces to `GRIP_TORQUE=200` (holds object gently)
  - **Other joints**: retries once, then reduces to `STALL_TORQUE=100` until target changes
  - Torque restores automatically when a new target is commanded
- **Calibration clamping** in `motors_bus.py _unnormalize()` — all Goal_Position writes are clamped to `[range_min, range_max]` from calibration. Motors cannot be commanded beyond physical limits.
- **Firmware backup** (set via `fix_eeprom.py`):
  - `Protection_Current=450` (~1.8A) — motor self-disables if software fails
  - `Over_Current_Protection_Time=150` — 1.5s grace for movements
  - `Max_Temperature_Limit=70` — thermal cutoff
  - `Max_Torque_Limit=600` — EEPROM default on power cycle

### Power station rules
- Each board on separate USB-C port (own 140W budget)
- Bus1 has 8 position-hold motors (arm+head) vs Bus2's 6+2wheels — bus1 draws more
- Low battery (<30%) reduces per-port current capacity → trips easier
- Head motors enable LAST with 0.5s delay after arms (prevents startup current spike)
- Never write `Max_Temperature_Limit=85` or `Over_Current_Protection_Time=200` — these weaken protection and let motors overheat

## Critical Bugs Found & Fixed

### 1. Missing `bus1.configure_motors()` (original XLeRobot bug)
`xlerobot_2wheels.py` had `bus2.configure_motors()` called twice (copy-paste error), `bus1` never called. This left bus1 motors with 500us response delay (vs 2us on bus2), causing "no status packet" errors.

### 2. `configure_motors()` called too late
Must be called in `connect()` immediately after `bus.connect()`, BEFORE `write_calibration()`. Otherwise calibration EEPROM writes happen with 500us delay.

### 3. `sync_write("Goal_Position")` kills the bus
Writing Goal_Position to all motors simultaneously via sync_write causes the bus to die. **Use individual `write()` calls instead.** Proven by test_bus.py: individual writes = 0 errors, sync_write = bus death.

### 4. `get_observation()` called 4x per loop
Each `p_control_action()` called `get_observation()` internally. Fixed to call once and pass `obs` to all controllers.

### 5. No calibration clamping on DEGREES mode
`_unnormalize()` in `motors_bus.py` did not clamp raw values to `[range_min, range_max]` for DEGREES mode motors. This allowed Goal_Position writes beyond physical limits → motor stalls against joint stop → burns out. Fixed by adding `min(max_, max(min_, raw))` clamp.

## Key Registers (STS3215)

| Register | Address | Type | Notes |
|----------|---------|------|-------|
| Torque_Limit | (48, 2) | SRAM | 0-2047. Set to 600 for C300 power station |
| Max_Torque_Limit | EEPROM | EEPROM | Torque_Limit resets to this on power cycle. Set to 600 via fix_eeprom.py |
| Protection_Current | (28, 2) | EEPROM | Default ~500 (~2A). Set to 450 via fix_eeprom.py |
| Max_Temperature_Limit | (13, 1) | EEPROM | Default 70. Do NOT increase — protects motors |
| Over_Current_Protection_Time | (38, 1) | EEPROM | Set to 150 for movement headroom |
| Present_Current | (69, 2) | Read-only | Used by StallDetector |
| Return_Delay_Time | EEPROM | EEPROM | Default 250 (500us). configure_motors() sets to 20 (40us) |
| P/I/D Coefficients | SRAM | SRAM | Set to 16/0/43 by XLeRobot |

## File Locations

| File | Purpose |
|------|---------|
| `lerobot/src/lerobot/robots/xlerobot_2wheels/xlerobot_2wheels.py` | Robot driver (motor control) |
| `lerobot/src/lerobot/robots/xlerobot_2wheels/config_xlerobot_2wheels.py` | COM port config (COM5, COM6) |
| `lerobot/src/lerobot/motors/motors_bus.py` | Motor bus with calibration clamping fix |
| `lerobot/examples/xlerobot/4_xlerobot_2wheels_teleop_keyboard.py` | Keyboard teleop + Ender 3 + bus selection |
| `lerobot/examples/xlerobot/8_xlerobot_2wheels_teleop_vr.py` | VR teleop + bus selection |
| `lerobot/examples/xlerobot/recalibrate.py` | Recalibrate single arm: `python recalibrate.py right` |
| `lerobot/examples/xlerobot/fix_eeprom.py` | Set firmware protection on all motors (run once) |
| `lerobot/examples/xlerobot/test_bus.py` | Bus diagnostic tool |
| `lerobot/examples/xlerobot/set_motor_id.py` | Set motor ID for replacement motors |
| `XLeRobot/XLeVR/web-ui/vr_app.js` | Quest 3 WebXR app (buttons[4]=X/A, buttons[5]=Y/B) |
| `XLeRobot/XLeVR/xlevr/inputs/vr_ws_server.py` | VR WebSocket server (passes buttons in metadata) |
| `~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json` | Calibration file |

## Ender 3 G-Code Rules

- **Never** run `G28` or `G28 X` — firmware hangs on missing X endstop
- Always `G92 X0` first to suppress missing X axis
- Home with `G28 Y` then `G28 Z` individually
- Use `G91` for relative mode
- Z feed: F300 (slow). Y feed: F3000

## Keyboard Teleop Controls

- Left arm: q/e (pan), w/s (forward/back), a/d (left/right), r/f (wrist roll), t/g (gripper)
- Right arm: 7/9 (pan), 8/2 (forward/back), 4/6 (left/right), //\* (wrist roll), +/- (gripper)
- Head: </> (pan), ,/. (tilt)
- Wheels: i/k/j/l (forward/back/rotate)
- Ender 3: Arrow Up/Down (Z lift), Arrow Right/Left (Y bed)

## Bus Selection

Both teleop scripts prompt at startup:
- `1` = Bus1 only (left arm + head)
- `2` = Bus2 only (right arm + wheels) — use when bus1 motors are damaged
- `3` = Both buses
- Ender 3: y/N prompt

## Environment

- Windows 11, Python 3.12, conda env "lerobot"
- LeRobot v0.5.1 (requires Python 3.12+)
- Working directory: `C:\Users\Antonio\Desktop\XLeRobot_Setup`
