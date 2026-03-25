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

## Critical Bugs Found & Fixed

### 1. Missing `bus1.configure_motors()` (original XLeRobot bug)
`xlerobot_2wheels.py` line 302-303 had `bus2.configure_motors()` called twice (copy-paste error), `bus1` never called. This left bus1 motors with 500us response delay (vs 2us on bus2), causing "no status packet" errors.

### 2. `configure_motors()` called too late
Must be called in `connect()` immediately after `bus.connect()`, BEFORE `write_calibration()`. Otherwise calibration EEPROM writes happen with 500us delay.

### 3. `sync_write("Goal_Position")` kills the bus
Writing Goal_Position to all motors simultaneously via sync_write causes the bus to die. **Use individual `write()` calls instead.** This was proven by test_bus.py: individual writes = 0 errors, sync_write = bus death.

### 4. Power station overcurrent
The Anker C300's per-port overcurrent protection trips when motors draw too much. Fix: `Torque_Limit=600` (29%). Sweet spot found by testing: 500 too weak, 650 trips on hard pushes, 600 is stable.

### 5. `get_observation()` called 4x per loop
Each `p_control_action()` called `get_observation()` internally. Fixed to call once and pass `obs` to all controllers.

## Key Registers (STS3215)

| Register | Address | Notes |
|----------|---------|-------|
| Torque_Limit | (48, 2) | SRAM. 0-2047. Set to 600 for C300 power station |
| Protection_Current | (28, 2) | EEPROM. Default ~500 (~2A). Per-motor sustained current limit |
| Max_Temperature_Limit | (13, 1) | EEPROM. Default ~70. Set to 85 to avoid false overheat |
| Over_Current_Protection_Time | (38, 1) | EEPROM. Set to 200 for more headroom |
| Return_Delay_Time | EEPROM | Default 250 (500us). configure_motors() sets to 0-20 (2-40us) |
| P/I/D Coefficients | SRAM | Set to 16/0/43 by XLeRobot |

## File Locations

| File | Purpose |
|------|---------|
| `lerobot/src/lerobot/robots/xlerobot_2wheels/xlerobot_2wheels.py` | Robot driver (motor control) |
| `lerobot/src/lerobot/robots/xlerobot_2wheels/config_xlerobot_2wheels.py` | COM port config (COM5, COM6) |
| `lerobot/examples/xlerobot/4_xlerobot_2wheels_teleop_keyboard.py` | Keyboard teleop + Ender 3 |
| `lerobot/examples/xlerobot/8_xlerobot_2wheels_teleop_vr.py` | VR teleop (needs updating with same fixes) |
| `XLeRobot/XLeVR/web-ui/vr_app.js` | Quest 3 WebXR app |
| `XLeRobot/XLeVR/xlevr/inputs/vr_ws_server.py` | VR WebSocket server |
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
- Wheels: handled by robot's built-in keys (i/k/j/l)
- Ender 3: Arrow Up/Down (Z lift), Arrow Right/Left (Y bed)

## Environment

- Windows 11, Python 3.12, conda env "lerobot"
- LeRobot v0.5.1 (requires Python 3.12+)
- Working directory: `C:\Users\Antonio\Desktop\XLeRobot_Setup`
