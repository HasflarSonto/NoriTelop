# NoriTelop — XLeRobot 2-Wheel Teleop Fixes

Patches and drivers for the [XLeRobot v0.4.0](https://github.com/Vector-Wangel/XLeRobot) 2-wheel differential drive robot, built on top of [LeRobot](https://github.com/huggingface/lerobot).

## What's in this repo

- **`src/lerobot/robots/xlerobot_2wheels/`** — Robot driver with bus reliability fixes
- **`src/lerobot/motors/motors_bus.py`** — Modified motors bus (patched `_sync_read` retry)
- **`src/lerobot/model/SO101Robot.py`** — Analytical IK solver for SO-101 arms
- **`examples/xlerobot/4_xlerobot_2wheels_teleop_keyboard.py`** — Keyboard teleop with Ender 3 support
- **`examples/xlerobot/test_bus.py`** — Bus communication diagnostic tool
- **`CLAUDE.md`** — Full project knowledge base (bugs found, register info, controls)

## Key fixes over upstream XLeRobot

1. **`configure_motors()` bug** — Original code called `bus2.configure_motors()` twice, never `bus1`. Bus1 motors had 500us response delay causing "no status packet" crashes.
2. **`sync_write` replaced with individual writes** — `sync_write("Goal_Position")` kills the half-duplex bus. Individual `write()` calls work reliably.
3. **Torque limiting for portable power** — `Torque_Limit=600` prevents tripping Anker SOLIX C300 overcurrent protection.
4. **Single `get_observation()` per loop** — Original called it 4x per iteration, flooding the bus.
5. **Error recovery** — `try/except` in main loop with 0.5s recovery instead of crashing.

## Setup

```bash
# 1. Install LeRobot (requires Python 3.12+)
git clone --depth=1 https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[feetech]"

# 2. Clone this repo
git clone https://github.com/HasflarSonto/NoriTelop.git

# 3. Copy files into lerobot
cp -r NoriTelop/src/lerobot/robots/xlerobot_2wheels lerobot/src/lerobot/robots/
cp NoriTelop/src/lerobot/model/SO101Robot.py lerobot/src/lerobot/model/
cp NoriTelop/src/lerobot/motors/motors_bus.py lerobot/src/lerobot/motors/
cp NoriTelop/examples/xlerobot/* lerobot/examples/xlerobot/

# 4. Update COM ports in config
# Edit src/lerobot/robots/xlerobot_2wheels/config_xlerobot_2wheels.py
# Set port1 and port2 to your COM ports

# 5. Run
cd lerobot/examples/xlerobot
python 4_xlerobot_2wheels_teleop_keyboard.py
```

## Hardware

- XLeRobot v0.4.0 (2-wheel, dual SO-101 arms, 2-DOF head)
- 16x Feetech STS3215 servos (12V)
- Optional: Modified Ender 3 (Z-lift + Y-bed, arrow keys)
- Optional: Meta Quest 3 (VR teleop — coming soon)
