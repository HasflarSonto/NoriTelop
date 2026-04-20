"""Recalibrate a single arm without touching the other.
Usage:
    python recalibrate.py right
    python recalibrate.py left
"""
import sys
import json
from pathlib import Path
from lerobot.robots.xlerobot_2wheels import XLerobot2WheelsConfig, XLerobot2Wheels
from lerobot.robots.xlerobot_2wheels._ports import get_bus1_port, get_bus2_port
from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode

if len(sys.argv) < 2 or sys.argv[1] not in ("left", "right"):
    print("Usage: python recalibrate.py [left|right]")
    sys.exit(1)

side = sys.argv[1]
calib_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"

robot_config = XLerobot2WheelsConfig(id="my_xlerobot_2wheels_lab")
robot = XLerobot2Wheels(robot_config)

if side == "right":
    bus = robot.bus2
    arm_motors = robot.right_arm_motors
    extra_motors = robot.base_motors
    port = get_bus2_port()
elif side == "left":
    bus = robot.bus1
    arm_motors = robot.left_arm_motors
    extra_motors = robot.head_motors
    port = get_bus1_port()

print(f"=== Recalibrating {side} arm on {port} ===\n")
print(f"Arm motors: {arm_motors}")
print(f"Extra motors: {extra_motors}\n")

# Connect only the needed bus
bus.connect()
bus.configure_motors(return_delay_time=20)
bus.disable_torque()

for name in arm_motors:
    bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

# Step 1: Homing
input(f"Move {side} arm motors to the MIDDLE of their range of motion and press ENTER...")
homing_offsets = bus.set_half_turn_homings(arm_motors)

# Extra motors get 0 offset
for name in extra_motors:
    homing_offsets[name] = 0

# Step 2: Range recording
full_turn = [m for m in extra_motors if "wheel" in m]
range_motors = [m for m in arm_motors + extra_motors if m not in full_turn]

print(f"\nMove all {side} arm joints through their FULL range of motion.")
print("Press ENTER when done...")
range_mins, range_maxes = bus.record_ranges_of_motion(range_motors)

for name in full_turn:
    range_mins[name] = 0
    range_maxes[name] = 4095

# Build calibration for this side
new_cal = {}
for name, motor in bus.motors.items():
    new_cal[name] = MotorCalibration(
        id=motor.id,
        drive_mode=0,
        homing_offset=homing_offsets.get(name, 0),
        range_min=range_mins[name],
        range_max=range_maxes[name],
    )

# Write to motors
bus.write_calibration(new_cal)

# Load existing calibration, merge, save
existing = {}
if calib_path.is_file():
    with open(calib_path) as f:
        raw = json.load(f)
    for name, data in raw.items():
        existing[name] = data

# Update with new calibration
for name, cal in new_cal.items():
    existing[name] = {
        "id": cal.id,
        "drive_mode": cal.drive_mode,
        "homing_offset": cal.homing_offset,
        "range_min": cal.range_min,
        "range_max": cal.range_max,
    }

calib_path.parent.mkdir(parents=True, exist_ok=True)
with open(calib_path, "w") as f:
    json.dump(existing, f, indent=4)

bus.port_handler.closePort()
print(f"\nCalibration saved to {calib_path}")
print(f"Only {side} arm was recalibrated. Other side unchanged.")
