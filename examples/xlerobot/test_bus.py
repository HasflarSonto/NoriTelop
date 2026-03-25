"""Test: individual Goal_Position writes vs sync_write."""
import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
import json
from pathlib import Path

nm = MotorNormMode.DEGREES

bus1_motors = {
    "left_arm_shoulder_pan": Motor(1, "sts3215", nm),
    "left_arm_shoulder_lift": Motor(2, "sts3215", nm),
    "left_arm_elbow_flex": Motor(3, "sts3215", nm),
    "left_arm_wrist_flex": Motor(4, "sts3215", nm),
    "left_arm_wrist_roll": Motor(5, "sts3215", nm),
    "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    "head_motor_1": Motor(7, "sts3215", nm),
    "head_motor_2": Motor(8, "sts3215", nm),
}

arm_motors = ["left_arm_shoulder_pan", "left_arm_shoulder_lift", "left_arm_elbow_flex",
              "left_arm_wrist_flex", "left_arm_wrist_roll", "left_arm_gripper"]
head_motors = ["head_motor_1", "head_motor_2"]

print("=== Test: Individual writes vs sync_write ===\n")

bus1 = FeetechMotorsBus(port="COM5", motors=bus1_motors)
bus1.connect()
bus1.configure_motors(return_delay_time=20)

# Load calibration
calib_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"
with open(calib_path) as f:
    calib_data = json.load(f)
bus1_calib = {}
for name in bus1_motors:
    if name in calib_data:
        c = calib_data[name]
        bus1_calib[name] = MotorCalibration(
            id=c["id"], drive_mode=c["drive_mode"],
            homing_offset=c["homing_offset"],
            range_min=c["range_min"], range_max=c["range_max"]
        )
bus1.write_calibration(bus1_calib)

bus1.disable_torque()
for name in arm_motors + head_motors:
    bus1.write("Operating_Mode", name, 0)
    bus1.write("P_Coefficient", name, 16)
    bus1.write("I_Coefficient", name, 0)
    bus1.write("D_Coefficient", name, 43)
    bus1.write("Torque_Limit", name, 800)

# Read positions
positions = bus1.sync_read("Present_Position", arm_motors + head_motors, normalize=False)
print(f"Positions: {positions}")

# Test A: Write Goal_Position individually (one motor at a time)
print("\nTest A: Individual Goal_Position writes...")
for name, pos in positions.items():
    try:
        bus1.write("Goal_Position", name, pos, normalize=False)
        print(f"  {name} = {pos}: OK")
    except Exception as e:
        print(f"  {name} = {pos}: FAIL - {e}")

# Check if bus still works
errs = 0
for i in range(20):
    try:
        bus1.sync_read("Present_Position", arm_motors, normalize=False)
    except:
        errs += 1
print(f"After individual writes: {errs}/20 errors")

# Enable torque one motor at a time
print("\nEnabling torque one motor at a time...")
for name in arm_motors + head_motors:
    try:
        bus1.write("Torque_Enable", name, 1)
        print(f"  {name}: torque ON")
    except Exception as e:
        print(f"  {name}: FAIL - {e}")
    time.sleep(0.05)  # 50ms between each motor

# Check reads
errs = 0
for i in range(50):
    try:
        bus1.sync_read("Present_Position", arm_motors, normalize=False)
        bus1.sync_read("Present_Position", head_motors, normalize=False)
    except:
        errs += 1
print(f"After staggered torque enable: {errs}/50 errors")

# Simulate teleop with individual writes
print("\nTeleop simulation (individual writes, 100 rounds):")
errs = 0
for i in range(100):
    try:
        pos = bus1.sync_read("Present_Position", arm_motors, normalize=False)
        for name, val in pos.items():
            bus1.write("Goal_Position", name, val, normalize=False)
        bus1.sync_read("Present_Position", head_motors, normalize=False)
    except Exception as e:
        errs += 1
        if errs <= 3:
            print(f"  error #{errs} at {i}: {str(e)[:80]}")
    time.sleep(0.02)
print(f"  Result: {errs}/100 errors")

# Now test with sync_write for comparison
print("\nTeleop simulation (sync_write, 100 rounds):")
errs2 = 0
for i in range(100):
    try:
        pos = bus1.sync_read("Present_Position", arm_motors, normalize=False)
        bus1.sync_write("Goal_Position", pos)
        bus1.sync_read("Present_Position", head_motors, normalize=False)
    except Exception as e:
        errs2 += 1
        if errs2 <= 3:
            print(f"  error #{errs2} at {i}: {str(e)[:80]}")
        if errs2 > 20:
            print("  stopping early")
            break
    time.sleep(0.02)
print(f"  Result: {errs2} errors")

print("\n--- Cleanup ---")
try:
    bus1.disable_torque()
except:
    print("  disable_torque failed")
bus1.disconnect()
print("Done.")
