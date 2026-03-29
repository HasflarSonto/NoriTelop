"""Test: move elbow_flex (motor 3) with reduced torque and slower speed."""
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

arm1 = [n for n in bus1_motors if "arm" in n]

print("=== TEST: MOTOR 3 (elbow_flex) WITH REDUCED TORQUE ===\n")

bus1 = FeetechMotorsBus(port="COM5", motors=bus1_motors)
bus1.connect()
bus1.configure_motors(return_delay_time=20)

calib_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"
with open(calib_path) as f:
    calib_data = json.load(f)
for name in bus1_motors:
    if name in calib_data:
        c = calib_data[name]
        bus1.calibration[name] = MotorCalibration(id=c["id"], drive_mode=c["drive_mode"],
            homing_offset=c["homing_offset"], range_min=c["range_min"], range_max=c["range_max"])
bus1.write_calibration(bus1.calibration)
bus1.disable_torque()

# Configure — give motor 3 lower torque
for name in arm1 + ["head_motor_1", "head_motor_2"]:
    bus1.write("Operating_Mode", name, 0)
    bus1.write("P_Coefficient", name, 16)
    bus1.write("I_Coefficient", name, 0)
    bus1.write("D_Coefficient", name, 43)
    if name == "left_arm_elbow_flex":
        bus1.write("Torque_Limit", name, 300)
        print(f"  {name}: Torque_Limit=300 (reduced)")
    else:
        bus1.write("Torque_Limit", name, 600)

# Enable torque
for name in arm1 + ["head_motor_1", "head_motor_2"]:
    pos = bus1.sync_read("Present_Position", [name], normalize=False)
    bus1.write("Goal_Position", name, pos[name], normalize=False)
    bus1.write("Torque_Enable", name, 1)
    time.sleep(0.05)
print("All bus1 torque ON\n")

# Read current elbow position
current = bus1.sync_read("Present_Position", ["left_arm_elbow_flex"], normalize=True)["left_arm_elbow_flex"]
print(f"Elbow flex current: {current:.1f} degrees")
diff = 0.0 - current

# Move elbow to zero VERY SLOWLY (100 steps, 50ms each = 5 seconds)
STEPS = 100
DELAY = 0.05
print(f"Moving elbow to zero: {current:.1f} -> 0.0 ({diff:+.1f} deg) in {STEPS} steps...")
errs = 0
for i in range(1, STEPS + 1):
    alpha = i / STEPS
    target = current + diff * alpha
    try:
        bus1.write("Goal_Position", "left_arm_elbow_flex", target, num_retry=5)
    except:
        errs += 1
    time.sleep(DELAY)

print(f"  Errors: {errs}/{STEPS}")

# Check bus
try:
    bus1.sync_read("Present_Position", arm1, normalize=False)
    print("  Bus1: ALIVE")
except:
    print("  *** BUS1 DEAD ***")

print("\nCleanup")
try: bus1.disable_torque()
except: print("  disable failed")
bus1.port_handler.closePort()
print("Done.")
