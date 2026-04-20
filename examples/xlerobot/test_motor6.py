"""Test motor 6 after swap — plug new motor into chain, run this."""
import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.robots.xlerobot_2wheels._ports import get_bus2_port

nm = MotorNormMode.DEGREES
motors = {
    "shoulder_pan": Motor(1, "sts3215", nm),
    "shoulder_lift": Motor(2, "sts3215", nm),
    "elbow_flex": Motor(3, "sts3215", nm),
    "wrist_flex": Motor(4, "sts3215", nm),
    "wrist_roll": Motor(5, "sts3215", nm),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

bus = FeetechMotorsBus(port=get_bus2_port(), motors=motors)
bus.connect()
bus.configure_motors(return_delay_time=20)
bus.disable_torque()

print("=== Full right arm test (motors 1-6) ===\n")

for name in motors:
    bus.write("Operating_Mode", name, 0)
    bus.write("P_Coefficient", name, 16)
    bus.write("I_Coefficient", name, 0)
    bus.write("D_Coefficient", name, 43)

for name in motors:
    pos = bus.sync_read("Present_Position", [name], normalize=False)
    bus.write("Goal_Position", name, pos[name], normalize=False)
    bus.write("Torque_Enable", name, 1)
    time.sleep(0.05)
    print(f"  {name}: torque ON at pos {pos[name]}")

time.sleep(0.5)

print("\nRapid read test (50 rounds)...")
errs = 0
for i in range(50):
    try:
        bus.sync_read("Present_Position", list(motors.keys()), normalize=False)
    except:
        errs += 1
print(f"  Errors: {errs}/50")

print("\nMove gripper +200 / -200...")
try:
    pos = bus.sync_read("Present_Position", ["gripper"], normalize=False)["gripper"]
    bus.write("Goal_Position", "gripper", pos + 200, normalize=False)
    time.sleep(1)
    bus.write("Goal_Position", "gripper", pos - 200, normalize=False)
    time.sleep(1)
    bus.write("Goal_Position", "gripper", pos, normalize=False)
    time.sleep(0.5)
    print("  Gripper movement: OK")
except Exception as e:
    print(f"  Gripper movement: FAILED - {e}")

bus.disable_torque()
bus.port_handler.closePort()
print("\nDone.")
