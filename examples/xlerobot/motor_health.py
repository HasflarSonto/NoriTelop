"""Motor Health Check — tests each motor individually to find damaged ones.
Usage:
    python motor_health.py 1   (test bus1: left arm + head)
    python motor_health.py 2   (test bus2: right arm + wheels + z-lift)
"""
import sys
import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.robots.xlerobot_2wheels._ports import get_bus1_port, get_bus2_port

nm = MotorNormMode.DEGREES

BUS1_MOTORS = {
    "left_arm_shoulder_pan": Motor(1, "sts3215", nm),
    "left_arm_shoulder_lift": Motor(2, "sts3215", nm),
    "left_arm_elbow_flex": Motor(3, "sts3215", nm),
    "left_arm_wrist_flex": Motor(4, "sts3215", nm),
    "left_arm_wrist_roll": Motor(5, "sts3215", nm),
    "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

BUS2_MOTORS = {
    "right_arm_shoulder_pan": Motor(1, "sts3215", nm),
    "right_arm_shoulder_lift": Motor(2, "sts3215", nm),
    "right_arm_elbow_flex": Motor(3, "sts3215", nm),
    "right_arm_wrist_flex": Motor(4, "sts3215", nm),
    "right_arm_wrist_roll": Motor(5, "sts3215", nm),
    "right_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    "base_left_wheel": Motor(9, "sts3215", nm),
    "base_right_wheel": Motor(10, "sts3215", nm),
    "z_lift": Motor(11, "sts3215", nm),
}

bus_arg = sys.argv[1] if len(sys.argv) > 1 else None
if bus_arg not in ("1", "2"):
    print("Usage: python motor_health.py [1|2]")
    sys.exit(1)

if bus_arg == "1":
    motors = BUS1_MOTORS
    port = get_bus1_port()
else:
    motors = BUS2_MOTORS
    port = get_bus2_port()
arm_motors = [n for n in motors if "arm" in n]
skip_motors = ["head_motor_1", "head_motor_2"]

print(f"=== Motor Health Check — bus{bus_arg} ({port}) ===\n")

bus = FeetechMotorsBus(port=port, motors=motors)
bus.connect()
bus.configure_motors(return_delay_time=20)
bus.disable_torque()

results = {}

for name in motors:
    if name in skip_motors:
        continue
    print(f"--- {name} (ID {motors[name].id}) ---")
    result = {"name": name, "issues": []}

    # Test 1: Can we read it?
    try:
        pos = bus.sync_read("Present_Position", [name], normalize=False)
        temp = bus.sync_read("Present_Temperature", [name], normalize=False)
        cur = bus.sync_read("Present_Current", [name], normalize=False)
        print(f"  Read:     pos={pos[name]}, temp={temp[name]}C, current={cur[name]}")
        if temp[name] > 50:
            result["issues"].append(f"HIGH TEMP: {temp[name]}C")
    except Exception as e:
        print(f"  Read:     FAILED — {e}")
        result["issues"].append("CANNOT READ")
        results[name] = result
        continue

    # Test 2: Enable torque (hold current position)
    try:
        bus.write("Torque_Limit", name, 600, normalize=False)
        bus.write("Goal_Position", name, pos[name], normalize=False)
        bus.write("Torque_Enable", name, 1)
        time.sleep(0.3)
        cur_hold = bus.sync_read("Present_Current", [name], normalize=False)[name]
        print(f"  Hold:     current={cur_hold} (should be <10)")
        if cur_hold > 20:
            result["issues"].append(f"HIGH HOLD CURRENT: {cur_hold}")
    except Exception as e:
        print(f"  Hold:     FAILED — {e}")
        result["issues"].append("TORQUE ENABLE FAILS (power trips?)")
        results[name] = result
        try:
            bus.write("Torque_Enable", name, 0)
        except:
            pass
        time.sleep(0.5)
        continue

    # Test 3: Move +50 steps and back (small movement)
    try:
        start = pos[name]
        target = start + 50
        bus.write("Goal_Position", name, target, normalize=False)
        time.sleep(0.5)
        new_pos = bus.sync_read("Present_Position", [name], normalize=False)[name]
        moved = abs(new_pos - start)
        cur_move = bus.sync_read("Present_Current", [name], normalize=False)[name]
        print(f"  Move +50: moved={moved} steps, current={cur_move}")

        if moved < 10:
            result["issues"].append(f"BARELY MOVED (+50 cmd, only moved {moved})")

        # Move back
        bus.write("Goal_Position", name, start, normalize=False)
        time.sleep(0.5)
        back_pos = bus.sync_read("Present_Position", [name], normalize=False)[name]
        back_err = abs(back_pos - start)
        print(f"  Return:   error={back_err} steps (should be <10)")
        if back_err > 20:
            result["issues"].append(f"POOR RETURN: {back_err} steps off")
    except Exception as e:
        print(f"  Move:     FAILED — {e}")
        result["issues"].append("MOVEMENT FAILS")

    # Test 4: Move +200 steps (bigger movement, checks for stiffness)
    try:
        start = bus.sync_read("Present_Position", [name], normalize=False)[name]
        target = start + 200
        bus.write("Goal_Position", name, target, normalize=False)
        time.sleep(1.0)
        new_pos = bus.sync_read("Present_Position", [name], normalize=False)[name]
        moved = abs(new_pos - start)
        cur_big = bus.sync_read("Present_Current", [name], normalize=False)[name]
        print(f"  Move+200: moved={moved} steps, current={cur_big}")

        if moved < 50:
            result["issues"].append(f"STIFF/JAMMED (+200 cmd, only moved {moved})")
        if cur_big > 40:
            result["issues"].append(f"HIGH CURRENT DURING MOVE: {cur_big}")

        # Return
        bus.write("Goal_Position", name, start, normalize=False)
        time.sleep(1.0)
    except Exception as e:
        print(f"  Big move: FAILED — {e}")
        result["issues"].append("BIG MOVEMENT FAILS")

    # Test 5: Bus still alive after this motor?
    try:
        bus.sync_read("Present_Position", [name], normalize=False)
    except:
        result["issues"].append("BUS DIES AFTER MOVEMENT")

    # Disable torque
    try:
        bus.write("Torque_Enable", name, 0)
    except:
        pass
    time.sleep(0.2)

    # Test 6: Can you turn it by hand? (check if motor is stiff when unpowered)
    print(f"  Manual:   Try turning {name} by hand now. Is it stiff? (y/n) ", end="", flush=True)
    stiff = input().strip().lower()
    if stiff == "y":
        result["issues"].append("STIFF WHEN UNPOWERED — gears likely damaged")

    results[name] = result
    print()

# Summary
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)

healthy = []
damaged = []
for name, r in results.items():
    if r["issues"]:
        damaged.append((name, r["issues"]))
    else:
        healthy.append(name)

if healthy:
    print(f"\nHEALTHY ({len(healthy)}):")
    for name in healthy:
        print(f"  {name} — OK")

if damaged:
    print(f"\nPROBLEMS ({len(damaged)}):")
    for name, issues in damaged:
        print(f"  {name}:")
        for issue in issues:
            print(f"    - {issue}")
else:
    print("\nAll motors healthy!")

print()
bus.port_handler.closePort()
