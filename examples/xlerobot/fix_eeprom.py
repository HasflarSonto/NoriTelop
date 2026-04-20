"""Set firmware protection on ALL motors — run once after motor swap."""
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.robots.xlerobot_2wheels._ports import get_bus1_port, get_bus2_port

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
bus2_motors = {
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

EEPROM_SETTINGS = {
    "Protection_Current": 450,           # ~1.8A max sustained (factory=500/~2A)
    "Over_Current_Protection_Time": 150,  # 1.5s grace for brief movements (factory=100)
    "Max_Temperature_Limit": 70,          # factory default thermal cutoff
    "Max_Torque_Limit": 2047,             # full torque in EEPROM (software handles limiting)
}

for label, port, motors in [("BUS1", get_bus1_port(), bus1_motors),
                              ("BUS2", get_bus2_port(), bus2_motors)]:
    print(f"\n=== {label} ===")
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect()
    bus.configure_motors(return_delay_time=20)
    bus.disable_torque()

    for name in motors:
        for reg, val in EEPROM_SETTINGS.items():
            try:
                bus.write(reg, name, val, normalize=False)
            except Exception as e:
                print(f"  {name} {reg}: FAILED - {e}")

    # Verify
    for reg in EEPROM_SETTINGS:
        vals = bus.sync_read(reg, list(motors.keys()), normalize=False)
        values = [str(v) for v in vals.values()]
        ok = all(v == str(EEPROM_SETTINGS[reg]) for v in values)
        print(f"  {reg}: {', '.join(values)} {'OK' if ok else 'MISMATCH'}")

    bus.port_handler.closePort()

print("\nDone. Firmware protection set on all motors.")
