"""Set a new motor's ID. Connect ONLY the new motor to the bus (unplug all others)."""
import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

TARGET_ID = 6
PORT = "COM6"

# New motor defaults to ID 1
motors = {"new_motor": Motor(1, "sts3215", MotorNormMode.RANGE_0_100)}

print(f"=== Set motor ID to {TARGET_ID} on {PORT} ===")
print("Make sure ONLY the new motor is connected!")
input("Press ENTER when ready...")

bus = FeetechMotorsBus(port=PORT, motors=motors)
bus.connect()

# Read current ID to confirm connection
try:
    pos = bus.sync_read("Present_Position", ["new_motor"], normalize=False)
    print(f"  Motor found at ID 1, position: {pos['new_motor']}")
except Exception as e:
    print(f"  Can't find motor at ID 1: {e}")
    print("  Is the motor plugged in? Is it the only one on the bus?")
    bus.port_handler.closePort()
    exit()

# Write new ID
print(f"\n  Writing ID = {TARGET_ID}...")
bus.write("Lock", "new_motor", 0)  # Unlock EEPROM
bus.write("ID", "new_motor", TARGET_ID, normalize=False)
time.sleep(0.5)

# Verify — reconnect with new ID
bus.port_handler.closePort()
motors2 = {"gripper": Motor(TARGET_ID, "sts3215", MotorNormMode.RANGE_0_100)}
bus2 = FeetechMotorsBus(port=PORT, motors=motors2)
bus2.connect()

try:
    pos = bus2.sync_read("Present_Position", ["gripper"], normalize=False)
    print(f"  Motor responding at ID {TARGET_ID}, position: {pos['gripper']}")
    print(f"\n  SUCCESS! Motor ID set to {TARGET_ID}.")
    print("  Now reconnect the full daisy chain and run test_motor6.py")
except Exception as e:
    print(f"  FAILED to read at ID {TARGET_ID}: {e}")

bus2.port_handler.closePort()
