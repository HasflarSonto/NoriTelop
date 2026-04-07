"""Set a new STS3215 motor's ID. No heavy imports — fast startup.
Connect ONLY the new motor to the bus (unplug all others)."""
import time
import serial

print("=== Set Motor ID (STS3215) ===\n")
PORT = input("Which port? (COM5/COM6) [COM5]: ").strip() or "COM5"
if PORT.isdigit():
    PORT = f"COM{PORT}"

HEADER = bytes([0xFF, 0xFF])

def checksum(packet):
    return (~sum(packet) & 0xFF)

def read_register(ser, motor_id, addr, length=1):
    ser.reset_input_buffer()
    pkt = [motor_id, 4, 0x02, addr, length]
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.02)
    resp = ser.read(ser.in_waiting)
    if len(resp) >= 6 + length:
        if length == 1:
            return resp[5]
        else:
            return resp[5] | (resp[6] << 8)
    return None

def write_register(ser, motor_id, addr, value, length=1):
    if length == 1:
        data = [value & 0xFF]
    else:
        data = [value & 0xFF, (value >> 8) & 0xFF]
    params = [addr] + data
    pkt = [motor_id, len(params) + 2, 0x03] + params
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.05)
    return ser.read(ser.in_waiting)

ser = serial.Serial(PORT, 1000000, timeout=0.1)
time.sleep(0.1)

# Scan IDs 1-16
print(f"Scanning {PORT} for motors (ID 1-16)...")
found = []
for test_id in range(1, 17):
    pos = read_register(ser, test_id, 56, 2)
    if pos is not None:
        print(f"  ID {test_id}: FOUND (position: {pos})")
        found.append(test_id)

if not found:
    print("\n  No motors found! Check cable and power.")
    ser.close()
    exit()

if len(found) > 1:
    print(f"\n  WARNING: Multiple motors found ({found}). Only ONE motor should be connected!")
    print("  Unplug all others first.")
    ser.close()
    exit()

current_id = found[0]
print(f"\nMotor found at ID {current_id}")
TARGET_ID = int(input(f"Set to which ID? (1-10): ").strip())

print(f"\n  Unlocking EEPROM...")
write_register(ser, current_id, 55, 0)

print(f"  Writing ID = {TARGET_ID}...")
write_register(ser, current_id, 5, TARGET_ID)
time.sleep(0.5)

print(f"  Verifying...")
pos = read_register(ser, TARGET_ID, 56, 2)
if pos is not None:
    print(f"  Motor responding at ID {TARGET_ID}, position: {pos}")
    print(f"\n  SUCCESS! ID changed from {current_id} to {TARGET_ID}.")
else:
    print(f"  FAILED — no response at ID {TARGET_ID}")

ser.close()
