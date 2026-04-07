"""Spin motor ID 1 continuously with overload protection.
R = reverse, Q = quit. Monitors current and auto-stops on stall."""
import time
import serial
import threading

PORT = input("Which port? (COM5/COM6) [COM5]: ").strip() or "COM5"
if PORT.isdigit():
    PORT = f"COM{PORT}"

MOTOR_ID = 1
SPEED = 5000  # 0-32767 range
HEADER = bytes([0xFF, 0xFF])
CURRENT_THRESHOLD = 40   # stall detection threshold
STALL_LOOPS = 10         # consecutive high-current reads before cutting
TORQUE_LIMIT = 600       # max torque (matches teleop setting)

direction = 1
running = True

def checksum(pkt):
    return (~sum(pkt) & 0xFF)

def write_reg(ser, mid, addr, val, length=1):
    data = [val & 0xFF] if length == 1 else [val & 0xFF, (val >> 8) & 0xFF]
    params = [addr] + data
    pkt = [mid, len(params) + 2, 0x03] + params
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.02)
    ser.read(ser.in_waiting)

def read_reg(ser, mid, addr, length=2):
    ser.reset_input_buffer()
    pkt = [mid, 4, 0x02, addr, length]
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.02)
    resp = ser.read(ser.in_waiting)
    if len(resp) >= 6 + length:
        return resp[5] | (resp[6] << 8) if length == 2 else resp[5]
    return None

def set_velocity(ser, speed):
    """STS3215: sign-magnitude, bit 15 = direction."""
    magnitude = min(abs(speed), 32767)
    raw = magnitude | 0x8000 if speed < 0 else magnitude
    write_reg(ser, MOTOR_ID, 46, raw, 2)

ser = serial.Serial(PORT, 1000000, timeout=0.1)
time.sleep(0.1)

# Verify motor exists
pos = read_reg(ser, MOTOR_ID, 56, 2)
if pos is None:
    print(f"No motor found at ID {MOTOR_ID} on {PORT}")
    ser.close()
    exit()

# Configure
write_reg(ser, MOTOR_ID, 55, 0)       # Unlock EEPROM
write_reg(ser, MOTOR_ID, 33, 1)       # Operating_Mode = velocity
write_reg(ser, MOTOR_ID, 48, TORQUE_LIMIT & 0xFF, 1)  # Torque_Limit low byte
# Write full 2-byte torque limit
write_reg(ser, MOTOR_ID, 48, TORQUE_LIMIT, 2)
write_reg(ser, MOTOR_ID, 40, 1)       # Torque_Enable

set_velocity(ser, SPEED * direction)
print(f"Motor {MOTOR_ID} spinning CW at speed {SPEED} on {PORT}")
print(f"Torque_Limit={TORQUE_LIMIT}, stall threshold={CURRENT_THRESHOLD}")
print("R + Enter = reverse, Q + Enter = quit\n")

# Stall monitor in background
stall_count = 0
def monitor():
    global running, stall_count
    while running:
        cur = read_reg(ser, MOTOR_ID, 69, 2)  # Present_Current addr=69
        temp = read_reg(ser, MOTOR_ID, 63, 1)  # Present_Temperature addr=63
        if cur is not None and temp is not None:
            if cur > CURRENT_THRESHOLD:
                stall_count += 1
                if stall_count >= STALL_LOOPS:
                    print(f"\n[PROTECT] Motor stalled! current={cur}, temp={temp}C — stopping")
                    set_velocity(ser, 0)
                    write_reg(ser, MOTOR_ID, 40, 0)  # Torque off
                    running = False
                    return
            else:
                stall_count = 0
        time.sleep(0.1)

monitor_thread = threading.Thread(target=monitor, daemon=True)
monitor_thread.start()

try:
    while running:
        cmd = input().strip().lower()
        if cmd == "r":
            direction *= -1
            set_velocity(ser, SPEED * direction)
            print(f"  Now spinning {'CW' if direction > 0 else 'CCW'} at {SPEED}")
        elif cmd == "q":
            break
except (KeyboardInterrupt, EOFError):
    pass

running = False
time.sleep(0.2)
set_velocity(ser, 0)
write_reg(ser, MOTOR_ID, 40, 0)
write_reg(ser, MOTOR_ID, 55, 0)
write_reg(ser, MOTOR_ID, 33, 0)
ser.close()
print("Stopped.")
