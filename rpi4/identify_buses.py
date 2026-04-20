"""Auto-identify which /dev/ttyACM* is bus1 vs bus2 by pinging motors.

Bus 1 is the one that responds to head motor IDs 7 or 8.
Bus 2 is the one that responds to wheel motor IDs 9 or 10 (or z-lift 11).

Prints to stdout in shell-eval friendly form:
    BUS1_SERIAL=<serial>
    BUS1_DEV=/dev/ttyACM?
    BUS2_SERIAL=<serial>
    BUS2_DEV=/dev/ttyACM?

Exits non-zero with a human-readable diagnostic if identification fails.
"""
import glob
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. sudo apt install python3-serial", file=sys.stderr)
    sys.exit(2)


HEADER = bytes([0xFF, 0xFF])
BUS1_UNIQUE_IDS = [7, 8]            # head motors live ONLY on bus1
BUS2_UNIQUE_IDS = [9, 10, 11]       # wheels + z-lift live ONLY on bus2


def checksum(pkt):
    return (~sum(pkt) & 0xFF)


def ping(ser, motor_id):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    pkt = [motor_id, 2, 0x01]
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.03)
    return ser.read(64)


def get_serial(dev):
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", dev],
            text=True, stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None
    for line in out.splitlines():
        if line.startswith("ID_SERIAL_SHORT="):
            return line.split("=", 1)[1]
    return None


def probe(dev):
    """Return ('bus1' | 'bus2' | 'unknown', responders_list)."""
    try:
        ser = serial.Serial(dev, baudrate=1000000, timeout=0.15)
    except Exception as e:
        return "open_failed", [str(e)]
    time.sleep(0.1)
    bus1_hits = [m for m in BUS1_UNIQUE_IDS if ping(ser, m)]
    bus2_hits = [m for m in BUS2_UNIQUE_IDS if ping(ser, m)]
    ser.close()
    if bus1_hits and not bus2_hits:
        return "bus1", bus1_hits
    if bus2_hits and not bus1_hits:
        return "bus2", bus2_hits
    if bus1_hits and bus2_hits:
        return "ambiguous", bus1_hits + bus2_hits
    return "silent", []


def main():
    devs = sorted(glob.glob("/dev/ttyACM*"))
    if len(devs) < 2:
        print(f"ERROR: expected 2 ttyACM devices, found {len(devs)}: {devs}", file=sys.stderr)
        sys.exit(1)
    if len(devs) > 2:
        print(f"WARN: found {len(devs)} ttyACM devices, probing all: {devs}", file=sys.stderr)

    results = {}
    for d in devs:
        tag, hits = probe(d)
        results[d] = (tag, hits, get_serial(d))
        print(f"# {d:16s} -> {tag:12s}  responders={hits}  serial={results[d][2]}", file=sys.stderr)

    bus1 = [(d, s) for d, (t, _, s) in results.items() if t == "bus1"]
    bus2 = [(d, s) for d, (t, _, s) in results.items() if t == "bus2"]

    if len(bus1) != 1 or len(bus2) != 1:
        print("\nERROR: could not uniquely identify bus1 and bus2.", file=sys.stderr)
        print("Are motors powered? Is the Feetech data bus cable plugged in?", file=sys.stderr)
        sys.exit(1)

    bus1_dev, bus1_serial = bus1[0]
    bus2_dev, bus2_serial = bus2[0]

    if not bus1_serial or not bus2_serial:
        print("ERROR: could not read ID_SERIAL_SHORT for one of the devices.", file=sys.stderr)
        sys.exit(1)

    print(f"BUS1_SERIAL={bus1_serial}")
    print(f"BUS1_DEV={bus1_dev}")
    print(f"BUS2_SERIAL={bus2_serial}")
    print(f"BUS2_DEV={bus2_dev}")


if __name__ == "__main__":
    main()
