"""Identify which serial device is bus1 vs bus2.

Pings head motors (IDs 7-8) on each candidate port; the port that responds
is bus1. The other port is bus2. Works against local COM/tty ports or the
socket://host:port bridge URLs used in RPi4 bridge mode.

Usage examples:
    python detect_buses.py COM5 COM6
    python detect_buses.py /dev/ttyUSB0 /dev/ttyUSB1
    python detect_buses.py socket://xlerobot.local:3001 socket://xlerobot.local:3002
"""
import sys
import time
import serial

HEADER = bytes([0xFF, 0xFF])
BUS1_HEAD_IDS = (7, 8)      # head motors live on bus1
BUS2_WHEEL_IDS = (9, 10)    # wheels live on bus2


def checksum(packet):
    return (~sum(packet) & 0xFF)


def ping(ser, motor_id):
    ser.reset_input_buffer()
    pkt = [motor_id, 2, 0x01]
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.03)
    return ser.read(64)


def probe(port):
    try:
        ser = serial.serial_for_url(port, baudrate=1000000, timeout=0.1)
    except Exception as e:
        return {"port": port, "error": str(e)}
    time.sleep(0.1)
    found = {}
    for mid in list(BUS1_HEAD_IDS) + list(BUS2_WHEEL_IDS):
        resp = ping(ser, mid)
        found[mid] = bool(resp)
    ser.close()
    return {"port": port, "ids": found}


def classify(result):
    ids = result.get("ids", {})
    head = any(ids.get(m) for m in BUS1_HEAD_IDS)
    wheel = any(ids.get(m) for m in BUS2_WHEEL_IDS)
    if head and not wheel:
        return "bus1"
    if wheel and not head:
        return "bus2"
    if head and wheel:
        return "ambiguous (both head and wheel motors respond)"
    return "unknown (no expected motors responded)"


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    for port in sys.argv[1:]:
        result = probe(port)
        if "error" in result:
            print(f"{port:40s}  FAILED TO OPEN: {result['error']}")
            continue
        label = classify(result)
        responded = [str(m) for m, ok in result["ids"].items() if ok]
        print(f"{port:40s}  -> {label}  (responded: {','.join(responded) or 'none'})")


if __name__ == "__main__":
    main()
