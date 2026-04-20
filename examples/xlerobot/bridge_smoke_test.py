"""Bridge smoke test — Phase 2 of the RPi4 migration.

Checks that the laptop can open socket:// URLs pointing at ser2net on the Pi
and that Feetech motors respond. No lerobot/scservo_sdk imports; just pyserial.

Usage:
    set XLEROBOT_BRIDGE=xlerobot.local
    python bridge_smoke_test.py

Or explicit:
    python bridge_smoke_test.py xlerobot.local
"""
import os
import sys
import time
import serial

HEADER = bytes([0xFF, 0xFF])

# Expected IDs on each bus (subset — enough to prove the link)
BUS1_PROBE = [1, 2, 3, 7, 8]             # left arm IDs 1-3 + head 7-8
BUS2_PROBE = [1, 2, 3, 9, 10, 11]        # right arm IDs 1-3 + wheels 9-10 + z-lift 11


def checksum(pkt):
    return (~sum(pkt) & 0xFF)


def ping(ser, motor_id):
    ser.reset_input_buffer()
    pkt = [motor_id, 2, 0x01]
    pkt.append(checksum(pkt))
    ser.write(HEADER + bytes(pkt))
    time.sleep(0.03)
    return ser.read(64)


def probe(url, ids, label):
    print(f"\n=== {label} :: {url} ===")
    try:
        t0 = time.perf_counter()
        ser = serial.serial_for_url(url, baudrate=1000000, timeout=0.15)
        open_ms = (time.perf_counter() - t0) * 1000
        print(f"  open: OK ({open_ms:.0f} ms)")
    except Exception as e:
        print(f"  open: FAILED — {e}")
        return False
    time.sleep(0.1)
    responding = []
    missing = []
    for mid in ids:
        resp = ping(ser, mid)
        (responding if resp else missing).append(mid)
    ser.close()
    print(f"  responding IDs: {responding}")
    if missing:
        print(f"  missing IDs:    {missing}")
    return len(responding) > 0 and not missing


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("XLEROBOT_BRIDGE")
    if not host:
        print("Set XLEROBOT_BRIDGE or pass host as argv[1].")
        print("Example: python bridge_smoke_test.py xlerobot.local")
        sys.exit(1)

    bus1_url = f"socket://{host}:3001"
    bus2_url = f"socket://{host}:3002"

    ok1 = probe(bus1_url, BUS1_PROBE, "BUS1 (left arm + head)")
    ok2 = probe(bus2_url, BUS2_PROBE, "BUS2 (right arm + wheels + z-lift)")

    print("\n=== Result ===")
    print(f"  bus1: {'PASS' if ok1 else 'FAIL'}")
    print(f"  bus2: {'PASS' if ok2 else 'FAIL'}")
    sys.exit(0 if (ok1 and ok2) else 1)


if __name__ == "__main__":
    main()
