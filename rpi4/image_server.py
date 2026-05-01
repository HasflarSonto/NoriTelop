#!/usr/bin/env python3
"""Phase 2: Pi-side multi-camera ZMQ image server.

Streams USB cameras attached to the RPi4 over ZMQ PUB sockets, one socket per
camera (one port per camera). The wire format matches lerobot's ZMQCamera
parser at lerobot/src/lerobot/cameras/zmq/camera_zmq.py:174-212, so the laptop
side can use ZMQCamera off the shelf:

    {"timestamps": {"<name>": float_seconds},
     "images":     {"<name>": "<base64-jpeg>"}}

Standalone — no lerobot import on the Pi to keep RAM footprint small. Uses
OpenCV + pyzmq directly. MJPEG fourcc on the V4L2 capture so the camera does
JPEG compression in hardware (saves USB bandwidth and Pi CPU).

Run alongside teleop_server.py — they share zero state and use different
ports. Either can crash without affecting the other.
"""
from __future__ import annotations

import argparse
import base64
import json
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass

import cv2
import zmq


# ───────────── Camera config ─────────────
# Edit indices to match the physical USB ports.
# `v4l2-ctl --list-devices` on the Pi shows the mapping.

@dataclass
class CamSpec:
    name: str
    index: int
    port: int
    fps: int = 30
    width: int = 640
    height: int = 480


CAMERAS = [
    CamSpec(name="head",        index=0, port=5555),
    CamSpec(name="right_wrist", index=2, port=5556),
]

JPEG_QUALITY = 80   # 80 is the widely-used sweet spot for IL: ~50 KB/frame, no visible artefacts.

# ──────────────────────────────────────────


log = logging.getLogger("image_server")


def _open_capture(cam: CamSpec) -> cv2.VideoCapture | None:
    """Open V4L2 capture in MJPEG mode with a single-frame buffer."""
    cap = cv2.VideoCapture(cam.index, cv2.CAP_V4L2)
    if not cap.isOpened():
        log.warning(f"[{cam.name}] V4L2 backend failed for index {cam.index}; trying default backend")
        cap = cv2.VideoCapture(cam.index)
    if not cap.isOpened():
        log.error(f"[{cam.name}] cannot open /dev/video{cam.index}")
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.height)
    cap.set(cv2.CAP_PROP_FPS, cam.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join(chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4))
    log.info(f"[{cam.name}] open {actual_w}x{actual_h}@{actual_fps:.0f}fps fourcc={fourcc_str}")
    return cap


def capture_and_publish(cam: CamSpec, ctx: zmq.Context, stop_event: threading.Event) -> None:
    """One thread per camera: capture → JPEG re-encode → PUB."""
    cap = _open_capture(cam)
    if cap is None:
        log.error(f"[{cam.name}] thread exiting (camera unavailable)")
        return

    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(f"tcp://*:{cam.port}")
    log.info(f"[{cam.name}] PUB tcp://*:{cam.port}")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    frame_times: list[float] = []
    fail_streak = 0
    last_status = time.monotonic()

    try:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                fail_streak += 1
                if fail_streak == 1 or fail_streak % 30 == 0:
                    log.warning(f"[{cam.name}] read failed (streak={fail_streak})")
                if fail_streak >= 150:
                    log.error(f"[{cam.name}] 150 consecutive read failures, exiting thread")
                    return
                time.sleep(0.05)
                continue
            fail_streak = 0

            capture_ts = time.monotonic()

            ok, jpg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue

            b64 = base64.b64encode(jpg.tobytes()).decode("ascii")
            msg = json.dumps({
                "timestamps": {cam.name: capture_ts},
                "images":     {cam.name: b64},
            })

            try:
                sock.send_string(msg, zmq.NOBLOCK)
            except zmq.Again:
                pass  # subscriber slow; drop frame (CONFLATE makes this safe)

            frame_times.append(capture_ts)
            now = time.monotonic()
            if now - last_status >= 10.0:
                cutoff = now - 5.0
                recent = [t for t in frame_times if t >= cutoff]
                frame_times[:] = recent
                fps = len(recent) / 5.0
                log.info(f"[{cam.name}] {fps:.1f} fps over last 5s; jpg≈{len(jpg)//1024} KB")
                last_status = now
    finally:
        try:
            cap.release()
        except Exception:
            pass
        try:
            sock.close(linger=0)
        except Exception:
            pass
        log.info(f"[{cam.name}] stopped")


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    ap = argparse.ArgumentParser(description="Pi-side ZMQ multi-camera publisher")
    ap.add_argument(
        "--cam",
        action="append",
        metavar="NAME:INDEX:PORT",
        help="override CAMERAS list: pass once per camera, e.g. --cam head:0:5555 --cam wrist:2:5556",
    )
    args = ap.parse_args()

    cams: list[CamSpec] = list(CAMERAS)
    if args.cam:
        cams = []
        for spec in args.cam:
            try:
                name, idx, port = spec.split(":")
                cams.append(CamSpec(name=name, index=int(idx), port=int(port)))
            except ValueError:
                log.error(f"bad --cam value {spec!r} (want NAME:INDEX:PORT)")
                return 2

    if not cams:
        log.error("no cameras configured")
        return 2

    stop_event = threading.Event()

    def _shutdown(signum, _frame):
        log.info(f"[main] caught signal {signum}, stopping…")
        stop_event.set()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    ctx = zmq.Context.instance()
    threads: list[threading.Thread] = []
    for cam in cams:
        t = threading.Thread(
            target=capture_and_publish,
            args=(cam, ctx, stop_event),
            name=f"cap-{cam.name}",
            daemon=False,
        )
        t.start()
        threads.append(t)

    log.info(f"[main] {len(threads)} camera thread(s) started; Ctrl-C to stop")

    while not stop_event.is_set():
        time.sleep(0.5)
        for t, cam in zip(threads, cams):
            if not t.is_alive():
                log.error(f"[main] camera thread {cam.name} died — shutting down")
                stop_event.set()
                break

    log.info("[main] joining camera threads…")
    for t in threads:
        t.join(timeout=3.0)

    try:
        ctx.term()
    except Exception:
        pass
    log.info("[main] shutdown complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
