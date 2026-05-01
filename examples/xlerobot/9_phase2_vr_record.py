#!/usr/bin/env python3
"""Phase 2 VR teleop + dataset recording for the XLeRobot 2-wheels.

Pulls together three connections to the Pi:
  - TCP control to teleop_server.py (port 7777) — sends VR-derived joint
    targets at FPS Hz, receives state telemetry every Pi loop tick.
  - ZMQ SUB to image_server.py (port 5555) — head camera frames.
  - ZMQ SUB to image_server.py (port 5556) — right-wrist camera frames.

VR teleop runs laptop-side (the Quest 3 is on the laptop's WiFi anyway). IK
runs on the laptop and the resulting joint goals are pushed to the Pi as
arm_targets; the Pi keeps doing tight P-control to motors at 50 Hz.

Action space is locked to the right arm (6 DOF). Head, wheels, left arm are
not commanded and not stored in the dataset. Match this with bus_choice="2"
on the Pi side.

Dataset is local-only (no push_to_hub).

How to run:
  1. On the Pi:  bash ~/NoriTelop/rpi4/start_camera_server.sh   (terminal 1)
                 bash ~/NoriTelop/rpi4/start_teleop.sh          (terminal 2)
  2. On the laptop:
       $env:XLEROBOT_BRIDGE = "172.20.10.7"
       python examples/xlerobot/9_phase2_vr_record.py
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import queue
import socket
import sys
import threading
import time
import traceback

import numpy as np
import pygame

from vr_monitor import VRMonitor
from lerobot.cameras.zmq import ZMQCamera, ZMQCameraConfig
from lerobot.model.SO101Robot import SO101Kinematics
from lerobot.utils.robot_utils import precise_sleep
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame
from lerobot.utils.constants import ACTION, OBS_STR

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


# ───────────────────────── CONFIG ─────────────────────────

PI_HOST = os.environ.get("XLEROBOT_BRIDGE", "172.20.10.7")
TCP_PORT = 7777
HEAD_CAM_PORT = 5555
WRIST_CAM_PORT = 5556

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30

TASK = "pick up the red block"
DATASET_REPO = "local/xlerobot_phase2_pickup"
DATASET_ROOT = "outputs/datasets/xlerobot_phase2_pickup"
NUM_EPISODES = 30
EPISODE_LEN_SEC = 15
RESET_TIME_SEC = 5
FPS = 30                       # recording rate (matches camera fps)
EPISODE_FRAMES = int(EPISODE_LEN_SEC * FPS)

# ──────────────────────────────────────────────────────────


RIGHT_JOINT_MAP = {
    "shoulder_pan":  "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex":    "right_arm_elbow_flex",
    "wrist_flex":    "right_arm_wrist_flex",
    "wrist_roll":    "right_arm_wrist_roll",
    "gripper":       "right_arm_gripper",
}
RIGHT_ARM_FULL_NAMES = list(RIGHT_JOINT_MAP.values())
RIGHT_ARM_STATE_KEYS = [f"{n}.pos" for n in RIGHT_ARM_FULL_NAMES]

JOINT_LIMITS = {
    "shoulder_pan":  (-90, 90),
    "shoulder_lift": (-90, 90),
    "elbow_flex":    (-90, 90),
    "wrist_flex":    (-90, 90),
    "wrist_roll":    (-90, 90),
    "gripper":       (0, 45),
}


# ───────── Virtual tactile sensor (grip force) ─────────
# STS3215 Present_Current is signed raw int (~6.5 mA per unit). Current is
# proportional to motor torque, so on the TPU gripper we get a continuous
# force-proxy signal as the jaws compress on a contact.
#
# Calibration recipe — run teleop_client.py to see live `[GRIP]` readings:
#   1. close gripper through air, no contact — note the highest values
#      you see during free motion. That's the friction floor (no contact).
#      Set GRIP_IDLE_CURRENT slightly above this so jogging registers as 0.
#   2. squeeze a firm object until you see "[SRV] stalled" from the server.
#      The raw value just before the stall is GRIP_MAX_CURRENT — that's
#      the natural saturation point set by StallDetector.
#
# Tuned from real measurements on Antonio's left_arm_gripper (2026-05):
#   - jogging in air: peaks at 7-18 raw
#   - pre-stall squeeze peak: ~90 raw
GRIP_IDLE_CURRENT = 20
GRIP_MAX_CURRENT  = 90


def compute_grip_force(raw_current: int) -> float:
    """Map signed Present_Current → normalized grip force in [0, 1].

    abs() because current direction = motion direction, not load sign — we
    care about magnitude. Linear-with-clamp is good enough for v1; switch
    to f**2 if soft-object resolution is too coarse (see plan).
    """
    mag = abs(raw_current)
    f = (mag - GRIP_IDLE_CURRENT) / max(1, GRIP_MAX_CURRENT - GRIP_IDLE_CURRENT)
    return max(0.0, min(1.0, f))


# ────────────────────── VR teleop ───────────────────────
# Ported from 8_xlerobot_2wheels_teleop_vr.py:77-294.
# Same algorithm as the direct-USB script 9 — laptop owns IK + delta control.

class SimpleTeleopArm:
    """VR delta-control + IK + soft limits. Produces target_positions in
    degrees keyed by short joint name. The recording loop reads this dict
    and ships values to the Pi as arm_targets."""

    def __init__(self, joint_map, initial_obs, kinematics, prefix="right", kp=1):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.kinematics = kinematics

        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        self.target_positions = {
            "shoulder_pan":  initial_obs.get(f"{prefix}_arm_shoulder_pan.pos", 0.0),
            "shoulder_lift": initial_obs.get(f"{prefix}_arm_shoulder_lift.pos", 0.0),
            "elbow_flex":    initial_obs.get(f"{prefix}_arm_elbow_flex.pos", 0.0),
            "wrist_flex":    initial_obs.get(f"{prefix}_arm_wrist_flex.pos", 0.0),
            "wrist_roll":    initial_obs.get(f"{prefix}_arm_wrist_roll.pos", 0.0),
            "gripper":       initial_obs.get(f"{prefix}_arm_gripper.pos", 0.0),
        }
        self.zero_pos = dict.fromkeys(self.target_positions, 0.0)

    def handle_vr_input(self, vr_goal, gripper_state=None):
        if vr_goal is None or not hasattr(vr_goal, "target_position") or vr_goal.target_position is None:
            return

        current_vr_pos = vr_goal.target_position
        if not hasattr(self, "prev_vr_pos"):
            self.prev_vr_pos = current_vr_pos
            return

        vr_x = (current_vr_pos[0] - self.prev_vr_pos[0]) * 220
        vr_y = (current_vr_pos[1] - self.prev_vr_pos[1]) * 70
        vr_z = (current_vr_pos[2] - self.prev_vr_pos[2]) * 70
        if abs(vr_x) > 50 or abs(vr_y) > 50 or abs(vr_z) > 50:
            self.prev_vr_pos = current_vr_pos
            return
        self.prev_vr_pos = current_vr_pos

        pos_scale = 0.01
        angle_scale = 4.0
        delta_limit = 0.01
        angle_limit = 8.0

        delta_x = max(-delta_limit, min(delta_limit, vr_x * pos_scale))
        delta_y = max(-delta_limit, min(delta_limit, vr_y * pos_scale))
        delta_z = max(-delta_limit, min(delta_limit, vr_z * pos_scale))

        self.current_x += -delta_z
        self.current_y += delta_y

        if hasattr(vr_goal, "wrist_flex_deg") and vr_goal.wrist_flex_deg is not None:
            if not hasattr(self, "prev_wrist_flex"):
                self.prev_wrist_flex = vr_goal.wrist_flex_deg
                return
            delta_pitch = (vr_goal.wrist_flex_deg - self.prev_wrist_flex) * angle_scale
            if abs(delta_pitch) > 30:
                self.prev_wrist_flex = vr_goal.wrist_flex_deg
                return
            delta_pitch = max(-angle_limit, min(angle_limit, delta_pitch))
            self.pitch = max(-90, min(90, self.pitch + delta_pitch))
            self.prev_wrist_flex = vr_goal.wrist_flex_deg

        if hasattr(vr_goal, "wrist_roll_deg") and vr_goal.wrist_roll_deg is not None:
            if not hasattr(self, "prev_wrist_roll"):
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            delta_roll = (vr_goal.wrist_roll_deg - self.prev_wrist_roll) * angle_scale
            if abs(delta_roll) > 30:
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            delta_roll = max(-angle_limit, min(angle_limit, delta_roll))
            new_roll = self.target_positions.get("wrist_roll", 0.0) + delta_roll
            self.target_positions["wrist_roll"] = max(-90, min(90, new_roll))
            self.prev_wrist_roll = vr_goal.wrist_roll_deg

        if abs(delta_x) > 0.001:
            delta_pan = max(-angle_limit, min(angle_limit, delta_x * 200.0))
            new_pan = self.target_positions.get("shoulder_pan", 0.0) + delta_pan
            self.target_positions["shoulder_pan"] = max(-180, min(180, new_pan))

        try:
            j2, j3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            alpha = 0.1
            self.target_positions["shoulder_lift"] = (1 - alpha) * self.target_positions.get("shoulder_lift", 0.0) + alpha * j2
            self.target_positions["elbow_flex"]    = (1 - alpha) * self.target_positions.get("elbow_flex", 0.0)    + alpha * j3
        except Exception as e:
            print(f"[{self.prefix}] VR IK failed: {e}")

        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            - self.target_positions["elbow_flex"]
            + self.pitch
        )

        if vr_goal.metadata.get("trigger", 0) > 0.5:
            self.target_positions["gripper"] = 45
        else:
            self.target_positions["gripper"] = 0.0

        for joint, (lo, hi) in JOINT_LIMITS.items():
            if joint in self.target_positions:
                self.target_positions[joint] = max(lo, min(hi, self.target_positions[joint]))


# ───────────────────── TCP control client ─────────────────────

class TcpControlClient:
    """Manages the persistent TCP connection to teleop_server.py.

    - connect(): handshake, returns initial_obs.
    - send_targets(): non-blocking send of arm_targets + wheel_targets.
    - get_latest_state(): atomic snapshot of last state telemetry from Pi.
    - background recv thread drains telemetry into the cache.
    """

    def __init__(self, host: str, port: int, bus_choice: str = "2"):
        self.host = host
        self.port = port
        self.bus_choice = bus_choice
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._latest_state: dict[str, float] = {}
        self._latest_currents: dict[str, int] = {}
        self._latest_ts_ns: int = 0
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        self._recv_thread: threading.Thread | None = None
        self._recv_buf = b""

    def connect(self) -> dict[str, float]:
        print(f"[TCP] connecting to {self.host}:{self.port} bus={self.bus_choice}")
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))

        # Handshake
        self.sock.sendall((json.dumps({"hello": {"bus_choice": self.bus_choice}}) + "\n").encode())
        self.sock.settimeout(15.0)
        buf = b""
        while b"\n" not in buf:
            chunk = self.sock.recv(8192)
            if not chunk:
                raise RuntimeError("server closed before ack")
            buf += chunk
        line, self._recv_buf = buf.split(b"\n", 1)
        ack = json.loads(line.decode("utf-8"))
        if not ack.get("ack"):
            raise RuntimeError(f"server refused: {ack.get('error', '?')}")
        initial_obs = ack.get("initial_obs", {})
        print(f"[TCP] ack received, {len(initial_obs)} initial positions")

        # Pre-seed state cache so first recording iter has valid data.
        with self._state_lock:
            self._latest_state = dict(initial_obs)
            self._latest_ts_ns = time.monotonic_ns()

        # Switch socket to blocking with a short timeout for the recv thread
        self.sock.settimeout(1.0)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True, name="tcp-recv")
        self._recv_thread.start()
        return initial_obs

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self.sock.recv(16384)
                if not chunk:
                    print("[TCP] server closed connection")
                    return
                self._recv_buf += chunk
                while b"\n" in self._recv_buf:
                    line, self._recv_buf = self._recv_buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError:
                        continue
                    if "state" in msg:
                        with self._state_lock:
                            self._latest_state = msg["state"]
                            self._latest_ts_ns = msg.get("ts_ns", time.monotonic_ns())
                    if "currents" in msg:
                        with self._state_lock:
                            self._latest_currents = msg["currents"]
                    if msg.get("stalled"):
                        print(f"[SRV] stalled: {msg['stalled']} (loop_hz={msg.get('loop_hz')})")
            except socket.timeout:
                continue
            except OSError:
                return

    def send_targets(self, arm_targets: dict[str, float], wheel_targets: dict[str, float]) -> bool:
        try:
            frame = {"arm_targets": arm_targets, "wheel_targets": wheel_targets}
            self.sock.sendall((json.dumps(frame) + "\n").encode())
            return True
        except OSError as e:
            print(f"[TCP] send failed: {e}")
            return False

    def get_latest_state(self) -> tuple[int, dict[str, float]]:
        with self._state_lock:
            return self._latest_ts_ns, dict(self._latest_state)

    def get_latest_currents(self) -> dict[str, int]:
        with self._state_lock:
            return dict(self._latest_currents)

    def disconnect(self) -> None:
        self._stop.set()
        try:
            self.sock.sendall((json.dumps({"bye": True}) + "\n").encode())
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


# ───────────────────── Dataset machinery ─────────────────────

def init_dataset(cameras_ft: dict[str, tuple[int, int, int]]) -> LeRobotDataset:
    """Build LeRobotDataset features = right-arm 6 DOF (action+state) + grip
    force (state only) + cameras (state only)."""
    action_features = {key: float for key in RIGHT_ARM_STATE_KEYS}
    # State carries the same 6 joints as action, plus the virtual tactile
    # signals: raw current (forensic / model-internal) and normalized force.
    state_joint_features = {
        **action_features,
        "right_arm_gripper_current": float,
        "right_arm_grip_force":      float,
    }
    features: dict[str, dict] = {}
    features.update(hw_to_dataset_features(action_features, ACTION))
    features.update(hw_to_dataset_features(state_joint_features, OBS_STR))
    features.update(hw_to_dataset_features(cameras_ft, OBS_STR))

    print(f"[REC] features: {list(features.keys())}")
    print(f"[REC] saving to {DATASET_ROOT} (repo_id={DATASET_REPO})")

    return LeRobotDataset.create(
        repo_id=DATASET_REPO,
        root=DATASET_ROOT,
        features=features,
        fps=FPS,
        image_writer_processes=4,
        image_writer_threads=4,
    )


def saving_dataset_worker(
    dataset: LeRobotDataset,
    frame_queue: queue.Queue,
    shutdown_event: threading.Event,
    saving_in_progress_event: threading.Event,
    reset_event: threading.Event,
) -> None:
    try:
        dataset.meta.update_chunk_settings(video_files_size_in_mb=0.001)
        frame_nr = 0
        episode = 0
        print(f"[REC] starting episode 0 — record for {EPISODE_LEN_SEC}s")
        while not shutdown_event.is_set():
            try:
                frame = frame_queue.get(timeout=1)
            except queue.Empty:
                continue

            dataset.add_frame(frame)
            frame_nr += 1

            if frame_nr >= EPISODE_FRAMES:
                print(f"[REC] saving episode {episode}…")
                saving_in_progress_event.set()
                dataset.save_episode()
                dataset.image_writer.wait_until_done()
                saving_in_progress_event.clear()

                episode += 1
                if episode >= NUM_EPISODES:
                    print(f"[REC] all {NUM_EPISODES} episodes captured.")
                    shutdown_event.set()
                    break

                reset_event.set()
                print(f"[REC] reset {RESET_TIME_SEC}s — place the scene back, then VR resumes recording")
                t0 = time.time()
                while time.time() - t0 < RESET_TIME_SEC and not shutdown_event.is_set():
                    time.sleep(0.1)
                while not frame_queue.empty():
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        break
                reset_event.clear()
                frame_nr = 0
                print(f"[REC] starting episode {episode} — record for {EPISODE_LEN_SEC}s")
    except Exception as e:
        logger.error(f"saving worker error: {e}", exc_info=True)
    finally:
        try:
            if dataset is not None:
                dataset.image_writer.wait_until_done()
        except Exception:
            pass


# ───────────────────── Main ─────────────────────

def main() -> int:
    print("XLeRobot Phase 2 VR Teleop + Recording")
    print("=" * 50)

    pygame.init()

    shutdown_event = threading.Event()
    saving_in_progress_event = threading.Event()
    reset_event = threading.Event()

    tcp: TcpControlClient | None = None
    head_cam: ZMQCamera | None = None
    wrist_cam: ZMQCamera | None = None
    vr_monitor: VRMonitor | None = None
    dataset: LeRobotDataset | None = None
    saving_thread: threading.Thread | None = None

    try:
        # 1. TCP control to teleop_server.py
        tcp = TcpControlClient(PI_HOST, TCP_PORT, bus_choice="2")
        try:
            initial_obs = tcp.connect()
        except Exception as e:
            print(f"[MAIN] TCP connect failed: {e}")
            return 1

        # 2. ZMQ cameras
        print(f"[MAIN] connecting cameras on {PI_HOST}…")
        head_cam = ZMQCamera(ZMQCameraConfig(
            server_address=PI_HOST, port=HEAD_CAM_PORT, camera_name="head",
            fps=CAM_FPS, width=CAM_WIDTH, height=CAM_HEIGHT,
        ))
        wrist_cam = ZMQCamera(ZMQCameraConfig(
            server_address=PI_HOST, port=WRIST_CAM_PORT, camera_name="right_wrist",
            fps=CAM_FPS, width=CAM_WIDTH, height=CAM_HEIGHT,
        ))
        try:
            head_cam.connect()
            wrist_cam.connect()
        except Exception as e:
            print(f"[MAIN] camera connect failed: {e}")
            return 1
        cameras = {"head": head_cam, "right_wrist": wrist_cam}
        print(f"[MAIN] cameras connected: {list(cameras)}")

        # 3. VR monitor
        print("[MAIN] initializing VR monitor…")
        vr_monitor = VRMonitor()
        if not vr_monitor.initialize():
            print("[MAIN] VR monitor init failed")
            return 1
        threading.Thread(
            target=lambda: asyncio.run(vr_monitor.start_monitoring()),
            daemon=True,
        ).start()
        print("[MAIN] VR ready")

        # 4. Init right-arm controller from Pi-supplied initial state
        obs0 = {f"{k}.pos": v for k, v in initial_obs.items()}
        for k in RIGHT_ARM_STATE_KEYS:
            obs0.setdefault(k, 0.0)
        right_arm = SimpleTeleopArm(RIGHT_JOINT_MAP, obs0, SO101Kinematics(), prefix="right")

        # 5. Dataset + saving worker
        cameras_ft = {name: (CAM_HEIGHT, CAM_WIDTH, 3) for name in cameras}
        dataset = init_dataset(cameras_ft)
        frame_queue: queue.Queue = queue.Queue(maxsize=200)
        saving_thread = threading.Thread(
            target=saving_dataset_worker,
            args=(dataset, frame_queue, shutdown_event, saving_in_progress_event, reset_event),
            daemon=False,
        )
        saving_thread.start()

        print("[MAIN] starting VR + record loop. Press ESC to abort.")
        loop_count = 0
        cam_timeouts = 0
        status_time = time.time()
        first_frame_warn = True

        # Wheel targets are static for arm-only IL — wheels stay at zero velocity.
        STATIC_WHEEL_TARGETS = {"x.vel": 0.0, "theta.vel": 0.0}

        while not shutdown_event.is_set():
            t0 = time.perf_counter()

            try:
                # VR -> target_positions
                dual_goals = vr_monitor.get_latest_goal_nowait()
                right_goal = dual_goals.get("right") if dual_goals else None
                if dual_goals is None:
                    time.sleep(0.01)
                    continue

                right_arm.handle_vr_input(right_goal)

                # Build arm_targets keyed by full motor names (no .pos), for the Pi.
                arm_targets = {full: right_arm.target_positions[short]
                               for short, full in RIGHT_JOINT_MAP.items()}
                if not tcp.send_targets(arm_targets, STATIC_WHEEL_TARGETS):
                    shutdown_event.set()
                    break

                # Pull latest state from TCP cache (updated by recv thread).
                _, state_raw = tcp.get_latest_state()
                state = {f"{k}.pos": v for k, v in state_raw.items()}

                # Pull latest images. read_latest is non-blocking; tolerates
                # transient WiFi hiccups by reusing the last cached frame.
                try:
                    head_img = head_cam.read_latest(max_age_ms=500)
                    wrist_img = wrist_cam.read_latest(max_age_ms=500)
                except (TimeoutError, RuntimeError) as e:
                    cam_timeouts += 1
                    if first_frame_warn:
                        print(f"[CAM] waiting for first frame… ({e})")
                        first_frame_warn = False
                    elif cam_timeouts <= 3 or cam_timeouts % 30 == 0:
                        print(f"[CAM] frame stale (#{cam_timeouts}): {e}")
                    time.sleep(0.05)
                    continue
                first_frame_warn = False

                # Enqueue frame only when actively recording.
                if not saving_in_progress_event.is_set() and not reset_event.is_set():
                    if all(k in state for k in RIGHT_ARM_STATE_KEYS):
                        action_values = {k: arm_targets[k.removesuffix(".pos")] for k in RIGHT_ARM_STATE_KEYS}
                        state_values = {k: state[k] for k in RIGHT_ARM_STATE_KEYS}
                        # Virtual tactile observations from Pi-shipped currents.
                        currents = tcp.get_latest_currents()
                        gripper_raw = currents.get("right_arm_gripper", 0)
                        state_values["right_arm_gripper_current"] = float(gripper_raw)
                        state_values["right_arm_grip_force"] = compute_grip_force(gripper_raw)
                        cam_values = {"head": head_img, "right_wrist": wrist_img}

                        action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
                        obs_frame = build_dataset_frame(
                            dataset.features, {**state_values, **cam_values}, prefix=OBS_STR
                        )
                        lerobot_frame = {**obs_frame, **action_frame, "task": TASK}
                        try:
                            frame_queue.put_nowait(lerobot_frame)
                        except queue.Full:
                            pass

                loop_count += 1
                dt_s = time.perf_counter() - t0
                precise_sleep(max(0.0, 1.0 / FPS - dt_s))

            except Exception as e:
                print(f"[LOOP] error: {e}")
                traceback.print_exc()
                time.sleep(0.05)

            now = time.time()
            if now - status_time >= 10:
                qsize = frame_queue.qsize()
                print(f"[STATUS] {loop_count} loops in 10s | queue {qsize} | cam_timeouts {cam_timeouts}")
                loop_count = 0
                cam_timeouts = 0
                status_time = now

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    shutdown_event.set()
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    shutdown_event.set()

    except Exception as e:
        print(f"[MAIN] unexpected error: {e}")
        traceback.print_exc()

    finally:
        print("[MAIN] shutting down…")
        shutdown_event.set()

        if saving_thread is not None and saving_thread.is_alive():
            print("[MAIN] waiting for saving worker to drain…")
            saving_thread.join(timeout=30)

        for cam in (head_cam, wrist_cam):
            if cam is not None:
                try:
                    cam.disconnect()
                except Exception:
                    pass

        if tcp is not None:
            tcp.disconnect()

        try:
            pygame.quit()
        except Exception:
            pass

        print(f"[MAIN] dataset at: {DATASET_ROOT}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
