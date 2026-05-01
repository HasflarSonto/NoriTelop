#!/usr/bin/env python3
"""
VR teleop + dataset recording for the XLeRobot 2-wheels robot.

Captures right-arm-only 6-DOF demos with the head + right-wrist USB cameras.
Drops each episode straight into a local LeRobotDataset that feeds into
`python -m lerobot.scripts.lerobot_train --policy.type=act ...`.

How to run:
    1. python -m lerobot.scripts.lerobot_find_cameras opencv
       (and edit the indices in xlerobot_2wheels_cameras_config() to match)
    2. start the XLeVR server (Quest 3)
    3. python examples/xlerobot/9_xlerobot_2wheels_teleop_vr_record.py

Knobs you might tweak live in the CONFIG block at the top.
"""

import asyncio
import logging
import queue
import sys
import threading
import time
import traceback

import numpy as np
import pygame

from vr_monitor import VRMonitor
from lerobot.robots.xlerobot_2wheels import XLerobot2WheelsConfig, XLerobot2Wheels
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.robot_utils import precise_sleep
from lerobot.model.SO101Robot import SO101Kinematics
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame
from lerobot.utils.constants import ACTION, OBS_STR

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


# ───────────────────────── CONFIG ─────────────────────────

TASK = "pick up the red block"
DATASET_REPO = "local/xlerobot_right_arm_pickup"
DATASET_ROOT = "outputs/datasets/xlerobot_right_arm_pickup"
NUM_EPISODES = 30
EPISODE_LEN_SEC = 15
RESET_TIME_SEC = 5
FPS = 30

EPISODE_FRAMES = int(EPISODE_LEN_SEC * FPS)

# ──────────────────────────────────────────────────────────


RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

RIGHT_ARM_STATE_KEYS = [f"{v}.pos" for v in RIGHT_JOINT_MAP.values()]

JOINT_LIMITS = {
    "shoulder_pan": (-90, 90),
    "shoulder_lift": (-90, 90),
    "elbow_flex": (-90, 90),
    "wrist_flex": (-90, 90),
    "wrist_roll": (-90, 90),
    "gripper": (0, 45),
}


class SimpleTeleopArm:
    """VR delta-control with IK + P control. Right-arm-only variant."""

    def __init__(self, joint_map, initial_obs, kinematics, prefix="right", kp=1):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.kinematics = kinematics

        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }

        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0

        self.last_vr_time = 0.0
        self.vr_deadzone = 0.001
        self.max_delta_per_frame = 0.005

        self.degree_step = 2
        self.xy_step = 0.005

        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }

    def handle_vr_input(self, vr_goal, gripper_state):
        if vr_goal is None:
            return
        if not hasattr(vr_goal, "target_position") or vr_goal.target_position is None:
            return

        current_vr_pos = vr_goal.target_position
        if not hasattr(self, "prev_vr_pos"):
            self.prev_vr_pos = current_vr_pos
            return

        vr_x = (current_vr_pos[0] - self.prev_vr_pos[0]) * 220
        vr_y = (current_vr_pos[1] - self.prev_vr_pos[1]) * 70
        vr_z = (current_vr_pos[2] - self.prev_vr_pos[2]) * 70

        if abs(vr_x) > 50 or abs(vr_y) > 50 or abs(vr_z) > 50:
            print(f"[{self.prefix}] Large VR jump detected, resetting baseline")
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
            x_scale = 200.0
            delta_pan = max(-angle_limit, min(angle_limit, delta_x * x_scale))
            new_pan = self.target_positions.get("shoulder_pan", 0.0) + delta_pan
            self.target_positions["shoulder_pan"] = max(-180, min(180, new_pan))

        try:
            joint2_target, joint3_target = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            alpha = 0.1
            self.target_positions["shoulder_lift"] = (1 - alpha) * self.target_positions.get("shoulder_lift", 0.0) + alpha * joint2_target
            self.target_positions["elbow_flex"] = (1 - alpha) * self.target_positions.get("elbow_flex", 0.0) + alpha * joint3_target
        except Exception as e:
            print(f"[{self.prefix}] VR IK failed: {e}")

        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"] - self.target_positions["elbow_flex"] + self.pitch
        )

        if vr_goal.metadata.get("trigger", 0) > 0.5:
            self.target_positions["gripper"] = 45
        else:
            self.target_positions["gripper"] = 0.0

        for joint, (lo, hi) in JOINT_LIMITS.items():
            if joint in self.target_positions:
                self.target_positions[joint] = max(lo, min(hi, self.target_positions[joint]))

    def p_control_action(self, obs):
        """Compute one P-control step. Caller passes pre-fetched obs (no bus reads here)."""
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


def get_vr_base_action(vr_goal, robot):
    """Right thumbstick → base velocity. Robot still drives wheels live; we just don't record them."""
    if vr_goal is None or not hasattr(vr_goal, "metadata"):
        return {"x.vel": 0.0, "theta.vel": 0.0}
    thumb = vr_goal.metadata.get("thumbstick", {})
    if not thumb:
        return {"x.vel": 0.0, "theta.vel": 0.0}
    thumb_x = thumb.get("x", 0)
    thumb_y = thumb.get("y", 0)
    speed_setting = robot.speed_levels[robot.speed_index]
    linear_speed = speed_setting["linear"]
    angular_speed = speed_setting["angular"]
    x_cmd = -thumb_y * linear_speed if abs(thumb_y) > 0.15 else 0.0
    theta_cmd = thumb_x * angular_speed if abs(thumb_x) > 0.15 else 0.0
    return {"x.vel": x_cmd, "theta.vel": theta_cmd}


class StallDetector:
    """Reduces Torque_Limit on stalled motors so plastic gears survive long demo sessions."""

    CURRENT_THRESHOLD = 30
    POS_EPSILON = 5
    STALL_LOOPS = 15
    NORMAL_TORQUE = 600
    STALL_TORQUE = 100
    GRIP_TORQUE = 200

    def __init__(self):
        self.prev_pos = {}
        self.stall_count = {}
        self.stalled = {}
        self.retried = {}
        self.stall_target = {}

    def update(self, bus, motor_names, current_targets=None):
        try:
            positions = bus.sync_read("Present_Position", motor_names, normalize=False)
            currents = bus.sync_read("Present_Current", motor_names, normalize=False)
        except Exception:
            return
        for name in motor_names:
            pos = positions.get(name, 0)
            cur = currents.get(name, 0)
            prev = self.prev_pos.get(name, pos)
            delta = abs(pos - prev)
            self.prev_pos[name] = pos
            is_gripper = "gripper" in name
            if self.stalled.get(name) and current_targets:
                new_target = current_targets.get(name)
                old_target = self.stall_target.get(name)
                if new_target is not None and old_target is not None:
                    if abs(new_target - old_target) > 10:
                        self.stalled[name] = False
                        self.retried[name] = False
                        self.stall_count[name] = 0
                        try:
                            bus.write("Torque_Limit", name, self.NORMAL_TORQUE, normalize=False)
                        except Exception:
                            pass
            if self.stalled.get(name):
                continue
            if cur > self.CURRENT_THRESHOLD and delta < self.POS_EPSILON:
                self.stall_count[name] = self.stall_count.get(name, 0) + 1
                if self.stall_count[name] >= self.STALL_LOOPS:
                    if is_gripper:
                        self.stalled[name] = True
                        self.stall_target[name] = pos
                        try:
                            bus.write("Torque_Limit", name, self.GRIP_TORQUE, normalize=False)
                        except Exception:
                            pass
                        print(f"[GRIP] {name} gripping (current={cur}) — holding at reduced torque")
                    elif not self.retried.get(name):
                        self.retried[name] = True
                        self.stall_count[name] = 0
                        print(f"[STALL] {name} stalled (current={cur}) — retrying once")
                    else:
                        self.stalled[name] = True
                        self.stall_target[name] = pos
                        try:
                            bus.write("Torque_Limit", name, self.STALL_TORQUE, normalize=False)
                        except Exception:
                            pass
                        print(f"[STALL] {name} stalled again — torque reduced until target changes")
            else:
                self.stall_count[name] = 0
                self.retried[name] = False


def init_dataset(robot):
    """Build LeRobotDataset features = right-arm 6 DOF + however many cameras are configured."""
    joint_features = {key: float for key in RIGHT_ARM_STATE_KEYS}

    features = {}
    features.update(hw_to_dataset_features(joint_features, ACTION))
    features.update(hw_to_dataset_features(joint_features, OBS_STR))
    features.update(hw_to_dataset_features(robot._cameras_ft, OBS_STR))

    if not robot._cameras_ft:
        print("[REC] WARNING: no cameras configured. Edit xlerobot_2wheels_cameras_config().")

    print(f"[REC] dataset features: {list(features.keys())}")
    print(f"[REC] saving to {DATASET_ROOT} (repo_id={DATASET_REPO})")

    dataset = LeRobotDataset.create(
        repo_id=DATASET_REPO,
        root=DATASET_ROOT,
        features=features,
        fps=FPS,
        image_writer_processes=4,
        image_writer_threads=4,
    )
    return dataset


def saving_dataset_worker(dataset, frame_queue, shutdown_event, saving_in_progress_event, reset_event):
    """Consumes frames from the queue, saves on episode boundaries, then triggers a reset window."""
    try:
        dataset.meta.update_chunk_settings(video_files_size_in_mb=0.001)
        frame_nr = 0
        episode = 0
        print(f"[REC] starting episode 0 — record for {EPISODE_LEN_SEC}s")
        while not shutdown_event.is_set():
            try:
                lerobot_frame = frame_queue.get(timeout=1)
            except queue.Empty:
                continue

            dataset.add_frame(lerobot_frame)
            frame_nr += 1

            if frame_nr >= EPISODE_FRAMES:
                print(f"[REC] saving episode {episode}...")
                saving_in_progress_event.set()
                dataset.save_episode()
                dataset.image_writer.wait_until_done()
                saving_in_progress_event.clear()

                episode += 1
                if episode >= NUM_EPISODES:
                    print(f"[REC] all {NUM_EPISODES} episodes captured. Shutting down.")
                    shutdown_event.set()
                    break

                reset_event.set()
                print(f"[REC] reset {RESET_TIME_SEC}s — place the scene back, then VR resumes recording")
                t0 = time.time()
                while time.time() - t0 < RESET_TIME_SEC and not shutdown_event.is_set():
                    time.sleep(0.1)
                # drain frames captured during save+reset
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


def _connect_bus2_and_cameras(robot):
    """Connect bus2 + cameras only. Skips bus1 entirely so left arm / head don't need to be powered.
    Mirrors the bus2-only init path in 8_xlerobot_2wheels_teleop_vr.py."""
    import json as _json
    from pathlib import Path as _Path
    from lerobot.motors import MotorCalibration as _MC

    robot.bus2.connect()
    robot.bus2.configure_motors(return_delay_time=20)

    calib_path = _Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"
    if calib_path.is_file():
        with open(calib_path) as f:
            calib_data = _json.load(f)
        cal = {}
        for name in robot.bus2.motors:
            if name in calib_data:
                c = calib_data[name]
                cal[name] = _MC(
                    id=c["id"],
                    drive_mode=c["drive_mode"],
                    homing_offset=c["homing_offset"],
                    range_min=c["range_min"],
                    range_max=c["range_max"],
                )
        robot.bus2.calibration = cal
        robot.bus2.write_calibration(cal)
    else:
        print(f"[MAIN] WARNING: no calibration at {calib_path}")

    robot.bus2.disable_torque()
    for name in robot.right_arm_motors:
        robot.bus2.write("Operating_Mode", name, 0)
        robot.bus2.write("P_Coefficient", name, 16)
        robot.bus2.write("I_Coefficient", name, 0)
        robot.bus2.write("D_Coefficient", name, 43)
        robot.bus2.write("Torque_Limit", name, 600)
    for name in robot.base_motors:
        robot.bus2.write("Operating_Mode", name, 1)
    for name in robot.right_arm_motors + robot.base_motors:
        try:
            pos = robot.bus2.sync_read("Present_Position", [name], normalize=False)
            robot.bus2.write("Goal_Position", name, pos[name], normalize=False)
            robot.bus2.write("Torque_Enable", name, 1)
        except Exception:
            print(f"[WARN] Failed to enable {name}")
        time.sleep(0.05)

    for cam_key, cam in robot.cameras.items():
        print(f"[MAIN] connecting camera {cam_key}...")
        cam.connect()
    print(f"[MAIN] cameras connected: {list(robot.cameras)}")


def main():
    print("XLeRobot 2-wheels VR Teleop + Recording")
    print("=" * 50)

    pygame.init()

    shutdown_event = threading.Event()
    saving_in_progress_event = threading.Event()
    reset_event = threading.Event()

    robot = None
    vr_monitor = None
    dataset = None
    dataset_thread = None

    try:
        robot_config = XLerobot2WheelsConfig(id="my_xlerobot_2wheels_lab")
        robot = XLerobot2Wheels(robot_config)

        try:
            _connect_bus2_and_cameras(robot)
        except Exception as e:
            print(f"[MAIN] connect failed: {e}")
            traceback.print_exc()
            return

        if not robot.cameras:
            print("[MAIN] no cameras configured — refusing to record. Edit xlerobot_2wheels_cameras_config().")
            return

        print("[MAIN] initializing VR monitor...")
        vr_monitor = VRMonitor()
        if not vr_monitor.initialize():
            print("[MAIN] VR monitor init failed")
            return
        vr_thread = threading.Thread(
            target=lambda: asyncio.run(vr_monitor.start_monitoring()), daemon=True
        )
        vr_thread.start()
        print("[MAIN] VR ready")

        # Initial obs from bus2 (skip bus1 entirely)
        obs = {}
        rp = robot.bus2.sync_read("Present_Position", robot.right_arm_motors, num_retry=5)
        obs.update({f"{k}.pos": v for k, v in rp.items()})
        wv = robot.bus2.sync_read("Present_Velocity", robot.base_motors, num_retry=5)
        obs.update(robot._wheel_raw_to_body(wv["base_left_wheel"], wv["base_right_wheel"]))

        kin_right = SO101Kinematics()
        right_arm = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, kin_right, prefix="right")

        # Move right arm to its zero pose
        action = right_arm.p_control_action(obs)
        for name, val in {k.replace(".pos", ""): v for k, v in action.items()}.items():
            try:
                robot.bus2.write("Goal_Position", name, val)
            except Exception:
                pass
        print("[OK] right arm at zero")

        # Set up dataset + saving worker
        dataset = init_dataset(robot)
        frame_queue = queue.Queue(maxsize=200)
        dataset_thread = threading.Thread(
            target=saving_dataset_worker,
            args=(dataset, frame_queue, shutdown_event, saving_in_progress_event, reset_event),
            daemon=False,
        )
        dataset_thread.start()

        print("[MAIN] starting VR + record loop. Press ESC to abort.")
        loop_count = 0
        error_count = 0
        stall_detector = StallDetector()
        status_time = time.time()

        while not shutdown_event.is_set():
            try:
                start_loop_t = time.perf_counter()

                dual_goals = vr_monitor.get_latest_goal_nowait()
                right_goal = dual_goals.get("right") if dual_goals else None
                if dual_goals is None:
                    time.sleep(0.01)
                    continue

                right_arm.handle_vr_input(right_goal, gripper_state=None)
                base_action = get_vr_base_action(right_goal, robot)

                # Read present state from bus2
                obs = {}
                right_pos = robot.bus2.sync_read("Present_Position", robot.right_arm_motors)
                obs.update({f"{k}.pos": v for k, v in right_pos.items()})
                wheel_vel = robot.bus2.sync_read("Present_Velocity", robot.base_motors)
                obs.update(robot._wheel_raw_to_body(wheel_vel["base_left_wheel"], wheel_vel["base_right_wheel"]))

                # Read camera frames (non-blocking — returns latest cached frame)
                for cam_key, cam in robot.cameras.items():
                    obs[cam_key] = cam.async_read()

                right_action = right_arm.p_control_action(obs)

                # Drive bus2 (write Goal_Position for arm + Goal_Velocity for wheels)
                for name, val in {k.replace(".pos", ""): v for k, v in right_action.items()}.items():
                    robot.bus2.write("Goal_Position", name, val)
                base_raw = robot._body_to_wheel_raw(base_action.get("x.vel", 0), base_action.get("theta.vel", 0))
                robot.bus2.sync_write("Goal_Velocity", base_raw)

                # Enqueue frame only when actively recording (not saving / not in reset window)
                if not saving_in_progress_event.is_set() and not reset_event.is_set():
                    action_values = {k: right_action[k] for k in RIGHT_ARM_STATE_KEYS}
                    state_values = {k: obs[k] for k in RIGHT_ARM_STATE_KEYS}
                    cam_values = {cam_key: obs[cam_key] for cam_key in robot.cameras}

                    action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
                    obs_frame = build_dataset_frame(
                        dataset.features, {**state_values, **cam_values}, prefix=OBS_STR
                    )
                    lerobot_frame = {**obs_frame, **action_frame, "task": TASK}
                    try:
                        frame_queue.put_nowait(lerobot_frame)
                    except queue.Full:
                        # saving thread fell behind — drop this frame instead of blocking control
                        pass

                loop_count += 1
                dt_s = time.perf_counter() - start_loop_t
                precise_sleep(max(0.0, 1.0 / FPS - dt_s))

            except (ConnectionError, RuntimeError, OSError, DeviceNotConnectedError) as e:
                error_count += 1
                if error_count <= 5 or error_count % 50 == 0:
                    print(f"[WARN] error #{error_count}: {e}")
                time.sleep(0.05)
                continue

            now = time.time()
            if now - status_time >= 10:
                err_pct = (error_count / max(loop_count + error_count, 1)) * 100
                vr_ok = "connected" if right_goal else "waiting"
                qsize = frame_queue.qsize()
                print(
                    f"[STATUS] {loop_count} loops | err {error_count} ({err_pct:.1f}%) | VR {vr_ok} | queue {qsize}"
                )
                loop_count = 0
                error_count = 0
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
        print("[MAIN] shutting down...")
        shutdown_event.set()

        if dataset_thread is not None and dataset_thread.is_alive():
            print("[MAIN] waiting for saving worker to drain...")
            dataset_thread.join(timeout=30)

        if robot is not None:
            try:
                for cam in robot.cameras.values():
                    try:
                        cam.disconnect()
                    except Exception:
                        pass
                robot.bus2.disconnect()
            except Exception:
                pass

        try:
            pygame.quit()
        except Exception:
            pass

        print(f"[MAIN] dataset at: {DATASET_ROOT}")


if __name__ == "__main__":
    sys.exit(main())
