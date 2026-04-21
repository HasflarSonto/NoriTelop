"""Phase 2: Pi-side teleop server.

Runs on the RPi4. Owns /dev/xlerobot_bus1 and /dev/xlerobot_bus2 directly via
scservo_sdk (no ser2net hop). Accepts one JSON-over-TCP client on port 7777
which sends pressed-keys frames ~50 Hz; this process runs the real 50 Hz
control loop (P-control on arms/head, smooth accel/decel on wheels, stall
detection).

Ser2net must be stopped before running this (use start_teleop.sh).

Protocol:
    Client hello:   {"hello": {"bus_choice": "1"|"2"|"3"}}
    Server ack:     {"ack": true, "initial_obs": {motor_name: pos, ...}}
    Client frame:   {"keys": ["w","a",...]}
    Client bye:     {"bye": true}

Dead-man: if no client frame for 500 ms, wheels zero and arm targets freeze.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np
import scservo_sdk as scs

# ---------------------------------------------------------------------------
# Register addresses (STS3215) — extracted from
# src/lerobot/motors/feetech/tables.py to avoid a lerobot dependency on the Pi.
# ---------------------------------------------------------------------------
REG = {
    "Return_Delay_Time": (7, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Velocity": (46, 2),      # sign-magnitude, bit 15
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),   # sign-magnitude, bit 15
    "Present_Current": (69, 2),    # sign-magnitude, bit 15
    "Maximum_Acceleration": (85, 1),
}

SIGN_BIT_15_REGS = {"Goal_Velocity", "Present_Velocity", "Present_Current"}

PORT_BUS1 = "/dev/xlerobot_bus1"
PORT_BUS2 = "/dev/xlerobot_bus2"
BAUDRATE = 1_000_000

TCP_PORT = 7777
FPS = 50
DEAD_MAN_SEC = 0.5
TELEMETRY_INTERVAL = 10

# Base speed — max at all times. Huge accel/decel makes the ramp instant
# (one frame), and all speed levels are maxed so u/o become no-ops.
BASE_ACCELERATION_RATE = 1000.0
BASE_DECELERATION_RATE = 1000.0
BASE_MAX_SPEED = 6.0
MIN_VELOCITY_THRESHOLD = 0.02
SPEED_LEVELS = [
    {"linear": 0.3, "angular": 90},
    {"linear": 0.3, "angular": 90},
    {"linear": 0.3, "angular": 90},
]
WHEEL_MAX_RAW = 4000  # was 3000; STS3215 physical max is ~4096 (60 RPM)
# From XLerobot2WheelsConfig defaults (same values as laptop).
TELEOP_KEYS = {
    "forward": "i", "backward": "k", "rotate_left": "j", "rotate_right": "l",
    "speed_up": "u", "speed_down": "o", "quit": "ESC",
}
WHEEL_RADIUS = 0.05
WHEELBASE = 0.125

# Z-lift (motor 11 on bus2): continuous-rotation lead screw, velocity-controlled
# like the wheels. No calibration entry required. Hold a key to drive.
Z_LIFT_UP_KEY = "y"
Z_LIFT_DOWN_KEY = "h"
Z_LIFT_VELOCITY = 4000  # raw ticks/sec; ~60 RPM motor (physical max)

# VR mode (ported from 8_xlerobot_2wheels_teleop_vr.py).
# Soft joint limits applied in user space after handle_vr_input; the final
# motor-safety clamp is still _clamp_goal_position at register-write time.
JOINT_LIMITS = {
    "shoulder_pan":  (-90, 90),
    "shoulder_lift": (-90, 90),
    "elbow_flex":    (-90, 90),
    "wrist_flex":    (-90, 90),
    "wrist_roll":    (-90, 90),
    "gripper":       (0, 45),
}
# Right-controller face buttons for Z-lift (per Phase 2.5 plan).
VR_Z_LIFT_UP_BUTTON = "a"
VR_Z_LIFT_DOWN_BUTTON = "b"

# Keymaps (mirror laptop script lines 29-51)
LEFT_KEYMAP = {
    "shoulder_pan+": "q", "shoulder_pan-": "e",
    "wrist_roll+": "r", "wrist_roll-": "f",
    "gripper+": "t", "gripper-": "g",
    "x+": "w", "x-": "s", "y+": "a", "y-": "d",
    "pitch+": "z", "pitch-": "x",
    "reset": "c",
    "head_motor_1+": "<", "head_motor_1-": ">",
    "head_motor_2+": ",", "head_motor_2-": ".",
}
RIGHT_KEYMAP = {
    "shoulder_pan+": "7", "shoulder_pan-": "9",
    "wrist_roll+": "/", "wrist_roll-": "*",
    "gripper+": "+", "gripper-": "-",
    "x+": "8", "x-": "2", "y+": "4", "y-": "6",
    "pitch+": "1", "pitch-": "3",
    "reset": "0",
}
LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}
RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}
HEAD_MOTOR_MAP = {"head_motor_1": "head_motor_1", "head_motor_2": "head_motor_2"}


# ---------------------------------------------------------------------------
# Bus + motor definitions
# ---------------------------------------------------------------------------
class NormMode(Enum):
    RANGE_M100_100 = "M100_100"
    RANGE_0_100 = "0_100"


@dataclass
class Motor:
    id: int
    norm_mode: NormMode


@dataclass
class Calibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


BUS1_MOTORS = {
    "left_arm_shoulder_pan": Motor(1, NormMode.RANGE_M100_100),
    "left_arm_shoulder_lift": Motor(2, NormMode.RANGE_M100_100),
    "left_arm_elbow_flex": Motor(3, NormMode.RANGE_M100_100),
    "left_arm_wrist_flex": Motor(4, NormMode.RANGE_M100_100),
    "left_arm_wrist_roll": Motor(5, NormMode.RANGE_M100_100),
    "left_arm_gripper": Motor(6, NormMode.RANGE_0_100),
    "head_motor_1": Motor(7, NormMode.RANGE_M100_100),
    "head_motor_2": Motor(8, NormMode.RANGE_M100_100),
}
BUS2_MOTORS = {
    "right_arm_shoulder_pan": Motor(1, NormMode.RANGE_M100_100),
    "right_arm_shoulder_lift": Motor(2, NormMode.RANGE_M100_100),
    "right_arm_elbow_flex": Motor(3, NormMode.RANGE_M100_100),
    "right_arm_wrist_flex": Motor(4, NormMode.RANGE_M100_100),
    "right_arm_wrist_roll": Motor(5, NormMode.RANGE_M100_100),
    "right_arm_gripper": Motor(6, NormMode.RANGE_0_100),
    "base_left_wheel": Motor(9, NormMode.RANGE_M100_100),
    "base_right_wheel": Motor(10, NormMode.RANGE_M100_100),
    "z_lift": Motor(11, NormMode.RANGE_M100_100),
}
LEFT_ARM_MOTORS = [n for n in BUS1_MOTORS if n.startswith("left_arm")]
HEAD_MOTORS = [n for n in BUS1_MOTORS if n.startswith("head")]
RIGHT_ARM_MOTORS = [n for n in BUS2_MOTORS if n.startswith("right_arm")]
BASE_MOTORS = [n for n in BUS2_MOTORS if n.startswith("base")]
Z_LIFT_MOTORS = ["z_lift"]


# ---------------------------------------------------------------------------
# Sign-magnitude encoding (for Goal_Velocity, Present_Velocity, Present_Current)
# ---------------------------------------------------------------------------
def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    max_mag = (1 << sign_bit) - 1
    mag = abs(value)
    if mag > max_mag:
        mag = max_mag
    return ((1 if value < 0 else 0) << sign_bit) | mag


def decode_sign_magnitude(val: int, sign_bit: int) -> int:
    dir_bit = (val >> sign_bit) & 1
    mag = val & ((1 << sign_bit) - 1)
    return -mag if dir_bit else mag


# ---------------------------------------------------------------------------
# Calibration-aware normalize / un-normalize.
# Mirrors src/lerobot/motors/motors_bus.py _normalize + _unnormalize for
# RANGE_M100_100 and RANGE_0_100 only (xlerobot_2wheels doesn't use DEGREES).
# apply_drive_mode=True, matching FeetechMotorsBus.apply_drive_mode.
# ---------------------------------------------------------------------------
def normalize_raw(raw: int, motor: Motor, cal: Calibration) -> float:
    """Raw servo ticks -> normalized float."""
    min_, max_, drive = cal.range_min, cal.range_max, bool(cal.drive_mode)
    bounded = min(max_, max(min_, raw))
    if motor.norm_mode is NormMode.RANGE_M100_100:
        norm = (((bounded - min_) / (max_ - min_)) * 200) - 100
        return -norm if drive else norm
    # RANGE_0_100
    norm = ((bounded - min_) / (max_ - min_)) * 100
    return 100 - norm if drive else norm


def clamp_and_unnormalize(val: float, motor: Motor, cal: Calibration) -> int:
    """Normalized float -> raw servo ticks, clamped to [range_min, range_max]."""
    min_, max_, drive = cal.range_min, cal.range_max, bool(cal.drive_mode)
    if motor.norm_mode is NormMode.RANGE_M100_100:
        v = -val if drive else val
        v = min(100.0, max(-100.0, v))
        raw = int(((v + 100) / 200) * (max_ - min_) + min_)
    else:
        v = 100 - val if drive else val
        v = min(100.0, max(0.0, v))
        raw = int((v / 100) * (max_ - min_) + min_)
    # Final clamp to physical range (redundant but matches motors_bus.py DEGREES path).
    return min(max_, max(min_, raw))


# ---------------------------------------------------------------------------
# Minimal Feetech bus wrapper
# ---------------------------------------------------------------------------
def _patch_setPacketTimeout(self, packet_length):  # noqa: N802
    # Mirrors src/lerobot/motors/feetech/feetech.py:patch_setPacketTimeout.
    # The stock scservo_sdk packet_timeout is tuned too tight for STS3215
    # EEPROM commits (~10-50 ms). Without this, bus init issues ~100 register
    # writes that silently time out and push the ack past the client's 10 s
    # window, causing "TimeoutError: timed out" on the laptop.
    self.packet_start_time = self.getCurrentTime()
    overhead_ms = 500 if str(getattr(self, "port_name", "")).startswith("socket://") else 50
    self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + overhead_ms


class FeetechBus:
    def __init__(self, port: str, motors: dict[str, Motor], calibration: dict[str, Calibration]):
        self.port = port
        self.motors = motors
        self.calibration = calibration
        self.port_handler = scs.PortHandler(port)
        self.port_handler.setPacketTimeout = _patch_setPacketTimeout.__get__(  # type: ignore[method-assign]
            self.port_handler, scs.PortHandler
        )
        self.packet_handler = scs.PacketHandler(0)
        self.id_to_name = {m.id: name for name, m in motors.items()}

    def connect(self) -> None:
        if not self.port_handler.openPort():
            raise RuntimeError(f"Could not open {self.port}")
        self.port_handler.setBaudRate(BAUDRATE)

    def disconnect(self) -> None:
        try:
            self.port_handler.closePort()
        except Exception:
            pass

    def ping_all(self) -> list[int]:
        ok = []
        for m in self.motors.values():
            _, comm, _ = self.packet_handler.ping(self.port_handler, m.id)
            if comm == scs.COMM_SUCCESS:
                ok.append(m.id)
        return ok

    # --- low-level register I/O ---
    def write(self, reg_name: str, motor_name: str, value: int) -> bool:
        addr, length = REG[reg_name]
        mid = self.motors[motor_name].id
        if reg_name in SIGN_BIT_15_REGS:
            value = encode_sign_magnitude(int(value), 15)
        if length == 1:
            comm, _ = self.packet_handler.write1ByteTxRx(self.port_handler, mid, addr, int(value) & 0xFF)
        else:
            comm, _ = self.packet_handler.write2ByteTxRx(self.port_handler, mid, addr, int(value) & 0xFFFF)
        return comm == scs.COMM_SUCCESS

    def read(self, reg_name: str, motor_name: str) -> int | None:
        addr, length = REG[reg_name]
        mid = self.motors[motor_name].id
        if length == 1:
            val, comm, _ = self.packet_handler.read1ByteTxRx(self.port_handler, mid, addr)
        else:
            val, comm, _ = self.packet_handler.read2ByteTxRx(self.port_handler, mid, addr)
        if comm != scs.COMM_SUCCESS:
            return None
        if reg_name in SIGN_BIT_15_REGS:
            val = decode_sign_magnitude(val, 15)
        return val

    def configure(self) -> None:
        """Replicates FeetechMotorsBus.configure_motors(return_delay_time=20)."""
        for name in self.motors:
            self.write("Return_Delay_Time", name, 20)
            self.write("Maximum_Acceleration", name, 254)
            self.write("Acceleration", name, 254)

    def write_calibration(self) -> None:
        """Push Homing_Offset + Min/Max_Position_Limit to each motor (matches
        FeetechMotorsBus.write_calibration)."""
        for name, cal in self.calibration.items():
            if name not in self.motors:
                continue
            self.write("Homing_Offset", name, cal.homing_offset)
            self.write("Min_Position_Limit", name, cal.range_min)
            self.write("Max_Position_Limit", name, cal.range_max)

    # --- sync ops ---
    def sync_read_positions(self, names: list[str]) -> dict[str, float]:
        """Returns {name: normalized_float} for Present_Position."""
        addr, length = REG["Present_Position"]
        gsr = scs.GroupSyncRead(self.port_handler, self.packet_handler, addr, length)
        for n in names:
            gsr.addParam(self.motors[n].id)
        comm = gsr.txRxPacket()
        out: dict[str, float] = {}
        if comm != scs.COMM_SUCCESS:
            return out
        for n in names:
            mid = self.motors[n].id
            if gsr.isAvailable(mid, addr, length):
                raw = gsr.getData(mid, addr, length)
                out[n] = normalize_raw(raw, self.motors[n], self.calibration[n])
        return out

    def sync_read_currents(self, names: list[str]) -> dict[str, int]:
        """Returns {name: signed_int} for Present_Current."""
        addr, length = REG["Present_Current"]
        gsr = scs.GroupSyncRead(self.port_handler, self.packet_handler, addr, length)
        for n in names:
            gsr.addParam(self.motors[n].id)
        comm = gsr.txRxPacket()
        out: dict[str, int] = {}
        if comm != scs.COMM_SUCCESS:
            return out
        for n in names:
            mid = self.motors[n].id
            if gsr.isAvailable(mid, addr, length):
                raw = gsr.getData(mid, addr, length)
                out[n] = decode_sign_magnitude(raw, 15)
        return out

    def write_positions(self, targets: dict[str, float], use_sync: bool = False) -> None:
        """targets: {name: normalized_float}. Applies calibration clamp + un-normalize."""
        addr, length = REG["Goal_Position"]
        raws: dict[str, int] = {}
        for n, v in targets.items():
            raws[n] = clamp_and_unnormalize(v, self.motors[n], self.calibration[n])
        if use_sync:
            gsw = scs.GroupSyncWrite(self.port_handler, self.packet_handler, addr, length)
            for n, raw in raws.items():
                mid = self.motors[n].id
                data = [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)]
                gsw.addParam(mid, data)
            gsw.txPacket()
            gsw.clearParam()
        else:
            for n, raw in raws.items():
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, self.motors[n].id, addr, raw & 0xFFFF
                )

    def write_wheel_velocities(self, raws: dict[str, int]) -> None:
        """raws: {name: signed_int} for wheel motors in velocity mode."""
        addr, length = REG["Goal_Velocity"]
        gsw = scs.GroupSyncWrite(self.port_handler, self.packet_handler, addr, length)
        for n, v in raws.items():
            enc = encode_sign_magnitude(int(v), 15)
            data = [scs.SCS_LOBYTE(enc), scs.SCS_HIBYTE(enc)]
            gsw.addParam(self.motors[n].id, data)
        gsw.txPacket()
        gsw.clearParam()


# ---------------------------------------------------------------------------
# SO101 kinematics (from src/lerobot/model/SO101Robot.py)
# ---------------------------------------------------------------------------
class SO101Kinematics:
    def __init__(self, l1: float = 0.1159, l2: float = 0.1350):
        self.l1, self.l2 = l1, l2

    def inverse_kinematics(self, x: float, y: float) -> tuple[float, float]:
        l1, l2 = self.l1, self.l2
        theta1_offset = math.atan2(0.028, 0.11257)
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
        r = math.sqrt(x * x + y * y)
        r_max = l1 + l2
        if r > r_max:
            x *= r_max / r; y *= r_max / r; r = r_max
        r_min = abs(l1 - l2)
        if 0 < r < r_min:
            x *= r_min / r; y *= r_min / r; r = r_min
        cos_t2 = -(r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))
        theta2 = math.pi - math.acos(cos_t2)
        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = beta + gamma
        j2 = max(-0.1, min(3.45, theta1 + theta1_offset))
        j3 = max(-0.2, min(math.pi, theta2 + theta2_offset))
        j2d = 90 - math.degrees(j2)
        j3d = math.degrees(j3) - 90
        return j2d, j3d


# ---------------------------------------------------------------------------
# Controllers (ported from 4_xlerobot_2wheels_teleop_keyboard.py)
# ---------------------------------------------------------------------------
class SimpleTeleopArm:
    def __init__(self, kin: SO101Kinematics, joint_map: dict[str, str], initial_obs: dict[str, float],
                 prefix: str = "left", kp: float = 0.81):
        self.kin = kin
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        self.degree_step = 3
        self.xy_step = 0.0081
        self.target_positions = dict.fromkeys(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"], 0.0
        )
        self.zero_pos = dict(self.target_positions)

    def handle_keys(self, key_state: dict[str, bool]) -> None:
        step = self.degree_step
        if key_state.get("shoulder_pan+"): self.target_positions["shoulder_pan"] += step
        if key_state.get("shoulder_pan-"): self.target_positions["shoulder_pan"] -= step
        if key_state.get("wrist_roll+"):   self.target_positions["wrist_roll"] += step
        if key_state.get("wrist_roll-"):   self.target_positions["wrist_roll"] -= step
        if key_state.get("gripper+"):      self.target_positions["gripper"] += step
        if key_state.get("gripper-"):      self.target_positions["gripper"] -= step
        if key_state.get("pitch+"):        self.pitch += step
        if key_state.get("pitch-"):        self.pitch -= step

        moved = False
        if key_state.get("x+"): self.current_x += self.xy_step; moved = True
        if key_state.get("x-"): self.current_x -= self.xy_step; moved = True
        if key_state.get("y+"): self.current_y += self.xy_step; moved = True
        if key_state.get("y-"): self.current_y -= self.xy_step; moved = True
        if moved:
            j2, j3 = self.kin.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = j2
            self.target_positions["elbow_flex"] = j3

        # wrist_flex coupled to pitch
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            - self.target_positions["elbow_flex"]
            + self.pitch
        )

    def reset(self) -> None:
        self.target_positions = dict(self.zero_pos)
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0

    def handle_vr_input(self, vr_goal) -> None:
        """VR delta-position control. Ported verbatim from
        examples/xlerobot/8_xlerobot_2wheels_teleop_vr.py:169-294 (monolith).
        Stateful: caches prev_vr_pos / prev_wrist_flex / prev_wrist_roll on self."""
        if vr_goal is None:
            return
        if not hasattr(vr_goal, "target_position") or vr_goal.target_position is None:
            return

        current_vr_pos = vr_goal.target_position  # [x, y, z] in meters
        if not hasattr(self, "prev_vr_pos"):
            self.prev_vr_pos = current_vr_pos
            return  # establish baseline

        vr_x = (current_vr_pos[0] - self.prev_vr_pos[0]) * 220
        vr_y = (current_vr_pos[1] - self.prev_vr_pos[1]) * 70
        vr_z = (current_vr_pos[2] - self.prev_vr_pos[2]) * 70

        # Controller reconnect guard — reset baseline and skip.
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

        self.current_x += -delta_z  # VR Z -> robot X (direction flipped)
        self.current_y += delta_y   # VR Y -> robot Y

        # Wrist pitch (from wrist_flex_deg delta)
        if getattr(vr_goal, "wrist_flex_deg", None) is not None:
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

        # Wrist roll (absolute joint target)
        if getattr(vr_goal, "wrist_roll_deg", None) is not None:
            if not hasattr(self, "prev_wrist_roll"):
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            delta_roll = (vr_goal.wrist_roll_deg - self.prev_wrist_roll) * angle_scale
            if abs(delta_roll) > 30:
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            delta_roll = max(-angle_limit, min(angle_limit, delta_roll))
            current_roll = self.target_positions.get("wrist_roll", 0.0)
            self.target_positions["wrist_roll"] = max(-90, min(90, current_roll + delta_roll))
            self.prev_wrist_roll = vr_goal.wrist_roll_deg

        # VR x-delta -> shoulder_pan
        if abs(delta_x) > 0.001:
            delta_pan = max(-angle_limit, min(angle_limit, delta_x * 200.0))
            current_pan = self.target_positions.get("shoulder_pan", 0.0)
            self.target_positions["shoulder_pan"] = max(-180, min(180, current_pan + delta_pan))

        # IK -> shoulder_lift / elbow_flex (smoothed)
        try:
            j2, j3 = self.kin.inverse_kinematics(self.current_x, self.current_y)
            alpha = 0.1
            self.target_positions["shoulder_lift"] = (
                (1 - alpha) * self.target_positions.get("shoulder_lift", 0.0) + alpha * j2
            )
            self.target_positions["elbow_flex"] = (
                (1 - alpha) * self.target_positions.get("elbow_flex", 0.0) + alpha * j3
            )
        except Exception as e:
            print(f"[{self.prefix}] VR IK failed: {e}")

        # wrist_flex coupled to pitch (same as keyboard path)
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            - self.target_positions["elbow_flex"]
            + self.pitch
        )

        # Gripper from trigger
        trigger = 0.0
        if getattr(vr_goal, "metadata", None):
            trigger = vr_goal.metadata.get("trigger", 0.0) or 0.0
        self.target_positions["gripper"] = 45 if trigger > 0.5 else 0.0

        # Clamp all joints to soft VR limits
        for j, (lo, hi) in JOINT_LIMITS.items():
            if j in self.target_positions:
                self.target_positions[j] = max(lo, min(hi, self.target_positions[j]))

    def p_control_targets(self, obs: dict[str, float]) -> dict[str, float]:
        """Returns {full_motor_name: normalized_target} to send as Goal_Position."""
        out: dict[str, float] = {}
        for j, full in self.joint_map.items():
            cur = obs.get(full, 0.0)
            err = self.target_positions[j] - cur
            out[full] = cur + self.kp * err
        return out


class SimpleHeadControl:
    def __init__(self, initial_obs: dict[str, float], kp: float = 0.81):
        self.kp = kp
        self.degree_step = 1
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def handle_keys(self, key_state: dict[str, bool]) -> None:
        if key_state.get("head_motor_1+"): self.target_positions["head_motor_1"] += self.degree_step
        if key_state.get("head_motor_1-"): self.target_positions["head_motor_1"] -= self.degree_step
        if key_state.get("head_motor_2+"): self.target_positions["head_motor_2"] += self.degree_step
        if key_state.get("head_motor_2-"): self.target_positions["head_motor_2"] -= self.degree_step

    def handle_vr_input(self, left_vr_goal) -> None:
        """Left-controller thumbstick drives head pan/tilt.
        Ported from 8_xlerobot_2wheels_teleop_vr.py:334-349."""
        if left_vr_goal is None or not getattr(left_vr_goal, "metadata", None):
            return
        thumb = left_vr_goal.metadata.get("thumbstick") or {}
        if not thumb:
            return
        tx = thumb.get("x", 0) or 0
        ty = thumb.get("y", 0) or 0
        step = 2  # monolith value (bigger than keyboard's 1)
        if abs(tx) > 0.1:
            self.target_positions["head_motor_1"] += step if tx > 0 else -step
        if abs(ty) > 0.1:
            self.target_positions["head_motor_2"] += step if ty > 0 else -step

    def reset(self) -> None:
        self.target_positions = dict(self.zero_pos)

    def p_control_targets(self, obs: dict[str, float]) -> dict[str, float]:
        out: dict[str, float] = {}
        for name in self.target_positions:
            cur = obs.get(name, 0.0)
            err = self.target_positions[name] - cur
            out[name] = cur + self.kp * err
        return out


class SmoothBaseController:
    def __init__(self):
        self.current_speed = 0.0
        self.last_time = time.time()
        self.last_direction = {"x.vel": 0.0, "theta.vel": 0.0}
        self.is_moving = False
        self.speed_index = 0

    def update(self, pressed_keys: set[str]) -> dict[str, float]:
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if TELEOP_KEYS["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, len(SPEED_LEVELS) - 1)
        if TELEOP_KEYS["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)

        base_keys = [TELEOP_KEYS["forward"], TELEOP_KEYS["backward"],
                     TELEOP_KEYS["rotate_left"], TELEOP_KEYS["rotate_right"]]
        any_pressed = any(k in pressed_keys for k in base_keys)

        base = {"x.vel": 0.0, "theta.vel": 0.0}
        if any_pressed:
            self.is_moving = True
            setting = SPEED_LEVELS[self.speed_index]
            lin, ang = setting["linear"], setting["angular"]
            if TELEOP_KEYS["forward"] in pressed_keys:      base["x.vel"] += lin
            if TELEOP_KEYS["backward"] in pressed_keys:     base["x.vel"] -= lin
            if TELEOP_KEYS["rotate_left"] in pressed_keys:  base["theta.vel"] += ang
            if TELEOP_KEYS["rotate_right"] in pressed_keys: base["theta.vel"] -= ang
            self.last_direction = base.copy()
            self.current_speed = min(self.current_speed + BASE_ACCELERATION_RATE * dt, BASE_MAX_SPEED)
        else:
            self.is_moving = False
            if self.current_speed > 0.01 and self.last_direction:
                base = self.last_direction.copy()
            self.current_speed = max(self.current_speed - BASE_DECELERATION_RATE * dt, 0.0)

        for k in base:
            orig = base[k]
            base[k] *= self.current_speed
            if self.current_speed > 0.01 and abs(base[k]) < MIN_VELOCITY_THRESHOLD:
                base[k] = MIN_VELOCITY_THRESHOLD if orig > 0 else -MIN_VELOCITY_THRESHOLD
        return base

    def stop(self) -> None:
        self.current_speed = 0.0
        self.is_moving = False
        self.last_direction = {"x.vel": 0.0, "theta.vel": 0.0}


class StallDetector:
    CURRENT_THRESHOLD = 30
    POS_EPSILON = 0.25
    TARGET_CHANGE = 0.5
    STALL_LOOPS = 15
    NORMAL_TORQUE = 600
    STALL_TORQUE = 100
    GRIP_TORQUE = 200

    def __init__(self):
        self.prev_pos: dict[str, float] = {}
        self.stall_count: dict[str, int] = {}
        self.stalled: dict[str, bool] = {}
        self.retried: dict[str, bool] = {}
        self.stall_target: dict[str, float] = {}

    def update(self, bus: FeetechBus, motor_names: list[str],
               positions: dict[str, float], targets: dict[str, float] | None = None) -> list[str]:
        currents = bus.sync_read_currents(motor_names)
        newly_stalled: list[str] = []
        for name in motor_names:
            pos = positions.get(name, 0.0)
            cur = currents.get(name, 0)
            prev = self.prev_pos.get(name, pos)
            delta = abs(pos - prev)
            self.prev_pos[name] = pos
            is_gripper = "gripper" in name

            if self.stalled.get(name) and targets:
                nt, ot = targets.get(name), self.stall_target.get(name)
                if nt is not None and ot is not None and abs(nt - ot) > self.TARGET_CHANGE:
                    self.stalled[name] = False
                    self.retried[name] = False
                    self.stall_count[name] = 0
                    bus.write("Torque_Limit", name, self.NORMAL_TORQUE)

            if self.stalled.get(name):
                continue

            if cur > self.CURRENT_THRESHOLD and delta < self.POS_EPSILON:
                self.stall_count[name] = self.stall_count.get(name, 0) + 1
                if self.stall_count[name] >= self.STALL_LOOPS:
                    if is_gripper:
                        self.stalled[name] = True
                        self.stall_target[name] = pos
                        bus.write("Torque_Limit", name, self.GRIP_TORQUE)
                        newly_stalled.append(name)
                        print(f"[GRIP] {name} gripping (current={cur}) — holding at reduced torque")
                    elif not self.retried.get(name):
                        self.retried[name] = True
                        self.stall_count[name] = 0
                        print(f"[STALL] {name} stalled (current={cur}) — retrying once")
                    else:
                        self.stalled[name] = True
                        self.stall_target[name] = pos
                        bus.write("Torque_Limit", name, self.STALL_TORQUE)
                        newly_stalled.append(name)
                        print(f"[STALL] {name} stalled again — torque reduced")
            else:
                self.stall_count[name] = 0
                self.retried[name] = False
        return newly_stalled


# ---------------------------------------------------------------------------
# Differential-drive wheel math (ported from xlerobot_2wheels.py:386-510)
# ---------------------------------------------------------------------------
def _degps_to_raw(degps: float) -> int:
    v = int(round(degps * (4096.0 / 360.0)))
    return max(-0x8000, min(0x7FFF, v))


def get_vr_base_action(right_vr_goal) -> dict[str, float]:
    """Right thumbstick -> base velocity command.
    Ported from 8_xlerobot_2wheels_teleop_vr.py:379-406. Uses the max
    speed level (same linear=0.3 angular=90 we already hard-code in
    Phase 2). Deadzone 0.15 matches monolith."""
    if right_vr_goal is None or not getattr(right_vr_goal, "metadata", None):
        return {"x.vel": 0.0, "theta.vel": 0.0}
    thumb = right_vr_goal.metadata.get("thumbstick") or {}
    if not thumb:
        return {"x.vel": 0.0, "theta.vel": 0.0}
    tx = thumb.get("x", 0) or 0
    ty = thumb.get("y", 0) or 0
    level = SPEED_LEVELS[-1]
    x_cmd = -ty * level["linear"] if abs(ty) > 0.15 else 0.0
    theta_cmd = tx * level["angular"] if abs(tx) > 0.15 else 0.0
    return {"x.vel": x_cmd, "theta.vel": theta_cmd}


class _VRGoal:
    """Lightweight attribute-bag that mimics ControlGoal for the ported
    monolith code (which accesses `.target_position`, `.wrist_roll_deg`,
    `.wrist_flex_deg`, `.metadata`)."""
    __slots__ = ("target_position", "wrist_roll_deg", "wrist_flex_deg", "metadata")

    def __init__(self, d: dict[str, Any] | None):
        if not d:
            self.target_position = None
            self.wrist_roll_deg = None
            self.wrist_flex_deg = None
            self.metadata = {}
            return
        pos = d.get("position")
        self.target_position = list(pos) if pos is not None else None
        self.wrist_roll_deg = d.get("wrist_roll_deg")
        self.wrist_flex_deg = d.get("wrist_flex_deg")
        self.metadata = {
            "trigger": d.get("trigger", 0.0),
            "thumbstick": d.get("thumbstick") or {},
            "buttons": d.get("buttons") or {},
        }


def body_to_wheel_raw(x: float, theta: float, max_raw: int = WHEEL_MAX_RAW) -> dict[str, int]:
    theta_rad = theta * (math.pi / 180.0)
    l_w = (x - theta_rad * WHEELBASE / 2) / WHEEL_RADIUS
    r_w = (x + theta_rad * WHEELBASE / 2) / WHEEL_RADIUS
    l_dps = l_w * (180.0 / math.pi)
    r_dps = r_w * (180.0 / math.pi)
    steps_per_deg = 4096.0 / 360.0
    max_computed = max(abs(l_dps) * steps_per_deg, abs(r_dps) * steps_per_deg)
    if max_computed > max_raw:
        scale = max_raw / max_computed
        l_dps *= scale; r_dps *= scale
    return {"base_left_wheel": -_degps_to_raw(r_dps), "base_right_wheel": _degps_to_raw(l_dps)}


# ---------------------------------------------------------------------------
# Calibration loading
# ---------------------------------------------------------------------------
def load_calibration(path: Path) -> dict[str, Calibration]:
    with open(path) as f:
        raw = json.load(f)
    out: dict[str, Calibration] = {}
    for name, c in raw.items():
        out[name] = Calibration(
            id=c["id"], drive_mode=c["drive_mode"],
            homing_offset=c["homing_offset"],
            range_min=c["range_min"], range_max=c["range_max"],
        )
    return out


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------
class LineReader:
    """Non-blocking newline-delimited reader with timeout."""
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buf = b""
        self.sock.setblocking(False)

    def poll(self, timeout: float) -> list[dict[str, Any]]:
        end = time.time() + timeout
        frames: list[dict[str, Any]] = []
        while True:
            remaining = end - time.time()
            if remaining <= 0:
                break
            try:
                self.sock.settimeout(max(0.001, remaining))
                chunk = self.sock.recv(4096)
            except (BlockingIOError, socket.timeout):
                break
            except OSError:
                break
            if not chunk:
                raise ConnectionError("client closed")
            self.buf += chunk
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                if line.strip():
                    try:
                        frames.append(json.loads(line.decode("utf-8")))
                    except json.JSONDecodeError:
                        pass
        return frames


def configure_bus_for_teleop(bus: FeetechBus, arm_names: list[str], head_names: list[str] = (),
                             wheel_names: list[str] = ()) -> None:
    """Per-motor setup matching 4_xlerobot_..._teleop_keyboard.py lines 634-674."""
    # Disable torque on all
    for n in list(arm_names) + list(head_names) + list(wheel_names):
        bus.write("Torque_Enable", n, 0)
        bus.write("Lock", n, 0)

    # Position-mode motors: arm + head
    for n in list(arm_names) + list(head_names):
        bus.write("Operating_Mode", n, 0)
        bus.write("P_Coefficient", n, 16)
        bus.write("I_Coefficient", n, 0)
        bus.write("D_Coefficient", n, 43)
        bus.write("Torque_Limit", n, 600)

    # Wheel motors: velocity mode
    for n in wheel_names:
        bus.write("Operating_Mode", n, 1)

    # Seed Goal_Position = Present_Position, enable torque, small stagger.
    for n in list(arm_names) + list(head_names) + list(wheel_names):
        addr, length = REG["Present_Position"]
        mid = bus.motors[n].id
        raw, comm, _ = bus.packet_handler.read2ByteTxRx(bus.port_handler, mid, addr)
        if comm == scs.COMM_SUCCESS:
            bus.packet_handler.write2ByteTxRx(
                bus.port_handler, mid, REG["Goal_Position"][0], raw & 0xFFFF
            )
        bus.write("Torque_Enable", n, 1)
        bus.write("Lock", n, 1)
        time.sleep(0.05)


def handle_client(conn: socket.socket, addr, args) -> None:
    print(f"[SRV] client connected: {addr}")
    reader = LineReader(conn)

    # Wait for hello (up to 5s)
    frames = reader.poll(5.0)
    hello = next((f for f in frames if "hello" in f), None)
    if hello is None:
        print("[SRV] no hello in 5s; closing"); conn.close(); return
    bus_choice = str(hello["hello"].get("bus_choice", "2"))
    mode = str(hello["hello"].get("mode", "keyboard")).lower()
    if mode not in ("keyboard", "vr"):
        mode = "keyboard"
    use_bus1 = bus_choice in ("1", "3")
    use_bus2 = bus_choice in ("2", "3")
    print(f"[SRV] hello: mode={mode} bus_choice={bus_choice} (bus1={use_bus1} bus2={use_bus2})")

    # Load calibration
    cal_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"
    if not cal_path.is_file():
        msg = f"calibration missing at {cal_path}"
        print(f"[SRV] {msg}")
        conn.sendall((json.dumps({"ack": False, "error": msg}) + "\n").encode())
        conn.close(); return
    calibration = load_calibration(cal_path)

    bus1 = bus2 = None
    try:
        if use_bus1:
            bus1 = FeetechBus(PORT_BUS1, BUS1_MOTORS, calibration)
            bus1.connect()
            bus1.configure()
            bus1.write_calibration()
            configure_bus_for_teleop(bus1, LEFT_ARM_MOTORS, HEAD_MOTORS)
        if use_bus2:
            bus2 = FeetechBus(PORT_BUS2, BUS2_MOTORS, calibration)
            bus2.connect()
            bus2.configure()
            bus2.write_calibration()
            configure_bus_for_teleop(bus2, RIGHT_ARM_MOTORS,
                                     wheel_names=BASE_MOTORS + Z_LIFT_MOTORS)

        # Initial obs: normalized positions for arms + head
        initial_obs: dict[str, float] = {}
        if bus1:
            initial_obs.update(bus1.sync_read_positions(LEFT_ARM_MOTORS + HEAD_MOTORS))
        if bus2:
            initial_obs.update(bus2.sync_read_positions(RIGHT_ARM_MOTORS))
        for j in LEFT_JOINT_MAP.values():
            initial_obs.setdefault(j, 0.0)
        for j in RIGHT_JOINT_MAP.values():
            initial_obs.setdefault(j, 0.0)
        for j in HEAD_MOTOR_MAP.values():
            initial_obs.setdefault(j, 0.0)

        # Build controllers
        kin_l, kin_r = SO101Kinematics(), SO101Kinematics()
        left_arm = SimpleTeleopArm(kin_l, LEFT_JOINT_MAP, initial_obs, prefix="left")
        right_arm = SimpleTeleopArm(kin_r, RIGHT_JOINT_MAP, initial_obs, prefix="right")
        head = SimpleHeadControl(initial_obs)
        base = SmoothBaseController()
        stall = StallDetector()

        # Send ack
        conn.sendall((json.dumps({"ack": True, "initial_obs": initial_obs}) + "\n").encode())

        # 50 Hz main loop
        print(f"[SRV] control loop starting @ {FPS} Hz")
        last_frame_time = time.time()
        pressed_keys: set[str] = set()
        vr_left_goal: _VRGoal | None = None
        vr_right_goal: _VRGoal | None = None
        loop_count = 0
        error_count = 0
        loop_dt = 1.0 / FPS
        t_last_report = time.time()
        frames_since_report = 0

        while True:
            loop_start = time.time()
            loop_count += 1

            # Read any waiting frames (non-blocking, ~1 ms budget)
            try:
                frames = reader.poll(0.001)
            except ConnectionError:
                print("[SRV] client disconnected")
                break
            for f in frames:
                if "bye" in f:
                    print("[SRV] client said bye"); raise StopIteration
                if "keys" in f:
                    pressed_keys = set(f["keys"])
                    last_frame_time = loop_start
                if "vr" in f:
                    v = f["vr"] or {}
                    vr_left_goal = _VRGoal(v.get("left"))
                    vr_right_goal = _VRGoal(v.get("right"))
                    last_frame_time = loop_start
                if f.get("reset_left") and use_bus1:
                    left_arm.reset()
                if f.get("reset_right") and use_bus2:
                    right_arm.reset()
                if f.get("reset_head") and use_bus1:
                    head.reset()

            dead_man = (loop_start - last_frame_time) > DEAD_MAN_SEC
            if dead_man:
                pressed_keys = set()
                vr_left_goal = None
                vr_right_goal = None
                base.stop()

            # Map inputs -> arm/head/base target updates
            if mode == "vr":
                if use_bus1:
                    left_arm.handle_vr_input(vr_left_goal)
                    head.handle_vr_input(vr_left_goal)
                if use_bus2:
                    right_arm.handle_vr_input(vr_right_goal)
            else:
                left_state = {a: (k in pressed_keys) for a, k in LEFT_KEYMAP.items()}
                right_state = {a: (k in pressed_keys) for a, k in RIGHT_KEYMAP.items()}
                if use_bus1:
                    left_arm.handle_keys(left_state)
                    head.handle_keys(left_state)
                if use_bus2:
                    right_arm.handle_keys(right_state)

            try:
                if use_bus1 and bus1 is not None:
                    pos = bus1.sync_read_positions(LEFT_ARM_MOTORS + HEAD_MOTORS)
                    if pos:
                        left_targets = left_arm.p_control_targets(pos)
                        head_targets = head.p_control_targets(pos)
                        combined = {**left_targets, **head_targets}
                        bus1.write_positions(combined, use_sync=args.sync_write)
                        stall.update(bus1, LEFT_ARM_MOTORS + HEAD_MOTORS, pos, combined)

                if use_bus2 and bus2 is not None:
                    pos2 = bus2.sync_read_positions(RIGHT_ARM_MOTORS)
                    if pos2:
                        right_targets = right_arm.p_control_targets(pos2)
                        bus2.write_positions(right_targets, use_sync=args.sync_write)
                        stall.update(bus2, RIGHT_ARM_MOTORS, pos2, right_targets)

                    # Base + Z-lift depend on mode
                    if mode == "vr":
                        base_action = get_vr_base_action(vr_right_goal)
                        right_buttons = (vr_right_goal.metadata.get("buttons") or {}
                                         ) if vr_right_goal else {}
                        z_vel = 0
                        if right_buttons.get(VR_Z_LIFT_UP_BUTTON):
                            z_vel = Z_LIFT_VELOCITY
                        elif right_buttons.get(VR_Z_LIFT_DOWN_BUTTON):
                            z_vel = -Z_LIFT_VELOCITY
                    else:
                        base_action = base.update(pressed_keys)
                        z_vel = 0
                        if Z_LIFT_UP_KEY in pressed_keys:
                            z_vel = Z_LIFT_VELOCITY
                        elif Z_LIFT_DOWN_KEY in pressed_keys:
                            z_vel = -Z_LIFT_VELOCITY

                    wheels = body_to_wheel_raw(base_action.get("x.vel", 0.0),
                                                base_action.get("theta.vel", 0.0))
                    wheels["z_lift"] = z_vel
                    bus2.write_wheel_velocities(wheels)
            except Exception as e:
                error_count += 1
                if error_count < 5 or error_count % 50 == 0:
                    print(f"[SRV] loop error #{error_count}: {e}")

            # Telemetry every N loops
            frames_since_report += 1
            if loop_count % TELEMETRY_INTERVAL == 0:
                now = time.time()
                hz = frames_since_report / max(0.001, (now - t_last_report))
                frames_since_report = 0
                t_last_report = now
                try:
                    tele = {"loop_hz": round(hz, 1), "errors": error_count,
                             "stalled": [n for n, s in stall.stalled.items() if s]}
                    conn.sendall((json.dumps(tele) + "\n").encode())
                except OSError:
                    print("[SRV] client write failed; ending session"); break

            # Pace the loop
            remaining = loop_dt - (time.time() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    except StopIteration:
        pass
    except Exception as e:
        print(f"[SRV] session error: {e}")
        import traceback; traceback.print_exc()
    finally:
        # Zero wheels + disable torque on close
        for b in (bus1, bus2):
            if b is None:
                continue
            try:
                if b is bus2:
                    b.write_wheel_velocities({
                        "base_left_wheel": 0, "base_right_wheel": 0, "z_lift": 0,
                    })
                for n in b.motors:
                    b.write("Torque_Enable", n, 0)
                    b.write("Lock", n, 0)
            except Exception:
                pass
            b.disconnect()
        try: conn.close()
        except Exception: pass
        print(f"[SRV] session ended; loops={loop_count} errors={error_count}")


def run_print_clamp_table() -> None:
    """Debug: print the clamp table for every motor, every norm-mode edge value."""
    cal_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"
    calibration = load_calibration(cal_path)
    all_motors = {**BUS1_MOTORS, **BUS2_MOTORS}
    for name, m in all_motors.items():
        c = calibration.get(name)
        if c is None:
            print(f"{name}: no calibration"); continue
        samples = [-100, -50, 0, 50, 100] if m.norm_mode is NormMode.RANGE_M100_100 else [0, 25, 50, 75, 100]
        raws = [clamp_and_unnormalize(v, m, c) for v in samples]
        print(f"{name:28s} id={c.id:2d} drive={c.drive_mode} range=[{c.range_min},{c.range_max}] -> {dict(zip(samples, raws))}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="Do not open any bus; accept one telnet hello + echo ack + exit.")
    ap.add_argument("--print-clamp-table", action="store_true",
                     help="Print calibration clamp outputs and exit.")
    ap.add_argument("--sync-write", action="store_true",
                     help="Use GroupSyncWrite for Goal_Position (WARNING: CLAUDE.md §3 — known to kill direct CH343 USB bus).")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=TCP_PORT)
    args = ap.parse_args()

    if args.print_clamp_table:
        run_print_clamp_table()
        return

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"[SRV] listening on {args.host}:{args.port} (dry_run={args.dry_run}, sync_write={args.sync_write})")

    try:
        while True:
            conn, addr = srv.accept()
            if args.dry_run:
                print(f"[SRV] dry-run client: {addr}")
                try:
                    data = conn.recv(4096)
                    print(f"[SRV] received: {data!r}")
                    conn.sendall((json.dumps({"ack": True, "initial_obs": {}}) + "\n").encode())
                except Exception as e:
                    print(f"[SRV] dry-run recv error: {e}")
                conn.close()
                return
            try:
                handle_client(conn, addr, args)
            except Exception as e:
                print(f"[SRV] handler crash: {e}")
                import traceback; traceback.print_exc()
    except KeyboardInterrupt:
        print("\n[SRV] shutting down")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
