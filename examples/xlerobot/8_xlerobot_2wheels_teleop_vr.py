#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
import asyncio
import logging
import math
import serial
import sys
import threading
import time
import traceback

# Third-party imports
import numpy as np
import pygame

# Local imports
from vr_monitor import VRMonitor
from lerobot.robots.xlerobot_2wheels import XLerobot2WheelsConfig, XLerobot2Wheels
from lerobot.utils.errors import DeviceNotConnectedError
# from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.robot_utils import precise_sleep
from lerobot.model.SO101Robot import SO101Kinematics

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint mapping configurations
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

HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

# Joint calibration coefficients - manually edit
# Format: [joint_name, zero_position_offset(degrees), scale_factor]
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # Joint1: zero position offset, scale factor
    ['shoulder_lift', 2.0, 0.97],     # Joint2: zero position offset, scale factor
    ['elbow_flex', 0.0, 1.05],        # Joint3: zero position offset, scale factor
    ['wrist_flex', 0.0, 0.94],        # Joint4: zero position offset, scale factor
    ['wrist_roll', 0.0, 0.5],        # Joint5: zero position offset, scale factor
    ['gripper', 0.0, 1.0],           # Joint6: zero position offset, scale factor
]

# Joint limits in degrees to prevent stalling
JOINT_LIMITS = {
    'shoulder_pan': (-90, 90),
    'shoulder_lift': (-90, 90),
    'elbow_flex': (-90, 90),
    'wrist_flex': (-90, 90),
    'wrist_roll': (-90, 90),
    'gripper': (0, 45),
}

# ─── Ender 3 motion platform config ──────────────────────────────────────────
ENDER3_PORT = "COM9"
ENDER3_BAUD = 115200
ENDER3_Z_STEP = 10    # mm per button press
ENDER3_Y_STEP = 50    # mm per button press
ENDER3_Z_FEED = 300   # mm/min (slow for Z)
ENDER3_Y_FEED = 3000  # mm/min


class Ender3Controller:
    """Controls Ender 3 Z-lift and Y-bed via G-code over serial."""

    def __init__(self, port=ENDER3_PORT, baud=ENDER3_BAUD):
        self.port_name = port
        self.baud = baud
        self.ser = None
        self._lock = threading.Lock()
        self._last_button_time = {}  # debounce tracking
        self._moving = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=2)
            time.sleep(2)  # wait for Marlin to boot
            self._drain()
            print(f"[ENDER3] Connected on {self.port_name}")
            self.send("G92 X0")       # suppress missing X axis
            self.send("M84 S0")       # disable stepper idle timeout (keep energized)
            self.send("G28 Y")        # home bed
            self.send("G28 Z")        # home lift
            self.send("G91")          # relative positioning mode
            print("[ENDER3] Homed and ready (relative mode)")
        except Exception as e:
            print(f"[ENDER3] Failed to connect: {e}")
            self.ser = None

    def send(self, cmd):
        if not self.ser:
            return
        with self._lock:
            self.ser.write(f"{cmd}\n".encode())
            time.sleep(0.2)
            self._drain()

    def _drain(self):
        while self.ser and self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"[ENDER3] {line}")

    def move_z(self, delta_mm):
        print(f"[ENDER3] Z {'up' if delta_mm > 0 else 'down'} {abs(delta_mm)}mm")
        self.send(f"G1 Z{delta_mm} F{ENDER3_Z_FEED}")

    def move_y(self, delta_mm):
        print(f"[ENDER3] Bed {'forward' if delta_mm > 0 else 'backward'} {abs(delta_mm)}mm")
        self.send(f"G1 Y{delta_mm} F{ENDER3_Y_FEED}")

    def emergency_stop(self):
        if self.ser:
            print("[ENDER3] EMERGENCY STOP")
            self.ser.write(b"M112\n")

    def disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
            print("[ENDER3] Disconnected")

    def is_moving(self):
        """Check if Ender 3 is currently executing a move."""
        return self._moving

    def handle_button(self, button_name, action_fn, *args, debounce_ms=600):
        """Call action_fn if button hasn't been pressed within debounce window."""
        now = time.time()
        last = self._last_button_time.get(button_name, 0)
        if (now - last) * 1000 > debounce_ms:
            self._last_button_time[button_name] = now
            self._moving = True
            action_fn(*args)
            time.sleep(0.5)  # pause to let power settle before arms resume
            self._moving = False


class SimpleTeleopArm:
    """
    A class for controlling a robot arm using VR input with delta action control.
    
    This class provides inverse kinematics-based arm control with proportional control
    for smooth movement and gripper operations based on VR controller input.
    """
    
    def __init__(self, joint_map, initial_obs, kinematics, prefix="right", kp=1):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.kinematics = kinematics
        
        # Initial joint positions - adapted for XLerobot observation format
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Delta control state variables for VR input
        self.last_vr_time = 0.0
        self.vr_deadzone = 0.001  # Minimum movement threshold
        self.max_delta_per_frame = 0.005  # Maximum position change per frame
        
        # Set step size
        self.degree_step = 2
        self.xy_step = 0.005
        
        # P control target positions, set to zero position
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }

    def move_to_zero_position(self, robot, steps=50, delay=0.03):
        print(f"[{self.prefix}] Moving to Zero Position gradually ({steps} steps)...")

        # Read current positions
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}

        print(f"[{self.prefix}] Current positions: {current}")

        # Interpolate from current to zero over multiple steps
        # Use shortest-path for wrist_roll to avoid wraparound
        for i in range(1, steps + 1):
            alpha = i / steps
            interp = {}
            for j in self.joint_map:
                start = current[j]
                end = self.zero_pos[j]
                diff = end - start
                # For wrist_roll, take the shortest path (wrap-aware)
                if j == 'wrist_roll':
                    if diff > 180:
                        diff -= 360
                    elif diff < -180:
                        diff += 360
                interp[f"{self.joint_map[j]}.pos"] = start + diff * alpha
            robot.send_action(interp)
            time.sleep(delay)

        # Reset internal state
        self.target_positions = self.zero_pos.copy()
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        self.last_vr_time = 0.0
        self.target_positions["wrist_flex"] = 0.0

    def handle_vr_input(self, vr_goal, gripper_state):
        """
        Handle VR input with delta action control - incremental position updates.
        
        Args:
            vr_goal: VR controller goal data containing target position and orientations
            gripper_state: Current gripper state (not used in current implementation)
        """
        if vr_goal is None:
            return
        
        # VR goal contains: target_position [x, y, z], wrist_roll_deg, wrist_flex_deg, gripper_closed
        if not hasattr(vr_goal, 'target_position') or vr_goal.target_position is None:
            return
            
        # Extract VR position data
        # Get current VR position
        current_vr_pos = vr_goal.target_position  # [x, y, z] in meters
        
        # Initialize previous VR position if not set
        if not hasattr(self, 'prev_vr_pos'):
            self.prev_vr_pos = current_vr_pos
            return  # Skip first frame to establish baseline

        # Calculate relative change (delta) from previous frame
        vr_x = (current_vr_pos[0] - self.prev_vr_pos[0]) * 220
        vr_y = (current_vr_pos[1] - self.prev_vr_pos[1]) * 70
        vr_z = (current_vr_pos[2] - self.prev_vr_pos[2]) * 70

        # If jump is too large (controller reconnect), reset baseline and skip
        if abs(vr_x) > 50 or abs(vr_y) > 50 or abs(vr_z) > 50:
            print(f"[{self.prefix}] Large VR jump detected, resetting baseline")
            self.prev_vr_pos = current_vr_pos
            return

        # Update previous position for next frame
        self.prev_vr_pos = current_vr_pos
        
        # Delta control parameters - adjust these for sensitivity
        pos_scale = 0.01  # Position sensitivity scaling
        angle_scale = 4.0  # Angle sensitivity scaling
        delta_limit = 0.01  # Maximum delta per update (meters)
        angle_limit = 8.0  # Maximum angle delta per update (degrees)
        
        delta_x = vr_x * pos_scale
        delta_y = vr_y * pos_scale  
        delta_z = vr_z * pos_scale
        
        # Limit delta values to prevent sudden movements
        delta_x = max(-delta_limit, min(delta_limit, delta_x))
        delta_y = max(-delta_limit, min(delta_limit, delta_y))
        delta_z = max(-delta_limit, min(delta_limit, delta_z))
        
        self.current_x += -delta_z  # yy: VR Z maps to robot x, change the direction
        self.current_y += delta_y  # yy:VR Y maps to robot y

        # Handle wrist angles with delta control - use relative changes
        if hasattr(vr_goal, 'wrist_flex_deg') and vr_goal.wrist_flex_deg is not None:
            if not hasattr(self, 'prev_wrist_flex'):
                self.prev_wrist_flex = vr_goal.wrist_flex_deg
                return

            delta_pitch = (vr_goal.wrist_flex_deg - self.prev_wrist_flex) * angle_scale
            # Reset baseline on large jump
            if abs(delta_pitch) > 30:
                self.prev_wrist_flex = vr_goal.wrist_flex_deg
                return
            delta_pitch = max(-angle_limit, min(angle_limit, delta_pitch))
            self.pitch += delta_pitch
            self.pitch = max(-90, min(90, self.pitch))
            self.prev_wrist_flex = vr_goal.wrist_flex_deg

        if hasattr(vr_goal, 'wrist_roll_deg') and vr_goal.wrist_roll_deg is not None:
            if not hasattr(self, 'prev_wrist_roll'):
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return

            delta_roll = (vr_goal.wrist_roll_deg - self.prev_wrist_roll) * angle_scale
            # Reset baseline on large jump
            if abs(delta_roll) > 30:
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            delta_roll = max(-angle_limit, min(angle_limit, delta_roll))
            
            current_roll = self.target_positions.get("wrist_roll", 0.0)
            new_roll = current_roll + delta_roll
            new_roll = max(-90, min(90, new_roll))  # Limit roll range
            self.target_positions["wrist_roll"] = new_roll
            
            # Update previous value for next frame
            self.prev_wrist_roll = vr_goal.wrist_roll_deg
        
        # VR Z axis controls shoulder_pan joint (delta control)
        if abs(delta_x) > 0.001:  # Only update if significant movement
            x_scale = 200.0  # Reduced scaling factor for delta control
            delta_pan = delta_x * x_scale
            delta_pan = max(-angle_limit, min(angle_limit, delta_pan))
            current_pan = self.target_positions.get("shoulder_pan", 0.0)
            new_pan = current_pan + delta_pan
            new_pan = max(-180, min(180, new_pan))  # Limit pan range
            self.target_positions["shoulder_pan"] = new_pan
        
        try:
            joint2_target, joint3_target = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            # Smooth transition to new joint positions,  Smoothing factor 0-1, lower = smoother
            alpha = 0.1
            self.target_positions["shoulder_lift"] = (1-alpha) * self.target_positions.get("shoulder_lift", 0.0) + alpha * joint2_target
            self.target_positions["elbow_flex"] = (1-alpha) * self.target_positions.get("elbow_flex", 0.0) + alpha * joint3_target
        except Exception as e:
            print(f"[{self.prefix}] VR IK failed: {e}")
        
        # Calculate wrist_flex to maintain end-effector orientation
        self.target_positions["wrist_flex"] = (-self.target_positions["shoulder_lift"] - 
                                               self.target_positions["elbow_flex"] + self.pitch)
   
        # Handle gripper state directly
        if vr_goal.metadata.get('trigger', 0) > 0.5:
            self.target_positions["gripper"] = 45
        else:
            self.target_positions["gripper"] = 0.0

        # Clamp all joints to safe limits
        for joint, (lo, hi) in JOINT_LIMITS.items():
            if joint in self.target_positions:
                self.target_positions[joint] = max(lo, min(hi, self.target_positions[joint]))

    def p_control_action(self, robot, obs=None):
        """
        Generate proportional control action based on target positions.

        Args:
            robot: Robot instance to get current observations
            obs: Pre-fetched observation dict (avoids redundant bus reads)

        Returns:
            dict: Action dictionary with position commands for each joint
        """
        if obs is None:
            obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


class SimpleHeadControl:
    """
    A class for controlling robot head motors using VR thumbstick input.
    
    Provides simple head movement control with proportional control for smooth operation.
    """
    
    def __init__(self, initial_obs, kp=1):
        self.kp = kp
        self.degree_step = 2  # Move 2 degrees each time
        # Initialize head motor positions
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def handle_vr_input(self, vr_goal):
        # Map VR input to head motor targets
        thumb = vr_goal.metadata.get('thumbstick', {})
        if thumb:
            thumb_x = thumb.get('x', 0)
            thumb_y = thumb.get('y', 0)
            if abs(thumb_x) > 0.1:
                if thumb_x > 0:
                    self.target_positions["head_motor_1"] += self.degree_step
                else:
                    self.target_positions["head_motor_1"] -= self.degree_step
            if abs(thumb_y) > 0.1:
                if thumb_y > 0:
                    self.target_positions["head_motor_2"] += self.degree_step
                else:
                    self.target_positions["head_motor_2"] -= self.degree_step
                    
    def move_to_zero_position(self, robot):
        print(f"[HEAD] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def p_control_action(self, robot, obs=None):
        """
        Generate proportional control action for head motors.

        Args:
            robot: Robot instance to get current observations
            obs: Pre-fetched observation dict (avoids redundant bus reads)

        Returns:
            dict: Action dictionary with position commands for head motors
        """
        if obs is None:
            obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action


def get_vr_base_action(vr_goal, robot):
    """
    Get base control from right thumbstick.
    Forward/back = Y axis, turn left/right = X axis.
    """
    if vr_goal is None or not hasattr(vr_goal, 'metadata'):
        return {"x.vel": 0.0, "theta.vel": 0.0}

    thumb = vr_goal.metadata.get('thumbstick', {})
    if not thumb:
        return {"x.vel": 0.0, "theta.vel": 0.0}

    thumb_x = thumb.get('x', 0)
    thumb_y = thumb.get('y', 0)

    speed_setting = robot.speed_levels[robot.speed_index]
    linear_speed = speed_setting["linear"]
    angular_speed = speed_setting["angular"]

    # Deadzone
    x_cmd = 0.0
    theta_cmd = 0.0
    if abs(thumb_y) > 0.15:
        x_cmd = -thumb_y * linear_speed  # Forward/back
    if abs(thumb_x) > 0.15:
        theta_cmd = thumb_x * angular_speed  # Turn left/right

    return {"x.vel": x_cmd, "theta.vel": theta_cmd}


# Base speed control parameters - adjustable slopes
BASE_ACCELERATION_RATE = 2.0  # acceleration slope (speed/second)
BASE_DECELERATION_RATE = 2.5  # deceleration slope (speed/second)
BASE_MAX_SPEED = 3.0          # maximum speed multiplier


class StallDetector:
    """Monitors motor current and position to detect stalls.
    - Gripper: on stall, reduce to GRIP_TORQUE immediately. No retry.
    - Other joints: retry once, then reduce to STALL_TORQUE until target changes."""

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


def get_vr_speed_control(vr_goal):
    """
    Get speed control from VR input with linear acceleration and deceleration.
    
    Linearly accelerates to maximum speed when holding any base control input,
    and linearly decelerates to 0 when released.
    
    Args:
        vr_goal: VR controller goal data
        
    Returns:
        float: Current speed multiplier (0.0 to BASE_MAX_SPEED)
    """
    global current_base_speed, last_update_time, is_accelerating
    
    # Initialize global variables
    if 'current_base_speed' not in globals():
        current_base_speed = 0.0
        last_update_time = time.time()
        is_accelerating = False
    
    current_time = time.time()
    dt = current_time - last_update_time
    last_update_time = current_time
    
    # Check if any base control input is active from VR
    any_base_input_active = False
    if vr_goal and hasattr(vr_goal, 'metadata'):
        thumb = vr_goal.metadata.get('thumbstick', {})
        if thumb:
            thumb_x = thumb.get('x', 0)
            thumb_y = thumb.get('y', 0)
            # Check if thumbstick is being used for base movement
            any_base_input_active = abs(thumb_x) > 0.2 or abs(thumb_y) > 0.2
    
    if any_base_input_active:
        # VR input active - accelerate
        if not is_accelerating:
            is_accelerating = True
            print("[BASE] Starting acceleration")
        
        # Linear acceleration
        current_base_speed += BASE_ACCELERATION_RATE * dt
        current_base_speed = min(current_base_speed, BASE_MAX_SPEED)
        
    else:
        # No VR input - decelerate
        if is_accelerating:
            is_accelerating = False
            print("[BASE] Starting deceleration")
        
        # Linear deceleration
        current_base_speed -= BASE_DECELERATION_RATE * dt
        current_base_speed = max(current_base_speed, 0.0)
    
    # Print current speed (optional, for debugging)
    if abs(current_base_speed) > 0.01:  # Only print when speed is not 0
        print(f"[BASE] Current speed: {current_base_speed:.2f}")
    
    return current_base_speed


def main():
    """
    Main function for VR teleoperation of XLerobot.
    
    Initializes the robot connection, VR monitoring, and runs the main control loop
    for dual-arm robot control with VR input.
    """
    print("XLerobot VR Control Example")
    print("="*50)
    
    # Initialize pygame for keyboard input handling
    pygame.init()

    try:
        # Choose buses
        print("\nWhich buses to enable?")
        print("  1 = Bus1 only (left arm + head)")
        print("  2 = Bus2 only (right arm + wheels)")
        print("  3 = Both buses")
        bus_choice = input("Enter choice [2]: ").strip() or "2"
        use_bus1 = bus_choice in ("1", "3")
        use_bus2 = bus_choice in ("2", "3")
        ender3_choice = input("Enable Ender 3 Z-axis? [y/N]: ").strip().lower()
        use_ender3 = ender3_choice == "y"

        robot_config = XLerobot2WheelsConfig(id="my_xlerobot_2wheels_lab")
        robot = XLerobot2Wheels(robot_config)

        # Connect only selected buses
        import json as _json
        from pathlib import Path as _Path
        from lerobot.motors import MotorCalibration as _MC

        try:
            if use_bus1:
                robot.bus1.connect()
                robot.bus1.configure_motors(return_delay_time=20)
            if use_bus2:
                robot.bus2.connect()
                robot.bus2.configure_motors(return_delay_time=20)

            # Load calibration
            calib_path = _Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json"
            if calib_path.is_file():
                with open(calib_path) as f:
                    calib_data = _json.load(f)
                if use_bus1:
                    cal = {}
                    for name in robot.bus1.motors:
                        if name in calib_data:
                            c = calib_data[name]
                            cal[name] = _MC(id=c["id"], drive_mode=c["drive_mode"],
                                homing_offset=c["homing_offset"], range_min=c["range_min"], range_max=c["range_max"])
                    robot.bus1.calibration = cal
                    robot.bus1.write_calibration(cal)
                if use_bus2:
                    cal = {}
                    for name in robot.bus2.motors:
                        if name in calib_data:
                            c = calib_data[name]
                            cal[name] = _MC(id=c["id"], drive_mode=c["drive_mode"],
                                homing_offset=c["homing_offset"], range_min=c["range_min"], range_max=c["range_max"])
                    robot.bus2.calibration = cal
                    robot.bus2.write_calibration(cal)

            # Configure active buses
            if use_bus1:
                robot.bus1.disable_torque()
                for name in robot.left_arm_motors + robot.head_motors:
                    robot.bus1.write("Operating_Mode", name, 0)
                    robot.bus1.write("P_Coefficient", name, 16)
                    robot.bus1.write("I_Coefficient", name, 0)
                    robot.bus1.write("D_Coefficient", name, 43)
                    robot.bus1.write("Torque_Limit", name, 600)
                for name in robot.left_arm_motors + robot.head_motors:
                    try:
                        pos = robot.bus1.sync_read("Present_Position", [name], normalize=False)
                        robot.bus1.write("Goal_Position", name, pos[name], normalize=False)
                        robot.bus1.write("Torque_Enable", name, 1)
                    except Exception:
                        print(f"[WARN] Failed to enable {name}")
                    time.sleep(0.05)

            if use_bus2:
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

            print(f"[MAIN] Connected — Bus1={'ON' if use_bus1 else 'OFF'} Bus2={'ON' if use_bus2 else 'OFF'}")
        except Exception as e:
            print(f"[MAIN] Failed to connect: {e}")
            import traceback; traceback.print_exc()
            return

        # Initialize VR monitor
        print("Initializing VR monitor...")
        vr_monitor = VRMonitor()
        if not vr_monitor.initialize():
            print("VR monitor initialization failed")
            return
        print("Starting VR monitoring...")
        vr_thread = threading.Thread(target=lambda: asyncio.run(vr_monitor.start_monitoring()), daemon=True)
        vr_thread.start()
        print("VR system ready")

        # Init arm and head instances
        obs = {}
        if use_bus1:
            try:
                lp = robot.bus1.sync_read("Present_Position", robot.left_arm_motors, num_retry=5)
                obs.update({f"{k}.pos": v for k, v in lp.items()})
                hp = robot.bus1.sync_read("Present_Position", robot.head_motors, num_retry=5)
                obs.update({f"{k}.pos": v for k, v in hp.items()})
            except Exception:
                use_bus1 = False
        if use_bus2:
            rp = robot.bus2.sync_read("Present_Position", robot.right_arm_motors, num_retry=5)
            obs.update({f"{k}.pos": v for k, v in rp.items()})
            wv = robot.bus2.sync_read("Present_Velocity", robot.base_motors, num_retry=5)
            obs.update(robot._wheel_raw_to_body(wv["base_left_wheel"], wv["base_right_wheel"]))
        for prefix in ["left_arm", "right_arm"]:
            for joint in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]:
                key = f"{prefix}_{joint}.pos"
                if key not in obs:
                    obs[key] = 0.0
        for h in ["head_motor_1.pos", "head_motor_2.pos"]:
            if h not in obs:
                obs[h] = 0.0

        kin_left = SO101Kinematics()
        kin_right = SO101Kinematics()
        left_arm = SimpleTeleopArm(LEFT_JOINT_MAP, obs, kin_left, prefix="left")
        right_arm = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, kin_right, prefix="right")
        head_control = SimpleHeadControl(obs)

        # Move active arms to zero
        if use_bus2:
            action = right_arm.p_control_action(robot, obs)
            for name, val in {k.replace(".pos", ""): v for k, v in action.items()}.items():
                try:
                    robot.bus2.write("Goal_Position", name, val)
                except Exception:
                    pass
            print("[OK] right arm at zero")

        # Ender 3
        ender3 = Ender3Controller()
        if use_ender3:
            print("Initializing Ender 3...")
            ender3.connect()

        # Main VR control loop — crash-proof with diagnostics
        print("Starting VR control loop. Press ESC to exit.")
        loop_count = 0
        error_count = 0
        ender3_moves = 0
        stall_detector = StallDetector()
        status_time = time.time()
        running = True

        try:
            while running:
                try:
                    # Get VR controller data
                    dual_goals = vr_monitor.get_latest_goal_nowait()
                    left_goal = dual_goals.get("left") if dual_goals else None
                    right_goal = dual_goals.get("right") if dual_goals else None

                    # Wait for VR connection before proceeding
                    if dual_goals is None:
                        time.sleep(0.01)
                        continue

                    # Skip arm updates while Ender 3 is moving
                    if ender3.ser and ender3.is_moving():
                        time.sleep(0.01)
                        continue

                    # Right arm + wheels only (bus2) — left arm motor damaged
                    right_arm.handle_vr_input(right_goal, gripper_state=None)
                    base_action = get_vr_base_action(right_goal, robot)

                    # Bus2 only reads
                    obs = {}
                    right_pos = robot.bus2.sync_read("Present_Position", robot.right_arm_motors)
                    obs.update({f"{k}.pos": v for k, v in right_pos.items()})
                    wheel_vel = robot.bus2.sync_read("Present_Velocity", robot.base_motors)
                    obs.update(robot._wheel_raw_to_body(wheel_vel["base_left_wheel"], wheel_vel["base_right_wheel"]))

                    right_action = right_arm.p_control_action(robot, obs)

                    # Bus2 only writes
                    for name, val in {k.replace(".pos", ""): v for k, v in right_action.items()}.items():
                        robot.bus2.write("Goal_Position", name, val)
                    base_raw = robot._body_to_wheel_raw(base_action.get("x.vel", 0), base_action.get("theta.vel", 0))
                    robot.bus2.sync_write("Goal_Velocity", base_raw)

                    loop_count += 1
                    time.sleep(0.02)

                    # ─── Ender 3 button controls ─────────────────────
                    if ender3.ser:
                        right_buttons = {}
                        left_buttons = {}
                        if right_goal and hasattr(right_goal, 'metadata') and right_goal.metadata:
                            right_buttons = right_goal.metadata.get('buttons', {}) if isinstance(right_goal.metadata, dict) else {}
                        if left_goal and hasattr(left_goal, 'metadata') and left_goal.metadata:
                            left_buttons = left_goal.metadata.get('buttons', {}) if isinstance(left_goal.metadata, dict) else {}

                        # Z-axis: Right B = up, Left Y(b) = down
                        if right_buttons.get('b'):
                            ender3.handle_button('z_up', ender3.move_z, ENDER3_Z_STEP)
                            ender3_moves += 1
                        if left_buttons.get('b'):
                            ender3.handle_button('z_down', ender3.move_z, -ENDER3_Z_STEP)
                            ender3_moves += 1
                        # Y-axis (bed): Right A = forward, Left X(a) = backward
                        if right_buttons.get('a'):
                            ender3.handle_button('y_fwd', ender3.move_y, ENDER3_Y_STEP)
                            ender3_moves += 1
                        if left_buttons.get('a'):
                            ender3.handle_button('y_back', ender3.move_y, -ENDER3_Y_STEP)
                            ender3_moves += 1

                except (ConnectionError, RuntimeError, OSError, DeviceNotConnectedError) as e:
                    error_count += 1
                    if error_count <= 5 or error_count % 50 == 0:
                        print(f"[WARN] Error #{error_count}: {e}")
                    time.sleep(0.05)
                    continue

                # ─── Diagnostic status line every 10s ─────────────
                now = time.time()
                if now - status_time >= 10:
                    elapsed = now - status_time
                    total = loop_count + error_count
                    err_pct = (error_count / total * 100) if total > 0 else 0
                    vr_ok = "connected" if (left_goal or right_goal) else "waiting"
                    print(f"[STATUS {int(now - status_time)}s] Loops: {loop_count} | Errors: {error_count} ({err_pct:.1f}%) | Ender3: {ender3_moves} moves | VR: {vr_ok}")
                    loop_count = 0
                    error_count = 0
                    ender3_moves = 0
                    status_time = now

                # Handle keyboard exit (press ESC to quit)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False

        finally:
            print("VR teleoperation ended.")
            try:
                robot.disconnect()
            except Exception:
                pass
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        
    finally:
        # Cleanup
        try:
            ender3.disconnect()
        except:
            pass
        try:
            pygame.quit()
        except:
            pass
        try:
            robot.disconnect()
        except:
            pass

if __name__ == "__main__":
    main()
