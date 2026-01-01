#!/usr/bin/env python
"""
Enhanced Robot Teleoperation with Data Recording
Combines your joystick control with automatic data collection
"""

import sys
import os
import pygame
import time
import numpy as np

sys.path.append("..")
from scservo_sdk import *
from data_collector import DataCollector

# ==============================================================================
# 1. SERVO AND COMMUNICATION CONFIGURATION
# ==============================================================================
GRIPPER_ID = 6
BASE_SERVO_ID = 1
SERVO_2_ID = 2
SERVO_3_ID = 3
SERVO_4_ID = 4
SERVO_5_ID = 5 
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'

# Servo limits (same as your original)
GRIPPER_MIN_POSITION = 80
GRIPPER_MAX_POSITION = 875
GRIPPER_MOVING_SPEED = 1500
GRIPPER_MOVING_ACC = 50

BASE_MIN_POSITION = 250
BASE_MAX_POSITION = 1100
BASE_MAX_SPEED = 200

SERVO_2_MIN_POSITION = 1130
SERVO_2_MAX_POSITION = 2400
SERVO_2_MAX_SPEED = 200

SERVO_3_MIN_POSITION = 1370
SERVO_3_MAX_POSITION = 2600
SERVO_3_MAX_SPEED = 200

SERVO_4_MIN_POSITION = 1080
SERVO_4_MAX_POSITION = 2550
SERVO_4_MAX_SPEED = 200

SERVO_5_MIN_POSITION = 900
SERVO_5_MAX_POSITION = 3500
SERVO_5_MAX_SPEED = 200

SERVO_ACC = 150

# ==============================================================================
# 2. JOYSTICK CONFIGURATION
# ==============================================================================
AXIS_0 = 0  # Servo 2
AXIS_1 = 1  # Base
AXIS_2 = 2  # Servo 5
AXIS_3 = 3  # Servo 4
AXIS_4 = 4  # Servo 3
AXIS_5 = 5  # Gripper

BUTTON_HOME = 2
BUTTON_START_RECORD = 0  # 'A' button - Start episode
BUTTON_STOP_RECORD = 1   # 'B' button - End episode

DEADZONE = 0.08

HOME_POSITIONS = {
    BASE_SERVO_ID: 496,
    SERVO_2_ID: 1170,
    SERVO_3_ID: 2232,
    SERVO_4_ID: 2861,
    SERVO_5_ID: 557,      
    GRIPPER_ID: 80
}

HOME_SPEED = 150
HOME_ACC = 100

LOOKAHEAD_MULTIPLIER = 180
SPEED_SMOOTHING_FACTOR = 0.3
MIN_SPEED = 30
STOP_DECEL_DISTANCE = 25
FEEDBACK_READ_INTERVAL = 1

# ==============================================================================
# 3. DATA COLLECTION CONFIGURATION
# ==============================================================================
DATASET_PATH = "./collected_data"
CAMERA_ID = 0
IMAGE_SIZE = (96, 96)
COLLECTION_FPS = 10  # Hz

# ==============================================================================
# 4. UTILITY FUNCTIONS (same as original)
# ==============================================================================
def clip_axis_to_0_1(value):
    return max(0.0, min(1.0, (value + 1.0) / 2.0))

def map_to_servo_position(normalized_value, min_pos, max_pos):
    return int(min_pos + normalized_value * (max_pos - min_pos))

def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    return value

def smooth_speed(current_speed, target_speed, smoothing_factor):
    return current_speed + (target_speed - current_speed) * smoothing_factor

def read_servo_position(packet_handler, servo_id):
    scs_present_position, scs_comm_result, scs_error = packet_handler.ReadPos(servo_id)
    if scs_comm_result == COMM_SUCCESS:
        return scs_present_position
    return None

# ==============================================================================
# 5. SERVO STATE CLASS (same as original)
# ==============================================================================
class ServoState:
    def __init__(self, servo_id, min_pos, max_pos, max_speed):
        self.servo_id = servo_id
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.max_speed = max_speed
        self.current_position = (min_pos + max_pos) // 2
        self.last_target = self.current_position
        self.current_speed = 0.0
        self.is_stopping = False
        self.stop_start_pos = None
        
    def update_position(self, pos):
        if pos is not None:
            self.current_position = pos
    
    def calculate_smooth_motion(self, axis_value, lookahead_mult, smoothing_factor):
        if abs(axis_value) > 0.001:
            self.is_stopping = False
            target_speed_raw = abs(axis_value) * self.max_speed
            target_speed_raw = max(MIN_SPEED, target_speed_raw)
            self.current_speed = smooth_speed(self.current_speed, target_speed_raw, smoothing_factor)
            direction = 1 if axis_value > 0 else -1
            lookahead_distance = int(axis_value * lookahead_mult)
            target_position = self.current_position + lookahead_distance
            target_position = max(self.min_pos, min(self.max_pos, target_position))
            return target_position, int(self.current_speed), False
        else:
            if not self.is_stopping:
                self.is_stopping = True
                self.stop_start_pos = self.current_position
            if self.current_speed > 5:
                decel_speed = max(0, self.current_speed * 0.6)
                self.current_speed = decel_speed
                target_position = self.current_position + int(STOP_DECEL_DISTANCE * (self.current_speed / self.max_speed))
                target_position = max(self.min_pos, min(self.max_pos, target_position))
                return target_position, max(5, int(self.current_speed)), False
            else:
                self.current_speed = 0
                return self.current_position, 0, True

# ==============================================================================
# 6. MAIN PROGRAM
# ==============================================================================
def main():
    # Initialize Pygame
    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()
    LOOP_FREQUENCY = 60

    # Joystick Setup
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick found!")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick: {joystick.get_name()}")

    # Servo Setup
    portHandler = PortHandler(DEVICENAME)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to open port {DEVICENAME}")
        pygame.quit()
        return
    print("✅ Servo port opened")

    # Ping servos
    for servo_id in [GRIPPER_ID, BASE_SERVO_ID, SERVO_2_ID, SERVO_3_ID, SERVO_4_ID, SERVO_5_ID]:
        scs_model, result, error = packetHandler.ping(servo_id)
        if result != COMM_SUCCESS:
            print(f"Cannot find servo ID {servo_id}!")
            portHandler.closePort()
            pygame.quit()
            return
        print(f"✅ Servo ID {servo_id} connected")

    # Initialize servo states
    servo_2_state = ServoState(SERVO_2_ID, SERVO_2_MIN_POSITION, SERVO_2_MAX_POSITION, SERVO_2_MAX_SPEED)
    base_state = ServoState(BASE_SERVO_ID, BASE_MIN_POSITION, BASE_MAX_POSITION, BASE_MAX_SPEED)
    servo_3_state = ServoState(SERVO_3_ID, SERVO_3_MIN_POSITION, SERVO_3_MAX_POSITION, SERVO_3_MAX_SPEED)
    servo_4_state = ServoState(SERVO_4_ID, SERVO_4_MIN_POSITION, SERVO_4_MAX_POSITION, SERVO_4_MAX_SPEED)
    servo_5_state = ServoState(SERVO_5_ID, SERVO_5_MIN_POSITION, SERVO_5_MAX_POSITION, SERVO_5_MAX_SPEED)
    all_servo_states = [base_state, servo_2_state, servo_3_state, servo_4_state, servo_5_state]

    # Read initial positions
    print("\nReading initial positions...")
    for state in all_servo_states:
        pos = read_servo_position(packetHandler, state.servo_id)
        if pos is not None:
            state.update_position(pos)
            state.last_target = pos
        print(f"Servo ID {state.servo_id}: {state.current_position}")

    gripper_current_position = read_servo_position(packetHandler, GRIPPER_ID) or GRIPPER_MIN_POSITION
    print(f"Gripper ID {GRIPPER_ID}: {gripper_current_position}")

    # ==============================================================================
    # DATA COLLECTOR INITIALIZATION
    # ==============================================================================
    data_collector = DataCollector(
        dataset_path=DATASET_PATH,
        camera_id=CAMERA_ID,
        image_size=IMAGE_SIZE,
        fps=COLLECTION_FPS,
        buffer_size=50
    )
    data_collector.start_camera()
    time.sleep(1.0)  # Camera warmup

    # State tracking
    going_home = False
    home_button_pressed_last = False
    start_button_pressed_last = False
    stop_button_pressed_last = False
    last_gripper_position = gripper_current_position
    feedback_counter = 0

    # Previous axis values
    prev_axis_0 = 0.0
    prev_axis_1 = 0.0
    prev_axis_2 = 0.0
    prev_axis_3 = 0.0
    prev_axis_4 = 0.0

    print("\n" + "="*70)
    print("🤖 ROBOT ARM CONTROL WITH DATA RECORDING")
    print("="*70)
    print(f"Button {BUTTON_START_RECORD} (A) - 🎬 START EPISODE RECORDING")
    print(f"Button {BUTTON_STOP_RECORD} (B) - ⏹️  STOP EPISODE RECORDING")
    print(f"Button {BUTTON_HOME} (X) - 🏠 GO TO HOME POSITION")
    print("ESC - Exit and save dataset")
    print("="*70)

    # ==============================================================================
    # MAIN CONTROL LOOP
    # ==============================================================================
    running = True
    last_record_time = time.time()
    
    try:
        while running:
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    running = False

            # Button states
            home_button = joystick.get_button(BUTTON_HOME) if joystick.get_numbuttons() > BUTTON_HOME else False
            start_button = joystick.get_button(BUTTON_START_RECORD) if joystick.get_numbuttons() > BUTTON_START_RECORD else False
            stop_button = joystick.get_button(BUTTON_STOP_RECORD) if joystick.get_numbuttons() > BUTTON_STOP_RECORD else False

            # === RECORDING CONTROLS ===
            if start_button and not start_button_pressed_last:
                if not data_collector.is_recording:
                    data_collector.start_episode()
            start_button_pressed_last = start_button

            if stop_button and not stop_button_pressed_last:
                if data_collector.is_recording:
                    data_collector.end_episode()
            stop_button_pressed_last = stop_button

            # === HOME BUTTON ===
            if home_button and not home_button_pressed_last:
                going_home = True
                # Send all servos to home
                for servo_id, home_pos in HOME_POSITIONS.items():
                    packetHandler.WritePosEx(servo_id, home_pos, HOME_SPEED, HOME_ACC)
                
                # Update states
                for state in all_servo_states:
                    state.current_speed = 0
                    state.last_target = HOME_POSITIONS[state.servo_id]
                    state.current_position = HOME_POSITIONS[state.servo_id]
                
                print("\n🏠 GOING TO HOME POSITION...")
            home_button_pressed_last = home_button

            # === FEEDBACK READING ===
            feedback_counter += 1
            if feedback_counter >= FEEDBACK_READ_INTERVAL:
                feedback_counter = 0
                for state in all_servo_states:
                    pos = read_servo_position(packetHandler, state.servo_id)
                    state.update_position(pos)
                gripper_fb = read_servo_position(packetHandler, GRIPPER_ID)
                if gripper_fb is not None:
                    gripper_current_position = gripper_fb

            # Check if home complete
            if going_home:
                all_at_home = all(abs(state.current_position - HOME_POSITIONS[state.servo_id]) < 50 
                                  for state in all_servo_states)
                if all_at_home:
                    going_home = False
                    print("✅ HOME POSITION REACHED\n")

            if going_home:
                clock.tick(LOOP_FREQUENCY)
                continue

            # === READ AXES ===
            axis_0 = joystick.get_axis(AXIS_0) if joystick.get_numaxes() > AXIS_0 else 0.0
            axis_1 = joystick.get_axis(AXIS_1) if joystick.get_numaxes() > AXIS_1 else 0.0
            axis_2 = joystick.get_axis(AXIS_2) if joystick.get_numaxes() > AXIS_2 else -1.0
            axis_3 = joystick.get_axis(AXIS_3) if joystick.get_numaxes() > AXIS_3 else 0.0
            axis_4 = joystick.get_axis(AXIS_4) if joystick.get_numaxes() > AXIS_4 else 0.0
            axis_5 = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0

            # === SERVO CONTROL (same as original) ===
            # Servo 2
            axis_0_f = apply_deadzone(axis_0, DEADZONE)
            target_pos, target_speed, stopped = servo_2_state.calculate_smooth_motion(axis_0_f, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR)
            if abs(target_pos - servo_2_state.last_target) > 3 or not stopped:
                packetHandler.WritePosEx(SERVO_2_ID, target_pos, target_speed, SERVO_ACC)
                servo_2_state.last_target = target_pos
            prev_axis_0 = axis_0_f

            # Base
            axis_1_f = apply_deadzone(axis_1, DEADZONE)
            target_pos, target_speed, stopped = base_state.calculate_smooth_motion(axis_1_f, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR)
            if abs(target_pos - base_state.last_target) > 3 or not stopped:
                packetHandler.WritePosEx(BASE_SERVO_ID, target_pos, target_speed, SERVO_ACC)
                base_state.last_target = target_pos
            prev_axis_1 = axis_1_f

            # Servo 3
            axis_4_f = apply_deadzone(axis_4, DEADZONE)
            target_pos, target_speed, stopped = servo_3_state.calculate_smooth_motion(axis_4_f, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR)
            if abs(target_pos - servo_3_state.last_target) > 3 or not stopped:
                packetHandler.WritePosEx(SERVO_3_ID, target_pos, target_speed, SERVO_ACC)
                servo_3_state.last_target = target_pos
            prev_axis_4 = axis_4_f

            # Servo 4
            axis_3_f = apply_deadzone(axis_3, DEADZONE)
            target_pos, target_speed, stopped = servo_4_state.calculate_smooth_motion(axis_3_f, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR)
            if abs(target_pos - servo_4_state.last_target) > 3 or not stopped:
                packetHandler.WritePosEx(SERVO_4_ID, target_pos, target_speed, SERVO_ACC)
                servo_4_state.last_target = target_pos
            prev_axis_3 = axis_3_f

            # Servo 5
            axis_2_clipped = max(0.0, (axis_2 + 1.0) / 2.0)
            axis_2_f = apply_deadzone(axis_2_clipped, DEADZONE)
            target_pos, target_speed, stopped = servo_5_state.calculate_smooth_motion(axis_2_f, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR)
            if abs(target_pos - servo_5_state.last_target) > 3 or not stopped:
                packetHandler.WritePosEx(SERVO_5_ID, target_pos, target_speed, SERVO_ACC)
                servo_5_state.last_target = target_pos
            prev_axis_2 = axis_2_f

            # Gripper
            gripper_control = clip_axis_to_0_1(axis_5)
            target_gripper = map_to_servo_position(gripper_control, GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION)
            if abs(target_gripper - last_gripper_position) >= 5:
                packetHandler.WritePosEx(GRIPPER_ID, target_gripper, GRIPPER_MOVING_SPEED, GRIPPER_MOVING_ACC)
                last_gripper_position = target_gripper

            # === DATA RECORDING ===
            current_time = time.time()
            if data_collector.is_recording and (current_time - last_record_time) >= (1.0 / COLLECTION_FPS):
                # Get current state (all 6 servo positions)
                state = np.array([
                    base_state.current_position,
                    servo_2_state.current_position,
                    servo_3_state.current_position,
                    servo_4_state.current_position,
                    servo_5_state.current_position,
                    gripper_current_position
                ], dtype=np.float32)

                # Get actions (last commanded positions)
                action = np.array([
                    base_state.last_target,
                    servo_2_state.last_target,
                    servo_3_state.last_target,
                    servo_4_state.last_target,
                    servo_5_state.last_target,
                    last_gripper_position
                ], dtype=np.float32)

                # Record
                data_collector.record_step(action, state)
                last_record_time = current_time

            clock.tick(LOOP_FREQUENCY)

    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")

    finally:
        print("\n🛑 Shutting down...")
        
        # Stop servos smoothly
        for state in all_servo_states:
            pos = read_servo_position(packetHandler, state.servo_id)
            if pos is not None:
                packetHandler.WritePosEx(state.servo_id, pos, 10, SERVO_ACC)
        
        time.sleep(0.2)
        
        # Close connections
        data_collector.close()
        portHandler.closePort()
        pygame.quit()
        
        # Show final stats
        data_collector.get_stats()
        
        print("✅ Shutdown complete")

if __name__ == "__main__":
    main()